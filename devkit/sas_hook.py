# -*- coding: utf-8 -*-
"""SAS v2 闭环自仿真 hook 入口（执行合同=docs/策略/规控-SASv2闭环自仿真执行规格-1002.md）。

挂点=simple_control.process 内全部 v_cmd 剂量消费后、IDM 前（MINE_EXACT/stpp 块之后）。
- off（缺省）：调用点短路，本模块不被 import → G0 字节中性。
- log：每 commit 窗建计划+审计落盘，v_cmd 原样（shadow/G2 数据源）。
- on：胜者接管 v_cmd（∈[0.1,12]）；原族继续消费 IDM，新跟踪族消费完整
  评分通过的加速度政策，均保留 jerk/rot。SAS 不改 ctrl._stpp_exec_v。
终版通道=scen_config.json 顶层 "sas": {"enabled": true}（平台零 env）；
回滚杆=config 置 false（无需改码重建）。

fail-closed 三层：调用点 try/except + 本入口 try/except + 引擎 None 计划回银行。
场景无关红线：本模块与 sas_core 零逐场名分支（审计日志字段除外）。
"""
import json
import os

# config 旗标缓存（None=未读；False/False 之外只读一次文件）
_CFG_FLAG = None
_MODE_CACHE = None  # config 激活时的生效模式（on/log；env 显式时不经此）


def cfg_enabled():
    """scen_config.json 顶层 sas.enabled（MINE_CFG 搜索链，与 simple_control 同序）。

    predictor.py 的 stash 条件共用本函数（单一事实源）；模块级仅 stdlib，
    可被 predictor 无副作用 import。
    """
    global _CFG_FLAG
    if _CFG_FLAG is None:
        flag = False
        for p in ([x for x in (
                os.environ.get("MINE_CFG", ""),
                os.environ.get("MINE_SCEN_CONFIG", ""),
                os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "centerlines", "scen_config.json"),
                os.path.join(os.getcwd(), "outputs", "scen_config.json"),
        ) if x]):
            if os.path.exists(p):
                try:
                    with open(p) as f:
                        flag = bool(json.load(f).get("sas", {}).get("enabled", False))
                except Exception:
                    flag = False
                break
        _CFG_FLAG = flag
    return _CFG_FLAG


def cfg_reload():
    """config 变更后强制重读（本地实验用；跑批内不调用）。"""
    global _CFG_FLAG
    _CFG_FLAG = None
    return cfg_enabled()


# 逐场上下文（name -> sas_core.SceneCtx；仿真进程一次一场，键控隔离）
_CTX = {}


def vcmd(ctrl, v_cmd, observation, path_waypoints, traj_future, mode="off"):
    """SAS 入口（call-site try/except 之上的第二层 fail-closed）。"""
    try:
        # ① rollout 副本候选注入（replica._sas_cand=Cand；同码链内短路——
        #    副本绝不进入下方重规划路径，递归结构性不存在）
        _c = getattr(ctrl, "_sas_cand", None)
        if _c is not None:
            if getattr(_c, "stpp_bank_shadow", False):
                from stpp_geometry import consume_bank
                return consume_bank(ctrl, v_cmd, observation, path_waypoints, traj_future, _c)
            if getattr(_c, "tracking", False) and hasattr(_c, "steering"):
                ctrl._stpp_exec_v = None
            v = _c(ctrl, v_cmd,
                   float(observation.get("test_setting", {}).get("t", 0.0)))
            ctrl._sas_exec_policy = "track" if getattr(_c, "tracking", False) else None
            ctrl._sas_geometry_candidate = _c if hasattr(_c, "steering") else None
            if getattr(ctrl, "_local_exit_capture_state", None) is not None:
                from local_exit_execution import record_command
                record_command(ctrl, _c, v,
                               float(observation.get("test_setting", {}).get("t", 0.0)))
            return v
        # ② stpp 接管帧让位（标记=stpp_hook 置位）；stpp_scenes 接管场整场让位
        #    （银行=stpp+exec 赢家，SAS 不混变量）
        if getattr(ctrl, "_stpp_exec_v", None) is not None:
            return v_cmd
        name = observation.get("test_setting", {}).get("scenario_name", "")
        try:
            import stpp_hook as _st
            if _st._takeover(name):
                return v_cmd
        except Exception:
            pass
        # ③ 模式解析：env 显式 > config（enabled→on / log→log）；进程内缓存一次
        m = mode if mode in ("log", "on") else None
        if m is None:
            global _MODE_CACHE
            if _MODE_CACHE is None:
                try:
                    with open(_first_cfg_path()) as _f:
                        _sc = json.load(_f).get("sas", {})
                    _MODE_CACHE = "log" if _sc.get("log") else "on"
                except Exception:
                    _MODE_CACHE = "on"
            m = _MODE_CACHE
        ctx = _CTX.get(name)
        if ctx is None:
            # 跨场景拆台：新场名到来=旧 SceneCtx（含 BitMap 全图 bool ~300-450MB）
            # 不再回访，主动释放防单进程多场（docker 40 场一进程）RSS 线性累积；
            # 单场进程首调用时 _CTX 为空，clear() 空操作=零行为差。
            _CTX.clear()
            import sas_core  # 激活才 import（numpy/pandas 延后进镜像进程）
            ctx = sas_core.SceneCtx(ctrl, observation, path_waypoints)
            _CTX[name] = ctx
        return ctx.vcmd(ctrl, v_cmd, observation, path_waypoints,
                        traj_future, m)
    except Exception:
        ctrl._sas_exec_policy = None
        ctrl._sas_geometry_candidate = None
        if os.environ.get("MINE_SAS_DBG", "0") == "1":
            import traceback
            traceback.print_exc()
        return v_cmd


def _first_cfg_path():
    for p in ([x for x in (
            os.environ.get("MINE_CFG", ""),
            os.environ.get("MINE_SCEN_CONFIG", ""),
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "centerlines", "scen_config.json"),
            os.path.join(os.getcwd(), "outputs", "scen_config.json"),
    ) if x]):
        if os.path.exists(p):
            return p
    return ""


def reset():
    """跨场景清理（__main__ 单进程逐场串行时无需调用；测试用）。"""
    _CTX.clear()
