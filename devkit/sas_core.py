# -*- coding: utf-8 -*-
"""SAS v2 闭环自仿真引擎（sas_hook 的实现层）。

执行合同=docs/策略/规控-SASv2闭环自仿真执行规格-1002.md §3。
本文件与波1（离线静态剂量筛查，10-02 终判死）无共享代码——v2 是
simple_control v_cmd 挂点内的在线闭环 rollout-argmax：

    每 commit 窗：对 K 个场景无关速度剖面候选，用「背景车 GT 未来 +
    MotionController 同码全链副本 + 逐位复刻植物/终局判定 + score_embed
    官方评分」滚到终局 → argmax → 胜者剖面接管本窗 v_cmd。

保真策略（R2 的结构性防御）：滚动体=执行体本身——rollout 逐步调用
replica.process()（真实控制链原码），植物/终局逐行照抄 controller.py
（_action_cheaker/_update_ego_and_t/_update_other_vehicles_to_t/
_update_end_status/_collision_detect/_get_poly），CACV 合成逐行照抄
predictor.py 递推（含舍入）。计划-执行失真（SIPP/exec 死因）结构性不存在。

场景无关红线：零逐场名分支（map 二分 jiangtong/dapai 与 scorer 的
stem 推断同款，属图级路由非逐场键）。fail-closed：候选异常→弃选；
全灭/引擎异常→银行 v_cmd（零下界）。

v1 注意（D1/D2 形态）：标量同码路径，单场 ~分钟级——只用于 shadow/G2
保真门；G3 算力门前必须完成向量化（K 候选 numpy 并行），勿直接上 40 场。
"""
import copy as _copy
import contextlib
import io
import json
import math
import os
import time
from pathlib import Path

import numpy as np

DT = 0.1
V_LO, V_HI = 0.1, 12.0
COMMIT_K = 20            # 重规划节拍（步=1.0s）# v45 裁决 A：10→20（G1 中性门 PASS 后翻转；节拍减半=窗数 ×~0.55、9_3_5 恒速型胜者不敏感）
HYST = 0.15              # 挑战者边际（分）
KEEP_CEIL = 100.0 - HYST + 1e-9  # 严格高于此界，任何挑战者均无法过迟滞
MIN_SWITCH = 5           # 最小切换间隔（步；熔断触发不受限）
SWITCH_WARN = 15         # 单场切换数报警线
N_MAX_CAP = 660          # 66s/0.1 硬顶（max_t 最大 66s）
WIN_MS_SOFT = 25000      # 单窗软预算（ms）：超则下一窗缩族
F1_SPEEDS = (5.5, 6.5, 7.25, 8.0, 9.0, 10.5, 12.0)
F2_VGO = (8.0, 10.5, 12.0)
F3_DELAYS = (0.5, 1.5, 3.0, 5.0)  # v44 起停用（F3 仅 2/1291 窗胜=g4d 归档 38 场
                                # 日志实数，v45 D 项勘误；at() 分支保留供回退）
# v44 F4 曲率自适应（F3 四槽换血；dapai 3_x/8_x S 池主轴）：v(s)=clip(c·√(F4_ALAT/κ_la(s)))
# c×前瞻 L_t 四档；κ 从静态路径折线窗首一次预计算（SceneCtx._f4_cands，零逐场键）
F4_GRID = ((0.85, 1.5), (0.85, 3.0), (1.0, 1.5), (1.0, 3.0))  # (c, L_t s)
F4_ALAT = 0.39   # 银行 eslat 实证剂量（v²κ=0.39<0.4 阈值全合规；V_LO=0.1 破 eslat 地板 3.0 盲区）
F4_ADEC = 2.5    # 剖面后向减速传播（m/s²，物理可实现性；IDM 在环照常平滑）

# ---- v41 事故防御（v40 平台 5_4_3 整发作废；归档=platform_eval/
# v40-sas_2026-10-04_0021_fail_raw.txt）----
CEIL_EXIT = 99.995           # F0 满分窗提前退出阈（总分上限 100；>100+HYST 不存在）
SAT_STREAK = 3               # v43 饱和停靠：连续 N 个「F0 满分窗」→ 整场回银行
SCENE_BUDGET_MS = 240_000.0  # 单场 SAS rollout 墙钟预算（r2：90s 首跑误熔断 15 场
                                # ——重场合法花销 123-173s（G3 容器实测），240s=其上
                                # 留 65s 余量；风暴场仍被绑 ≤~4min 后回银行）
GLOBAL_BUDGET_MS = 7_600_000.0  # 进程级总预算（v42：v41 首读实测 5400s 触帽于
                                # #139【v44 法医勘误（v45 D 项）：v41 帽实为
                                # ∈(#124,#125]；#124-138 满分 ceiling 场 Δ=0 与
                                # 银行不可分辨，#139 系当时可辨下界；下文推导
                                # 沿用旧口径，量级结论不变】
                                # → 尾段 44 场还需 ~1980s → 不触帽全量 ≈7380s
                                # ±15%。7200 会在 9_3_x 块（#174-180=9_3_5 +21.16 所在）
                                # 触帽；7600=不触帽概率 ~60%+最坏触帽墙钟 7600+1520
                                # ≈2.53h<死亡区 ~2.7h（8400 最坏 2.67h 拒）。本地 G4
                                # Σ1391s/容器 1919s<两帽 → 门内零行为差（可析证明））

_GLOBAL_MS = 0.0             # 进程级累计（平台单进程语义；本地分 shard 各自累计）

# D3 加速（同码链零改写；全部无损，验收=G2 F0 recs 逐位不变）
FORK_ENABLED = os.environ.get("MINE_SAS_FORK", "1") == "1"
MAX_WORKERS = max(1, min(15, (os.cpu_count() or 4) - 1))
CHILD_TIMEOUT_S = 240.0


def _child_roll(pipe_w, sctx, ctrl, observation, cand, step):
    """fork 子进程：滚单候选并回传。args 经 fork 继承免 pickle；
    os._exit(0) 跳过 atexit/stdio 冲刷（继承的缓冲不得双写）。"""
    try:
        r = sctx._rollout(ctrl, observation, cand, step)
        if r is None:
            msg = ("dead", None)
        else:
            lite = {"score": r["score"], "end": r["end"],
                    "arrival": r["arrival"], "recs": r["recs"],
                    "detail": r["detail"]}
            # 串行与fork回传同一在线未来后缀；关闭模式不新增载荷。
            if "retiming_ego_rows" in r:
                lite["retiming_ego_rows"] = r["retiming_ego_rows"]
            if "braking_execution_audit" in r:
                lite["braking_execution_audit"] = r["braking_execution_audit"]
            if "local_exit_execution_audit" in r:
                lite["local_exit_execution_audit"] = r["local_exit_execution_audit"]
            msg = ("ok", lite)
    except Exception:
        msg = ("dead", None)
    try:
        pipe_w.send(msg)
        pipe_w.close()
    except Exception:
        pass
    os._exit(0)

_DEVKIT = Path(__file__).resolve().parent
_LAB_ROOT = _DEVKIT.parent


def _datasets_dir() -> str:
    return os.environ.get("MINE_DATASETS") or str(_LAB_ROOT / "datasets")


def _stpp_takeover(name):
    """stpp_scenes 接管场：SAS 整场让位（银行=stpp+exec 赢家，不混变量）。"""
    try:
        import stpp_hook
        return stpp_hook._takeover(name)
    except Exception:
        return False


class Cand:
    """速度剖面候选。at(t, bank_v)=剖面在 t 的 v_cmd（F0/F3 放行后=银行链值）。
    同一实例既做 rollout 注入（sas_hook 短路），也做实跑本窗取值。
    F4：p/q 复用为 (t_abs, v) 数组（0.1s 网格时间表），np.interp 纯查表
    ——rollout 副本与实跑同表=闭环一致（表由窗首 _f4_cands 一次性构建）。"""

    __slots__ = ("label", "kind", "p", "q")

    def __init__(self, label, kind, p=None, q=None):
        self.label, self.kind, self.p, self.q = label, kind, p, q

    def at(self, t, bank_v):
        k = self.kind
        if k == "F0":
            return bank_v
        if k == "F1":
            return self.p
        if k == "F2":
            return 0.1 if t < self.p else self.q
        if k == "F3":
            return bank_v if t >= self.p else 0.1
        if k == "F4":
            return float(np.interp(t, self.p, self.q))
        return bank_v

    def __call__(self, ctrl, bank_v, t):
        return self.at(t, bank_v)


class SpatialCand:
    """速度包络按执行轨迹的空间进度取值；数组全程只读共享。"""

    kind = "F5"

    def __init__(self, original, rs, rx, ry, ss):
        if len(ss) != len(original.q):
            raise ValueError("空间剖面长度不一致")
        self.label = "SP_EXEC_" + original.label
        self.rs, self.rx, self.ry = rs, rx, ry
        self.ss, self.speeds = ss, original.q
        for a in (rs, rx, ry, ss, self.speeds):
            a.setflags(write=False)

    def __call__(self, ctrl, bank_v, t):
        near = int(np.argmin((self.rx - ctrl._ego_x) ** 2
                             + (self.ry - ctrl._ego_y) ** 2))
        return float(np.interp(self.rs[near], self.ss, self.speeds))


class TrackingCand:
    """带加速度跟踪的完整闭环政策，须经真足迹评分参选。"""

    kind = "F6"
    tracking = True

    def __init__(self, base):
        self.base = base
        self.label = "TRACK_" + base.label

    def __call__(self, ctrl, bank_v, t):
        return self.base(ctrl, bank_v, t)


class SceneCtx:
    """逐场上下文：静态快照 + 前缀记录 + 窗缓存 + 重规划状态机。"""

    def __init__(self, ctrl, observation, path_waypoints):
        ts = observation["test_setting"]
        self.name = ts.get("scenario_name", "")
        self.t0 = float(ts.get("t", 0.0))
        self.dt = float(ts.get("dt", 0.1))
        self.max_t = float(ts.get("max_t", 66.0))
        self.goal = {"x": list(ts["goal"]["x"]), "y": list(ts["goal"]["y"])}
        self._ts_tmpl = dict(ts)  # 合成 observation 的 test_setting 模板（t/end 逐合成步覆写）
        self.path = path_waypoints  # planner 一次性产物（__main__:194），全程只读共享
        self.ego_shape = dict(observation["vehicle_info"]["ego"]["shape"])
        self.traj = None            # 首步从 stpp_hook._TRAJ_LATEST 取
        self.prefix = []            # [t,x,y,yaw,v]（真实前缀=官方 CSV 行语义）
        self.bank_seq = {}          # step -> 银行链 v_cmd（F3 参照/审计）
        self.plan = None            # (cand, score, recs{step:(x,y)})
        self.last_plan_step = -10**9
        self.last_switch_step = -10**9
        self.n_switch = 0
        self._win_ms = 0.0
        self._shrink = False
        self._last_bg_set = None
        self._road = None           # (lookup, image, x0, y0) 惰性
        self._exp_t = None
        self._exp_src = ""
        self._audit = None
        self._dead_note = None
        self._sas_off = None    # v41 场级熔断原因（offpath/scene_budget/global_budget）
        self._scene_ms = 0.0    # v41 单场 rollout 墙钟累计
        self._ceil_streak = 0   # v43 饱和停靠：连续 F0 满分窗计数
        # D3：合成表按 key3 字符串跨窗复用（_synth 输出只依赖 key3/round2 键，
        # 0.1 网格时刻对 ulp 噪声双舍入稳定→键同则值逐位同）；bg 凸包同理
        self._memo = {}     # key3 -> (bg_vi, tf)
        self._bpolys = {}   # (vid, key3) -> shapely 凸包
        self._f4_kap = None  # v44 κ 表逐场缓存 (rs, kap, rx, ry)——path 静态，窗首只重算进度+剖面
        self._exec_enabled = bool(ctrl._sas_cfg.get("execution_profile", False))
        self._exec_ready = False
        self._exec_candidates = []  # 首窗胜过原族才缓存；后窗最多增一个政策
        self._trace_enabled = bool(ctrl._sas_cfg.get("trajectory_tracking", False))
        self._geometry_enabled = bool(ctrl._sas_cfg.get("hairpin_geometry", False))
        self._trace_refreshes = 0
        self._trace_last_t = None
        self._bezier_enabled = bool(ctrl._sas_cfg.get("bezier_geometry", False))
        self._bezier_ready = False
        self._goal_pose_enabled = bool(ctrl._sas_cfg.get("goal_pose_geometry", False))
        self._goal_pose_ready = False
        self._event_enabled = bool(ctrl._sas_cfg.get("event_geometry", False))
        self._event_ready = False
        self._bootstrap_enabled = bool(ctrl._sas_cfg.get("bootstrap_route", False))
        self._bootstrap_ready = False
        self._retiming_enabled = bool(ctrl._sas_cfg.get("executed_retiming_ff", False))
        self._full_ff_enabled = bool(ctrl._sas_cfg.get("full_ff_enabled", False))
        self._full_ff_ready = False
        self._braking_enabled = bool(ctrl._sas_cfg.get("braking_reachability_enabled", False))
        self._braking_ready = False
        self._retiming_ready = False
        self._retiming_collect_rows = False
        self._retiming_pending = []
        self._local_exit_enabled = bool(ctrl._sas_cfg.get("local_exit_enabled", False))
        self._local_exit_ready = False
        self._local_exit_observed = []
        self._local_exit_capture_t = None
        self._full_trace_sqp_enabled = bool(ctrl._sas_cfg.get("full_trace_sqp_enabled", False))
        self._full_trace_sqp_ready = False
        self._full_trace_sqp_diagnostic = None

    # ---------------- 审计 ----------------
    def _log(self, obj):
        try:
            if self._audit is None:
                d = _LAB_ROOT / "outputs" / "sas_log"
                d.mkdir(parents=True, exist_ok=True)
                self._audit = open(d / f"{self.name}.jsonl", "a")
            self._audit.write(json.dumps(obj, ensure_ascii=False) + "\n")
            self._audit.flush()
        except Exception:
            self._audit = None

    # ---------------- 每步入口（sas_hook 调） ----------------
    def vcmd(self, ctrl, v_cmd, observation, path_waypoints, traj_future, mode):
        ts = observation["test_setting"]
        t = float(ts.get("t", 0.0))
        step = int(round((t - self.t0) / self.dt))
        ego = observation["vehicle_info"]["ego"]
        self.prefix.append([t, float(ego["x"]), float(ego["y"]),
                            float(ego["yaw_rad"]), float(ego["v_mps"])])
        self.bank_seq[step] = float(v_cmd)

        # v41 离径熔断（防御② r3=终形）：真实 ego 预瞄失败（simple_control 全
        # 路径无点 < prev_dist=14+1.5v，floor 11m）= v40 事故的精确签名 → SAS
        # 本场整场让位回银行（F1 已把该分支跟踪回退到最近路径点=恢复弧）。
        # r1/r2 的距离代理（OFFPATH_M=8m，r2 加内部点守卫）被 g4c 串行批实测
        # 两处误杀后废除：r1 归因"端点大 d（nid=0/n-1）"系未验假设（r1 事件
        # 无 nid 字段），r2 补记 nid 后实锤两处均为**内部点**——1_3_10 t=0
        # d=9.28@nid=1618/2336（并入段出身几何，预瞄成功）、9_3_5 t=27.6
        # d=8.21@nid=1369/1960（F1@12 满速胜者线程合法割角，v40-G4 同位
        # =100 分）。距预瞄死区尚远即熔断=用猜测替代信号；本触发=预瞄搜索
        # 结果原样布尔，场景无关、零假阳性（公开 40 银行 33 轮+SAS G4 零该
        # 分支触达=死分支证明）。sticky（fail-closed 方向=银行=零下界）。
        if self._sas_off is None and getattr(ctrl, "_preview_fail", False):
            self._sas_off = "preview_fail"
            self.plan = None
            self._log({"t": t, "ev": "sas_off", "why": "preview_fail"})
        if self._sas_off is not None:
            # 几何已偏离银行路径时，预算停止重规划但继续本窗已完整验证的
            # 安全轨迹。强制切回银行会失去该几何的安全保证。
            if (self._sas_off in ("scene_budget", "global_budget") and mode == "on"
                    and self.plan is not None and hasattr(self.plan[0], "steering")):
                cand = self.plan[0]
                ctrl._sas_exec_policy = "track"
                ctrl._sas_geometry_candidate = cand
                return min(V_HI, max(V_LO, float(cand(ctrl, v_cmd, t))))
            return v_cmd

        if self.traj is None:
            try:
                import stpp_hook
                self.traj = stpp_hook._TRAJ_LATEST.get(self.name)
            except Exception:
                self.traj = None
            if self.traj is None:
                if self._dead_note is None:
                    self._dead_note = "no_traj"
                    self._log({"t": t, "ev": "fail_closed", "why": "traj stash absent"})
                return v_cmd
        if self._exp_t is None:
            self._init_exp_time(ctrl)

        breaker = self._breaker(observation, step)
        if step - self.last_plan_step >= COMMIT_K or breaker:
            t_start = time.perf_counter()
            try:
                self._replan(ctrl, observation, step, t, mode, breaker)
            except Exception:
                if os.environ.get("MINE_SAS_DBG", "0") == "1":
                    import traceback
                    traceback.print_exc()
                self.plan = None  # fail-closed 全窗回银行
                self.last_plan_step = step  # r3（H7）：异常同样 10 步退避
            self._win_ms = (time.perf_counter() - t_start) * 1000.0
            # v41 双层墙钟预算（防御③）：单场 >240s 或进程累计 >1.5h → SAS
            # 剩余窗口全部让位回银行。单场帽根除风暴场（v40 的 5_4_3 一场
            # 12-17min）；全局帽给平台 183 场单进程一个 ≤~2h 的硬上界
            # （超发先例全是 ~30-60min 银行速度，SAS 全量首触真实上限）。
            # r3（对抗审计 H7）：武装门去掉 plan 非None 条件——all_dead/异常
            # 路径 plan=None 时 _scene_ms 照计但旧门永不熔断=残留 10Hz 全族
            # 扇出无界风暴类（历史 46 日志零 all_dead，但正是 v40 事故等级）。
            # 配套：all_dead/异常路径推进 last_plan_step=10 步退避（预算兜底）。
            global _GLOBAL_MS
            self._scene_ms += self._win_ms
            _GLOBAL_MS += self._win_ms
            if self._sas_off is None:
                why = None
                if self._scene_ms > SCENE_BUDGET_MS:
                    why = "scene_budget"
                elif _GLOBAL_MS > GLOBAL_BUDGET_MS:
                    why = "global_budget"
                if why is not None:
                    self._sas_off = why
                    retain_geometry = self.plan is not None and hasattr(self.plan[0], "steering")
                    if not retain_geometry:
                        self.plan = None
                    self._log({"t": t, "ev": "sas_off", "why": why,
                               "retained_geometry": retain_geometry,
                               "scene_ms": round(self._scene_ms, 0),
                               "global_ms": round(_GLOBAL_MS, 0)})

        if mode == "on" and self.plan is not None:
            v = self.plan[0](ctrl, v_cmd, t)
            ctrl._sas_exec_policy = "track" if getattr(self.plan[0], "tracking", False) else None
            ctrl._sas_geometry_candidate = self.plan[0] if hasattr(self.plan[0], "steering") else None
            return min(V_HI, max(V_LO, float(v)))
        return v_cmd

    # ---------------- 触发器（3.3 事件触发） ----------------
    def _breaker(self, observation, step):
        # v45（裁决 C）：返回类型字符串化（"drift"/"bg"/None）供审计日志归因——
        # 所有调用点只消费真值（if/or/bool()），None 与 False 同假、非空串同真，
        # 控制流逐位不变（G1 中性门实证）。
        if self.plan is None:
            return None
        cand, _score, recs = self.plan
        exp = recs.get(step)
        if exp is not None:
            ego = observation["vehicle_info"]["ego"]
            if math.hypot(ego["x"] - exp[0], ego["y"] - exp[1]) > 1.5:
                return "drift"  # 实测熔断：保真漂移超阈
        bg_now = frozenset(k for k in observation["vehicle_info"] if k != "ego")
        if self._last_bg_set is not None and bg_now != self._last_bg_set:
            return "bg"  # 前车集合突变
        return None

    # ---------------- expected_time（官方优先，自算兜底） ----------------
    def _init_exp_time(self, ctrl):
        try:
            off = (ctrl._expected_times or {}).get(self.name)
            if off:
                self._exp_t, self._exp_src = float(off), "official"
                return
        except Exception:
            pass
        cx = float(np.mean(self.goal["x"]))
        cy = float(np.mean(self.goal["y"]))
        p0 = self.prefix[0] if self.prefix else [self.t0, 0.0, 0.0, 0.0, 0.0]
        self._exp_t = round(math.hypot(cx - p0[1], cy - p0[2]) / 5.0, 1)
        self._exp_src = "chord5"

    # ---------------- 候选族（3.2；波1 死轴全部族外） ----------------
    def _families(self, step, t):
        cands = [Cand("F0", "F0")]
        cands += [Cand(f"F1@{v:g}", "F1", v) for v in F1_SPEEDS]
        taus = self._seed_taus(t)
        if not self._shrink:
            cands += [Cand(f"F2:{tau:g}+{vg:g}", "F2", tau, vg)
                      for tau in taus for vg in F2_VGO]
        else:
            cands += [Cand(f"F2:{taus[0]:g}+8", "F2", taus[0], 8.0)] if taus else []
        cands += self._f4_cands(t)   # v44：F3×4 → F4 曲率自适应×4；族基数结构性有界
        cands += self._exec_candidates
        # 基础族 ≤21；缓存新政策与真实在位者最多再占两槽。首窗额外筛选
        # 分阶段进行。算力由筛选和迟滞上界短路控制，保留 F0/真实在位者，
        # 禁止头部切片导致 F4 或在位者被无条件砍掉。
        return cands

    def _execution_candidates(self, reference, t):
        """用安全基线的真实执行轨迹重建曲率；规划路径与转向链保持。

        只在首个非满分窗口筛选四档空间剖面和三个标量跟踪政策，避免
        每个 commit 窗重复展开离线网格。后续始终比较真实在位续跑分。
        """
        out = [TrackingCand(Cand("F0", "F0")),
               TrackingCand(Cand("F1@8", "F1", 8.0)),
               TrackingCand(Cand("F1@12", "F1", 12.0))]
        xy = [[self.prefix[-1][1], self.prefix[-1][2]]]
        xy += [list(p) for _, p in sorted(reference["recs"].items())]
        old_path, old_kap = self.path, self._f4_kap
        try:
            self.path, self._f4_kap = xy, None
            profiles = self._f4_cands(t)
            execution_kap = self._f4_kap
        finally:
            self.path, self._f4_kap = old_path, old_kap
        if execution_kap is not None:
            rs, kap, rx, ry = execution_kap
            near = int(np.argmin((rx - xy[0][0]) ** 2 + (ry - xy[0][1]) ** 2))
            ss = rs[1:-1][rs[1:-1] >= rs[near] - 0.5]
            out += [TrackingCand(SpatialCand(c, rs, rx, ry, ss)) for c in profiles]
        if self._trace_enabled and reference["end"] == 4:
            self._trace_last_t = t
            from trajectory_candidates import TraceCand
            for coefficient in (.85, 1.):
                for look in (6., 10., 16.):
                    try:
                        out.append(TraceCand(xy, coefficient, look, t))
                    except Exception as error:
                        self._log({"ev": "policy_drop", "t": t, "kind": "trace",
                                   "error_type": type(error).__name__})
        if self._geometry_enabled:
            from trajectory_candidates import hairpin_candidates
            try:
                out += hairpin_candidates(self, t, Path(_datasets_dir()))
            except Exception as error:
                self._log({"ev": "policy_drop", "t": t, "kind": "hairpin",
                           "error_type": type(error).__name__})
        return out

    # ---------------- v44 F4 曲率自适应候选（窗首一次预计算）----------------
    def _f4_cands(self, t):
        """静态路径折线 → κ(s)（2m 重采样+三点外接圆）→ 限速剖面（前瞻窗 max 包络
        迭代+后向减速传播）→ 弧长时间表。κ 表逐场缓存；ego 进度每窗重取。
        全程场景无关（只吃 self.path 几何）；异常/短路径返空=fail-safe 少 4 候选。
        单测三勘误（tmp_scripts/test_f4_profile.py）：①后向传播符号（−c·idx 版把
        长直路起点拖到 V_LO）；②前瞻必须是窗 [i,i+la] 的 κ max 包络——点采样 κ(s+la)
        在曲线尾段自洽坏不动点（快→看得远→窗过曲线末端→见直线→不减速，混合场弧尾
        6 格 v=12 实锤）；③ego 进度=重采样折线最近点（rs 是弧长，首版 hypot(rs−x,
        rs−y) 把弧长当坐标，圆弧场 s_now 误判 +10m）。"""
        try:
            if self._f4_kap is None:
                pts = np.asarray([[w[0], w[1]] for w in self.path], dtype=float)
                if len(pts) < 8:
                    return []
                seg = np.hypot(*(pts[1:] - pts[:-1]).T)
                s = np.concatenate([[0.0], np.cumsum(seg)])
                if float(s[-1]) < 20.0:
                    return []
                rs = np.arange(0.0, float(s[-1]), 2.0)   # 2m 重采样（原点距 ~0.1m 逐点 κ 噪声大）
                rx = np.interp(rs, s, pts[:, 0])
                ry = np.interp(rs, s, pts[:, 1])
                ax, ay = rx[:-2], ry[:-2]
                bx, by = rx[1:-1], ry[1:-1]
                cx, cy = rx[2:], ry[2:]
                area2 = np.abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax))
                lab_ = np.hypot(bx - ax, by - ay)
                lbc_ = np.hypot(cx - bx, cy - by)
                lca_ = np.hypot(ax - cx, ay - cy)
                denom = np.maximum(lab_ * lbc_ * lca_, 1e-9)
                kap = np.where(area2 > 1e-9, 2.0 * area2 / denom, 0.0)
                self._f4_kap = (rs, np.minimum(kap, 0.5), rx, ry)  # κ 帽 0.5：v_min≈0.75-0.88，防长停
            rs, kap, rx, ry = self._f4_kap
            ks_s = rs[1:-1]                 # κ 采样弧长（三点中心）
            if self.prefix:                 # vcmd 已 append 本步 ego → 恒非空；防御式兜底=路径首点
                ego_xy = np.asarray([self.prefix[-1][1], self.prefix[-1][2]])
            else:
                ego_xy = np.asarray([self.path[0][0], self.path[0][1]], dtype=float)
            near = int(np.argmin((rx - ego_xy[0]) ** 2 + (ry - ego_xy[1]) ** 2))
            s_now = float(rs[near])
            m = ks_s >= s_now - 0.5
            if int(m.sum()) < 4:
                return []
            ss, kk = ks_s[m], kap[m]
            # 前瞻窗 max 包络：la=clip(lt·v,4,30)m → 窗格数 w=la/2∈[2,15]；
            # pref_max[i, w−1]=max kk[i:i+w]（行前缀 max 一次算全 w；右延拓=末值）
            kk_pad = np.concatenate([kk, np.full(15, kk[-1])])
            pref_max = np.maximum.accumulate(
                np.lib.stride_tricks.sliding_window_view(kk_pad, 16), axis=1)
            rows = np.arange(len(kk))
            out = []
            for c, lt in F4_GRID:
                v = np.full_like(ss, V_HI)
                for _ in range(3):          # 前瞻迭代：v 决定窗长，窗 max 包络决定 v
                    w_idx = np.clip((lt * v / 2.0).astype(int), 2, 15) - 1
                    k_eff = pref_max[rows, w_idx]
                    v = np.clip(c * np.sqrt(F4_ALAT / np.maximum(k_eff, 1e-4)), V_LO, V_HI)
                # 后向减速传播 v²[i]=min_{j≥i}(v[j]²+2a(s_j−s_i))：i 点须低到能减速到达
                # 未来任意 j 的限速。Δs=2 均匀网格 → y=v²+c·idx 的 reverse-cummin 再 −c·i
                v2 = v * v
                idx = np.arange(len(v2), dtype=float)
                _c = 2.0 * F4_ADEC * 2.0
                acc = np.minimum.accumulate((v2 + _c * idx)[::-1])[::-1]
                v = np.sqrt(np.maximum(acc - _c * idx, V_LO * V_LO))
                vm = np.maximum((v[:-1] + v[1:]) / 2.0, 0.5)   # 时间参数化（0.5 地板防除零）
                t_abs = np.concatenate([[t], t + np.cumsum(np.diff(ss) / vm)])
                out.append(Cand(f"F4c{c:g}L{lt:g}", "F4", t_abs, v))
            return out
        except Exception:
            return []

    def _seed_taus(self, t_now):
        """F2 τ 播种：ego 剩余路径走廊内背景车 GT 占用窗（前/中/后 ≤3）。
        纯数据驱动（占用窗=帧位到路径采样点距离 < 半宽和+2m），零逐场键。"""
        try:
            pts = np.asarray([[w[0], w[1]] for w in self.path], dtype=float)
            if len(pts) < 2:
                return []
            # 路径剩余段（ego 最近点 → 终点）每 ~5m 采样
            ego_xy = np.asarray([self.prefix[-1][1], self.prefix[-1][2]]) if self.prefix \
                else pts[0]
            near = int(np.argmin(np.hypot(pts[:, 0] - ego_xy[0], pts[:, 1] - ego_xy[1])))
            seg = pts[near:]
            step_i = max(1, int(5.0 / max(np.mean(np.hypot(*(seg[1:] - seg[:-1]).T)), 0.5)))
            samp = seg[::step_i]
            half_ego = math.hypot(self.ego_shape["width"], self.ego_shape["length"]) / 2.0
            wins = []
            for vid, tab in self.traj.items():
                if vid == "ego":
                    continue
                sh = tab.get("shape") or {}
                half = math.hypot(float(sh.get("width", 5.0)), float(sh.get("length", 9.0))) / 2.0
                r = half_ego + half + 2.0
                ts_in, ts_out = None, None
                # 数值序遍历（字典键是字符串时间——裸 sorted() 是字典序，
                # "10.0"<"2.0" 会把占用窗扫成乱序 → τ 播种失效，10-02 实修）
                tkeys = []
                for k in tab.keys():
                    if k in ("shape", -1, "-1"):
                        continue
                    try:
                        tkeys.append((float(k), k))
                    except (TypeError, ValueError):
                        continue
                for tk, k in sorted(tkeys):
                    if tk < t_now - 0.5:
                        continue
                    if tk < t_now - 0.5:
                        continue
                    fr = tab[k]
                    dd = np.min(np.hypot(samp[:, 0] - fr["x"], samp[:, 1] - fr["y"]))
                    if dd < r:
                        ts_in = tk if ts_in is None else ts_in
                        ts_out = tk
                    elif ts_in is not None and ts_out is not None and tk - ts_out > 0.3:
                        break  # 窗结束
                if ts_in is not None and ts_out is not None and ts_out > t_now + 1.0:
                    wins.append((ts_in, ts_out))
            taus = set()
            for a, b in wins:
                taus.add(round(b + 0.5, 1))
                taus.add(round((a + b) / 2.0, 1))
            return sorted(x for x in taus if x >= t_now + 2.0)[:3]
        except Exception:
            return []

    # ---------------- v43 饱和停靠 ----------------
    def _park_check(self, t):
        """连续 SAT_STREAK 个「F0 满分窗」（plan=F0 且分 ≥CEIL_EXIT）→ sas_off=
        "saturated" 整场回银行，后续窗零 rollout。中性论证（F5 的场景级推广）：
        ego 沿 F0 计划=银行链确定性推进，F0 后续窗分恒等（rollout==执行已证）⇒
        之后任何重规划只能续期 F0 → 与直接回银行逐位同 v_cmd 流。在位者非 F0
        的满分窗不计数（parking 会把胜者剖面切回银行=行为改变，如 1_3_10 早窗
        F2 形态——该形态走现行 F5 短路续期不停车）。收益=ceiling 场后续窗免
        F0 rollout（~1s/窗）→ 进程级预算/墙钟释放（g4d 预注册：公开 37 场中
        24 场 t=2.0 即停）。"""
        if self._sas_off is not None:
            return
        if (self.plan is not None and self.plan[0].label == "F0"
                and self.plan[1] >= CEIL_EXIT):
            self._ceil_streak += 1
            if self._ceil_streak >= SAT_STREAK:
                self._sas_off = "saturated"
                self.plan = None
                self._log({"t": t, "ev": "sas_off", "why": "saturated",
                           "streak": self._ceil_streak})
        else:
            self._ceil_streak = 0

    # ---------------- 重规划（3.3/3.4） ----------------
    def _replan(self, ctrl, observation, step, t, mode, breaker):
        bootstrap_window_started = (time.perf_counter() if
                                    self._bootstrap_enabled or self._retiming_enabled or self._local_exit_enabled
                                    or self._full_trace_sqp_enabled else None)
        if self._local_exit_enabled and not self._local_exit_ready:
            self._local_exit_observed = []
        self._last_bg_set = frozenset(k for k in observation["vehicle_info"] if k != "ego")
        cands = self._families(step, t)
        incumbent = self.plan[0] if self.plan is not None else cands[0]
        # v46：真实在位对象必须参评。F4 同 label 的新表是挑战者，不能替代
        # 旧表的续跑分；F2 的旧 tau 离开播种集合后也不能丢掉下界。
        # 标量族同名即同政策，复用旧对象避免重复滚动；带表族保留两者。
        if self.plan is not None and not any(c is incumbent for c in cands):
            same = next((i for i, c in enumerate(cands)
                         if c.label == incumbent.label), None)
            if incumbent.kind in ("F0", "F1", "F2", "F3") and same is not None:
                cands[same] = incumbent
            else:
                cands.append(incumbent)
        # D3 memo 修剪：只留本窗起 ±1s 的合成表/凸包（防长场内存无界增长）
        if len(self._memo) > 700:
            for k in list(self._memo):
                try:
                    if float(k) < t - 1.0:
                        del self._memo[k]
                except ValueError:
                    del self._memo[k]
        if len(self._bpolys) > 30000:
            self._bpolys = {k2: v for k2, v in self._bpolys.items()
                            if float(k2[1]) >= t - 1.0}
        # v41 满分窗提前退出（防御④，运行时主削减）：F0（或在位者）rollout 已达
        # CEIL_EXIT ⇒ 任何挑战者不可能满足换挡条件（需 ≥ 在位分+HYST > 100，
        # 总分上限 100=数学不可达）⇒ 扇出结果不可能改变 plan → 跳过 20 候选扇出。
        # bit 中性论证：非满分窗走原路全族；满分窗原代码的 plan 也只能是在位者
        # 续期（margin 永假）→ 与本短路给同一 (cand, score, recs)。
        # r2 族内守卫（对抗审计 b2 子类）：在位者∉本窗候选族（F2 τ 过期 churn）时
        # 原代码 inc_r=None→margin 空真→可 SWITCH 到族内者（或保留陈旧 recs），
        # 与本短路的"另滚在位者续期"可分叉——此窗不走短路，fall through 全族
        # （f0_result 直传保逐位同）。历史 3688 窗 off-family 在位 44 窗全为
        # F2 churn 且与满分窗零巧合；1434 个满分窗在位者 100% 族内 → 收益零损。
        f0 = cands[0]
        ri_fast = None
        if (self.plan is not None and incumbent.kind != "F0"
                and self.plan[1] >= KEEP_CEIL):
            ri_fast = self._rollout(ctrl, observation, incumbent, step)
        keep_fast = (ri_fast is not None and ri_fast["score"] >= KEEP_CEIL
                     and not ri_fast["detail"]["vehicle_collision"]
                     and not ri_fast["detail"]["road_boundary_collision"])
        r0 = None if keep_fast else self._rollout(ctrl, observation, f0, step)
        _inc_in_family = any(c is incumbent for c in cands)
        # v47：严格超过 100−HYST 即无可切换挑战者，不必等到 99.995；
        # ri 仍滚真实在位者。满分停车的 CEIL_EXIT 判据不变。
        if keep_fast or (r0 is not None and r0["score"] >= KEEP_CEIL and _inc_in_family):
            inc0 = incumbent
            plan_c, plan_r = (inc0, ri_fast) if keep_fast else (f0, r0)
            if not keep_fast and inc0 is not f0 and inc0.label != f0.label:
                ri = ri_fast if ri_fast is not None else self._rollout(ctrl, observation, inc0, step)
                plan_c, plan_r = ((inc0, ri) if ri is not None
                                  and ri["score"] >= KEEP_CEIL else (None, None))
            if plan_c is not None:
                self.plan = (plan_c, plan_r["score"], plan_r["recs"])
                self.last_plan_step = step
                self._shrink = self._win_ms > WIN_MS_SOFT
                self._log({
                    "t": t, "step": step, "ev": "replan", "mode": mode,
                    "breaker": bool(breaker), "brk": breaker or None,
                    "ms": round(self._win_ms, 1),
                    "ceil": True, "winner": plan_c.label,
                    "score": round(plan_r["score"], 4),
                    "n_switch": self.n_switch, "switch_warn":
                        self.n_switch > SWITCH_WARN,
                    "exp_t": self._exp_t, "exp_src": self._exp_src,
                    "cands": [{"c": plan_c.label, "s": round(plan_r["score"], 4),
                               "end": plan_r["end"], "arr": plan_r["arrival"]}],
                })
                self._park_check(t)
                return
        results = self._fanout(ctrl, observation, cands, step, r0)
        if self._exec_enabled and not self._exec_ready:
            self._exec_ready = True
            safe = [(c, r) for c, r in results if r is not None
                    and not r["detail"]["vehicle_collision"]
                    and not r["detail"]["road_boundary_collision"]]
            if safe:
                reference = max(safe, key=lambda cr: cr[1]["score"])[1]
                added = self._execution_candidates(reference, t)
                trial = self._fanout(ctrl, observation, [f0] + added, step, r0)[1:]
                # 新控制政策必须在官方真足迹评分器中零撞/零压。
                trial = [(c, r) for c, r in trial if r is not None
                         and not r["detail"]["vehicle_collision"]
                         and not r["detail"]["road_boundary_collision"]]
                if trial:
                    top_c, top_r = max(trial, key=lambda cr: cr[1]["score"])
                    # 首窗胜过原族才长期保留；落败政策不占每窗 rollout 槽。
                    # F0 与真实在位者仍始终参评，不影响它们的续跑比较。
                    if top_r["score"] >= reference["score"] + HYST:
                        self._exec_candidates = [top_c]
                    results.extend(trial)
        # 首个非满分窗由当前安全到达参考生成有界Bezier捷径。原族/在位者
        # 始终参评；新路必须经官方实际足迹静态预筛和完整动态rollout。
        if self._bezier_enabled and not self._bezier_ready:
            self._bezier_ready = True
            safe_arrivals = [(c, r) for c, r in results if r is not None and r["end"] == 4
                             and not r["detail"]["vehicle_collision"]
                             and not r["detail"]["road_boundary_collision"]]
            if safe_arrivals:
                reference = max(safe_arrivals, key=lambda cr: cr[1]["score"])[1]
                if reference["score"] < KEEP_CEIL:
                    from geometry_proposals import bezier_candidates
                    try:
                        added, audit = bezier_candidates(self, reference, t, Path(_datasets_dir()))
                    except Exception as error:
                        added, audit = [], []
                        self._log({"ev": "policy_drop", "t": t, "kind": "bezier",
                                   "error_type": type(error).__name__})
                    self._log({"ev": "geometry_built", "t": t, "kind": "bezier",
                               "proposals": len(audit),
                               "static_accepted": sum(r["accepted"] for r in audit),
                               "policy_count": len(added)})
                    if added:
                        trial = self._fanout(ctrl, observation, [f0] + added, step, r0)[1:]
                        trial = [(c, r) for c, r in trial if r is not None
                                 and not r["detail"]["vehicle_collision"]
                                 and not r["detail"]["road_boundary_collision"]]
                        if trial:
                            top_c, top_r = max(trial, key=lambda cr: cr[1]["score"])
                            if top_r["score"] >= reference["score"] + HYST:
                                self._exec_candidates = [top_c]
                            results.extend(trial)
        # 每场首个非满分窗联合搜索目标入口位置和到达朝向。静态最多
        # 75个几何，动态最多12政策；保留原族和真实在位计划的下界。
        if self._goal_pose_enabled and not self._goal_pose_ready:
            self._goal_pose_ready = True
            safe_arrivals = [(c, r) for c, r in results if r is not None and r["end"] == 4
                             and not r["detail"]["vehicle_collision"]
                             and not r["detail"]["road_boundary_collision"]]
            if safe_arrivals:
                reference = max(safe_arrivals, key=lambda cr: cr[1]["score"])[1]
                if reference["score"] < KEEP_CEIL:
                    from goal_pose_geometry import goal_pose_candidates
                    try:
                        added, audit = goal_pose_candidates(self, reference, t, Path(_datasets_dir()))
                    except Exception as error:
                        added, audit = [], []
                        self._log({"ev": "policy_drop", "t": t, "kind": "goal_pose",
                                   "error_type": type(error).__name__})
                    self._log({"ev": "geometry_built", "t": t, "kind": "goal_pose",
                               "proposals": len(audit),
                               "static_accepted": sum(r["accepted"] for r in audit),
                               "policy_count": len(added)})
                    if added:
                        trial = self._fanout(ctrl, observation, [f0] + added, step, r0)[1:]
                        trial = [(c, r) for c, r in trial if r is not None
                                 and not r["detail"]["vehicle_collision"]
                                 and not r["detail"]["road_boundary_collision"]]
                        if trial:
                            top_c, top_r = max(trial, key=lambda cr: cr[1]["score"])
                            if top_r["score"] >= reference["score"] + HYST:
                                self._exec_candidates = [top_c]
                            results.extend(trial)
        # 由安全到达参考的道路转弯事件构建最多3条C2样条、6个政策。
        # 每场仅展开一次；官方真实足迹和完整动态安全均须通过。
        if self._event_enabled and not self._event_ready:
            self._event_ready = True
            safe_arrivals = [(c, r) for c, r in results if r is not None and r["end"] == 4
                             and not r["detail"]["vehicle_collision"]
                             and not r["detail"]["road_boundary_collision"]]
            if safe_arrivals:
                reference = max(safe_arrivals, key=lambda cr: cr[1]["score"])[1]
                if reference["score"] < KEEP_CEIL:
                    from event_geometry import event_candidates
                    try:
                        added, audit = event_candidates(self, reference, t, Path(_datasets_dir()))
                    except Exception as error:
                        added, audit = [], []
                        self._log({"ev": "policy_drop", "t": t, "kind": "event",
                                   "error_type": type(error).__name__})
                    self._log({"ev": "geometry_built", "t": t, "kind": "event",
                               "proposals": len(audit),
                               "static_accepted": sum(r["accepted"] for r in audit),
                               "policy_count": len(added)})
                    if added:
                        trial = self._fanout(ctrl, observation, [f0] + added, step, r0)[1:]
                        trial = [(c, r) for c, r in trial if r is not None
                                 and not r["detail"]["vehicle_collision"]
                                 and not r["detail"]["road_boundary_collision"]]
                        if trial:
                            top_c, top_r = max(trial, key=lambda cr: cr[1]["score"])
                            if top_r["score"] >= reference["score"] + HYST:
                                self._exec_candidates = [top_c]
                            results.extend(trial)
        # 初始参考未必描述中途跟车/让行之后的几何。每场最多追加两次，
        # 间隔至少6s；只从当前安全到达rollout重建，落败族不长期占槽。
        if (self._trace_enabled and self._exec_ready and self._trace_refreshes < 2
                and (self._trace_last_t is None or t - self._trace_last_t >= 6. - 1e-8)):
            safe_arrivals = [(c, r) for c, r in results if r is not None and r["end"] == 4
                             and not r["detail"]["vehicle_collision"]
                             and not r["detail"]["road_boundary_collision"]]
            if safe_arrivals:
                reference = max(safe_arrivals, key=lambda cr: cr[1]["score"])[1]
                if reference["score"] < KEEP_CEIL:
                    from trajectory_candidates import TraceCand
                    xy = [[self.prefix[-1][1], self.prefix[-1][2]]]
                    xy += [list(p) for _, p in sorted(reference["recs"].items())]
                    added = []
                    for coefficient in (.85, 1.):
                        for look in (6., 10., 16.):
                            try:
                                added.append(TraceCand(xy, coefficient, look, t))
                            except Exception as error:
                                self._log({"ev": "policy_drop", "t": t, "kind": "trace_refresh",
                                           "error_type": type(error).__name__})
                    self._trace_refreshes += 1
                    self._trace_last_t = t
                    trial = self._fanout(ctrl, observation, [f0] + added, step, r0)[1:]
                    trial = [(c, r) for c, r in trial if r is not None
                             and not r["detail"]["vehicle_collision"]
                             and not r["detail"]["road_boundary_collision"]]
                    if trial:
                        top_c, top_r = max(trial, key=lambda cr: cr[1]["score"])
                        if top_r["score"] >= reference["score"] + HYST:
                            self._exec_candidates = [top_c]
                        results.extend(trial)
        # 原族与既有几何均无严格安全到达参考时，才尝试一次地图路线救援。
        # helper失败不改原plan/cache，完整墙钟由vcmd既有预算统一计账。
        if self._bootstrap_enabled and not self._bootstrap_ready:
            try:
                from bootstrap_route import bootstrap_trials
                bootstrap_results = list(bootstrap_trials(
                    self, ctrl, observation, results, incumbent, step, t,
                    Path(_datasets_dir()), bootstrap_window_started,
                    SCENE_BUDGET_MS, _GLOBAL_MS, GLOBAL_BUDGET_MS, HYST))
            except Exception as error:
                self._log({"ev": "bootstrap_helper_import_or_dispatch_error",
                           "t": t, "error_type": type(error).__name__})
            else:
                results.extend(bootstrap_results)
        # 完整原两负/一full流程之后，至多追加一个当前在线制动余量政策。
        # 只产生参评结果，切换与迟滞仍由下方原selector决定。
        if ((self._retiming_enabled and not self._retiming_ready)
                or (self._braking_enabled and not self._braking_ready)):
            old_collect = self._retiming_collect_rows
            try:
                from retiming_braking_trials import retiming_trials
                retiming_results = list(retiming_trials(
                    self, ctrl, observation, results, incumbent, step, t,
                    bootstrap_window_started, SCENE_BUDGET_MS, _GLOBAL_MS,
                    GLOBAL_BUDGET_MS, HYST, KEEP_CEIL))
            except Exception as error:
                self._retiming_pending = []
                self._log({"ev": "retiming_ff_helper_import_or_dispatch_error",
                           "t": t, "error_type": type(error).__name__})
            else:
                results.extend(retiming_results)
            finally:
                self._retiming_collect_rows = old_collect
        # 原v59流程完成后才尝试一次局部退弯及一次内嵌后段速度修正。
        if self._local_exit_enabled and not self._local_exit_ready:
            try:
                from local_exit_trials import local_exit_trials
                local_results = local_exit_trials(
                    self, ctrl, observation, results, incumbent, step, t,
                    bootstrap_window_started, SCENE_BUDGET_MS, _GLOBAL_MS,
                    GLOBAL_BUDGET_MS, HYST, KEEP_CEIL)
                results.extend(local_results)
            except Exception as error:
                self._log({"ev": "local_exit_dispatch_error", "t": t,
                           "error_type": type(error).__name__})
        # 跟踪政策在每个窗口均须零撞/零压；首窗安全不代表从别的政策
        # 到达的新状态仍安全。银行族保留原评分语义，供失败时回退。
        alive = [(c, r) for c, r in results if r is not None
                 and (not getattr(c, "tracking", False)
                      or not (r["detail"]["vehicle_collision"]
                              or r["detail"]["road_boundary_collision"]))]
        if not alive:
            self.plan = None
            self.last_plan_step = step  # r3（H7）：推进节拍=10 步退避，防 plan=None 时 10Hz 重规划风暴
            self._log({"t": t, "ev": "all_dead"})
            self._park_check(t)   # plan=None → streak 归零（连续性语义）
            return
        selector_state = (self.plan, self.last_switch_step, self.n_switch)
        inc_r = self._select_original(alive, incumbent, step, breaker)
        self.last_plan_step = step
        self._shrink = self._win_ms > WIN_MS_SOFT
        if self._retiming_enabled:
            old_exec_candidates = self._exec_candidates
            try:
                from retiming_ff_trials import commit_selected
                commit_selected(self, t)
            except Exception as error:
                self._exec_candidates = old_exec_candidates
                self._retiming_pending = []
                self._log({"ev": "retiming_ff_cache_commit_error",
                           "t": t, "error_type": type(error).__name__})
        if self._full_trace_sqp_enabled and not self._full_trace_sqp_ready:
            try:
                from full_trace_sqp_trials import full_trace_trials
                extra = full_trace_trials(
                    self, ctrl, observation, results, incumbent, step, t, mode,
                    bootstrap_window_started, SCENE_BUDGET_MS, _GLOBAL_MS,
                    GLOBAL_BUDGET_MS, HYST, KEEP_CEIL)
            except Exception as error:
                extra = []
                self._full_trace_sqp_ready = True
                self._log({"ev": "full_trace_dispatch_error", "t": t,
                           "error_type": type(error).__name__, "error": str(error)})
            if extra:
                # 先前选中的真实对象只用于source认证；最终仍用原selector和
                # 窗首迟滞/切换状态，在全部原候选加一个新对象上决定。
                self.plan, self.last_switch_step, self.n_switch = selector_state
                results.extend(extra)
                alive.extend(extra)
                inc_r = self._select_original(alive, incumbent, step, breaker)
                selected = next((c for c, _ in extra if self.plan is not None
                                 and self.plan[0] is c), None)
                if selected is not None:
                    # 所有既有cache对象保留，只追加原selector最终选择的对象。
                    if not any(c is selected for c in self._exec_candidates):
                        self._exec_candidates.append(selected)
                self._log({"ev": "full_trace_selector_commit", "t": t,
                           "selected": self.plan[0].label if self.plan else None,
                           "selected_identity": id(self.plan[0]) if self.plan else None,
                           "candidate_identity": id(extra[0][0]),
                           "selected_extra_same_object": selected is extra[0][0],
                           "original_selector_used": True,
                           "original_switch_count": self.n_switch,
                           "score": self.plan[1] if self.plan else None})
        self._log({
            "t": t, "step": step, "ev": "replan", "mode": mode,
            "breaker": bool(breaker), "brk": breaker or None,
            "ms": round(self._win_ms, 1), "n_alive": len(alive),
            "winner": self.plan[0].label if self.plan else None,
            "score": round(self.plan[1], 4) if self.plan else None,
            "incumbent": incumbent.label,
            "incumbent_score": round(inc_r["score"], 4) if inc_r else None,
            "retained": self.plan is not None and self.plan[0] is incumbent,
            "n_switch": self.n_switch,
            "switch_warn": self.n_switch > SWITCH_WARN,
            "exp_t": self._exp_t, "exp_src": self._exp_src,
            "cands": [{"c": c.label, "s": round(r["score"], 4), "end": r["end"],
                       "arr": r["arrival"]} for c, r in results if r is not None],
        })
        self._park_check(t)

    def _select_original(self, alive, incumbent, step, breaker):
        # 平票序：total → arrival 早 → F0 优先（kind 序已是 F0 最前）
        best_c, best_r = min(
            alive, key=lambda cr: (-cr[1]["score"],
                                   cr[1]["arrival"] if cr[1]["arrival"] is not None else 9e9,
                                   0 if cr[0].kind == "F0" else 1))
        inc_r = next((r for c, r in alive if c is incumbent), None)
        switched = False
        if best_c is not incumbent:
            margin_ok = inc_r is None or best_r["score"] >= inc_r["score"] + HYST
            interval_ok = (step - self.last_switch_step) >= MIN_SWITCH or breaker
            if margin_ok and interval_ok:
                switched = True
                self.last_switch_step = step
                self.n_switch += 1
        if best_c is incumbent:
            self.plan = (best_c, best_r["score"], best_r["recs"])
        elif switched:
            self.plan = (best_c, best_r["score"], best_r["recs"])
        else:
            # 挑战者未过迟滞：incumbent 续期（用本轮 fresh rollout 记录）
            if inc_r is not None:
                self.plan = (incumbent, inc_r["score"], inc_r["recs"])
        return inc_r

    # ---------------- 候选并行（D3：fork 扇出，同码链零改写） ----------------
    def _fanout(self, ctrl, observation, cands, step, f0_result=None):
        """F0（cands[0]）父进程先滚——暖 BitMap/评分地图缓存/全量 memo，
        其余候选 fork 并行（子进程 COW 继承缓存）。**回填调度（D3 v2）**：
        MAX_WORKERS 槽位空一补一、无波屏障——原 15+6 两波的尾波不再被首波
        最慢候选阻塞；完成序与 argmax 无关（结果按下标回填保序）=bit 中性，
        MINE_SAS_FORK=0 → 串行回退（A/B 验收通道）。返回 [(cand, result|None)] 保序。
        v41：满分窗短路时 _replan 已滚过 F0——f0_result 直传免二次滚（同参同
        memo 态=逐位同值）。"""
        results = [(cands[0], f0_result if f0_result is not None else
                    self._rollout(ctrl, observation, cands[0], step))]
        rest = cands[1:]
        if len(rest) < 2 or not FORK_ENABLED:
            for c in rest:
                results.append((c, self._rollout(ctrl, observation, c, step)))
            return results
        import multiprocessing as _mp
        ctx = _mp.get_context("fork")
        out = {}                          # idx -> result|None（按提交序回填）
        pending = list(enumerate(rest))
        running = []                      # (pr, proc, idx, cand)
        deadline = time.monotonic() + CHILD_TIMEOUT_S
        while pending or running:
            while pending and len(running) < MAX_WORKERS:
                idx, c = pending.pop(0)
                pr, pw = ctx.Pipe(False)
                p = ctx.Process(target=_child_roll,
                                args=(pw, self, ctrl, observation, c, step),
                                daemon=True)
                p.start()
                pw.close()               # 父进程关写端（子独占，EOF 语义干净）
                running.append((pr, p, idx, c))
            done = [i for i, (pr, _, _, _) in enumerate(running) if pr.poll()]
            if not done:
                if time.monotonic() > deadline:   # 整窗看护：超时=剩余候选判死
                    for pr, p, idx, c in running:
                        p.terminate()
                        p.join(5)
                        pr.close()
                        out.setdefault(idx, None)
                    running = []
                    continue
                time.sleep(0.02)
                continue
            for i in sorted(done, reverse=True):
                pr, p, idx, c = running.pop(i)
                p.join(5)
                r = None
                try:
                    tag, r = pr.recv()
                    r = r if tag == "ok" else None
                    if r is not None and getattr(c, "_braking_execution_guard", False):
                        from executed_braking_reachability import execution_audit_valid
                        if not execution_audit_valid(r, c, len(r.get("recs", {}))):
                            r = None
                    if r is not None and getattr(c, "_local_exit_policy", False):
                        from local_exit_execution import valid as local_valid
                        if not local_valid(r, c, len(r.get("recs", {}))):
                            r = None
                except Exception:
                    r = None
                out[idx] = r
                pr.close()
        for i, c in enumerate(rest):
            results.append((c, out.get(i)))
        return results

    # ---------------- rollout（同码链 + 逐位植物/终局） ----------------
    def _rollout(self, ctrl, observation, cand, step_now):
        # v41 副本静默（防御⑤）：replica.process() 与真实循环共用同一组打印
        # （预瞄失败等）——v40 事故中 21 副本×逐步同步打印=平台日志体积爆炸
        # 的主源之一。rollout 期间 stdout 全吸（真实循环打印不受影响；审计走
        # 独立文件句柄）。fork 子进程调用的也是本包装=同样静默。
        with contextlib.redirect_stdout(io.StringIO()):
            result = self._rollout_impl(ctrl, observation, cand, step_now)
        if self._local_exit_enabled and not self._local_exit_ready and result is not None:
            self._local_exit_observed.append((cand, result))
        return result

    def _rollout_impl(self, ctrl, observation, cand, step_now):
        recs = {}
        guarded = bool(getattr(cand, "_braking_execution_guard", False))
        capture_t = self._local_exit_capture_t
        local_guard = capture_t is not None or bool(getattr(cand, "_local_exit_policy", False))
        anchor_controller = None
        if local_guard:
            import local_exit_execution
        rep = None
        try:
            rep = _copy.copy(ctrl)
            # 可变态隔离（仿真侧浅别名陷阱：process 内 clear/append 的容器换新）
            rep._projection_vehicle_S = []  # list（process 每步 clear；_projectVehicle append）
            rep._dbg = []
            rep._sas_cand = cand          # sas_hook 短路→候选注入同码链
            rep._last_acc = float(ctrl._last_acc)
            if local_guard:
                local_exit_execution.begin(rep, cand, self.dt)
            if guarded:
                cand.execution_audit_begin(rep)
            ego0 = observation["vehicle_info"]["ego"]
            x, y = float(ego0["x"]), float(ego0["y"])
            yaw, v = float(ego0["yaw_rad"]), float(ego0["v_mps"])
            length = float(self.ego_shape["length"])
            t_syn = float(observation["test_setting"].get("t", 0.0))
            n_cap = min(N_MAX_CAP, int(round((self.max_t - t_syn) / self.dt)) + 2)
            end = -1
            rows = []
            for k in range(1, n_cap + 1):
                key = str(np.around(t_syn, 3))
                ent = self._memo.get(key)
                if ent is None:
                    ent = self._synth(t_syn)
                    self._memo[key] = ent
                bg_vi, tf = ent
                vi = {"ego": {"x": x, "y": y, "yaw_rad": yaw, "v_mps": v,
                              "acc_mpss": rep._last_acc, "shape": self.ego_shape}}
                vi.update(bg_vi)
                obs_syn = {"vehicle_info": vi,
                           "test_setting": dict(self._ts_tmpl, t=t_syn, end=-1)}
                if guarded:
                    cand.execution_audit_before_frame(rep, t_syn, v, rep._last_acc)
                if local_guard:
                    local_exit_execution.before_frame(rep, t_syn, v, rep._last_acc)
                acc, delta = rep.process(vi, self.path, tf, obs_syn)
                if local_guard:
                    local_exit_execution.after_frame(rep, acc, delta)
                    if capture_t is not None and t_syn == capture_t:
                        anchor_controller = _copy.copy(rep)
                        anchor_controller._local_exit_capture_state = None
                if guarded:
                    cand.execution_audit_after_frame(rep, acc)
                # env._action_cheaker（controller.py:333-336 verbatim）
                a = float(np.clip(acc, -15, 15))
                rot = float(np.clip(delta, -1, 1))
                # env._update_ego_and_t（controller.py:382-390 verbatim，旧速更新位置）
                x = x + v * np.cos(yaw) * self.dt
                y = y + v * np.sin(yaw) * self.dt
                yaw = yaw + v / length * 1.7 * np.tan(rot) * self.dt
                v = v + a * self.dt
                if v < 0:
                    v = 0.0
                t_syn = t_syn + self.dt
                # 终局判定用 t+dt 的背景表（官方 step 序=先 _update_other_vehicles_to_t
                # 刷新 bg 再 _update_end_status；红队 10-03 确证修：原实现传 t 时刻
                # vi=滞后一帧，近碰窗误翻 ego@t+dt vs bg@t）。key_n 与下一循环步首
                # 的 key 逐位同串（同一浮点加法）——零额外合成成本
                key_n = str(np.around(t_syn, 3))
                ent2 = self._memo.get(key_n)
                if ent2 is None:
                    ent2 = self._synth(t_syn)
                    self._memo[key_n] = ent2
                end = self._end_check(x, y, yaw, t_syn, ent2[0], key_n)
                rows.append((t_syn, x, y, yaw, v))
                recs[step_now + k] = (x, y)
                if end != -1:
                    break
            if end == -1:
                end = 1  # n_cap 兜底=超时形态
            detail = self._score(rows, end)
            result = {"score": detail["total"], "end": end,
                      "arrival": t_syn if end == 4 else None,
                      "recs": recs, "detail": detail}
            # 数字评分不变；首次合格在线几何带回未来执行行供有界重定时。
            # helper的两条新政策显式收集，其他窗口及关闭模式不扩展回传。
            capture_rows = (self._retiming_collect_rows or local_guard or
                            (self._local_exit_enabled and not self._local_exit_ready
                             and getattr(cand, "tracking", False)
                             and callable(getattr(cand, "steering", None))) or
                            (self._braking_enabled and not self._braking_ready
                             and getattr(cand, "tracking", False)
                             and callable(getattr(cand, "steering", None)) and end == 4
                             and detail["vehicle_collision"] is False
                             and detail["road_boundary_collision"] is False
                             and detail.get("road_check") == "bitmap_footprint") or
                            (self._retiming_enabled and not self._retiming_ready
                             and getattr(cand, "tracking", False)
                             and hasattr(cand, "steering") and end == 4
                             and detail["vehicle_collision"] is False
                             and detail["road_boundary_collision"] is False
                             and detail.get("road_check") == "bitmap_footprint"
                             and result["score"] < KEEP_CEIL))
            if capture_rows:
                result["retiming_ego_rows"] = rows
            if local_guard:
                result["local_exit_execution_audit"] = local_exit_execution.finish(rep, cand, len(rows))
                if capture_t is not None:
                    result["local_exit_anchor_controller"] = anchor_controller
            if guarded:
                result["braking_execution_audit"] = cand.execution_audit_finish(rep, len(rows))
            return result
        except Exception:
            if os.environ.get("MINE_SAS_DBG", "0") == "1":
                import traceback
                traceback.print_exc()
            return None  # 候选判死（argmax 弃选；全灭→fail-closed 银行）
        finally:
            if local_guard and rep is not None:
                local_exit_execution.reset(rep)
            if guarded and rep is not None:
                cand.execution_audit_reset(rep)

    # ---------------- 合成 vehicle_info / traj_future（逐位照抄） ----------------
    def _synth(self, t):
        """t 时刻背景表 + CACV 预测。键序/字段=controller._update_other_vehicles_to_t
        与 predictor wrapper（predictor.py:63-81）逐位同构。ego 由调用方注入。"""
        key3 = str(np.around(t, 3))       # controller.py:403
        vi, tf = {}, {}
        for vid, tab in self.traj.items():
            if vid == "ego":
                continue
            fr = tab.get(key3)
            if fr is None:
                continue                  # 离场（末键后无鬼影）
            vi[vid] = {"x": fr["x"], "y": fr["y"], "yaw_rad": fr["yaw_rad"],
                       "v_mps": fr["v_mps"], "yawrate_radps": fr["yawrate_radps"],
                       "acc_mpss": fr["acc_mpss"], "shape": tab["shape"]}
            tf[vid] = self._cacv(tab, t)
        return vi, tf

    def _cacv(self, tab, t):
        """CACV 合成（predictor.py:58-81 + 274-311 verbatim，含全部舍入）。"""
        t_now_ = round(float(t), 2)
        out = {-1: {}, "shape": tab["shape"]}
        seed = tab.get(str(t_now_))
        if seed is None:                  # 0.1 网格边缘浮点错位兜底：邻近键
            seed = tab.get(key3_nearest(tab, t))
            if seed is None:
                return out
        out[str(t_now_)] = seed
        x, y = seed["x"], seed["y"]
        v, yaw = seed["v_mps"], seed["yaw_rad"]
        acc, yr = seed["acc_mpss"], seed["yawrate_radps"]
        dt = self.dt
        delta_t = self.max_t - t_now_
        n = int(delta_t / dt) if delta_t < 5.0 else int(5.0 / dt)
        t_ = t_now_
        for _ in range(n):
            t_ += dt
            x += dt * (v * np.cos(yaw) + acc * np.cos(yaw) * 0.5 * dt
                       + yr * v * np.sin(yaw) * 0.5 * dt)
            y += dt * (v * np.sin(yaw) + acc * np.sin(yaw) * 0.5 * dt
                       + yr * v * np.cos(yaw) * 0.5 * dt)
            yaw += dt * yr
            v += dt * acc
            if dt < 0.1:
                str_time = str(round(t_, 3))
            else:
                str_time = str(round(t_, 2))
            out[str_time] = {"x": round(x, 2), "y": round(y, 2), "v": round(v, 2),
                             "a": round(acc, 2), "yaw": round(yaw, 3)}
        return out

    # ---------------- 终局判定（controller.py:417-458 verbatim） ----------------
    def _end_check(self, x, y, yaw, t, vi, key3=None):
        status = -1
        if t > 0.5 and self._collide(x, y, yaw, vi, key3):
            status = 2
        if t >= self.max_t:
            status = max(status, 1)
        if self._road_hit(x, y, yaw):   # 位图越界 IndexError 不吞=候选判死（env 同款崩溃语义）
            status = max(status, 3)
        if self._in_goal(x, y):
            status = max(status, 4)
        return status

    def _collide(self, x, y, yaw, vi, key3=None):
        """controller._collision_detect(:460-476)+_get_poly(:478-498) verbatim：
        真足迹 convex_hull，仅 ego 相关对，t>0.5 宽限在 _end_check。
        D3 无损加速：① bg 凸包按 (vid,key3) 记忆——同窗各候选经 memo 见到的是
        同一份 bg 表（同键同值）；② bbox 必要条件预筛——bbox⊇凸包，bbox 不交
        ⇒ 凸包不交 ⇒ intersects() 本就返回 False，跳过纯省时（返回值逐位同）。"""
        try:
            from shapely.geometry import Polygon
            import common.utils as utils
            ec = np.asarray(utils.calculate_vehicle_corners(
                self.ego_shape["length"], self.ego_shape["width"],
                self.ego_shape["locationPoint2Head"],
                self.ego_shape["locationPoint2Rear"], x, y, yaw), dtype=float)
            e0x, e0y = ec[:, 0].min(), ec[:, 1].min()
            e1x, e1y = ec[:, 0].max(), ec[:, 1].max()
            ego_poly = None
            for vid, veh in vi.items():
                if vid == "ego":
                    continue
                pk = (vid, key3) if key3 is not None else None
                p = self._bpolys.get(pk) if pk is not None else None
                if p is None:
                    p = self._poly(Polygon, utils, float(veh["x"]), float(veh["y"]),
                                   float(veh["yaw_rad"]), veh["shape"])
                    if pk is not None:
                        self._bpolys[pk] = p
                b = p.bounds
                if e1x < b[0] or b[2] < e0x or e1y < b[1] or b[3] < e0y:
                    continue                       # bbox 不交：凸包必不交（无损跳过）
                if ego_poly is None:
                    ego_poly = self._poly(Polygon, utils, x, y, yaw, self.ego_shape)
                if ego_poly.intersects(p):
                    return True
            return False
        except Exception:
            return False

    @staticmethod
    def _poly(Polygon, utils, x, y, yaw, shape):
        # controller._get_poly(:486-503) verbatim（unpack 名序与多边形顶点序照抄；
        # convex_hull 使顶点序无几何影响，逐位对齐以杜绝任何歧义）
        fl, fr, rl, rr = utils.calculate_vehicle_corners(
            shape["length"], shape["width"], shape["locationPoint2Head"],
            shape["locationPoint2Rear"], x, y, yaw)
        corners = np.asarray([fl, fr, rr, rl, fl], dtype=float)
        return Polygon(corners).convex_hull

    def _in_goal(self, x, y):
        try:
            import common.utils as utils
            return bool(utils.is_inside_polygon(x, y, self.goal))
        except Exception:
            return False

    def _road_hit(self, x, y, yaw):
        """controller.py:438-449 verbatim：全图位图 + CollisionLookup（类型三分支
        :345-350 同款；lookup.py 查询含 0.1 格与 ×2 像素语义，逐位同码）。"""
        lk, img, x0, y0 = self._road_kit()
        return bool(lk.collision_detection(x - x0, y - y0, yaw, img))

    def _road_kit(self):
        if self._road is None:
            from dynamic_scenes.lookup import CollisionLookup, VehicleType
            sh = self.ego_shape
            if sh["length"] >= 8.5 and sh["width"] <= 9.5:
                typ = VehicleType.MineTruck_XG90G
            elif sh["length"] >= 12.5 and sh["width"] <= 13.6:
                typ = VehicleType.MineTruck_NTE200
            else:
                typ = VehicleType.MineTruck_XG90G
            from map_expansion.bit_map import BitMap
            # 官方 scenarioOrganizer 同款首段映射（红队 10-03：子串匹配在全部
            # 223 个实际场名上等价，此处硬化为逐字同构消除残余破格名风险）
            loc = ("guangdong_dapai"
                   if self.name.split("_", 1)[0] == "dapai" else "jiangxi_jiangtong")
            bm = BitMap(_datasets_dir(), loc, "bitmap_mask")
            utm = bm.bitmap_info["bitmap_mask_PNG"]["UTM_info"]
            self._road = (CollisionLookup(type=typ), bm.image_ndarray,
                          float(utm["local_x_range"][0]), float(utm["local_y_range"][0]))
        return self._road

    # ---------------- 评分（前缀固定 + 后缀重算；整条轨迹泛函） ----------------
    def _score(self, rows, end):
        import score_embed as se
        se.install_map_cache()
        rows_all = self.prefix + [list(r) for r in rows]
        n = len(rows_all)
        # 官方评分器消费 recorder CSV（t→1dp，x/y/yaw/v→4dp，recorder.py:18-39）；
        # 假想帧对齐同精度（红队 10-03：阈值邻域帧的违规翻转噪声消除）
        times = np.round([r[0] for r in rows_all], 1)
        ego = {"x": np.round([r[1] for r in rows_all], 4),
               "y": np.round([r[2] for r in rows_all], 4),
               "yaw": np.round([r[3] for r in rows_all], 4),
               "v": np.round([r[4] for r in rows_all], 4)}
        bg = {}
        for vid, tab in self.traj.items():
            if vid == "ego":
                continue
            sh = tab.get("shape") or {}
            xs = np.full(n, np.nan)
            ys = np.full(n, np.nan)
            yaws = np.full(n, np.nan)
            vs = np.full(n, np.nan)
            for i, tt in enumerate(times):
                fr = tab.get(str(np.around(tt, 3)))
                if fr is not None:
                    xs[i], ys[i] = fr["x"], fr["y"]
                    yaws[i], vs[i] = fr["yaw_rad"], fr["v_mps"]
            bg[str(vid)] = {"x": xs, "y": ys, "yaw": yaws, "v": vs,
                            "width": float(sh.get("width", 5.0)),
                            "head": float(sh.get("locationPoint2Head", 3.0)),
                            "rear": float(sh.get("locationPoint2Rear", 3.0))}
        sh_e = self.ego_shape
        frame = se.build_frame(times, ego, bg, terminal=end,
                               ego_shape={"width": sh_e["width"], "length": sh_e["length"],
                                          "head": sh_e["locationPoint2Head"],
                                          "rear": sh_e["locationPoint2Rear"]})
        return se.score_frame(frame, self.name, Path(_datasets_dir()), self._exp_t)


def key3_nearest(tab, t):
    """浮点错位兜底：表内与 t 最近的时间键（正常路径不触达）。"""
    best, bd = None, 9e9
    for k in tab.keys():
        try:
            d = abs(float(k) - t)
        except (TypeError, ValueError):
            continue
        if d < bd:
            best, bd = k, d
    return best
