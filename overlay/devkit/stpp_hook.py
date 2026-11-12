#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""stpp_hook —— MINE_EXACT 精确未来对时速度规划 hook（v1，E0-E2）。

挂载=simple_control.py E-slat 消费后、IDM 前（judge 定案 wf_6f0189cf-ab2）。
两平面单开关 MINE_EXACT=off|log|on（缺省 off，call-site 短路=字节中性）：
  off  = 本模块不被调用（predictor 也不注入）
  log  = 建计划+审计落盘，v_cmd 原样返回（数据面激活→IDM leader 精确化，行为≠锚）
  on   = stpp_scenes 接管场覆写 v_cmd；非接管场 v_cmd 原样

机制=离线探针验证版（st_probe v3，39 场 Δ+80.7 分·场）在线化：
  路径冻结重采样 DS=1.0m → 精确 s-t 占用表（OBB 4 轴 SAT + 参考点前偏移
  +0.2m 缓冲，KD 预筛限近距帧，ego/bg 矿卡同款偏移）→ SIPP 窗 Dijkstra
  （等待=站立方窗滑动 + 直边兜底）→ 双帽（紧=构造性 √(0.4/κ) / 松=12.0）
  镜像评分取优 → v(s) 剖面按 ego 弧长逐步取值。

接管语义（与既有逐场剂量键的关系，显式记账非静默）：
  vel0/alat/yield/eslat/vprof 的 v_cmd 剂量=被 stpp 计划**取代**（挂载点在其
  消费之后；审计记录该场被取代的键集）；idm_scenes（IDM a_max）/rot_scenes
  （转向 alat 钳）/look_scenes（预瞄）作用于挂载点之后的 IDM 与转向链
  =**保持原效**。批次 10 场全部带剂量键——按此语义全部纳入，A/B 单变量=
  开关层，逐场归因以取代记账为准。
安全契约：返回 ∈[0.1,12.0]；任何异常 fail-closed 返回传入 v_cmd；只读复用
控制器缓存（_nearest_pt_id），严禁写安全层状态；零随机零时钟。
场景无关：零逐场名分支（stpp_scenes 只判是否接管，参数全部场景内自算）。
"""
import json
import math
import os
import time as _time

import numpy as np

DS = 1.0            # 路径格距 (m)（探针验证口径）
DT = 0.1            # 占用表时间步 (s)
V_STEP = 0.5        # v 级距（探针 0.25 的预算折半版）
ACCELS = (-2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0)   # ⊂ 有效包络 [+2.5,−10]
V_BAR_FLOOR = 0.15  # 单格平均速度下限（防 dt→∞）
SAT_BUF = 0.2       # SAT 双侧数值缓冲 (m)
R_PRE = 13.0        # KD 预筛半径（≥ ego/bg 外接半径和 10.8+0.2+裕量）
ALAT_TH = 0.4       # 评分器侧加速度阈值 (m/s²)
V_MAX, V_MIN = 12.0, 0.1
REPLAN_K = 25       # 周期重规划步数（另有时序漂移触发）
DRIFT_T = 0.8       # |t_actual − t_plan(s_ego)| 超此值即重规划 (s)
TIGHT_WIN = 1.5     # 镜像到达优先窗：tight 须比 relaxed 快逾此值才胜出 (s)
NT_CAP = 700        # 占用表时间列上限

_CTX = {}           # scenario_name -> _PlanCtx（模块级单例，逐场键控）
_STPP = None        # stpp_scenes 键集（惰性装载）
_TRAJ_LATEST = {}   # scenario_name -> 官方 traj 引用（predictor 每步 stash，
                    # v2 旁路：占用表直读原始 schema，veh_traj_predicted 纯净）


def _cfg():
    # MINE_CFG：候选 config 注入缝（SAS 扫描；缺省空=原路径零变化）
    p = os.environ.get("MINE_CFG") or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "centerlines", "scen_config.json")
    return json.load(open(p))


def _takeover(name):
    global _STPP
    if _STPP is None:
        try:
            v = _cfg().get("stpp_scenes", {})
            _STPP = set(v.keys()) if isinstance(v, dict) else set()
        except Exception:
            _STPP = set()
        # MINE_STPP_EXTRA：筛查专用 env 扩展接管集（逗号分隔，不进包；
        # 缺省空=零变更；入批赢家走 stpp_scenes config 键）
        _extra = os.environ.get("MINE_STPP_EXTRA", "")
        if _extra:
            _STPP |= {x.strip() for x in _extra.split(",") if x.strip()}
    return name in _STPP


def _superseded(name):
    """该场被 stpp 取代的 v_cmd 上游剂量键（仅记账，A/B 归因用）。"""
    try:
        cfg = _cfg()
        return [k for k in ("vel0", "alat_scenes", "yield_scenes", "vprof_scenes")
                if name in cfg.get(k, {})]
    except Exception:
        return []


def _box_corners(cx, cy, yaw, w, l, off=0.0):
    """OBB 角点（含 SAT_BUF 膨胀；off=参考点→盒心前偏移）。"""
    hw, hl = w / 2 + SAT_BUF, l / 2 + SAT_BUF
    c_, s_ = math.cos(yaw), math.sin(yaw)
    cx, cy = cx + off * c_, cy + off * s_
    dx = np.array([hl, hl, -hl, -hl])
    dy = np.array([hw, -hw, -hw, hw])
    return np.column_stack([cx + dx * c_ - dy * s_, cy + dx * s_ + dy * c_])


def _overlap_subset(ego_c, bg_c):
    """SAT 重叠 (n,)。ego_c (n,4,2)；bg_c (4,2)（角序 [前左,前右,后右,后左]）。"""
    out = np.ones(len(ego_c), dtype=bool)
    for p, q in ((0, 1), (0, 3)):                 # ego 宽轴/长轴
        ax = ego_c[:, p] - ego_c[:, q]            # (n,2)
        pe = np.einsum("ikj,ij->ik", ego_c, ax)   # (n,4)
        pb = ax @ bg_c.T                          # (n,4)
        out &= ~((pe.max(1) < pb.min(1)) | (pb.max(1) < pe.min(1)))
    for p, q in ((0, 1), (0, 3)):                 # bg 宽轴/长轴
        ax = bg_c[p] - bg_c[q]                    # (2,)
        pe = ego_c @ ax                           # (n,4)
        pb = bg_c @ ax                            # (4,)
        out &= ~((pe.max(1) < pb.min()) | (pb.max() < pe.min(1)))
    return out


def _resample(xy, ds=DS):
    """折线 → 等弧长重采样 (n,3)[x,y,yaw] + 三点曲率（中值窗 9 平滑）+ 源弧长表。"""
    seg = np.hypot(*(xy[1:] - xy[:-1]).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(16, int(s[-1] / ds))
    s_new = np.linspace(0.0, s[-1], n)
    px = np.interp(s_new, s, xy[:, 0])
    py = np.interp(s_new, s, xy[:, 1])
    d = xy[1:] - xy[:-1]
    s_mid = (s[:-1] + s[1:]) / 2.0          # 段中点弧长（与段 yaw 同长 n−1）
    yaw = np.interp(s_new, s_mid, np.arctan2(d[:, 1], d[:, 0]))
    ax_, ay_ = px[1:-1] - px[:-2], py[1:-1] - py[:-2]
    bx_, by_ = px[2:] - px[1:-1], py[2:] - py[1:-1]
    cx_, cy_ = px[2:] - px[:-2], py[2:] - py[:-2]
    a_len = np.hypot(ax_, ay_); b_len = np.hypot(bx_, by_); c_len = np.hypot(cx_, cy_)
    area2 = np.abs(ax_ * by_ - bx_ * ay_)
    kap_mid = np.where(a_len * b_len * c_len > 1e-9,
                       2 * area2 / np.maximum(a_len * b_len * c_len, 1e-12), 0.0)
    kap = np.concatenate([[0.0], kap_mid, [0.0]])
    k = 4
    pad = np.pad(kap, k, mode="edge")
    kap = np.array([np.median(pad[i:i + 2 * k + 1]) for i in range(n)])
    return np.column_stack([px, py, yaw]), kap, s


class _PlanCtx:
    """单场计划上下文：路径/曲率/goal 格一次建，占用+DP 周期或触发重建。"""

    def __init__(self, ctrl, observation, path_waypoints):
        xy = np.asarray([[p[0], p[1]] for p in path_waypoints], dtype=float)
        self.path3, self.kap, self.s_wp = _resample(xy)
        self.n = len(self.kap)
        ego = observation["vehicle_info"]["ego"]
        shp = ego.get("shape", {}) or {}
        self.ew = float(shp.get("width", 4.0))
        self.el = float(shp.get("length", 9.0))
        self.eoff = (float(shp.get("locationPoint2Head", 0.0))
                     - float(shp.get("locationPoint2Rear", 0.0))) / 2.0
        # 横向跟踪余量：纯跟踪 14+1.5v 预瞄在弯道切内侧，真车系统性偏出中线
        # （3_2_1 实测 κ≈0.032 偏 2.3m）；占用表必须验真车可达域而非中线格
        lat_eps = np.minimum(2.5, 70.0 * np.maximum(self.kap, 0.0))
        self.ego_c = np.stack([_box_corners(x, y, yaw,
                                           self.ew + 2.0 * float(lat_eps[i]),
                                           self.el, self.eoff)
                               for i, (x, y, yaw) in enumerate(self.path3)])  # (n,4,2)
        self.name = observation["test_setting"].get("scenario_name", "")
        # KD 预筛树（路径格中心）
        try:
            from scipy.spatial import cKDTree
            self.tree = cKDTree(self.path3[:, :2])
        except Exception:
            self.tree = None
        # goal 格=路径首入 goal 多边形（评分到达语义：ego 定位点）
        ts = observation["test_setting"]
        gx = np.asarray(ts["goal"]["x"], dtype=float)
        gy = np.asarray(ts["goal"]["y"], dtype=float)
        try:
            from matplotlib.path import Path as MPath
            hits = np.where(MPath(np.column_stack([gx, gy])).contains_points(
                self.path3[:, :2]))[0]
        except Exception:
            hits = np.array([], dtype=int)
        self.i_goal = int(hits[0]) if len(hits) else self.n - 1
        # t_exp=spawn→goal 质心弦长/5（score_result 同式）
        sp = np.array([float(ego["x"]), float(ego["y"])])
        self.t_exp = max(1e-3, float(np.hypot(gx.mean() - sp[0],
                                              gy.mean() - sp[1])) / 5.0)
        self.plan = None            # (plan_s, plan_v, plan_t, kind)
        self.n_plan = -10 ** 9
        self.steps = 0
        self.logged_sup = False

    # ---------------- 占用表（直读官方 traj 原始 schema）----------------
    def _occupancy(self, t_now, max_t, i_ego=0):
        traj = _TRAJ_LATEST.get(self.name)
        nT = min(int((max_t - t_now) / DT) + 2, NT_CAP)
        occ = np.zeros((self.n, nT), dtype=bool)
        if not isinstance(traj, dict):
            return occ
        tq = t_now + np.arange(nT) * DT
        for veh in traj.values():
            if not isinstance(veh, dict):
                continue
            shp = veh.get("shape") or {}
            L, W = float(shp.get("length", 0.0)), float(shp.get("width", 0.0))
            if L <= 0 or W <= 0:
                continue
            off = (float(shp.get("locationPoint2Head", 0.0))
                   - float(shp.get("locationPoint2Rear", 0.0))) / 2.0
            ts_, xs, ys, yaws = [], [], [], []
            for k, fr in veh.items():
                try:
                    t = float(k)
                except (TypeError, ValueError):
                    continue
                if isinstance(fr, dict) and "x" in fr:
                    ts_.append(t)
                    xs.append(fr["x"]); ys.append(fr["y"])
                    # 原始 schema yaw_rad；容 fallback 短 schema yaw
                    yaws.append(fr.get("yaw_rad", fr.get("yaw", 0.0)))
            if not ts_:
                continue
            tv = np.array(ts_, float)
            order = np.argsort(tv)
            tv = tv[order]
            xv = np.array(xs, float)[order]
            yv = np.array(ys, float)[order]
            av = np.array(yaws, float)[order]
            # 各时间列取最近帧（容差 0.05s：车末键后无鬼影=车已离场）
            idx = np.clip(np.searchsorted(tv, tq), 0, len(tv) - 1)
            ok = np.abs(tv[idx] - tq) <= 0.05 + 1e-9
            ks = np.where(ok)[0]
            if not len(ks):
                continue
            for c in ks:                       # 逐候选帧：KD 预筛 + 局部 SAT
                bx, by, byaw = xv[idx[c]], yv[idx[c]], av[idx[c]]
                if self.tree is not None:
                    d, _ = self.tree.query([bx, by])
                    if d > R_PRE:
                        continue
                    cells = self.tree.query_ball_point([bx, by], R_PRE)
                else:
                    dd = np.hypot(self.path3[:, 0] - bx, self.path3[:, 1] - by)
                    cells = np.where(dd <= R_PRE)[0]
                if not len(cells):
                    continue
                bg_c = _box_corners(bx, by, byaw, W, L, off)
                occ[cells, c] |= _overlap_subset(self.ego_c[cells], bg_c)
        if os.environ.get("MINE_EXACT_DBG", "0") == "1":
            rows = []
            for veh in traj.values():
                if isinstance(veh, dict):
                    ts_ = []
                    for k in veh:
                        try:
                            ts_.append(float(k))
                        except (TypeError, ValueError):
                            pass
                    rows.append((len(ts_), round(min(ts_), 1) if ts_ else None,
                                 round(max(ts_), 1) if ts_ else None))
            blk = {i: [round(t_now + c * DT, 1) for c in np.where(occ[i])[0][:8]]
                   for i in range(max(0, i_ego - 3), min(self.n, i_ego + 9))
                   if occ[i].any()}
            print(f"[OCCDBG] t={t_now:.1f} vehs={rows} blk_near_ego={blk}", flush=True)
        return occ

    # ---------------- SIPP 窗 Dijkstra ----------------
    def _dp(self, occ, v_cap_arr, v0, i_ego, t_now, want_profile=False):
        """最早到达；等待只在 v=0（站立方窗滑动+直边兜底，探针 v3 三修复全保留）。"""
        import heapq
        nV = int(V_MAX / V_STEP) + 1
        N, nT = occ.shape
        INF = float("inf")
        T = [[INF] * nV for _ in range(N)]
        par = {} if want_profile else None

        def free_iv(col):
            iv, k, n = [], 0, len(col)
            while k < n:
                if not col[k]:
                    j = k
                    while j < n and not col[j]:
                        j += 1
                    iv.append((k * DT, (j - 1) * DT)); k = j
                else:
                    k += 1
            return iv

        fiv = [free_iv(occ[i]) for i in range(N)]
        j0 = min(nV - 1, int(round(v0 / V_STEP)))
        T[i_ego][j0] = t_now
        pq = [(t_now, i_ego, j0)]
        t_goal, j_goal = INF, None
        while pq:
            t, i, j = heapq.heappop(pq)
            if t > T[i][j] + 1e-12:
                continue
            if i >= self.i_goal:
                t_goal, j_goal = t, j
                break
            v = j * V_STEP
            k0 = min(max(int(round((t - t_now) / DT)), 0), nT - 1)
            for a in ACCELS:
                vn = math.sqrt(max(0.0, v * v + 2 * a * DS))
                if v <= 0.0 and vn <= 0.0:
                    continue    # 零速挪格幻影边（E2-3 撞车根因）：v=0 不加速
                                # 却以 dt=2DS/地板速 挪格，绕过站立窗占用检查
                jn = min(nV - 1, int(round(vn / V_STEP)))
                vn = jn * V_STEP
                cap = v_cap_arr[i + 1]
                if vn > cap + 1e-9:
                    jn = int(cap / V_STEP)
                    if jn < 0:
                        continue
                    vn = jn * V_STEP
                dt = 2 * DS / max(v + vn, 2 * V_BAR_FLOOR)
                if j == 0:
                    # 站立方窗：τ∈[t, b_end]，到达 τ+dt 落在 i+1 某空闲窗
                    win = None
                    if k0 >= 0 and not occ[i, k0]:
                        for a0, b0 in fiv[i]:
                            if a0 <= k0 * DT <= b0 or a0 <= (t - t_now) <= b0:
                                win = (a0, b0)
                                break
                    best = None
                    if win is not None:
                        for w_s, w_e in fiv[i + 1]:
                            cand = max(w_s, (t - t_now) + dt)
                            if cand <= w_e and cand - dt <= win[1] + 1e-9:
                                best = cand if best is None else min(best, cand)
                    # 直边兜底（保证=无等待版超集）
                    k2 = int(round(((t - t_now) + dt) / DT))
                    if k2 < nT and not occ[i + 1, k2]:
                        best = ((t - t_now) + dt if best is None
                                else min(best, (t - t_now) + dt))
                    if best is None:
                        continue
                    rel = best
                else:
                    rel = (t - t_now) + dt
                    k2 = int(round(rel / DT))
                    if k2 >= nT or occ[i + 1, k2]:
                        continue
                # 离开格过渡占用守卫：[k0,k2] 内 ego 盒仍压 cell i（慢边=站立
                # 挪动的真实占用；3_2_1 教训=只查到达列会把 ego 停进扫掠走廊）
                if k2 >= k0 and occ[i, k0:k2 + 1].any():
                    continue
                t2 = t_now + rel
                if t2 < T[i + 1][jn] - 1e-12:
                    T[i + 1][jn] = t2
                    if want_profile:
                        par[(i + 1, jn)] = (i, j)
                    heapq.heappush(pq, (t2, i + 1, jn))
        if j_goal is None:
            return None, None
        if not want_profile:
            return t_goal, None
        chain = []
        i, j = self.i_goal, j_goal
        while (i, j) in par:
            chain.append((i, j, T[i][j]))
            i, j = par[(i, j)]
        chain.append((i, j, T[i][j]))
        chain.reverse()
        return t_goal, chain

    # ---------------- 重规划 ----------------
    def _replan(self, observation, traj_future, t_now, i_ego):
        t0 = _time.perf_counter()
        max_t = float(observation["test_setting"]["max_t"])
        v0 = float(observation["vehicle_info"]["ego"].get("v_mps", 0.0))
        occ = self._occupancy(t_now, max_t, i_ego)
        kap_c = np.maximum(self.kap, 1e-9)
        cands = []
        for kind, caps in (("tight", np.minimum(np.sqrt(ALAT_TH / kap_c), V_MAX)),
                           ("relaxed", np.full(self.n, V_MAX))):
            arr, chain = self._dp(occ, caps, v0, i_ego, t_now, want_profile=True)
            if arr is None:
                continue
            # 镜像评分仅做变体选择（E2 教训：alat 构造性估计在执行层不兑现
            # ±0.04 vs 假设 +6~10；到达才是兑现杠杆）→ 到达优先，
            # tight 须快 >TIGHT_WIN 才胜出（alat 作次级平票项）
            dirty = 0.0
            for c in range(len(chain) - 1):
                i, j, tc = chain[c]
                if (j * V_STEP) ** 2 * self.kap[i] > ALAT_TH:
                    dirty += max(1e-3, chain[c + 1][2] - tc)
            frac = dirty / max(arr - t_now, 1e-3)
            cands.append((arr, 1.0 - frac, kind, arr, chain, frac))
            if os.environ.get("MINE_EXACT_DBG", "0") == "1":
                print(f"[RPDBG] t={t_now:.1f} {kind}: arr={arr:.2f} frac={frac:.2f}", flush=True)
        self.last_ms = round((_time.perf_counter() - t0) * 1000.0, 1)
        if not cands:
            self.plan = None
            return
        # 到达优先（min arr）；同窗内取 alat 分高者
        best_arr = min(c[0] for c in cands)
        finalists = [c for c in cands if c[0] <= best_arr + TIGHT_WIN]
        _, _, kind, arr, chain, frac = max(finalists, key=lambda c: c[1])
        self.plan = (np.array([c[0] * DS for c in chain]),
                     np.array([c[1] * V_STEP for c in chain]),
                     np.array([c[2] for c in chain]), kind)
        self.last_arr, self.last_frac = arr, frac
        self.n_plan = self.steps

    # ---------------- 每步入口 ----------------
    def vcmd(self, ctrl, v_cmd, observation, traj_future, mode):
        t_now = float(observation["test_setting"]["t"])
        self.steps += 1
        npid = getattr(ctrl, "_nearest_pt_id", None)
        s_ego = float(self.s_wp[int(npid)]) if npid is not None else 0.0
        i_ego = min(int(s_ego / DS), self.n - 2)
        if i_ego >= self.i_goal:
            return v_cmd                      # 已过 goal 格：交还银行控制
        drift = None
        if self.plan is not None:
            k = int(np.searchsorted(self.plan[0], s_ego, side="right")) - 1
            if 0 <= k < len(self.plan[2]):
                drift = t_now - float(self.plan[2][k])
        if (self.plan is None or self.steps - self.n_plan >= REPLAN_K
                or (drift is not None and abs(drift) > DRIFT_T)):
            try:
                self._replan(observation, traj_future, t_now, i_ego)
            except Exception:
                self.plan = None
        if mode == "log":
            self._audit(t_now, s_ego, v_cmd, drift)
            return v_cmd
        if self.plan is None:
            self._audit(t_now, s_ego, v_cmd, drift)
            return v_cmd                      # fail-closed=银行行为
        k = int(np.searchsorted(self.plan[0], s_ego, side="right")) - 1
        if k < 0:
            return v_cmd
        # 时间索引取速：k_t=计划期望 ego 于 t_now 所在格。等待格记录的是
        # 到达速 0，按格取速会把"等到 τ 后提速"锁死在 V_MIN（E2-4 撞车
        # 根因之二）；与实际格取 max=落后追赶、超前不越慢区
        k_t = int(np.searchsorted(self.plan[2], t_now, side="right")) - 1
        kk = min(max(k_t, k), len(self.plan[1]) - 1)
        v_t = float(self.plan[1][kk])
        if not math.isfinite(v_t):
            return v_cmd
        # v=0 等待格=计划性站立：命令 V_MIN（IDM/jerk 链收敛至近停），
        # 绝不回退银行 v_cmd（会把 ego 开进计划刚避开的冲突窗）
        out = V_MIN if v_t <= 0.0 else min(V_MAX, max(V_MIN, v_t))
        ctrl._stpp_exec_v = out                    # 计划接管帧（exec 模式速度设定值）
        self._audit(t_now, s_ego, v_cmd, drift)   # 记银行入参（v_plan=计划值）
        return out

    def _audit(self, t_now, s_ego, v_cmd, drift):
        """重规划级审计（E1 门数据：可行性/预算/离线-在线到达偏差）。"""
        try:
            d = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "..", "outputs", "stpp_log")
            os.makedirs(d, exist_ok=True)
            k = -1
            if self.plan is not None:
                k = max(int(np.searchsorted(self.plan[0], s_ego, side="right")) - 1, 0)
            rec = {"t": round(t_now, 1), "s": round(s_ego, 1),
                   "plan": self.plan is not None,
                   "kind": self.plan[3] if self.plan is not None else None,
                   "arrival": (round(float(self.plan[2][-1]), 2)
                               if self.plan is not None else None),
                   "v_plan": (round(float(self.plan[1][k]), 2)
                              if self.plan is not None else None),
                   "v_cmd_banked": round(v_cmd, 2),
                   "ms": getattr(self, "last_ms", None),
                   "drift": None if drift is None else round(drift, 2)}
            if self.steps == 1 and not self.logged_sup:
                rec["superseded_keys"] = _superseded(self.name)
                rec["t_exp"] = round(self.t_exp, 2)
                rec["i_goal_s"] = self.i_goal * DS
                self.logged_sup = True
            with open(os.path.join(d, f"{self.name}.jsonl"), "a") as f:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
        except Exception:
            pass


def vcmd(ctrl, v_cmd, observation, path_waypoints, traj_future, mode="on"):
    """入口（call-site try/except 之上再做一层 fail-closed）。"""
    # SAS rollout 副本让位（红队 10-03：MINE_EXACT=log 下副本会经本函数污染
    # 模块级 _PlanCtx 共享态——steps 计数/伪 replan/审计混入合成行；真链 ctrl
    # 无 _sas_cand 属性，此门对真实链零影响。副本继承的 _stpp_exec_v 在非
    # 接管场恒为 None，IDM 消费链不受影响）
    if getattr(ctrl, "_sas_cand", None) is not None:
        return v_cmd
    # MINE_STPP_EXEC（缺省 off=字节中性）：本帧计划实际接管才置 _stpp_exec_v，
    # simple_control 据此旁路 IDM 改为速度跟踪（排队族 58m 泡执行层解锁，10-02）
    ctrl._stpp_exec_v = None
    try:
        name = observation.get("test_setting", {}).get("scenario_name", "")
        if mode == "on" and not _takeover(name):
            return v_cmd
        ctx = _CTX.get(name)
        if ctx is None:
            ctx = _PlanCtx(ctrl, observation, path_waypoints)
            _CTX[name] = ctx
        return ctx.vcmd(ctrl, v_cmd, observation, traj_future, mode)
    except Exception:
        import os as _osd
        if _osd.environ.get("MINE_EXACT_DBG", "0") == "1":
            import traceback
            traceback.print_exc()
        return v_cmd
