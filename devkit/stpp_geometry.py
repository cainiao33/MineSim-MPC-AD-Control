"""STPP 首帧几何选策：原银行同码影子、完整安全评分、整场兑现。

候选在 MotionController.process 入口选定，预瞄前可见。XY为地图米制
坐标；完整自车行采用[t,x,y,yaw_rad,v_mps]。背景仅用 Predictor 的官方
traj stash，不读取外部CSV。每场只搜索一次，最多六条goal-pose加一次重定时。
"""

import copy
import math
import os
from pathlib import Path
import time


MISSING = object()


class BankCand:
    """银行标记不提供tracking/steering；SAS挂点补调用原STPP。"""

    kind = "F0"
    label = "F0_STPP_BANK"
    stpp_bank_shadow = True

    def __init__(self, mode):
        self.mode = mode
        self.frames = 0
        self.failed = False

    def __call__(self, ctrl, bank_v, t):
        self.failed = True
        raise RuntimeError("STPP银行必须经专用SAS影子挂点")


class SearchState:
    """每个真实控制器仅一次尝试；副本只读取政策，不进入选策。"""

    def __init__(self, name):
        self.name = name
        self._audit = None
        self.ctx = None
        self.candidate = None
        self.bank_result = None
        self.selected_result = None
        self.search_ms = 0.
        self.reason = None


def consume_bank(ctrl, bank_v, observation, path, future, cand):
    """上游速度原样传给原STPP；保留其本帧exec标记供原加速度链消费。"""
    import stpp_hook

    if not getattr(ctrl, "_stpp_bank_shadow", False):
        cand.failed = True
        raise RuntimeError("银行影子标记不能用于真实执行")
    ctrl._sas_exec_policy = None
    ctrl._sas_geometry_candidate = None
    previous = getattr(ctrl, "_sas_cand", MISSING)
    if previous is not cand:
        cand.failed = True
        raise RuntimeError("STPP银行候选标记不一致")
    del ctrl._sas_cand
    try:
        value = stpp_hook.vcmd(ctrl, bank_v, observation, path, future, cand.mode)
        cand.frames += 1
        return value
    except BaseException:
        cand.failed = True
        raise
    finally:
        ctrl._sas_cand = previous


def isolated_rollout(ctx, ctrl, observation, cand, snapshot):
    """全horizon隔离STPP上下文；finally恢复真实字典和审计方法原引用。"""
    import stpp_hook

    real_ctx = stpp_hook._CTX
    real_traj = stpp_hook._TRAJ_LATEST
    real_audit = stpp_hook._PlanCtx._audit
    prior_score = ctx.__dict__.get("_score", MISSING)
    score_method = ctx._score
    rows_captured = []
    replica_seed = copy.copy(ctrl)
    # 原rollout还会换新projection/debug容器；这里不写真实控制器或场景。
    replica_seed._stpp_bank_shadow = isinstance(cand, BankCand)

    def capture_score(rows, end):
        rows_captured.extend(ctx.prefix + [list(row) for row in rows])
        return score_method(rows, end)

    try:
        stpp_hook._CTX = copy.deepcopy(snapshot)
        stpp_hook._TRAJ_LATEST = {ctx.name: ctx.traj}
        stpp_hook._PlanCtx._audit = lambda self, *a, **k: None
        ctx._score = capture_score
        result = ctx._rollout(replica_seed, observation, cand, 0)
        if isinstance(cand, BankCand) and (cand.failed or result is not None
                                         and cand.frames != len(result["recs"])):
            return None
        if result is not None:
            result["ego_rows"] = rows_captured
        return result
    finally:
        stpp_hook._CTX = real_ctx
        stpp_hook._TRAJ_LATEST = real_traj
        stpp_hook._PlanCtx._audit = real_audit
        if prior_score is MISSING:
            ctx.__dict__.pop("_score", None)
        else:
            ctx._score = prior_score


def safe_arrival(result):
    if not isinstance(result, dict) or result.get("end") != 4:
        return False
    detail = result.get("detail", {})
    score = result.get("score")
    return (isinstance(score, (int, float)) and math.isfinite(score)
            and detail.get("vehicle_collision") is False
            and detail.get("road_boundary_collision") is False)


def summary(cand, result):
    if result is None:
        return {"c": cand.label, "dead": True}
    detail = result.get("detail", {})
    return {"c": cand.label, "dead": False, "s": result.get("score"),
            "end": result.get("end"), "arr": result.get("arrival"),
            "vehicle_collision": detail.get("vehicle_collision"),
            "road_boundary_collision": detail.get("road_boundary_collision")}


def prepare(ctrl, observation, path, future):
    """入口预选，不调用真实STPP、不更改真实控制器的动力学或进度状态。"""
    state = getattr(ctrl, "_stpp_geometry_state", None)
    if state is not None:
        # 全horizon已验证在位者整场兑现，预算停止不使其切回另一条道路。
        return state.candidate
    if getattr(ctrl, "_sas_cand", None) is not None:
        return None
    mode = os.environ.get("MINE_STPP_GEOMETRY", "on")
    if (not ctrl._sas_cfg.get("stpp_geometry", False) or mode not in ("on", "bank")
            or os.environ.get("MINE_EXACT", "off") != "on"
            or os.environ.get("MINE_STPP_EXEC", "off") != "on"):
        return None
    import stpp_hook
    name = observation.get("test_setting", {}).get("scenario_name", "")
    if not stpp_hook._takeover(name):
        return None

    started = time.perf_counter()
    import sas_core
    state = SearchState(name)
    ctrl._stpp_geometry_state = state
    records, geometry_audit = [], []
    proposed, selected, selected_result = None, None, None
    error_type = None
    ctx = None
    budget_reason = None
    retiming_audit = None
    try:
        ctx = sas_core.SceneCtx(ctrl, observation, path)
        state.ctx = ctx
        ego = observation["vehicle_info"]["ego"]
        ctx.prefix = [[ctx.t0, float(ego["x"]), float(ego["y"]),
                       float(ego["yaw_rad"]), float(ego["v_mps"])]]
        ctx.traj = stpp_hook._TRAJ_LATEST.get(name)
        ctx._init_exp_time(ctrl)
        if sas_core._GLOBAL_MS >= sas_core.GLOBAL_BUDGET_MS:
            state.reason = "global_budget_before_search"
            return None
        if ctx.traj is None:
            state.reason = "no_traj"
            return None
        # 原控制器入口到STPP之间没有_CTX消费，这就是本帧pre-call状态。
        snapshot = copy.deepcopy(stpp_hook._CTX)
        bank = BankCand(os.environ.get("MINE_EXACT", "on"))
        baseline = isolated_rollout(ctx, ctrl, observation, bank, snapshot)
        state.bank_result = baseline
        records.append(summary(bank, baseline))
        if not safe_arrival(baseline):
            state.reason = "bank_not_safe_arrival"
            return None
        if mode == "bank":
            state.reason = "bank_verification_mode"
            return None
        if ((time.perf_counter() - started) * 1000. >= sas_core.SCENE_BUDGET_MS
                or sas_core._GLOBAL_MS + (time.perf_counter() - started) * 1000.
                >= sas_core.GLOBAL_BUDGET_MS):
            state.reason = "budget_before_geometry"
            return None
        from goal_pose_geometry import goal_pose_candidates
        candidates, geometry_audit = goal_pose_candidates(
            ctx, baseline, ctx.t0, Path(sas_core._datasets_dir()),
            coefficients=(.85,), looks=(6.,), max_paths=6)
        if len(candidates) > 6:
            raise RuntimeError("STPP几何动态政策超过六个上界")
        trials = []
        prior_results = [(bank, baseline)]
        for cand in candidates:
            # 搜索前沿及时停止，不开始下一段重计算；所有已花时间仍计账。
            elapsed = (time.perf_counter() - started) * 1000.
            if (elapsed >= sas_core.SCENE_BUDGET_MS or
                    sas_core._GLOBAL_MS + elapsed >= sas_core.GLOBAL_BUDGET_MS):
                state.reason = "budget_during_geometry"
                break
            result = isolated_rollout(ctx, ctrl, observation, cand, snapshot)
            prior_results.append((cand, result))
            records.append(summary(cand, result))
            if safe_arrival(result):
                trials.append((cand, result))
        if trials:
            proposed, result = max(trials, key=lambda pair: pair[1]["score"])
            if (ctrl._sas_cfg.get('stpp_retiming_enabled', False)
                    and result['score'] >= baseline['score'] + sas_core.HYST):
                state.stpp_retiming_source = proposed
                state.stpp_retiming_qualified = ()
                try:
                    from stpp_retiming_trials import stpp_retiming_trials
                    qualified, retiming_audit = stpp_retiming_trials(
                        ctx, ctrl, observation, proposed, result, prior_results, snapshot,
                        started, sas_core.SCENE_BUDGET_MS, sas_core._GLOBAL_MS,
                        sas_core.GLOBAL_BUDGET_MS, sas_core.HYST)
                    trials.extend(qualified)
                    state.stpp_retiming_qualified = tuple(qualified)
                    records.extend(retiming_audit.get('records', []))
                except Exception as error:
                    retiming_audit = dict(predictions=0, qualified=0, records=[],
                        stop_reason='helper_import_exception_previous_preserved',
                        error_type=type(error).__name__, error_message=str(error))
                state.stpp_retiming_audit = retiming_audit
                proposed, result = max(trials, key=lambda pair: pair[1]['score'])
            if result["score"] >= baseline["score"] + sas_core.HYST:
                selected, selected_result = proposed, result
                state.reason = "geometry_selected"
            elif state.reason is None:
                state.reason = "below_hysteresis"
        elif state.reason is None:
            state.reason = "no_safe_geometry"
    except Exception as error:
        # 仅首帧未兑现的候选失败：保持真实STPP和控制器状态，回原银行。
        selected, selected_result = None, None
        state.reason = "search_exception"
        error_type = type(error).__name__
    finally:
        search_ms = (time.perf_counter() - started) * 1000.
        state.search_ms = search_ms
        # 与普通SAS共享原进程计数器和阈值，不另开预算或提高帽。
        sas_core._GLOBAL_MS += search_ms
        if ctx is not None:
            ctx._win_ms = search_ms
            ctx._scene_ms += search_ms
            if ctx._scene_ms >= sas_core.SCENE_BUDGET_MS:
                budget_reason = "scene_budget"
        if sas_core._GLOBAL_MS >= sas_core.GLOBAL_BUDGET_MS:
            budget_reason = "global_budget"
        if budget_reason is not None or state.reason in ("budget_before_geometry", "budget_during_geometry"):
            selected, selected_result = None, None
            state.reason = budget_reason or state.reason
        baseline = state.bank_result
        bank_score = baseline.get("score") if isinstance(baseline, dict) else None
        winner_score = selected_result["score"] if selected_result is not None else bank_score
        event = {"ev": "stpp_geometry_search", "t": observation.get("test_setting", {}).get("t"),
                 "kind": "goal_pose", "mode": mode, "search_ms": search_ms,
                 "policy_count": len(records) - int(bool(records)),
                 "max_new_policies": 6, "coefficient": .85, "look": 6.,
                 "winner": selected.label if selected is not None else BankCand.label,
                 "score": winner_score, "bank_score": bank_score,
                 "delta": winner_score - bank_score if winner_score is not None and bank_score is not None else None,
                 "reason": state.reason, "error_type": error_type,
                 "scene_ms": ctx._scene_ms if ctx is not None else search_ms,
                 "global_ms": sas_core._GLOBAL_MS, "cands": records,
                 "geometry_audit": geometry_audit}
        if retiming_audit is not None:
            qualified = state.stpp_retiming_qualified
            retiming_audit.update(
                selector_commit_selected=selected.label if selected is not None else BankCand.label,
                selector_commit_exact_qualified_object=any(selected is cand for cand, _ in qualified),
                selector_commit_candidate_is_qualified=any(selected is cand for cand, _ in qualified),
                selector_commit_source_is_original_base=(selected is state.stpp_retiming_source
                    or any(selected is cand and cand.base is state.stpp_retiming_source for cand, _ in qualified)))
            event['max_new_policies'] = 7
            event['stpp_retiming'] = retiming_audit
        # launcher可沿用SceneCtx._log重定向，此事件与普通replan耗时分开汇总。
        try:
            sas_core.SceneCtx._log(ctx if ctx is not None else state, event)
        except Exception:
            # 审计/准备未完成时不能在下一帧才突然接管首帧参考。
            selected, selected_result = None, None
            state.reason = "audit_exception"
        state.candidate, state.selected_result = selected, selected_result
        if retiming_audit is not None:
            qualified = state.stpp_retiming_qualified
            retiming_audit.update(
                selector_commit_selected=selected.label if selected is not None else BankCand.label,
                selector_commit_exact_qualified_object=any(selected is cand for cand, _ in qualified),
                selector_commit_candidate_is_qualified=any(selected is cand for cand, _ in qualified),
                selector_commit_source_is_original_base=(selected is state.stpp_retiming_source
                    or any(selected is cand and cand.base is state.stpp_retiming_source for cand, _ in qualified)))
    return state.candidate
