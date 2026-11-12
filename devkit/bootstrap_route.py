"""无安全到达参考时的一次性路线救援；不改原计划及其缓存。

构造与逐条完整rollout均位于原_replan计时范围，预算只在政策边界检查。
2秒仅限制姿态搜索提案循环，不代表含地图、足迹和rollout的硬总耗时。
"""

import math
import time


MAX_POLICIES = 2
PROPOSAL_BUDGET_S = 2.0
MIN_REMAINING_S = 20.0


def strict_safe_arrival(result):
    """缺项、异常类型及非有限分数/到达时间均不作安全到达证据。"""
    if not isinstance(result, dict) or result.get("end") != 4:
        return False
    detail = result.get("detail")
    if not isinstance(detail, dict):
        return False
    if (detail.get("vehicle_collision") is not False
            or detail.get("road_boundary_collision") is not False
            or detail.get("road_check") != "bitmap_footprint"):
        return False
    try:
        return (math.isfinite(float(result["score"]))
                and math.isfinite(float(result["arrival"])))
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def _finite_score(result):
    try:
        score = float(result["score"])
        return score if math.isfinite(score) else None
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


def _remaining(ctx, window_started_at, scene_limit_ms, global_used_ms,
               global_limit_ms):
    # 两个累计值尚未包含当前窗口；扣除已花的窗口墙钟后才是可用余量。
    elapsed = max(0., time.perf_counter() - window_started_at)
    return ((scene_limit_ms - ctx._scene_ms) / 1000. - elapsed,
            (global_limit_ms - global_used_ms) / 1000. - elapsed)


def bootstrap_trials(ctx, ctrl, observation, results, incumbent, step, t,
                     datasets_dir, window_started_at, scene_limit_ms,
                     global_used_ms, global_limit_ms, hysteresis):
    """返回最多两条合格(cand,result)，最终切换仍由原selector裁决。"""
    # 必须在导入地图构造模块、读取地图或写ready之前检查。
    if any(strict_safe_arrival(result) for _, result in results):
        return []
    if not getattr(ctx, "_bootstrap_enabled", False) or ctx._bootstrap_ready:
        return []
    remaining = _remaining(ctx, window_started_at, scene_limit_ms,
                           global_used_ms, global_limit_ms)
    if min(remaining) < MIN_REMAINING_S:
        ctx._log({"ev": "bootstrap_skipped", "t": t, "why": "budget_boundary",
                  "scene_remaining_s": round(remaining[0], 6),
                  "global_remaining_s": round(remaining[1], 6)})
        return []

    # 同原alive语义：银行族保持原评分下界，tracking族拒绝任何碰撞。
    original_alive = []
    for candidate, result in results:
        if not isinstance(result, dict) or _finite_score(result) is None:
            continue
        detail = result.get("detail")
        if getattr(candidate, "tracking", False):
            if (not isinstance(detail, dict)
                    or detail.get("vehicle_collision") is not False
                    or detail.get("road_boundary_collision") is not False):
                continue
        original_alive.append((candidate, result))
    baseline_score = max((_finite_score(r) for _, r in original_alive), default=None)
    safe_unfinished = [r for _, r in original_alive if isinstance(r.get("detail"), dict)
                       and r["detail"].get("vehicle_collision") is False
                       and r["detail"].get("road_boundary_collision") is False]
    reference = max(safe_unfinished, key=_finite_score) if safe_unfinished else {"recs": {}}

    ctx._bootstrap_ready = True  # 真正尝试构造前置位，异常也不重试。
    started = time.perf_counter()
    predictions, qualified = [], []
    audit = {"ev": "bootstrap_built", "t": t, "step": step,
             "attempts": 1, "max_policies": MAX_POLICIES, "plans": 0,
             "predictions": 0, "accepts": 0, "proposal_soft_budget_s": PROPOSAL_BUDGET_S,
             "minimum_policy_remaining_s": MIN_REMAINING_S,
             "scene_remaining_before_s": round(remaining[0], 6),
             "global_remaining_before_s": round(remaining[1], 6),
             "baseline_score": baseline_score, "records": predictions,
             "elapsed_is_counted_by_replan_only": True,
             "total_wall_hard_cap_claimed": False}
    try:
        build_started = time.perf_counter()
        from bitmap_route import build_candidates
        candidates, construction = build_candidates(
            ctx, reference, incumbent, datasets_dir, budget_s=PROPOSAL_BUDGET_S)
        audit["build_elapsed_s"] = time.perf_counter() - build_started
        candidates = list(candidates)[:MAX_POLICIES]
        audit["plans"] = len(candidates)
        audit["construction"] = {k: construction.get(k) for k in (
            "construction", "expanded", "node_count", "reached_route_count",
            "minimum_turn_radius_m", "elapsed_s", "search_elapsed_s", "budget_s")}
        audit["static_records"] = [{k: record.get(k) for k in (
            "candidate", "accepted", "reason", "road_boundary_check",
            "search_time_cost_s", "terminal_yaw")}
            for record in construction.get("records", [])]

        for candidate in candidates:
            remaining = _remaining(ctx, window_started_at, scene_limit_ms,
                                   global_used_ms, global_limit_ms)
            if min(remaining) < MIN_REMAINING_S:
                audit["stopped_before_policy"] = getattr(candidate, "label", "")
                audit["stop_reason"] = "budget_boundary"
                break
            rollout_started = time.perf_counter()
            record = {"candidate": getattr(candidate, "label", ""),
                      "accepted": False}
            predictions.append(record)
            try:
                result = ctx._rollout(ctrl, observation, candidate, step)
            finally:
                record["rollout_elapsed_s"] = time.perf_counter() - rollout_started
            if not isinstance(result, dict):
                record["reason"] = "dead"
                continue
            score = _finite_score(result)
            detail = result.get("detail")
            try:
                arrival = float(result["arrival"])
                arrival = arrival if math.isfinite(arrival) else None
            except (KeyError, TypeError, ValueError, OverflowError):
                arrival = None
            record.update(end=result.get("end"), score=score,
                          arrival=arrival,
                          vehicle_collision=detail.get("vehicle_collision")
                          if isinstance(detail, dict) else None,
                          road_boundary_collision=detail.get("road_boundary_collision")
                          if isinstance(detail, dict) else None,
                          road_check=detail.get("road_check")
                          if isinstance(detail, dict) else None)
            if not strict_safe_arrival(result):
                record["reason"] = "not_strict_safe_arrival"
                continue
            if baseline_score is not None and score < baseline_score + hysteresis:
                record["reason"] = "below_hysteresis"
                continue
            record.update(accepted=True, reason="qualified")
            qualified.append((candidate, result))
    except Exception as error:
        qualified = []
        audit["error_type"] = type(error).__name__
        audit["stop_reason"] = "helper_exception"
        for record in predictions:
            if record["accepted"]:
                record.update(accepted=False, reason="discarded_after_helper_exception")
        if predictions and "reason" not in predictions[-1]:
            predictions[-1]["reason"] = "rollout_exception"
    finally:
        audit["predictions"] = len(predictions)
        audit["accepts"] = len(qualified)
        audit["rollout_elapsed_s"] = sum(r["rollout_elapsed_s"] for r in predictions)
        audit["elapsed_s"] = time.perf_counter() - started
        remaining = _remaining(ctx, window_started_at, scene_limit_ms,
                               global_used_ms, global_limit_ms)
        audit["scene_remaining_after_s"] = round(remaining[0], 6)
        audit["global_remaining_after_s"] = round(remaining[1], 6)
        ctx._log(audit)
    return qualified
