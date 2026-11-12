"""当前在线安全tracking rollout的一次性负前馈重定时候选门。

全程位于原_replan计账范围。最多两次完整fixed-point政策预测，20秒软
预算和场景/进程剩余量只在政策边界检查，不宣称硬截断完整rollout。
本模块从不直接切换plan，也不在原selector选择前改_exec_candidates。
"""

import math
import time


ROWS_KEY = 'retiming_ego_rows'
MAX_POLICIES = 2
SOFT_BUDGET_S = 20.
MIN_REMAINING_S = 20.
CONVERGENCE_MPS = .03


def strict_safe_arrival(result):
    if not isinstance(result, dict) or result.get('end') != 4:
        return False
    detail = result.get('detail')
    if not isinstance(detail, dict):
        return False
    if (detail.get('vehicle_collision') is not False
            or detail.get('road_boundary_collision') is not False
            or detail.get('road_check') != 'bitmap_footprint'):
        return False
    try:
        return math.isfinite(float(result['score'])) and math.isfinite(float(result['arrival']))
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def _remaining(ctx, window_started_at, scene_limit_ms, global_used_ms, global_limit_ms):
    elapsed = max(0., time.perf_counter() - window_started_at)
    return ((scene_limit_ms - ctx._scene_ms) / 1000. - elapsed,
            (global_limit_ms - global_used_ms) / 1000. - elapsed)


def retiming_trials(ctx, ctrl, observation, results, incumbent, step, t,
                   window_started_at, scene_limit_ms, global_used_ms,
                   global_limit_ms, hysteresis, keep_ceiling):
    """返回有资格参评的(cand,result)，只使用本窗在线内存预测行。"""
    if not getattr(ctx, '_retiming_enabled', False) or ctx._retiming_ready:
        return []
    arrivals = [(candidate, result) for candidate, result in results if strict_safe_arrival(result)]
    if not arrivals:
        return []
    # 与原selector的total/arrival/F0平票顺序一致，不为低分几何引入额外政策。
    base, reference = min(arrivals, key=lambda pair: (
        -pair[1]['score'], pair[1]['arrival'], 0 if pair[0].kind == 'F0' else 1))
    if (reference['score'] >= keep_ceiling or not getattr(base, 'tracking', False)
            or not callable(getattr(base, 'steering', None))):
        return []
    tail = reference.get(ROWS_KEY)
    if not tail or not ctx.prefix:
        ctx._log({'ev': 'retiming_ff_skipped', 't': t, 'why': 'no_online_execution_rows'})
        return []
    remaining = _remaining(ctx, window_started_at, scene_limit_ms, global_used_ms, global_limit_ms)
    if min(remaining) < MIN_REMAINING_S:
        ctx._log({'ev': 'retiming_ff_skipped', 't': t, 'why': 'budget_boundary',
                  'scene_remaining_s': remaining[0], 'global_remaining_s': remaining[1]})
        return []

    # 空间表从本窗真实自车状态开始；历史prefix只由原_score保留评分，
    # 不进入重定时空间投影，避免在late window跳回初始车道分支。
    rows = [list(ctx.prefix[-1])] + [list(row) for row in tail]
    ctx._retiming_ready = True
    started = time.perf_counter()
    qualified, records, previous = [], [], None
    audit = {'ev': 'retiming_ff_built', 't': t, 'step': step,
             'base': base.label, 'baseline_score': reference['score'],
             'max_policies': MAX_POLICIES, 'soft_budget_s': SOFT_BUDGET_S,
             'minimum_policy_remaining_s': MIN_REMAINING_S,
             'total_wall_hard_cap_claimed': False,
             'elapsed_is_counted_by_replan_only': True,
             'row_source': 'current_state_plus_online_same_code_rollout_suffix',
             'reference_start_t': rows[0][0], 'historical_prefix_rows_excluded': len(ctx.prefix) - 1,
             'external_csv_used': False, 'steering_changed': False, 'records': records}
    try:
        from executed_retiming_ff import build_candidate
        from executed_retiming import profile_delta
        for iteration in range(1, MAX_POLICIES + 1):
            remaining = _remaining(ctx, window_started_at, scene_limit_ms,
                                   global_used_ms, global_limit_ms)
            if (min(remaining) < MIN_REMAINING_S
                    or time.perf_counter() - started >= SOFT_BUDGET_S):
                audit['stop_reason'] = 'budget_boundary'
                break
            candidate, profile_audit = build_candidate(base, rows, iteration)
            delta = profile_delta(previous, candidate) if previous is not None else None
            if delta and delta['max_abs_speed_change_mps'] <= CONVERGENCE_MPS:
                audit['stop_reason'] = 'profile_converged'
                audit['converged_next_profile'] = delta
                break
            record = {'candidate': candidate.label, 'iteration': iteration, 'accepted': False,
                      'profile_delta': delta,
                      'profile': {key: profile_audit.get(key) for key in (
                          'input_rows', 'reference_length_m', 'minimum_cap_mps', 'max_cap_mps',
                          'max_actual_alat_mps2', 'faster_station_count', 'slower_station_count',
                          'acceleration_feedforward', 'controller_p_gain')}}
            records.append(record)
            began = time.perf_counter()
            old_collect = ctx._retiming_collect_rows
            try:
                ctx._retiming_collect_rows = True
                result = ctx._rollout(ctrl, observation, candidate, step)
            finally:
                ctx._retiming_collect_rows = old_collect
                record['rollout_elapsed_s'] = time.perf_counter() - began
            if not strict_safe_arrival(result):
                record['reason'] = 'not_strict_safe_arrival'
                audit['stop_reason'] = 'unsafe_or_incomplete_new_execution'
                break
            record.update(score=result['score'], end=result['end'], arrival=result['arrival'],
                          delta_vs_online_reference=result['score'] - reference['score'])
            if result['score'] >= reference['score'] + hysteresis:
                record.update(accepted=True, reason='qualified_for_original_selector')
                qualified.append((candidate, result))
            else:
                record['reason'] = 'below_hysteresis'
            new_tail = result.get(ROWS_KEY)
            if not new_tail:
                audit['stop_reason'] = 'missing_next_execution_rows'
                break
            rows = [list(ctx.prefix[-1])] + [list(row) for row in new_tail]
            previous = candidate
    except Exception as error:
        qualified = []
        audit['error_type'] = type(error).__name__
        audit['stop_reason'] = 'helper_exception'
        for record in records:
            if record['accepted']:
                record.update(accepted=False, reason='discarded_after_helper_exception')
    finally:
        audit['predictions'] = len(records)
        audit['accepts'] = len(qualified)
        audit['elapsed_s'] = time.perf_counter() - started
        ctx._log(audit)
    # 保留身份用于selector之后的缓存提交；此时原cache仍不发生变化。
    ctx._retiming_pending = [candidate for candidate, _ in qualified]
    return qualified


def commit_selected(ctx, t):
    """仅缓存原selector实际选择的合格身份；其它结果不改既有执行槽。"""
    if ctx.plan is None:
        ctx._retiming_pending = []
        return False
    candidate = ctx.plan[0]
    if not any(candidate is pending for pending in ctx._retiming_pending):
        ctx._retiming_pending = []
        return False
    ctx._exec_candidates = [candidate]
    ctx._retiming_pending = []
    ctx._log({'ev': 'retiming_ff_cached', 't': t, 'candidate': candidate.label,
              'cache_commit_after_original_selector': True})
    return True
