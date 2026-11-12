"""完整保留v58两负/一full门，再至多尝试一个在线制动余量政策。

观察代理只收集本窗原helper完整严格安全预测。新增资格仍由原selector
决定，pending身份及旧资格不变；静态标量影子从不代替完整执行评分。
"""

import time

from retiming_ff_trials import MIN_REMAINING_S, _remaining, strict_safe_arrival
from retiming_full_ff_trials import _future_rows, retiming_trials as full_ff_trials


MAX_TOTAL_POLICIES = 4
MAX_BRAKING_POLICIES = 1
STATIC_BUDGET_S = 8.


class _FullFFObserver:
    __slots__ = ('_observed_ctx', '_safe_predictions')

    def __init__(self, ctx):
        object.__setattr__(self, '_observed_ctx', ctx)
        object.__setattr__(self, '_safe_predictions', [])

    def __getattr__(self, name):
        return getattr(object.__getattribute__(self, '_observed_ctx'), name)

    def __setattr__(self, name, value):
        setattr(object.__getattribute__(self, '_observed_ctx'), name, value)

    def _rollout(self, ctrl, observation, candidate, step):
        ctx = object.__getattribute__(self, '_observed_ctx')
        result = ctx._rollout(ctrl, observation, candidate, step)
        if strict_safe_arrival(result):
            object.__getattribute__(self, '_safe_predictions').append((candidate, result))
        return result


def retiming_trials(ctx, ctrl, observation, results, incumbent, step, t,
                   window_started_at, scene_limit_ms, global_used_ms,
                   global_limit_ms, hysteresis, keep_ceiling):
    """完整原v58流程先执行；新增政策胜过全部先前严格安全最高分。"""
    arguments = (ctrl, observation, results, incumbent, step, t, window_started_at,
                 scene_limit_ms, global_used_ms, global_limit_ms, hysteresis, keep_ceiling)
    if not getattr(ctx, '_braking_enabled', False):
        return full_ff_trials(ctx, *arguments)
    started = time.perf_counter()
    observer = _FullFFObserver(ctx)
    qualified = full_ff_trials(observer, *arguments)
    prior_pending = ctx._retiming_pending
    prior_qualified = qualified
    prior_pending_count = len(prior_pending)
    if getattr(ctx, '_braking_ready', False):
        return qualified
    audit = dict(ev='braking_reachability_built', t=t, step=step,
                 braking_enabled=True, max_braking_policies=MAX_BRAKING_POLICIES,
                 max_total_policies=MAX_TOTAL_POLICIES, static_budget_s=STATIC_BUDGET_S,
                 minimum_policy_remaining_s=MIN_REMAINING_S,
                 total_wall_hard_cap_claimed=False, elapsed_is_counted_by_replan_only=True,
                 external_csv_used=False, real_env_history_used=False, steering_changed=False,
                 previous_qualifications_preserved=len(qualified), attempted=False,
                 predictions=0, accepts=0)
    try:
        observed = object.__getattribute__(observer, '_safe_predictions')
        prior = [(candidate, result) for candidate, result in list(results) + observed
                 if strict_safe_arrival(result)]
        if not prior:
            audit['stop_reason'] = 'no_strict_safe_current_reference'
            return qualified
        tracking = [(candidate, result) for candidate, result in prior
                    if getattr(candidate, 'tracking', False)
                    and callable(getattr(candidate, 'steering', None))]
        if not tracking:
            audit['stop_reason'] = 'no_strict_safe_tracking_reference'
            return qualified
        # 稳定同分/同到达时刻按先到；base就是当前源对象，不剥wrapper。
        source_candidate, source = min(tracking, key=lambda pair: (
            -pair[1]['score'], pair[1]['arrival']))
        threshold = max(result['score'] for _, result in prior)
        audit.update(source_candidate=source_candidate.label,
                     source_reference_score=source['score'],
                     source_reference_arrival=source['arrival'],
                     qualification_reference_score=threshold,
                     required_braking_score=threshold + hysteresis)
        if threshold >= keep_ceiling:
            audit['stop_reason'] = 'keep_ceiling'
            return qualified
        remaining = _remaining(ctx, window_started_at, scene_limit_ms,
                               global_used_ms, global_limit_ms)
        audit.update(scene_remaining_s=remaining[0], global_remaining_s=remaining[1])
        if min(remaining) < MIN_REMAINING_S:
            audit['stop_reason'] = 'budget_boundary'
            return qualified
        rows = _future_rows(ctx, source)
        audit.update(base=source_candidate.label, reference_start_t=rows[0][0],
                     input_rows=len(rows), historical_prefix_rows_excluded=len(ctx.prefix) - 1,
                     row_source='current_state_plus_strict_safe_tracking_online_suffix')
        # 预算/源未就绪可延后；任何一次builder调用（含拒绝）都是本场唯一尝试。
        ctx._braking_ready = True
        audit['attempted'] = True
        from executed_braking_reachability import build_candidate, execution_audit_valid
        candidate, profile = build_candidate(
            source_candidate, rows, iteration=1, initial_acceleration=float(ctrl._last_acc),
            dt=float(ctx.dt), max_duration_s=min(66., float(ctx.max_t) - float(t)),
            budget_s=min(STATIC_BUDGET_S, min(remaining)))
        audit['profile'] = {name: profile.get(name) for name in (
            'accepted', 'reason', 'policy_count', 'candidate', 'memory_source',
            'initial_acceleration_mps2', 'dt_s', 'raw_curvature_proposal',
            'reference_arrival_s', 'shadow_arrival_s', 'scalar_steps', 'elapsed_s',
            'static_budget_s', 'shadow_is_physical_rollout', 'shadow_is_safety_certificate')}
        if candidate is None:
            audit['stop_reason'] = 'braking_builder_refused'
            return qualified
        audit['candidate'] = candidate.label
        # 静态构造消耗也属于原replan预算；完整同码只在政策边界启动。
        remaining = _remaining(ctx, window_started_at, scene_limit_ms,
                               global_used_ms, global_limit_ms)
        if min(remaining) < MIN_REMAINING_S:
            audit['stop_reason'] = 'budget_boundary_after_static_build'
            return qualified
        began = time.perf_counter()
        old_collect = ctx._retiming_collect_rows
        try:
            ctx._retiming_collect_rows = True
            audit['predictions'] = 1
            result = ctx._rollout(ctrl, observation, candidate, step)
        finally:
            ctx._retiming_collect_rows = old_collect
            audit['rollout_elapsed_s'] = time.perf_counter() - began
        if not strict_safe_arrival(result):
            audit['stop_reason'] = 'not_strict_safe_arrival'
            return qualified
        _future_rows(ctx, result)
        if not execution_audit_valid(result, candidate,
                                     len(result['retiming_ego_rows'])):
            audit['stop_reason'] = 'braking_execution_contract_failed'
            return qualified
        audit.update(score=result['score'], end=result['end'], arrival=result['arrival'],
                     delta_vs_best_prior_safe=result['score'] - threshold,
                     braking_execution_audit=result['braking_execution_audit'])
        if result['score'] >= threshold + hysteresis:
            prior_pending.append(candidate)
            qualified = qualified + [(candidate, result)]
            audit.update(accepts=1, stop_reason='qualified_for_original_selector')
        else:
            audit['stop_reason'] = 'below_best_prior_hysteresis'
    except Exception as error:
        del prior_pending[prior_pending_count:]
        ctx._retiming_pending = prior_pending
        qualified = prior_qualified
        audit.update(error_type=type(error).__name__, error_message=str(error),
                     stop_reason='braking_helper_exception_previous_preserved')
    finally:
        audit['elapsed_s'] = time.perf_counter() - started
        audit['pending_identity_preserved'] = ctx._retiming_pending is prior_pending
        ctx._log(audit)
    return qualified
