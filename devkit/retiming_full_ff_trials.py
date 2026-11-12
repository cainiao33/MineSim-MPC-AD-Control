"""逐字保留两次负前馈门，再至多尝试一次完整前馈。

观察代理只转发当前真实ctx的状态与bound方法，记录本窗严格安全的
负前馈结果；不改原helper、原selector、评分或预算。新增政策失败时
保留负前馈门已经返回的资格及身份，不在selector之前提交cache。
"""

import math
import time

from retiming_ff_trials import (
    MIN_REMAINING_S, ROWS_KEY, SOFT_BUDGET_S, _remaining,
    retiming_trials as negative_trials, strict_safe_arrival,
)


MAX_TOTAL_POLICIES = 3
MAX_FULL_FF_POLICIES = 1


class _NegativeObserver:
    """所有helper状态读写属于真实ctx；rollout仍用真实ctx的bound方法。"""

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


def _future_rows(ctx, result):
    """只接受完整当前状态及未来尾行，历史prefix不进入空间表。"""
    if not ctx.prefix:
        raise ValueError('missing_current_prefix_state')
    tail = result.get(ROWS_KEY)
    if not isinstance(tail, (list, tuple)) or not tail:
        raise ValueError('missing_online_negative_future_rows')
    rows = [list(ctx.prefix[-1])] + [list(row) for row in tail]
    if len(rows) < 5:
        raise ValueError('insufficient_online_negative_future_rows')
    if any(len(row) != 5 for row in rows):
        raise ValueError('invalid_online_negative_row_shape')
    rows = [[float(value) for value in row] for row in rows]
    if any(not math.isfinite(value) for row in rows for value in row):
        raise ValueError('nonfinite_online_negative_rows')
    if any(row[4] < 0. for row in rows):
        raise ValueError('negative_online_reference_speed')
    dt = float(ctx.dt)
    if not math.isfinite(dt) or dt <= 0.:
        raise ValueError('invalid_current_dt')
    if any(abs(after[0] - before[0] - dt) > 1e-6
           for before, after in zip(rows, rows[1:])):
        raise ValueError('future_rows_must_be_complete_current_suffix')
    if abs(rows[-1][0] - float(result['arrival'])) > 1e-6:
        raise ValueError('future_rows_do_not_reach_reported_arrival')
    return rows


def retiming_trials(ctx, ctrl, observation, results, incumbent, step, t,
                   window_started_at, scene_limit_ms, global_used_ms,
                   global_limit_ms, hysteresis, keep_ceiling):
    """原两轮负门的结果不变，完整前馈须胜过全部先前安全预测。"""
    arguments = (ctrl, observation, results, incumbent, step, t, window_started_at,
                 scene_limit_ms, global_used_ms, global_limit_ms, hysteresis, keep_ceiling)
    if not getattr(ctx, '_full_ff_enabled', False):
        return negative_trials(ctx, *arguments)
    started = time.perf_counter()
    observer = _NegativeObserver(ctx)
    qualified = negative_trials(observer, *arguments)
    # 原helper已完成自己的异常语义；新增门必须保留其结果及pending身份。
    negative_pending = ctx._retiming_pending
    negative_qualified = qualified
    negative_pending_count = len(negative_pending)
    if getattr(ctx, '_full_ff_ready', False):
        return qualified
    audit = {'ev': 'retiming_full_ff_built', 't': t, 'step': step,
             'full_ff_enabled': True, 'max_full_ff_policies': MAX_FULL_FF_POLICIES,
             'max_total_policies': MAX_TOTAL_POLICIES,
             'soft_budget_s': SOFT_BUDGET_S,
             'minimum_policy_remaining_s': MIN_REMAINING_S,
             'total_wall_hard_cap_claimed': False,
             'elapsed_is_counted_by_replan_only': True,
             'external_csv_used': False, 'steering_changed': False,
             'negative_qualifications_preserved': len(qualified),
             'predictions': 0, 'accepts': 0, 'attempted': False}
    try:
        safe_negative = object.__getattribute__(observer, '_safe_predictions')
        if not safe_negative:
            audit['stop_reason'] = 'no_strict_safe_negative_reference'
            return qualified
        # min稳定：同分先到，再完全平票保留第一次负预测；不读外部CSV。
        source_candidate, source = min(safe_negative, key=lambda pair: (
            -pair[1]['score'], pair[1]['arrival']))
        originals = [result for _, result in results if strict_safe_arrival(result)]
        if not originals:
            audit['stop_reason'] = 'no_strict_safe_current_original_reference'
            return qualified
        threshold_reference = max([result['score'] for result in originals]
                                  + [result['score'] for _, result in safe_negative])
        audit.update(source_negative=source_candidate.label,
                     source_negative_score=source['score'],
                     source_negative_arrival=source['arrival'],
                     qualification_reference_score=threshold_reference,
                     required_full_score=threshold_reference + hysteresis)
        if threshold_reference >= keep_ceiling:
            audit['stop_reason'] = 'keep_ceiling'
            return qualified
        remaining = _remaining(ctx, window_started_at, scene_limit_ms,
                               global_used_ms, global_limit_ms)
        audit.update(scene_remaining_s=remaining[0], global_remaining_s=remaining[1])
        if (min(remaining) < MIN_REMAINING_S
                or time.perf_counter() - started >= SOFT_BUDGET_S):
            audit['stop_reason'] = 'budget_boundary'
            return qualified
        rows = _future_rows(ctx, source)
        base = source_candidate.base
        if not getattr(base, 'tracking', False) or not callable(getattr(base, 'steering', None)):
            raise ValueError('negative_reference_lost_original_steering')
        audit.update(base=base.label, reference_start_t=rows[0][0], input_rows=len(rows),
                     historical_prefix_rows_excluded=len(ctx.prefix) - 1,
                     row_source='current_state_plus_strict_safe_negative_online_suffix')
        # attempt标志属于真实ctx，builder/rollout异常也不能触发第二次full。
        ctx._full_ff_ready = True
        audit['attempted'] = True
        from executed_retiming_full_ff import build_candidate
        candidate, profile = build_candidate(base, rows, iteration=1)
        audit.update(candidate=candidate.label,
                     profile={key: profile.get(key) for key in (
                         'input_rows', 'reference_length_m', 'minimum_cap_mps', 'max_cap_mps',
                         'max_actual_alat_mps2', 'acceleration_feedforward', 'controller_p_gain')})
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
        audit.update(score=result['score'], end=result['end'], arrival=result['arrival'],
                     delta_vs_best_prior_safe=result['score'] - threshold_reference)
        if result['score'] >= threshold_reference + hysteresis:
            # selector之后仍由原commit_selected缓存；不改既有执行槽。
            negative_pending.append(candidate)
            qualified = qualified + [(candidate, result)]
            audit.update(accepts=1, stop_reason='qualified_for_original_selector')
        else:
            audit['stop_reason'] = 'below_best_prior_hysteresis'
    except Exception as error:
        del negative_pending[negative_pending_count:]
        ctx._retiming_pending = negative_pending
        qualified = negative_qualified
        audit.update(error_type=type(error).__name__, error_message=str(error),
                     stop_reason='full_helper_exception_negative_preserved')
    finally:
        audit['elapsed_s'] = time.perf_counter() - started
        audit['negative_pending_identity_preserved'] = ctx._retiming_pending is negative_pending
        ctx._log(audit)
    return qualified
