"""当前安全在线政策的保前缀退弯和一次实际曲率后缀修正。

只返回原selector可参评的对象，保留既有pending及资格。新增完整政策
至多两个；源同对象重演单独计账。20秒软预算只在阶段边界检查，
不声称完整rollout硬截断。所有构造行均为当前状态加本窗在线future。
"""

import copy
import math
import time
from pathlib import Path

from retiming_ff_trials import MIN_REMAINING_S, SOFT_BUDGET_S, _remaining, strict_safe_arrival
from retiming_full_ff_trials import _future_rows


MAX_LOCAL_POLICIES = 2
MAX_TOTAL_POLICIES = 6
STATIC_BUDGET_S = 8.
EXECUTION_KEY = 'local_exit_execution_audit'


def _max_delta(left, right):
    if len(left) != len(right) or not left:
        raise ValueError('incomplete_comparison_rows')
    if any(len(a) != len(b) for a, b in zip(left, right)):
        raise ValueError('comparison_row_shape_mismatch')
    values = [(float(a), float(b)) for ra, rb in zip(left, right) for a, b in zip(ra, rb)]
    if not all(math.isfinite(a) and math.isfinite(b) for a, b in values):
        raise ValueError('nonfinite_comparison_rows')
    return max(abs(a - b) for a, b in values)


def _execution_frames(result, rows, ctrl, dt):
    """真实挂点审计之外，再核对完整逐帧P/clip/jerk和加速度连续性。"""
    audit = result.get(EXECUTION_KEY)
    if not isinstance(audit, dict) or audit.get('pass') is not True:
        raise ValueError('complete_local_execution_audit_required')
    if not math.isfinite(float(audit.get('dt_s', float('nan')))) or abs(float(audit['dt_s']) - dt) > 1e-12:
        raise ValueError('local_execution_dt_mismatch')
    frames = audit.get('frames')
    if not isinstance(frames, list) or len(frames) != len(rows) - 1:
        raise ValueError('complete_local_execution_frames_required')
    previous = float(ctrl._last_acc)
    if not math.isfinite(previous) or not -6. <= previous <= 2.5:
        raise ValueError('invalid_initial_controller_acceleration')
    keys = ('t', 'command_mps', 'actual_acceleration_mps2', 'final_steering_rad',
            'previous_acceleration_mps2', 'speed_mps')
    for index, frame in enumerate(frames):
        values = {key: float(frame[key]) for key in keys}
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError('nonfinite_local_execution_frame')
        if (not .1 <= values['command_mps'] <= 12.
                or abs(values['t'] - rows[index][0]) > 1e-10
                or abs(values['speed_mps'] - rows[index][4]) > 1e-10
                or abs(values['previous_acceleration_mps2'] - previous) > 1e-12):
            raise ValueError('local_execution_frame_state_or_chain_mismatch')
        requested = max(-6., min(2.5, 2. * (values['command_mps'] - values['speed_mps'])))
        expected = max(previous - 1., min(previous + 1., requested))
        if (abs(expected - values['actual_acceleration_mps2']) > 1e-12
                or abs(max(0., values['speed_mps'] + expected * dt) - rows[index + 1][4]) > 1e-10):
            raise ValueError('local_execution_original_p_clip_jerk_mismatch')
        previous = values['actual_acceleration_mps2']
    return frames


def _prefix_check(source_rows, rows, source_frames, frames, cut):
    """包含cut状态；cut动作属于后缀，仅比较cut之前完整动作。"""
    original = [row for row in source_rows if row[0] <= cut + 1e-7]
    if len(rows) < len(original):
        raise ValueError('local_prefix_rows_missing')
    state_delta = _max_delta(original, rows[:len(original)])
    keys = ('t', 'command_mps', 'actual_acceleration_mps2', 'final_steering_rad')
    old_actions = [[frame[key] for key in keys] for frame in source_frames if frame['t'] < cut - 1e-7]
    new_actions = [[frame[key] for key in keys] for frame in frames if frame['t'] < cut - 1e-7]
    action_delta = _max_delta(old_actions, new_actions)
    return dict(pass_prefix=state_delta <= 1e-10 and action_delta <= 1e-12,
                cut_time_s=cut, rows_through_cut=len(original),
                action_frames_before_cut=len(old_actions),
                max_five_state_delta=state_delta, max_action_delta=action_delta)


def local_exit_trials(ctx, ctrl, observation, results, incumbent, step, t,
                      window_started_at, scene_limit_ms, global_used_ms,
                      global_limit_ms, hysteresis, keep_ceiling):
    """最多F11/F12两个完整预测，最终对象仍由原selector决定。"""
    if not getattr(ctx, '_local_exit_enabled', False) or getattr(ctx, '_local_exit_ready', False):
        return []
    started = time.perf_counter()
    pending = ctx._retiming_pending
    qualified, owned = [], []
    old_collect = ctx._retiming_collect_rows
    had_capture = hasattr(ctx, '_local_exit_capture_t')
    old_capture = getattr(ctx, '_local_exit_capture_t', None)
    audit = dict(ev='local_exit_built', t=t, step=step, max_local_policies=MAX_LOCAL_POLICIES,
                 max_total_policies=MAX_TOTAL_POLICIES, soft_budget_s=SOFT_BUDGET_S,
                 static_budget_s=STATIC_BUDGET_S, minimum_policy_remaining_s=MIN_REMAINING_S,
                 total_wall_hard_cap_claimed=False, elapsed_is_counted_by_replan_only=True,
                 external_csv_used=False, predictions=0, source_replays=0, accepts=0,
                 records=[], previous_pending_count=len(pending))

    def boundary(stage):
        remaining = _remaining(ctx, window_started_at, scene_limit_ms, global_used_ms, global_limit_ms)
        audit.update(scene_remaining_s=remaining[0], global_remaining_s=remaining[1])
        if min(remaining) < MIN_REMAINING_S or time.perf_counter() - started >= SOFT_BUDGET_S:
            audit.update(stop_reason='budget_boundary', budget_stage=stage)
            return False
        return True

    def predict(candidate, phase):
        if ctx._retiming_pending is not pending:
            raise RuntimeError('original_pending_identity_changed')
        candidate._local_exit_policy = True
        began = time.perf_counter()
        ctx._retiming_collect_rows = True
        try:
            audit['predictions'] += 1
            result = ctx._rollout(ctrl, observation, candidate, step)
        finally:
            ctx._retiming_collect_rows = old_collect
        record = dict(candidate=candidate.label, phase=phase, accepted=False,
                      rollout_elapsed_s=time.perf_counter() - began)
        audit['records'].append(record)
        if not strict_safe_arrival(result):
            record['reason'] = 'not_strict_safe_arrival'
            return result, None, None, record
        rows = _future_rows(ctx, result)
        frames = _execution_frames(result, rows, ctrl, float(ctx.dt))
        record.update(score=result['score'], end=result['end'], arrival=result['arrival'],
                      execution_audit=result[EXECUTION_KEY])
        return result, rows, frames, record

    def qualify(candidate, result, record, threshold):
        record.update(qualification_reference_score=threshold,
                      required_score=threshold + hysteresis,
                      delta_vs_best_prior_safe=result['score'] - threshold)
        if result['score'] >= threshold + hysteresis:
            if ctx._retiming_pending is not pending:
                raise RuntimeError('original_pending_identity_changed')
            pending.append(candidate)
            owned.append(candidate)
            qualified.append((candidate, result))
            record.update(accepted=True, reason='qualified_for_original_selector')
        else:
            record['reason'] = 'below_best_prior_hysteresis'

    def rollback():
        pending[:] = [candidate for candidate in pending if not any(candidate is ours for ours in owned)]
        ctx._retiming_pending = pending
        qualified.clear()
        audit['accepts'] = 0
        for record in audit['records']:
            if record['accepted']:
                record.update(accepted=False, reason='discarded_after_local_helper_exception')

    try:
        observed = list(getattr(ctx, '_local_exit_observed', []))
        pairs = list(results) + observed
        unique = {(id(candidate), id(result)): (candidate, result) for candidate, result in pairs
                  if strict_safe_arrival(result)}
        prior = list(unique.values())
        tracking = [(candidate, result) for candidate, result in prior
                    if getattr(candidate, 'tracking', False) and callable(getattr(candidate, 'steering', None))]
        if not tracking:
            audit['stop_reason'] = 'no_complete_safe_tracking_source'
            return qualified
        source_candidate, reference = min(tracking, key=lambda pair: (-pair[1]['score'], pair[1]['arrival']))
        best_prior = max(result['score'] for _, result in prior)
        audit.update(source_candidate=source_candidate.label, source_reference_score=reference['score'],
                     highest_prior_score=best_prior, observed_prior_count=len(observed))
        if best_prior >= keep_ceiling:
            audit['stop_reason'] = 'keep_ceiling'
            return qualified
        if not boundary('before_alignment'):
            return qualified
        reference_rows = _future_rows(ctx, reference)
        from control_local_curvature_exit import alignment_anchor, build_candidate
        from control_geometry_executed_suffix import refine_candidate
        from sas_core import _datasets_dir
        vertices = list(zip(ctx.goal['x'], ctx.goal['y']))
        if len(vertices) > 1 and vertices[0] == vertices[-1]:
            vertices = vertices[:-1]
        index, anchor = alignment_anchor(reference_rows, vertices)
        audit['anchor'] = anchor
        if index is None:
            ctx._local_exit_ready = True
            audit['stop_reason'] = 'no_alignment_event'
            return qualified
        cut = float(anchor['cut_time_s'])
        if not boundary('before_source_replay'):
            return qualified
        ctx._local_exit_ready = True
        ctx._retiming_collect_rows, ctx._local_exit_capture_t = True, cut
        began = time.perf_counter()
        try:
            audit['source_replays'] = 1
            source = ctx._rollout(ctrl, observation, source_candidate, step)
        finally:
            ctx._retiming_collect_rows = old_collect
            if had_capture:
                ctx._local_exit_capture_t = old_capture
            else:
                del ctx._local_exit_capture_t
            audit['source_replay_elapsed_s'] = time.perf_counter() - began
        if not strict_safe_arrival(source):
            audit['stop_reason'] = 'source_replay_not_strict_safe'
            return qualified
        source_rows = _future_rows(ctx, source)
        replay_delta = _max_delta(reference_rows, source_rows)
        audit['source_replay_max_five_state_delta'] = replay_delta
        if replay_delta > 1e-10:
            audit['stop_reason'] = 'source_replay_not_identical'
            return qualified
        source_frames = _execution_frames(source, source_rows, ctrl, float(ctx.dt))
        anchor_ctrl = source.get('local_exit_anchor_controller')
        if anchor_ctrl is None or index >= len(source_frames):
            raise ValueError('source_cut_controller_missing')
        pose = [[float(getattr(anchor_ctrl, key)) for key in ('_ego_x', '_ego_y', '_ego_yaw', '_ego_v')]]
        if (_max_delta(pose, [source_rows[index][1:]]) > 1e-10
                or abs(float(anchor_ctrl._last_acc) - source_frames[index]['actual_acceleration_mps2']) > 1e-12):
            raise ValueError('source_cut_controller_not_same_frame')
        construction = copy.copy(ctx)
        construction.prefix = [list(ctx.prefix[-1])]
        best_prior = max(best_prior, float(source['score']))
        if not boundary('before_geometry_build'):
            return qualified
        began = time.perf_counter()
        geometry, proposal = build_candidate(source_candidate, source_rows, construction, anchor_ctrl,
                                             Path(_datasets_dir()), budget_s=STATIC_BUDGET_S)
        audit.update(geometry_build_elapsed_s=time.perf_counter() - began, geometry_proposal=proposal)
        if geometry is None:
            audit['stop_reason'] = 'geometry_constructor_refused'
            return qualified
        if geometry.base is not source_candidate or float(geometry.cut_time) != cut:
            raise ValueError('geometry_source_identity_or_cut_changed')
        if not boundary('before_geometry_prediction'):
            return qualified
        geometry_result, geometry_rows, geometry_frames, record = predict(geometry, 'geometry')
        if geometry_rows is None:
            audit['stop_reason'] = 'geometry_not_strict_safe'
            return qualified
        prefix = _prefix_check(source_rows, geometry_rows, source_frames, geometry_frames, cut)
        record['prefix_check'] = prefix
        if not prefix['pass_prefix']:
            audit['stop_reason'] = 'geometry_prefix_rejected'
            return qualified
        qualify(geometry, geometry_result, record, best_prior)
        best_prior = max(best_prior, float(geometry_result['score']))
        audit['highest_prior_including_geometry_score'] = best_prior
        if best_prior >= keep_ceiling:
            audit['stop_reason'] = 'geometry_keep_ceiling'
            return qualified
        if not boundary('before_suffix_build'):
            return qualified
        began = time.perf_counter()
        candidate, refinement = refine_candidate(geometry, geometry_rows, construction, iteration=1)
        audit.update(suffix_build_elapsed_s=time.perf_counter() - began, suffix_refinement=refinement)
        if candidate is None:
            audit['stop_reason'] = 'suffix_constructor_refused'
            return qualified
        if (candidate is geometry or candidate.base is not source_candidate
                or candidate.suffix.base is not geometry.suffix
                or candidate.label == geometry.label or candidate.kind == geometry.kind):
            raise ValueError('refined_inner_trace_or_prefix_identity_changed')
        if not boundary('before_suffix_prediction'):
            return qualified
        result, rows, frames, record = predict(candidate, 'suffix_refinement')
        if rows is None:
            audit['stop_reason'] = 'refined_not_strict_safe'
            return qualified
        prefix = _prefix_check(source_rows, rows, source_frames, frames, cut)
        geometry_prefix = _prefix_check(geometry_rows, rows, geometry_frames, frames, cut)
        record.update(prefix_check=prefix, geometry_vs_refined_prefix_check=geometry_prefix)
        if not prefix['pass_prefix'] or not geometry_prefix['pass_prefix']:
            audit['stop_reason'] = 'refined_prefix_rejected'
            return qualified
        qualify(candidate, result, record, best_prior)
        audit['stop_reason'] = record['reason']
    except Exception as error:
        rollback()
        audit.update(error_type=type(error).__name__, error_message=str(error),
                     stop_reason='local_helper_exception_previous_preserved')
    finally:
        ctx._retiming_collect_rows = old_collect
        if had_capture:
            ctx._local_exit_capture_t = old_capture
        elif hasattr(ctx, '_local_exit_capture_t'):
            del ctx._local_exit_capture_t
        audit.update(accepts=len(qualified), elapsed_s=time.perf_counter() - started,
                     pending_identity_preserved=ctx._retiming_pending is pending)
        try:
            ctx._log(audit)
        except Exception as error:
            rollback()
            audit.update(error_type=type(error).__name__, error_message=str(error),
                         stop_reason='local_log_exception_previous_preserved')
    return qualified
