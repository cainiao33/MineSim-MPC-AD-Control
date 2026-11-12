"""原STPP首帧Trace胜者的一次实际曲率NEGFF；原selector决定整场兑现。

只使用本窗原胜者完整safe在线ego_rows，不重演原银行或几何，不读取CSV。
一个新增完整政策、20秒阶段边界软预算；原scene/global预算只由prepare计账。
候选表只读，逐帧命令和原Trace转向使用原local_exit_execution合同。
"""

import hashlib
import math
import time

import numpy as np

from executed_retiming import build_profile
from executed_retiming_ff import ExecutedRetimingFeedforwardCand
from trajectory_candidates import TraceCand


SOFT_BUDGET_S = 20.
MIN_REMAINING_S = 20.
MAX_NEW_POLICIES = 1
MAX_PRIOR_POLICIES = 7  # 银行1 + 原goal6。
MAX_NONBANK_POLICIES = 7  # 原goal6 + 新bridge1。
MAX_TOTAL_POLICIES = 8  # 银行1 + 原goal6 + 新bridge1。


class GuardedNEGFF(ExecutedRetimingFeedforwardCand):
    """原NEGFF速度表/调用不变；转向显式委托同base，消费唯一同帧packet。"""

    kind = 'F14_STPP_EXECUTED_RETIMING_NEGATIVE_FF'
    _local_exit_policy = True

    def __setattr__(self, key, value):
        if self.__dict__.get('_sealed', False):
            raise AttributeError('stpp_retiming_candidate_is_immutable')
        object.__setattr__(self, key, value)

    def __init__(self, base, profile, iteration=1):
        if not isinstance(base, TraceCand) or iteration != 1:
            raise ValueError('one_original_trace_iteration_required')
        super().__init__(base, profile, iteration)
        self.label = 'STPP_' + self.label
        self._sealed = True

    def command(self, ctrl, bank_v, t, budget=None):
        t = float(t)
        state = getattr(ctrl, '_local_exit_frame', None)
        if (state is not None and state[0] is self and state[1] == id(ctrl)
                and (not state[5] or abs(state[2]-t) < 1e-8)):
            raise RuntimeError('stpp_retiming_duplicate_or_unconsumed_command')
        pose = tuple(float(getattr(ctrl, k)) for k in ('_ego_x', '_ego_y', '_ego_yaw', '_ego_v'))
        if not all(math.isfinite(x) for x in (*pose, t)):
            raise ValueError('nonfinite_stpp_retiming_frame')
        ctrl._local_exit_frame = (self, id(ctrl), t, True, pose, False, False)
        value = float(super().__call__(ctrl, bank_v, t))
        if not math.isfinite(value) or not .1 <= value <= 12.:
            raise ValueError('invalid_original_negative_ff_command')
        ctrl._local_exit_frame = (self, id(ctrl), t, True, pose, False, True)
        return value

    def __call__(self, ctrl, bank_v, t):
        return self.command(ctrl, bank_v, t)

    def steering(self, ctrl):
        state = getattr(ctrl, '_local_exit_frame', None)
        pose = tuple(float(getattr(ctrl, k)) for k in ('_ego_x', '_ego_y', '_ego_yaw', '_ego_v'))
        if (state is None or state[0] is not self or state[1] != id(ctrl)
                or state[5] or not state[6] or state[4] != pose):
            raise RuntimeError('stpp_retiming_steering_requires_unique_same_frame_command')
        value = float(self.base.steering(ctrl))
        if not math.isfinite(value) or abs(value) > 1.:
            raise ValueError('invalid_original_trace_steering')
        ctrl._local_exit_frame = state[:5] + (True, True)
        return value


def strict_safe(result):
    if not isinstance(result, dict) or result.get('end') != 4:
        return False
    detail = result.get('detail', {})
    try:
        return (detail.get('vehicle_collision') is False
            and detail.get('road_boundary_collision') is False
            and detail.get('road_check') == 'bitmap_footprint'
            and math.isfinite(float(result['score']))
            and math.isfinite(float(result['arrival'])))
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def _signature(policy):
    fields = {}
    for key, value in vars(policy).items():
        if isinstance(value, np.ndarray):
            if value.flags.writeable or not np.isfinite(value).all():
                raise ValueError('finite_readonly_policy_arrays_required')
            fields[key] = (value.shape, str(value.dtype), hashlib.sha256(value.tobytes()).hexdigest())
        elif key == 'base':
            fields[key] = id(value)
        else:
            fields[key] = value
    return fields


def _online_rows(ctx, result):
    rows = np.asarray(result.get('ego_rows', []), dtype=float)
    if (rows.ndim != 2 or rows.shape[1] != 5 or len(rows) < 5
            or not np.isfinite(rows).all() or np.any(rows[:, 4] < 0.)
            or abs(float(ctx.dt)-.1) > 1e-12
            or np.max(np.abs(np.diff(rows[:, 0])-ctx.dt)) > 1e-8
            or len(ctx.prefix) != 1 or np.max(np.abs(rows[0]-ctx.prefix[0])) > 1e-10
            or abs(rows[-1, 0]-float(result['arrival'])) > 1e-9
            or len(result.get('recs', {})) != len(rows)-1):
        raise ValueError('complete_current_online_rows_required')
    return rows


def build_guarded_candidate(base, rows):
    profile, audit = build_profile(rows)
    candidate = GuardedNEGFF(base, profile, iteration=1)
    audit.update(candidate=candidate.label, iteration=1, steering_source=base.label,
        steering_delegated=True, tracking_preserved=True, reference_uses_external_csv=False,
        acceleration_feedforward='negative_only', controller_p_gain=2.,
        feedforward_formula='selected_cap + min(0, .5*d(cap**2)/ds) / 2',
        acceleration_clip_mps2=[-2.5, 2.5],
        selected_station='current_or_ahead_whichever_cap_is_smaller',
        reference_acceleration_mps2=candidate._retime_acceleration.tolist(),
        original_ff_class_call_preserved=True, unique_command_steering_guard=True)
    return candidate, audit


def stpp_retiming_trials(ctx, ctrl, observation, source, reference, priors, snapshot,
                        window_started_at, scene_limit_ms, global_used_ms,
                        global_limit_ms, hysteresis, enabled=True):
    """返回最多一个合格pair及审计；不写原trials/pending/预算或真实控制器。"""
    started = time.perf_counter()
    qualified = []
    audit = dict(mechanism='selected_stpp_trace_executed_curvature_negative_ff',
        enabled=bool(enabled), external_csv_used=False, source_replays=0,
        max_new_policies=MAX_NEW_POLICIES, max_total_policies=MAX_TOTAL_POLICIES,
        max_prior_policies=MAX_PRIOR_POLICIES, max_nonbank_policies=MAX_NONBANK_POLICIES,
        soft_budget_s=SOFT_BUDGET_S, minimum_policy_remaining_s=MIN_REMAINING_S,
        hard_wall_cap_claimed=False, elapsed_counted_by_prepare_only=True,
        predictions=0, qualified=0, records=[], geometry_changed=False,
        coefficient_changed=False, look_changed=False)

    def boundary(stage):
        elapsed_ms = (time.perf_counter()-window_started_at)*1000.
        remaining = ((scene_limit_ms-ctx._scene_ms-elapsed_ms)/1000.,
                     (global_limit_ms-global_used_ms-elapsed_ms)/1000.)
        audit.update(scene_remaining_s=remaining[0], global_remaining_s=remaining[1])
        if min(remaining) < MIN_REMAINING_S or time.perf_counter()-started >= SOFT_BUDGET_S:
            audit.update(stop_reason='budget_boundary_previous_preserved', budget_stage=stage)
            return False
        return True

    try:
        if not enabled:
            audit['stop_reason'] = 'disabled'
            return qualified, audit
        if not isinstance(source, TraceCand) or not strict_safe(reference):
            audit['stop_reason'] = 'no_native_strict_safe_trace_winner'
            return qualified, audit
        if not any(candidate is source and result is reference for candidate, result in priors):
            raise ValueError('native_source_same_object_prior_required')
        safe = [(candidate, result) for candidate, result in priors if strict_safe(result)]
        highest = max(float(result['score']) for _, result in safe)
        threshold = highest + float(hysteresis)
        audit.update(source_candidate=source.label, source_reference_score=reference['score'],
            source_identity_matches_prior=True, prior_count=len(priors), prior_safe_count=len(safe),
            highest_prior_score=highest, required_score=threshold, original_hyst=float(hysteresis),
            prior_safe_candidates=[dict(candidate=c.label, score=r['score']) for c, r in safe])
        if len(priors) > MAX_PRIOR_POLICIES or threshold > 100.+1e-9:
            audit['stop_reason'] = 'prior_policy_cap_or_score_ceiling'
            return qualified, audit
        rows = _online_rows(ctx, reference)
        original = _signature(source)
        if not boundary('before_build'):
            return qualified, audit
        candidate, proposal = build_guarded_candidate(source, rows)
        audit['proposal'] = proposal
        signature = _signature(candidate)
        if candidate.base is not source or _signature(source) != original:
            raise ValueError('original_trace_geometry_identity_changed')
        if not boundary('before_prediction'):
            return qualified, audit
        from stpp_geometry import isolated_rollout, summary
        began = time.perf_counter()
        audit['predictions'] = 1
        result = isolated_rollout(ctx, ctrl, observation, candidate, snapshot)
        record = summary(candidate, result)
        record.update(qualified=False, required_score=threshold,
                      rollout_elapsed_s=time.perf_counter()-began)
        audit['records'].append(record)
        if _signature(source) != original or _signature(candidate) != signature:
            raise ValueError('policy_body_changed_during_prediction')
        audit['policy_bodies_unchanged'] = True
        if not boundary('after_prediction'):
            return qualified, audit
        if not strict_safe(result):
            audit['stop_reason'] = 'prediction_not_strict_bitmap_safe'
            return qualified, audit
        from local_exit_execution import valid
        from local_exit_trials import _execution_frames
        new_rows = _online_rows(ctx, result)
        if not valid(result, candidate, len(new_rows)-1):
            raise ValueError('original_local_exit_execution_contract_failed')
        frames = _execution_frames(result, new_rows, ctrl, float(ctx.dt))
        if any(abs(float(frame['final_steering_rad'])) > 1. for frame in frames):
            raise ValueError('final_steering_outside_env_domain')
        record.update(execution_audit=result['local_exit_execution_audit'],
                      delta_vs_highest_prior=float(result['score'])-highest)
        if not boundary('before_qualification'):
            return qualified, audit
        if result['score'] < threshold:
            audit['stop_reason'] = 'below_highest_prior_hysteresis'
            return qualified, audit
        qualified.append((candidate, result))
        record['qualified'] = True
        audit.update(qualified=1, qualified_candidate=candidate.label,
                     stop_reason='qualified_for_original_selector')
    except Exception as error:
        qualified.clear()
        for record in audit['records']:
            record['qualified'] = False
        audit.update(qualified=0, stop_reason='helper_exception_previous_preserved',
                     error_type=type(error).__name__, error_message=str(error))
    finally:
        audit['elapsed_s'] = time.perf_counter()-started
    return qualified, audit
