"""首窗当前安全赢家的全轨迹SQP；只返回一个原selector参评对象。"""

import hashlib
import json
import math
import time

import numpy as np

from control_full_trace_qp_contract import (
    FullTraceSource, IdentityReceipt, json_native, require,
)
from control_full_trace_native_domain_contract import FrozenNativeDomainTableCand
from control_full_trace_goal_dispatch import build_candidate
from control_full_trace_qp_segmented_sqp_factorized_backend import (
    FullTraceSegmentedSQPFactorizedBackendAdapter,
)
from full_trace_execution import ExecutionCapture, register_table
from full_trace_checks import (
    complete_alat_check, execution_actions, execution_rows6,
    first_actual_action_bridge, identity_verification, policy_tree_signature,
    readonly_policy_tree, table_execution_check, zero_safety_violations,
)
from retiming_ff_trials import (
    MIN_REMAINING_S, SOFT_BUDGET_S, _remaining, strict_safe_arrival,
)
import local_exit_execution


def _digest(value):
    wire = json.dumps(json_native(value), sort_keys=True, allow_nan=False,
                      separators=(',', ':')).encode()
    return hashlib.sha256(wire).hexdigest()


def _phase(ctx, ctrl, observation, candidate, step, phase, audit):
    capture = ExecutionCapture(candidate, phase, ctx.dt)
    existed = '_full_trace_capture' in vars(ctrl)
    old_capture = getattr(ctrl, '_full_trace_capture', None)
    old_collect = ctx._retiming_collect_rows
    started = time.perf_counter()
    try:
        ctrl._full_trace_capture = capture
        ctx._retiming_collect_rows = True
        result = ctx._rollout(ctrl, observation, candidate, step)
    finally:
        ctx._retiming_collect_rows = old_collect
        if existed:
            ctrl._full_trace_capture = old_capture
        else:
            del ctrl._full_trace_capture
    tail = result.get('retiming_ego_rows', []) if result is not None else []
    rows = [list(ctx.prefix[0])] + [list(row) for row in tail] if tail else []
    execution = capture.finish(len(tail))
    audit[phase] = dict(candidate=candidate.label, candidate_identity=id(candidate),
        wall_s=time.perf_counter() - started, frame_count=execution['frame_count'],
        execution_pass=execution['pass'], score=result.get('score') if result else None,
        arrival=result.get('arrival') if result else None,
        strict_safe=strict_safe_arrival(result), zero_violations=zero_safety_violations(result))
    require(strict_safe_arrival(result) and zero_safety_violations(result)
            and execution['pass'] is True, 'full_trace_' + phase + '_execution_or_safety_failed')
    if getattr(candidate, '_local_exit_policy', False):
        require(local_exit_execution.valid(result, candidate, len(tail)),
                'full_trace_' + phase + '_native_execution_failed')
    return result, rows, execution, capture.controller


def full_trace_trials(ctx, ctrl, observation, results, incumbent, step, t, mode,
                      window_started_at, scene_limit_ms, global_used_ms,
                      global_limit_ms, hysteresis, keep_ceiling):
    if not ctx._full_trace_sqp_enabled or ctx._full_trace_sqp_ready:
        return []
    ctx._full_trace_sqp_ready = True
    started = time.perf_counter()
    plan_before, cache_before = ctx.plan, ctx._exec_candidates
    cache_ids = [id(c) for c in cache_before]
    pending_before = ctx._retiming_pending
    pending_ids = [id(c) for c in pending_before]
    audit = dict(ev='full_trace_sqp_built', t=float(t), step=int(step),
        mechanism='F40_or_F45_live_goal_geometry_dispatch', source_replays=0,
        identity_predictions=0, optimized_predictions=0, qualified=0,
        native_Env_runs_in_helper=0, max_source_replays=1,
        max_identity_predictions=1, max_optimized_predictions=1,
        static_budget_s=8., final_static_reserve_s=1.5,
        soft_budget_s=SOFT_BUDGET_S, minimum_policy_remaining_s=MIN_REMAINING_S,
        total_wall_hard_cap_claimed=False, elapsed_is_counted_by_replan_only=True,
        external_csv_used=False, parameter_grid=False, first_window_only=True,
        prior_safe_candidates=[], accepted=False)
    qualified = []

    def boundary(stage):
        remaining = _remaining(ctx, window_started_at, scene_limit_ms,
                               global_used_ms, global_limit_ms)
        audit.update(scene_remaining_s=remaining[0], global_remaining_s=remaining[1])
        if min(remaining) < MIN_REMAINING_S or time.perf_counter() - started >= SOFT_BUDGET_S:
            audit.update(stop_reason='budget_boundary', budget_stage=stage)
            return False
        return True

    try:
        if mode != 'on' or step != 0 or len(ctx.prefix) != 1 or float(ctx.dt) != .1:
            audit['stop_reason'] = 'whole_first_window_on_mode_required'
            return qualified
        require(float(hysteresis) == .15, 'original_hysteresis_required')
        if plan_before is None:
            audit['stop_reason'] = 'no_native_selected_plan'
            return qualified
        base, score, recs = plan_before
        pairs = list(results) + list(getattr(ctx, '_local_exit_observed', []))
        unique = {(id(c), id(r)): (c, r) for c, r in pairs if strict_safe_arrival(r)}
        prior = list(unique.values())
        matching = [(c, r) for c, r in prior if c is base
                    and r['score'] == score and r['recs'] is recs]
        if (len(matching) != 1 or getattr(base, 'tracking', False) is not True
                or not callable(getattr(base, 'steering', None))):
            audit['stop_reason'] = 'native_selected_complete_safe_tracking_source_required'
            return qualified
        reference = matching[0][1]
        highest = max(float(r['score']) for _, r in prior)
        audit.update(source_candidate=base.label, source_candidate_identity=id(base),
            source_reference_score=float(score), highest_prior_score=highest,
            required_score=highest + hysteresis, original_hyst=hysteresis,
            prior_safe_candidates=[dict(candidate=c.label, score=float(r['score']),
                                        arrival=float(r['arrival'])) for c, r in prior])
        if highest >= keep_ceiling or highest + hysteresis >= 100.:
            audit['stop_reason'] = 'no_score_headroom'
            return qualified
        if not zero_safety_violations(reference):
            audit['stop_reason'] = 'source_has_safety_violations'
            return qualified
        reference_tail = reference.get('retiming_ego_rows', [])
        if not 1 <= len(reference_tail) <= 430:
            audit['stop_reason'] = 'bounded_complete_live_source_rows_required'
            return qualified
        signature = policy_tree_signature(base)
        if not readonly_policy_tree(signature):
            audit['stop_reason'] = 'readonly_source_policy_arrays_required'
            return qualified
        if not boundary('before_source_replay'):
            return qualified
        audit['source_replays'] = 1
        fixed, source_rows, source_execution, anchor = _phase(
            ctx, ctrl, observation, base, step, 'source', audit)
        reference_rows = np.asarray([list(ctx.prefix[0])] + [list(r) for r in reference_tail])
        actual_rows = np.asarray(source_rows)
        require(reference_rows.shape == actual_rows.shape,
                'source_replay_complete_row_shape_changed')
        error = float(np.max(np.abs(reference_rows - actual_rows)))
        source_alat = complete_alat_check(source_rows)
        audit.update(source_replay_max_five_state_delta=error,
                     source_complete_alat_check=source_alat)
        require(error <= 1e-10 and abs(fixed['score'] - score) <= 1e-8
                and source_alat['pass_alat'] is True
                and signature == policy_tree_signature(base),
                'same_selected_source_replay_not_identical')
        commands, raw, final = execution_actions(source_execution)
        rotations = getattr(anchor, '_rot_scenes', {})
        rotation = rotations.get(ctx.name)
        if rotation is None:
            rotation = rotations.get('_default')
        source = FullTraceSource(ctx=ctx, policy=base, result=reference, path=ctx.path,
            original_process=type(ctrl)._process_original, controller=anchor,
            rows6=execution_rows6(source_rows, source_execution), commands=commands,
            raw_delta=raw, final_delta=final, ego_shape=ctx.ego_shape, rot_config=rotation,
            execution=source_execution, online_same_object=True, external_csv_used=False,
            complete_actual_verified=True)
        contract_before = source.signature()
        if not boundary('before_identity_prediction'):
            return qualified
        identity = FrozenNativeDomainTableCand(source, role='identity')
        register_table(identity)
        identity_before = identity.signature()
        audit['identity_predictions'] = 1
        result, rows, execution, _ = _phase(ctx, ctrl, observation, identity, step, 'identity', audit)
        model, alat = table_execution_check(identity, rows, execution), complete_alat_check(rows)
        native = local_exit_execution.valid(result, identity, len(execution['frames']))
        verification = identity_verification(source, identity, result, rows, execution,
            model, alat, strict_safe_arrival, zero_safety_violations,
            native_execution_valid=native)
        require(contract_before == source.signature() and identity_before == identity.signature(),
                'identity_prediction_changed_source_or_table')
        receipt = IdentityReceipt(contract_before, identity, result, verification, new_complete_rollouts=1)
        audit.update(identity_verification=verification, identity_qualified=receipt.valid_for(source),
                     source_contract_sha256=_digest(contract_before))
        require(audit['identity_qualified'] is True, 'complete_identity_not_qualified')
        if not boundary('before_static_builder'):
            return qualified
        from sas_core import _datasets_dir
        adapter = FullTraceSegmentedSQPFactorizedBackendAdapter(source, _datasets_dir())
        candidate, proposal = build_candidate(source, receipt, tuple(r for _, r in prior), adapter.backend())
        audit.update(proposal=proposal, backend_evidence=adapter.evidence)
        require(contract_before == source.signature() and signature == policy_tree_signature(base),
                'full_trace_builder_changed_source')
        if candidate is None:
            audit['stop_reason'] = proposal['reason']
            return qualified
        require(isinstance(candidate, FrozenNativeDomainTableCand) and candidate.source is source
                and candidate.role == 'optimized' and proposal['accepted'] is True
                and proposal['candidate_count'] == 1
                and proposal['complete_policy_predictions'] == proposal['Env_runs'] == 0,
                'one_distinct_immutable_static_candidate_required')
        bridge = first_actual_action_bridge(candidate, source)
        audit['first_actual_action_bridge'] = bridge
        require(bridge['pass_bridge'] is True, 'first_actual_action_bridge_failed')
        if not boundary('before_optimized_prediction'):
            return qualified
        candidate_before = candidate.signature()
        register_table(candidate)
        audit['optimized_predictions'] = 1
        result, rows, execution, _ = _phase(ctx, ctrl, observation, candidate, step, 'optimized', audit)
        model, alat = table_execution_check(candidate, rows, execution), complete_alat_check(rows)
        audit.update(optimized_model_check=model, optimized_complete_alat_check=alat,
                     optimized_candidate=candidate.label, optimized_candidate_identity=id(candidate))
        require(model['pass_model'] is True and alat['pass_alat'] is True
                and candidate_before == candidate.signature()
                and contract_before == source.signature() and signature == policy_tree_signature(base),
                'optimized_prediction_did_not_match_complete_static_candidate')
        if result['score'] < highest + hysteresis:
            audit['stop_reason'] = 'optimized_below_highest_original_safe_prior_hyst'
            return qualified
        require(ctx.plan is plan_before and ctx._exec_candidates is cache_before
                and [id(c) for c in cache_before] == cache_ids
                and ctx._retiming_pending is pending_before
                and [id(c) for c in pending_before] == pending_ids,
                'original_plan_cache_or_pending_changed_inside_helper')
        qualified.append((candidate, result))
        audit.update(accepted=True, qualified=1, stop_reason='qualified_for_original_selector',
                     candidate_signature=candidate.signature())
    except Exception as error:
        qualified.clear()
        audit.update(accepted=False, qualified=0, stop_reason='full_trace_production_refused',
                     error_type=type(error).__name__, error=str(error))
    finally:
        audit.update(elapsed_s=time.perf_counter() - started,
            original_plan_identity_preserved=ctx.plan is plan_before,
            original_cache_identity_preserved=ctx._exec_candidates is cache_before,
            original_pending_identity_preserved=ctx._retiming_pending is pending_before)
        ctx._full_trace_sqp_diagnostic = json_native(audit)
        ctx._log(ctx._full_trace_sqp_diagnostic)
    return qualified
