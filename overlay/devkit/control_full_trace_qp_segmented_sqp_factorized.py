"""F40: bounded SQP, recentered only on complete safe nonlinear models.

The original source and identity are admitted once. Each new center is formed
from a full first-goal model which already passed every 0.1s physical gate.
Affine proposals remain provisional; only one final candidate may receive
native prediction and same-object Env verification after the original HYST.
"""

import math
import time
from dataclasses import replace
from types import SimpleNamespace

import numpy as np

import control_native_bank_phase as bank
from control_full_trace_qp_segmented_gt import (
    _admit, _action, _realize, _trial, _state, _finite, _pose, _score, _goal_geometry, _goal_cost,
    _earlier_target, _alat_linearization, _body_exit_corridor_rows, _final_bound,
    _Constraints, _QPBudget, _Chain, FullTraceSource, IdentityReceipt,
    FullTraceRefusal, FullTraceState, FrozenFullTraceTableCand, require, frozen_array,
    MAX_STEPS, SOURCE_SCALAR_LIMIT, MIN_FINAL_SCALARS, LOWER, UPPER, SCALES,
    REGULARIZATION, QP_DIRECTION_DIMENSION, QP_ITERATION_LIMIT, QP_ACTIVE_FACE_LIMIT,
    QP_TRIAL_MULTIPLIERS, QP_TRUST_RADIUS, QP_NUMERIC_TOLERANCE, QP_TARGET_INSET_M,
    QUANTIZATION_EPSILON, EXCLUDED_NOMINAL_BRANCH_PREFIXES, TURN_CURVATURE_CUTOFF,
    PARAMETER_ORDER, LOCAL_CENTERS, LOCAL_HALF_WIDTH, PHASE_SLOT, GLOBAL_COMMAND_SLOT,
    RELATIVE_RAW_STEERING_SCALE, NATIVE_STEERING_LIMIT, PHASE_ADVANCE_SCALE,
)
from control_qp_segmented_box import certify_direction_inside_exact_box
from control_qp_segmented_dominance import (
    prune_exact_redundancies as prune_box_implied_halfspaces,
    certify_original_removed_halfspaces,
)
from control_sqp_work_meter import SQPWorkMeter, MAX_SCALAR_STEPS

SQP_PASS_LIMIT = 3
QP_SOLVE_LIMIT = 3
MAX_TRIALS = 9
TRIAL_SCALAR_LIMIT = 800000


def _qp_constraints(source, chain, earlier, budget, audit):
    rows, sensitivity = chain.rows6, chain.sensitivities
    budget.charge('qp_source_yaw_unwrap_rows', len(rows))
    yaw = np.unwrap(rows[:, 3])
    radius = QP_TRUST_RADIUS
    # Source stencils must admit d=0. The prospective earlier ending is a
    # different constraint and may have negative slack which the QP must fix.
    quantization = []
    for index in range(len(rows)):
        budget.charge('qp_quantization_row_bounds')
        value, _, speed, omega, domega, weight = _alat_linearization(
            rows, sensitivity, index, recorded=True, unwrapped=yaw)
        q = QUANTIZATION_EPSILON * weight
        base = abs(speed) * q + QUANTIZATION_EPSILON * abs(omega) + QUANTIZATION_EPSILON * q
        slope = q * float(np.sum(np.abs(sensitivity[index, 4]))) + QUANTIZATION_EPSILON * float(np.sum(np.abs(domega)))
        slack = .4 - abs(value) - base
        require(slack >= 0., 'qp_source_quantization_envelope_excludes_identity')
        if slope > 0.:
            radius = min(radius, slack / slope)
        quantization.append((base, slope))
    require(math.isfinite(radius) and radius > 0., 'qp_quantization_trust_box_empty')
    # Avoid turning an exact equality into a negative floating-point slack.
    if radius < QP_TRUST_RADIUS:
        radius *= 1. - 1e-10
    constraints = _Constraints(budget)
    for axis in range(QP_DIRECTION_DIMENSION):
        unit = np.eye(QP_DIRECTION_DIMENSION)[axis]
        constraints.add(unit, min(radius, UPPER[axis] - chain.parameters[axis]), 'box_upper_%d' % axis)
        constraints.add(-unit, min(radius, chain.parameters[axis] - LOWER[axis]), 'box_lower_%d' % axis)
    budget.charge('qp_actual_goal_box_geometry_reads')
    goal = _goal_geometry(source)
    satisfied = (rows[earlier, 1:3] >= goal[[0, 2]]) & (rows[earlier, 1:3] <= goal[[1, 3]])
    for axis in range(2):
        if satisfied[axis]:
            gradient = sensitivity[earlier, axis + 1]
            constraints.add(gradient, goal[2 * axis + 1] - rows[earlier, axis + 1],
                            'goal_free_axis_upper_%d' % axis)
            constraints.add(-gradient, rows[earlier, axis + 1] - goal[2 * axis],
                            'goal_free_axis_lower_%d' % axis)
    require(chain.body_margins is not None and chain.body_margins.shape == (len(rows),)
            and np.all(chain.body_margins > 0.), 'positive_current_source_body_margins_required')
    budget.charge('qp_actual_body_shape_reads')
    body_radius = math.hypot(max(float(source.ego_shape['locationPoint2Head']),
                                float(source.ego_shape['locationPoint2Rear'])),
                             float(source.ego_shape['width']) / 2.)
    exit_indices = []
    for index, row in enumerate(rows):
        budget.charge('qp_exit_geometry_predicate_rows')
        angle = (float(row[3] - rows[0, 3]) + math.pi) % (2. * math.pi) - math.pi
        delta_index = min(index, len(chain.final_delta) - 1)
        curvature = abs(1.7 * math.tan(float(chain.final_delta[delta_index])) / source.vehicle_length)
        if abs(angle) < math.pi / 2. or curvature > TURN_CURVATURE_CUTOFF:
            continue
        budget.charge('qp_exit_corridor_stencils_formed')
        for normal in _body_exit_corridor_rows(row, sensitivity[index], body_radius):
            constraints.add(normal, 1. * float(chain.body_margins[index]),
                            'body_exit_pose_corridor_%d' % index)
        exit_indices.append(index)
    audit.update(qp_goal_free_axis=list(map(bool, satisfied)),
                 qp_goal_satisfied_axes_use_actual_box=True,
                 qp_goal_fixed_coordinate_penalty_for_satisfied_axes=False,
                 qp_exit_corridor_source_indices=exit_indices,
                 qp_exit_corridor_clearance_fraction=1.,
                 qp_exit_corridor_body_radius_m=body_radius,
                 qp_exit_corridor_is_affine_proposal_only=True)
    max_quantized_upper = 0.
    for index in range(len(rows)):
        budget.charge('qp_ALAT_stencils_formed')
        raw, gradient, _, _, _, _ = _alat_linearization(rows, sensitivity, index, unwrapped=yaw)
        continuous, rec_gradient, _, _, _, _ = _alat_linearization(
            rows, sensitivity, index, recorded=True, unwrapped=yaw)
        base, slope = quantization[index]
        error = base + radius * slope
        max_quantized_upper = max(max_quantized_upper, abs(continuous) + error)
        for sign in (-1., 1.):
            constraints.add(sign * gradient, .391 - sign * raw, 'raw_ALAT_%d' % index)
            constraints.add(sign * rec_gradient, .4 - error - sign * continuous, 'recorder_ALAT_%d' % index)
    budget.charge('qp_prospective_terminal_stencil')
    for recorded, limit in ((False, .391), (True, .4)):
        value, gradient, speed, omega, domega, weight = _alat_linearization(
            rows, sensitivity, earlier, terminal=True, recorded=recorded, unwrapped=yaw)
        q = QUANTIZATION_EPSILON * weight
        error = (abs(speed) + radius * float(np.sum(np.abs(sensitivity[earlier, 4])))) * q
        error += QUANTIZATION_EPSILON * (abs(omega) + radius * float(np.sum(np.abs(domega)))) + QUANTIZATION_EPSILON * q
        for sign in (-1., 1.):
            constraints.add(sign * gradient, limit - (error if recorded else 0.) - sign * value,
                            'prospective_terminal_recorder' if recorded else 'prospective_terminal_raw')
    # All nominal-branch faces in this stage were already excluded.
    # Their construction/export and per-row charges are not executed.
    # Full nonlinear/process/Env gates remain authoritative.
    budget.charge('qp_nominal_face_stage_elision')
    for plane in chain.traffic_planes:
        budget.charge('qp_GT_projection_rows')
        normal=np.asarray(plane.pose_normal) @ sensitivity[plane.index, 1:4]
        constraints.add(normal, plane.bound, 'GT_plane_%d_%s_%d_%d' % (plane.index,plane.actor,int(plane.recorded),plane.corner))
    audit.update(qp_live_GT_separation_planes=len(chain.traffic_planes),qp_GT_planes_are_affine_proposals_only=True)
    audit.update(qp_step_lower=np.maximum(LOWER - chain.parameters, -radius).tolist(),
                 qp_step_upper=np.minimum(UPPER - chain.parameters, radius).tolist(),
                 qp_recenter_parameters=chain.parameters.tolist())
    matrix, bounds = constraints.arrays()
    _finite(matrix, bounds)
    audit.update(qp_excluded_nominal_face_formation_elided=True,
        qp_excluded_nominal_faces_saved_and_charged=False,
        qp_excluded_nominal_branch_faces=constraints.excluded_nominal_faces,
        qp_excluded_nominal_branch_face_count=len(constraints.excluded_nominal_faces),
        qp_source_P2_jerk_branch_preservation_required=False,
        qp_source_nominal_branch_preservation_required=False,
        qp_source_phase_cell_preservation_required=False,
        qp_native_command_raw_clipping_gates_required=True,
        qp_policy_source_phase_is_identity=False,
        qp_physical_action_limits_unchanged=True,
        qp_native_steering_domain_rad=[-1., 1.],
        qp_excluded_nominal_branch_prefixes=list(EXCLUDED_NOMINAL_BRANCH_PREFIXES),
        qp_original_nonlinear_P2_clip_jerk_gates_required=True,
        qp_constraint_count=len(bounds), qp_trust_radius=radius,
        qp_trust_restricted_by_quantization=radius < QP_TRUST_RADIUS,
        qp_source_quantization_max_upper=max_quantized_upper,
        qp_quantization_bounds_are_local_affine_only=True,
        qp_source_rows_zero_step_feasible=True,
        qp_prospective_terminal_zero_step_required=False,
        qp_phase_right_cell_only=False, qp_constraint_labels=list(constraints.labels),
        qp_bounded_array_copies_in_per_row_formation_charge=True)
    return matrix, bounds


def _relinearize(source, chain, meter):
    """Differentiate saved safe states; never admit a partial or unsafe trial."""
    require(len(chain.rows6) == len(chain.commands) + 1,
            'complete_recenter_chain_shape')
    sensitivity = np.zeros((6, QP_DIRECTION_DIMENSION))
    derivatives = [sensitivity.copy()]
    meter.charge('sqp_recenter_sensitivity_retention')
    for index, (before, actual) in enumerate(zip(chain.rows6, chain.rows6[1:])):
        meter.charge('sqp_recenter_algebra_checks')
        meter.charge('sqp_recenter_sensitivity_updates')
        command, raw, dc, dr = _action(source, index, chain.parameters, chain.turn_basis)
        model, delta, acceleration, sensitivity = _realize(
            before, command, raw, dc, dr, sensitivity,
            source.vehicle_length, source.rot_config)
        require(np.max(np.abs(model - actual)) <= 1e-8
                and command == chain.commands[index]
                and raw == chain.raw_delta[index]
                and abs(delta - chain.final_delta[index]) <= 1e-10
                and abs(acceleration - actual[5]) <= 1e-10,
                'safe_recenter_action_and_model_not_bound')
        meter.charge('sqp_recenter_sensitivity_retention')
        derivatives.append(sensitivity.copy())
    result = frozen_array(np.asarray(derivatives),
                          (len(chain.rows6), 6, QP_DIRECTION_DIMENSION))
    return replace(chain, sensitivities=result)


def _rank(chain, earlier, goal):
    # Prefer earlier arrivals. At equal arrival time prefer smaller goal error
    # at the frozen earlier deadline, so a safe correction can be recentered.
    index = min(earlier, len(chain.rows6) - 1)
    error = _goal_cost(chain.rows6[index], np.zeros((6, QP_DIRECTION_DIMENSION)), goal)[0]
    return (-chain.static_score, error)


def _solve_qp(quadratic,gradient,matrix,bounds,budget,audit):
    from control_qp_segmented_factorized_primal import solve_factorized_identity_start_qp
    return solve_factorized_identity_start_qp(quadratic,gradient,matrix,bounds,budget,audit,
        objective_jacobian=np.asarray(audit['qp_objective_factor']),
        objective_residual=np.asarray(audit['qp_objective_residual']),
        regularization_center=np.asarray(audit['qp_regularization_center']),
        regularization=REGULARIZATION,iteration_limit=QP_ITERATION_LIMIT,
        face_limit=QP_ACTIVE_FACE_LIMIT,tolerance=QP_NUMERIC_TOLERANCE)


def build_candidate(source, identity_receipt, priors, backend, *, clock=time.perf_counter):
    meter = SQPWorkMeter(clock)
    candidate = None
    audit = dict(mechanism='factorized_complete_safe_model_recentered_segmented11D_SQP',
        accepted=False, candidate_count=0, qp_direction_dimension=11,
        qp_solve_limit=QP_SOLVE_LIMIT, qp_iteration_limit=QP_ITERATION_LIMIT,
        qp_active_face_limit=QP_ACTIVE_FACE_LIMIT, qp_solve_count=0,
        qp_iterations=0, sqp_pass_limit=SQP_PASS_LIMIT, sqp_passes=[],
        maximum_model_trials=MAX_TRIALS, trials=[], max_scalar_steps=MAX_SCALAR_STEPS,
        trial_scalar_stage_limit=TRIAL_SCALAR_LIMIT, source_scalar_stage_limit=15000,
        static_budget_s=8., final_reserve_s=1.5, complete_policy_predictions=0,
        Env_runs=0, graph_searches=0, parameter_grid=False, branch_grid=False,
        physical_infeasibility_claimed=False, physical_parameter_family_changed_from_parent=False,
        external_csv_used=False, original_process_executed_by_helper=False,
        qp_trial_multipliers=list(QP_TRIAL_MULTIPLIERS), parameter_order=list(PARAMETER_ORDER),
        maximum_KKT_size=22, vector_work_units=3, matrix_work_units=5,
        recenter_requires_complete_strict_safe_model=True,
        original_action_contract_unchanged=True, native_steering_limit_rad=1.,
        source_admission_steering_limit_rad=.6, phase_advance_slot=10)
    try:
        original, threshold, signature, goal, scale = _admit(
            source, identity_receipt, priors, backend, meter)
        original = replace(original,
            body_margins=backend.source_body_margins(meter),
            traffic_planes=backend.source_traffic_planes(meter))
        require(meter.scalar_steps <= SOURCE_SCALAR_LIMIT, 'live_GT_plane_source_stage_cap')
        best = current = original
        stage_start = meter.scalar_steps
        audit.update(identity_receipt_verified=True, identity_complete_rollouts=1,
            highest_physically_safe_prior_plus_hyst=threshold,
            source_admission_scalar=stage_start, optimization_stage_scalar_start=stage_start,
            source_frames=len(source.commands), source_static_score=original.static_score,
            source_linearization_diagnostics=original.diagnostics,
            retained_complete_identity_incumbent=True)
        meter.search = True
        for pass_index in range(SQP_PASS_LIMIT):
            meter.check('before_SQP_pass')
            if pass_index:
                linearization_start = meter.scalar_steps
                current = _relinearize(source, current, meter)
                current = replace(current,
                    body_margins=backend.current_body_margins(current.rows6, meter),
                    traffic_planes=backend.current_traffic_planes(current.rows6, meter))
                require(meter.scalar_steps - linearization_start <= SOURCE_SCALAR_LIMIT,
                        'recenter_linearization_stage_cap')
            meter.charge('sqp_center_diagnostic_rows', len(current.rows6))
            pa = dict(index=pass_index, source_frames=len(current.commands),
                center_parameters=current.parameters.tolist(),
                center_score=current.static_score, center_rows6=current.rows6.tolist(),
                center_commands=current.commands.tolist(), center_raw_delta=current.raw_delta.tolist(),
                center_final_delta=current.final_delta.tolist(),
                center_sensitivities=current.sensitivities.tolist(),
                center_complete_strict_safe=True, center_external_csv_used=False,
                center_safety_evidence='identity' if pass_index == 0 else 'complete_safe_trial',
                scalar_before=meter.scalar_steps, qp_solve_count=0, qp_iterations=0,
                trial_indices=[])
            audit['sqp_passes'].append(pa)
            maximum_final = _final_bound(backend, len(source.commands))
            reserve = max(MIN_FINAL_SCALARS, maximum_final + len(source.commands) + 4)
            worst_trial = 19 * len(source.commands) + 1
            stop_scalar = min(MAX_SCALAR_STEPS - reserve - worst_trial,
                              stage_start + TRIAL_SCALAR_LIMIT - worst_trial)
            budget = _QPBudget(meter, stop_scalar)
            pa.update(qp_stop_scalar=stop_scalar, qp_reserved_full_trial_scalar=worst_trial,
                      qp_reserved_final_scalar=reserve)
            view = SimpleNamespace(rows6=current.rows6, commands=current.commands, ctx=source.ctx)
            earlier, target, duration_limit = _earlier_target(view, threshold, goal, budget)
            pa.update(qp_fixed_earlier_goal_index=earlier, qp_earlier_duration_limit_s=duration_limit,
                      qp_target_xy=target.tolist(), qp_fixed_earlier_frame=True)
            budget.charge('qp_objective_build')
            jacobian = current.sensitivities[earlier, 1:3]
            residual = current.rows6[earlier, 1:3] - target
            violated = ((current.rows6[earlier, 1:3] < goal[[0, 2]]) |
                        (current.rows6[earlier, 1:3] > goal[[1, 3]]))
            require(np.any(violated), 'earlier_goal_frame_must_have_unsatisfied_axis')
            J, residual = jacobian[violated], residual[violated]
            quadratic = J.T @ J + REGULARIZATION * np.eye(QP_DIRECTION_DIMENSION)
            gradient = J.T @ residual + REGULARIZATION * current.parameters
            pa.update(qp_objective_factor=J.tolist(),qp_objective_residual=residual.tolist(),
                      qp_regularization_center=current.parameters.tolist())
            pa.update(qp_quadratic=quadratic.tolist(), qp_gradient=gradient.tolist(),
                      qp_goal_objective_violated_axes=list(map(bool, violated)),
                      qp_goal_objective_is_distance_to_box=True)
            matrix, bounds = _qp_constraints(source, current, earlier, budget, pa)
            matrix, bounds = prune_box_implied_halfspaces(
                matrix, bounds, pa['qp_constraint_labels'], pa['qp_step_lower'],
                pa['qp_step_upper'], budget, pa)
            try:
                direction = _solve_qp(quadratic, gradient, matrix, bounds, budget, pa)
                certify_direction_inside_exact_box(direction, budget, pa)
                certify_original_removed_halfspaces(direction, budget, pa)
            finally:
                audit['qp_solve_count'] += pa['qp_solve_count']
                audit['qp_iterations'] += pa['qp_iterations']
            require(float(np.linalg.norm(direction)) > 1e-12 and pa['qp_objective_change'] < 0.,
                    'qp_no_nonzero_local_merit_improvement')
            improved = current
            for multiplier in QP_TRIAL_MULTIPLIERS:
                meter.check('before_full_trace_trial')
                if (meter.scalar_steps + worst_trial + reserve > MAX_SCALAR_STEPS
                        or meter.scalar_steps - stage_start + worst_trial > TRIAL_SCALAR_LIMIT):
                    audit['optimizer_stop'] = 'full_trace_trial_scalar_reserve_reached'
                    break
                parameters = current.parameters + multiplier * direction
                require(np.all(parameters >= LOWER) and np.all(parameters <= UPPER),
                        'recenter_absolute_parameter_box')
                report = dict(index=len(audit['trials']), sqp_pass=pass_index,
                    multiplier=multiplier, parameters=parameters.tolist(),
                    attempted_frames=0, model_advanced_frames=0, complete=False,
                    feasible=False, became_best=False, scalar_before=meter.scalar_steps)
                pa['trial_indices'].append(report['index'])
                audit['trials'].append(report)
                try:
                    trial = _trial(source, parameters, backend, meter, goal, scale,
                                   report, original.turn_basis)
                    trial = replace(trial, turn_basis=original.turn_basis)
                    report.update(complete=True, feasible=True,
                        actual_model_first_goal_frame=len(trial.commands),
                        model_arrival_s=float(trial.rows6[-1, 0]),
                        static_score=trial.static_score, merit=trial.merit)
                    if _rank(trial, earlier, goal) < _rank(improved, earlier, goal):
                        improved, report['became_best'] = trial, True
                    if trial.static_score > best.static_score:
                        best = trial
                except (FullTraceRefusal, bank.NativeBankPhaseRefusal) as error:
                    report['reason'] = str(error)
                except bank.SearchStop as error:
                    report['reason'] = audit['optimizer_stop'] = str(error)
                    break
                finally:
                    report['scalar_after'] = meter.scalar_steps
            pa.update(scalar_after=meter.scalar_steps,
                      selected_center_parameters=improved.parameters.tolist(),
                      selected_center_score=improved.static_score)
            if best.static_score > threshold:
                break
            if improved is current:
                audit['optimizer_stop'] = 'no_complete_safe_recenter_improvement'
                break
            current = improved
        meter.search = False
        meter.check('before_final_full_trace')
        require(best.static_score > threshold, 'no_complete_SQP_trial_above_original_prior_hyst')
        reserve = _final_bound(backend, len(best.commands)) + len(best.commands) + 4
        require(meter.scalar_steps + reserve <= MAX_SCALAR_STEPS,
                'final_full_trace_scalar_reserve_missing')
        gate = backend.final_gate(best.rows6,
            np.column_stack((best.rows6[:-1, 0], best.commands, best.raw_delta)),
            best.final_delta, meter)
        require(isinstance(gate, dict) and all(gate.get(key) is True for key in (
            'complete_first_goal_chain', 'actual_bitmap_polygon_all_rows',
            'absolute_GT_all_rows', 'raw_and_recorder_ALAT_all_rows',
            'TTC_and_lateral_all_rows', 'static_only'))
            and gate.get('original_process_actions_verified', False) is False
            and gate.get('original_bank_process_actions_verified', False) is False,
            'complete_original_static_gate_required_without_process_claim')
        meter.charge('frozen_candidate_frame_checks', len(best.commands))
        candidate = FrozenFullTraceTableCand(source, best.rows6, best.commands,
                                             best.raw_delta, best.final_delta, role='optimized')
        meter.charge('final_source_signature_reads')
        require(source.signature() == signature, 'source_mutated_by_SQP')
        audit.update(candidate_count=1, model_frames=len(best.commands),
            model_arrival_s=float(best.rows6[-1, 0]), static_score=best.static_score,
            static_gain=best.static_score - source.result['score'],
            parameters=best.parameters.tolist(), final_gate=gate,
            model_time_is_actual_Env_arrival=False)
        meter.check('last_publication')
        audit['accepted'] = True
    except (FullTraceRefusal, bank.NativeBankPhaseRefusal, bank.SearchStop) as error:
        candidate = None
        audit.update(accepted=False, candidate_count=0, reason=str(error))
    except Exception as error:
        candidate = None
        audit.update(accepted=False, candidate_count=0, reason='sqp_backend_or_numeric_exception',
                     error=f'{type(error).__name__}: {error}')
    audit.update(scalar_steps=meter.scalar_steps, counters=dict(meter.counts))
    audit.setdefault('reason', 'accepted' if candidate is not None else
                     audit.get('optimizer_stop', 'not_accepted'))
    elapsed = clock() - meter.started
    audit['elapsed_s'] = elapsed if math.isfinite(elapsed) and elapsed >= 0. else None
    if not math.isfinite(elapsed) or not 0. <= elapsed < 8.:
        candidate = None
        audit.update(accepted=False, candidate_count=0, reason='total_soft_budget_final_metadata')
    return candidate, audit
