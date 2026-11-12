"""F20：完整恒等种子、单次五维局部凸QP及固定比例全轨迹提案。"""

import math
import time
from dataclasses import dataclass

import numpy as np

import control_native_bank_phase as bank
from control_full_trace_qp_contract import (
    FrozenFullTraceTableCand, FullTraceSource, IdentityReceipt,
    FullTraceRefusal, FullTraceState, require,
)


MAX_TRIALS = 3
MAX_STEPS = 430
SOURCE_SCALAR_LIMIT = 6500
TRIAL_SCALAR_LIMIT = 18500
MIN_FINAL_SCALARS = 5000
SCALES = np.array([.15, .03, .03, 2., .5])
LOWER = np.array([-1., -1., -1., 0., 0.])
UPPER = np.ones(5)
REGULARIZATION = 1e-6
QP_DIRECTION_DIMENSION = 5
QP_SOLVE_LIMIT = 1
QP_ITERATION_LIMIT = 10
QP_ACTIVE_FACE_LIMIT = 5
QP_TRIAL_MULTIPLIERS = (1., .5, .25)
QP_TRUST_RADIUS = .25
QP_NUMERIC_TOLERANCE = 1e-9
QP_TARGET_INSET_M = 1e-6
QUANTIZATION_EPSILON = 5e-5 + 1e-12


@dataclass(frozen=True)
class FullTraceBackend:
    selected: object
    pose: object
    final_gate: object
    final_charge_bound: object
    original_process: object


@dataclass(frozen=True)
class _Chain:
    rows6: np.ndarray
    commands: np.ndarray
    raw_delta: np.ndarray
    final_delta: np.ndarray
    merit: float
    gradient: np.ndarray
    static_score: float
    parameters: np.ndarray
    sensitivities: object = None
    action_derivatives: object = None
    diagnostics: object = None


def _finite(*values):
    require(all(np.isfinite(value).all() for value in values),
            'finite_continuation_numeric_required')


def _state(row):
    return FullTraceState(*map(float, row))


def _clip(value, derivative, lower, upper):
    # Strictly saturated branches have zero derivative. Equality uses the
    # inward one-sided derivative only as a proposal, never a gate certificate.
    if value < lower:
        return lower, np.zeros(5)
    if value > upper:
        return upper, np.zeros(5)
    return value, derivative


def _sample(array, phase):
    require(0. <= phase <= len(array) - 1., 'source_phase_outside_full_table')
    if len(array) == 1:
        return float(array[0]), 0.
    left = min(int(math.floor(phase)), len(array) - 2)
    slope = float(array[left + 1] - array[left])
    return float(array[left]) + (phase - left) * slope, slope


def _nominal_action_derivatives(source, index):
    """Pre-clip right-cell derivatives at the one identity linearization."""
    count = len(source.commands)
    u = index / (count - 1.) if count > 1 else 0.
    weight = 4. * u * (1. - u)
    phase = .5 * u * (1. - u) * (count - 1)
    _, command_slope = _sample(source.commands, float(index))
    raw, raw_slope = _sample(source.raw_delta, float(index))
    dc, dr = np.zeros(5), np.zeros(5)
    dc[3], dc[4] = 2. * weight, command_slope * phase
    dr[0], dr[1], dr[2], dr[4] = .15 * weight * raw, .03 * weight, .03 * weight * (2. * u - 1.), raw_slope * phase
    return np.stack((dc, dr))


def _action(source, index, parameters):
    count = len(source.commands)
    u = index / (count - 1.) if count > 1 else 0.
    weight = 4. * u * (1. - u)
    odd = weight * (2. * u - 1.)
    phase_derivative = .5 * u * (1. - u) * (count - 1)
    phase = index + parameters[4] * phase_derivative
    command, command_slope = _sample(source.commands, phase)
    raw, raw_slope = _sample(source.raw_delta, phase)
    dc, dr = np.zeros(5), np.zeros(5)
    dc[3], dc[4] = 2. * weight, command_slope * phase_derivative
    dr[0], dr[1], dr[2] = .15 * weight * raw, .03 * weight, .03 * odd
    dr[4] = raw_slope * phase_derivative * (1. + .15 * parameters[0] * weight)
    raw = raw * (1. + .15 * parameters[0] * weight) + .03 * parameters[1] * weight + .03 * parameters[2] * odd
    command += 2. * parameters[3] * weight
    if not np.any(parameters):
        # Preserve exact original command/raw bytes at identity. Derivatives
        # still describe the declared right-cell proposal parameterization.
        command, raw = float(source.commands[index]), float(source.raw_delta[index])
    command, dc = _clip(command, dc, .1, 12.)
    # The capped equality uses its outward cone in the QP. No fictitious
    # speed sensitivity is propagated through a command already at its cap.
    if command == .1 or command == 12.:
        dc = np.zeros(5)
    raw, dr = _clip(raw, dr, -.6, .6)
    _finite(command, raw, dc, dr)
    return command, raw, dc, dr


def _realize(row, command, raw, dc, dr, sensitivity, length, rotation):
    """Exact P2/jerk/original old-speed rot and plant plus 6x5 sensitivity."""
    t, x, y, yaw, speed, previous = map(float, row)
    dspeed, dprevious = sensitivity[4], sensitivity[5]
    acceleration, da = _clip(2. * (command - speed),
                              2. * (dc - dspeed), -6., 2.5)
    if acceleration > previous + 1.:
        acceleration, da = previous + 1., dprevious.copy()
    elif acceleration < previous - 1.:
        acceleration, da = previous - 1., dprevious.copy()
    delta, dd = raw, dr.copy()
    if rotation not in (None, 'off', {}) and speed > .1:
        coefficient = float(rotation.get('a_rot', .38)) * length / 1.7
        cap = math.atan(coefficient / (speed * speed))
        if abs(raw) > cap:
            sign = math.copysign(1., raw)
            delta = sign * cap
            dd = sign * (-2. * coefficient / (speed ** 3)
                         / (1. + (coefficient / speed ** 2) ** 2)) * dspeed
    after = np.array([t + .1, x + speed * math.cos(yaw) * .1,
        y + speed * math.sin(yaw) * .1,
        yaw + speed / length * 1.7 * math.tan(delta) * .1,
        max(0., speed + .1 * acceleration), acceleration])
    derivative = sensitivity.copy()
    derivative[1] += .1 * (dspeed * math.cos(yaw) - speed * math.sin(yaw) * sensitivity[3])
    derivative[2] += .1 * (dspeed * math.sin(yaw) + speed * math.cos(yaw) * sensitivity[3])
    derivative[3] += .17 / length * (dspeed * math.tan(delta)
        + speed / math.cos(delta) ** 2 * dd)
    derivative[4] = dspeed + .1 * da if after[4] > 0. else np.zeros(5)
    derivative[5] = da
    _finite(after, derivative, delta, acceleration)
    return after, delta, acceleration, derivative


def _goal_geometry(source):
    goal = source.ctx.goal
    x, y = np.asarray(goal['x'], dtype=float), np.asarray(goal['y'], dtype=float)
    require(x.size > 0 and y.size > 0, 'nonempty_actual_goal_box_required')
    _finite(x, y)
    return np.array([x.min(), x.max(), y.min(), y.max()])


def _goal_cost(row, sensitivity, goal):
    dx = min(float(row[1] - goal[0]), 0.) + max(float(row[1] - goal[1]), 0.)
    dy = min(float(row[2] - goal[2]), 0.) + max(float(row[2] - goal[3]), 0.)
    cost = dx * dx + dy * dy
    gradient = 2. * (dx * sensitivity[1] + dy * sensitivity[2])
    _finite(cost, gradient)
    return cost, gradient


def _score(source, terminal):
    duration = float(terminal[0] - source.rows6[0, 0])
    expected = float(source.ctx._exp_t)
    require(math.isfinite(expected) and expected > 0. and duration > 0.,
            'positive_original_expected_and_arrival_time_required')
    # Arithmetic only for complete zero-violation arrivals. Actual score is
    # still owned by the single later original prediction/Env verification.
    return 80. + 20. * min(1., expected / duration)


def _pose(backend, row, meter):
    # Backend owns pose_gates plus every real nested map/GT query charge.
    evidence = backend.pose(_state(row), meter)
    require(all(getattr(evidence, name) is True for name in (
        'runtime_road_safe', 'bitmap_polygon_safe', 'GT_collision_free',
        'ttc_safe', 'lateral_distance_safe')), 'full_trace_pose_not_strict_safe')
    require(isinstance(evidence.in_goal, bool), 'actual_boolean_first_goal_required')
    return evidence


def _admit(source, receipt, priors, backend, meter):
    require(isinstance(source, FullTraceSource), 'current_full_tracking_source_required')
    require(isinstance(receipt, IdentityReceipt), 'new_identity_receipt_required')
    meter.charge('identity_receipt_reads')
    require(receipt.valid_for(source) and receipt.actual_result is not source.result,
            'new_whole_policy_identity_not_verified')
    meter.charge('source_signature_reads')
    signature = source.signature()
    selected, result = meter.call('selected_source_queries', backend.selected)
    require(selected is source.policy and result is source.result,
            'source_not_actual_live_selected_policy_result')
    require(backend.original_process is source.original_process,
            'original_process_identity_changed')
    require(source.online_same_object is True and source.external_csv_used is False
            and source.complete_actual_verified is True,
            'complete_actual_online_tracking_source_required')
    require(isinstance(priors, tuple) and any(value is source.result for value in priors),
            'all_original_priors_including_same_source_required')
    require(bank._physically_safe(source.result) and
            all(float(source.result['detail']['ratios'][key]) == 0.
                for key in ('ttc', 'lat', 'alat')), 'source_not_complete_strict_safe')
    require(bank._physically_safe(receipt.actual_result)
            and abs(float(receipt.actual_result['score']) - source.result['score']) <= 1e-8,
            'identity_actual_result_not_same_safe_score')
    safe_scores = [float(value['score']) for value in priors if bank._physically_safe(value)]
    require(safe_scores and max(safe_scores) + .15 < 100., 'no_score_headroom')
    threshold = max(safe_scores) + .15
    require(1 <= len(source.commands) <= MAX_STEPS and source.dt == .1,
            'complete_bounded_point_one_source_required')
    require(np.max(np.abs(source.raw_delta)) <= .6 and np.max(np.abs(source.final_delta)) <= .6,
            'source_steering_domain_exceeded')
    require(abs(float(source.controller._last_acc) - source.rows6[0, 5]) <= 1e-12,
            'source_initial_acceleration_mismatch')
    rotation = source.rot_config
    if rotation not in (None, 'off', {}):
        require(isinstance(rotation, dict) and math.isfinite(float(rotation.get('a_rot', .38)))
                and float(rotation.get('a_rot', .38)) > 0., 'valid_original_rotation_required')
    meter.charge('goal_geometry_reads')
    goal = _goal_geometry(source)
    zero = np.zeros(5)
    scale = max(_goal_cost(source.rows6[0], np.zeros((6, 5)), goal)[0], 1.) * len(source.commands)
    first = _pose(backend, source.rows6[0], meter)
    require(first.in_goal is False, 'source_initial_already_in_goal')
    execution = source.execution
    require(execution['pass'] is True and execution['frame_count'] == len(source.commands),
            'complete_actual_source_audit_required')
    sensitivity, merit, gradient = np.zeros((6, 5)), 0., np.zeros(5)
    meter.charge('source_sensitivity_retention')
    sensitivities, action_derivatives = [sensitivity.copy()], []
    diagnostic_sensitivity, diagnostic_actions = [sensitivity.tolist()], []
    for index, packet in enumerate(execution['frames']):
        meter.charge('source_frame_contract_checks')
        before, after = source.rows6[index:index + 2]
        raw_packets = packet.get('steering', ())
        require(packet['pass_frame'] is True and packet['command_calls'] == packet['steering_calls'] == 1
                and packet.get('original_process_identity') == id(source.original_process)
                and packet.get('actual_path_identity') == id(source.path)
                and packet['command_mps'] == source.commands[index]
                and packet['final_steering_rad'] == source.final_delta[index]
                and packet['actual_acceleration_mps2'] == after[5]
                and packet['previous_acceleration_mps2'] == before[5]
                and len(raw_packets) == 1 and raw_packets[0]['raw_steering_rad'] == source.raw_delta[index],
                'source_actual_tracking_frame_not_bound')
        meter.charge('source_old_speed_P2_rot_algebra_checks')
        meter.charge('source_sensitivity_frame_updates')
        command, raw, dc, dr = _action(source, index, zero)
        model, delta, acceleration, sensitivity = _realize(before, command, raw, dc, dr,
            sensitivity, source.vehicle_length, rotation)
        meter.charge('source_sensitivity_retention')
        sensitivities.append(sensitivity.copy())
        action_derivatives.append(_nominal_action_derivatives(source, index))
        diagnostic_sensitivity.append(sensitivity.tolist())
        diagnostic_actions.append(action_derivatives[-1].tolist())
        require(np.max(np.abs(model - after)) <= 1e-8 and abs(delta - source.final_delta[index]) <= 1e-10
                and abs(acceleration - after[5]) <= 1e-10, 'source_actual_P2_rot_six_state_mismatch')
        meter.charge('source_closed_ALAT_checks')
        bank._alat_closed(_state(source.rows6[index - 1]) if index else None,
                          _state(before), _state(after))
        evidence = _pose(backend, after, meter)
        require(evidence.in_goal is (index == len(source.commands) - 1),
                'source_first_goal_not_terminal')
        value, derivative = _goal_cost(after, sensitivity, goal)
        merit += value / scale
        gradient += derivative / scale
        require(meter.scalar_steps <= SOURCE_SCALAR_LIMIT, 'source_admission_stage_cap')
    meter.charge('source_terminal_ALAT_checks')
    bank._terminal_alat(_state(source.rows6[-2]), _state(source.rows6[-1]))
    require(meter.scalar_steps <= SOURCE_SCALAR_LIMIT, 'source_admission_stage_cap')
    require(abs(_score(source, source.rows6[-1]) - source.result['score']) <= 1e-8,
            'zero_violation_source_score_formula_mismatch')
    _finite(merit, gradient, scale)
    return _Chain(source.rows6, source.commands, source.raw_delta, source.final_delta,
                  merit, gradient, float(source.result['score']), zero,
                  np.asarray(sensitivities), np.asarray(action_derivatives),
                  dict(state_order=['t', 'x', 'y', 'yaw', 'speed', 'last_acc'],
                       parameter_order=['raw_relative_shape', 'raw_even_bias', 'raw_odd_bias',
                                        'command_lift', 'monotonic_phase_advance'],
                       action_derivative_order=['command_before_clip', 'raw_before_clip'],
                       six_state_sensitivities=diagnostic_sensitivity,
                       nominal_action_derivatives=diagnostic_actions,
                       native_export_counted_in_source_retention=True)), threshold, signature, goal, scale


def _trial(source, parameters, backend, meter, goal, scale, report):
    rows, commands, raws, deltas = [source.rows6[0].copy()], [], [], []
    sensitivity, gradient = np.zeros((6, 5)), np.zeros(5)
    merit = REGULARIZATION * float(parameters @ parameters)
    gradient += 2. * REGULARIZATION * parameters
    for index in range(len(source.commands)):
        report['attempted_frames'] = index + 1
        report['stage'] = 'command_proposal'
        meter.charge('model_command_proposals')
        command, raw, dc, dr = _action(source, index, parameters)
        meter.charge('model_P2_rot_realizations')
        meter.charge('model_plant_steps')
        meter.charge('model_sensitivity_frame_updates')
        after, delta, acceleration, sensitivity = _realize(rows[-1], command, raw, dc, dr,
            sensitivity, source.vehicle_length, source.rot_config)
        if index == 0:
            require(np.max(np.abs(after - source.rows6[1])) <= 1e-8
                    and command == source.commands[0] and raw == source.raw_delta[0]
                    and abs(delta - source.final_delta[0]) <= 1e-10,
                    'first_actual_action_six_state_not_preserved')
        rows.append(after)
        commands.append(command); raws.append(raw); deltas.append(delta)
        report['model_advanced_frames'] = index + 1
        report['stage'] = 'frame_diagnostic'
        meter.charge('model_frame_diagnostic_records')
        report['last_model_frame'] = dict(index=index, before6=rows[-2].tolist(),
            after6=after.tolist(), command=float(command), raw=float(raw),
            final=float(delta), actual_acc=float(acceleration))
        report['stage'] = 'closed_ALAT'
        meter.charge('model_closed_ALAT_checks')
        bank._alat_closed(_state(rows[-3]) if len(rows) >= 3 else None,
                          _state(rows[-2]), _state(after))
        report['stage'] = 'whole_pose'
        evidence = _pose(backend, after, meter)
        value, derivative = _goal_cost(after, sensitivity, goal)
        merit += value / scale
        gradient += derivative / scale
        if evidence.in_goal:
            report['stage'] = 'terminal_ALAT'
            meter.charge('model_terminal_ALAT_checks')
            bank._terminal_alat(_state(rows[-2]), _state(after))
            _finite(merit, gradient)
            return _Chain(np.asarray(rows), np.asarray(commands), np.asarray(raws), np.asarray(deltas),
                          merit, gradient, _score(source, after), parameters.copy())
    raise FullTraceRefusal('model_full_horizon_without_first_goal')


def _final_bound(backend, count):
    bound = backend.final_charge_bound(count)
    require(type(bound) is int and bound >= 0,
            'finite_exact_final_charge_bound_required')
    return bound


class _QPBudget:
    def __init__(self, meter, stop_scalar):
        self.meter, self.stop_scalar = meter, stop_scalar

    def charge(self, kind, count=1):
        if self.meter.scalar_steps + count > self.stop_scalar:
            raise bank.SearchStop('qp_full_trial_and_final_scalar_reserve')
        self.meter.charge(kind, count)


class _Constraints:
    def __init__(self, budget):
        self.budget, self.rows, self.bounds, self.labels = budget, [], [], []
        self.indices = {}

    def add(self, row, bound, label):
        self.budget.charge('qp_constraint_rows_formed')
        row = np.asarray(row, dtype=float)
        _finite(row, bound)
        require(row.shape == (5,), 'five_dimensional_halfspace_required')
        norm = float(np.linalg.norm(row))
        if norm == 0.:
            require(bound >= 0., 'qp_constant_halfspace_infeasible_' + label)
            return
        row, bound = row / norm, float(bound) / norm
        key = row.tobytes()
        if key in self.indices:
            index = self.indices[key]
            if bound < self.bounds[index]:
                self.bounds[index], self.labels[index] = bound, label
        else:
            self.indices[key] = len(self.rows)
            self.rows.append(row); self.bounds.append(bound); self.labels.append(label)

    def arrays(self):
        return np.asarray(self.rows), np.asarray(self.bounds)


def _earlier_target(source, threshold, goal, budget):
    budget.charge('qp_fixed_earlier_target')
    require(80. < threshold < 100., 'bounded_original_safe_prior_threshold_required')
    duration_limit = 20. * float(source.ctx._exp_t) / (threshold - 80.)
    budget.charge('qp_earlier_target_rows_scanned', len(source.rows6))
    times = source.rows6[:, 0] - source.rows6[0, 0]
    indices = np.flatnonzero((times > 0.) & (times < duration_limit))
    indices = indices[indices < len(source.commands)]
    require(len(indices) > 0, 'no_earlier_discrete_goal_target')
    index = int(indices[-1])
    lower, upper = goal[[0, 2]], goal[[1, 3]]
    inset = np.minimum(QP_TARGET_INSET_M, .1 * (upper - lower))
    target = np.clip(source.rows6[index, 1:3], lower + inset, upper - inset)
    return index, target, duration_limit


def _alat_linearization(rows, sensitivities, index, *, terminal=False, recorded=False,
                        unwrapped=None):
    """Signed raw v*gradient(unwrap(yaw),t) and its fixed-stencil Jacobian."""
    count = len(rows) - 1
    indices = ([max(0, index - 1), index] if terminal or index == count else
               ([0, 1] if index == 0 else [index - 1, index, index + 1]))
    require(indices[-1] > indices[0], 'nonempty_ALAT_stencil_required')
    yaw = np.unwrap(rows[:, 3]) if unwrapped is None else unwrapped
    times = [round(float(rows[i, 0]), 1) if recorded else float(rows[i, 0]) for i in indices]
    steps = np.diff(times)
    require(np.isfinite(steps).all() and np.all(steps > 0.), 'positive_ALAT_stencil_time_required')
    if len(indices) == 2:
        weights = np.array([-1., 1.]) / steps[0]
    else:
        left, right = steps
        weights = np.array([-right / (left * (left + right)),
            (right - left) / (left * right), left / (right * (left + right))])
    omega = float(weights @ yaw[indices])
    domega = weights @ sensitivities[indices, 3]
    speed = float(rows[index, 4])
    gradient = omega * sensitivities[index, 4] + speed * domega
    _finite(omega, domega, gradient)
    return speed * omega, gradient, speed, omega, domega, float(np.sum(np.abs(weights)))


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
    for axis in range(5):
        unit = np.eye(5)[axis]
        constraints.add(unit, min(radius, UPPER[axis]), 'box_upper_%d' % axis)
        constraints.add(-unit, min(radius, -LOWER[axis]), 'box_lower_%d' % axis)
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
    for index, before in enumerate(rows[:-1]):
        budget.charge('qp_source_branch_frames')
        dc, dr = chain.action_derivatives[index]
        speed, previous = float(before[4]), float(before[5])
        dv, dp = sensitivity[index, 4], sensitivity[index, 5]
        command, raw = float(source.commands[index]), float(source.raw_delta[index])
        if command == 12.:
            constraints.add(-dc, 0., 'command_capped_outward')
            dc = np.zeros(5)
        elif command == .1:
            constraints.add(dc, 0., 'command_lower_capped_outward')
            dc = np.zeros(5)
        else:
            constraints.add(dc, 12. - command, 'command_upper_free')
            constraints.add(-dc, command - .1, 'command_lower_free')
        constraints.add(dr, .6 - raw, 'raw_upper_free')
        constraints.add(-dr, .6 + raw, 'raw_lower_free')
        requested, derivative = 2. * (command - speed), 2. * (dc - dv)
        if requested > 2.5:
            constraints.add(-derivative, requested - 2.5, 'P2_upper_clamped')
            acceleration, da = 2.5, np.zeros(5)
        elif requested < -6.:
            constraints.add(derivative, -6. - requested, 'P2_lower_clamped')
            acceleration, da = -6., np.zeros(5)
        else:
            constraints.add(derivative, 2.5 - requested, 'P2_upper_free')
            constraints.add(-derivative, requested + 6., 'P2_lower_free')
            acceleration, da = requested, derivative
        difference = acceleration - previous
        if difference > 1.:
            constraints.add(dp - da, difference - 1., 'jerk_upper_clamped')
            da = dp
        elif difference < -1.:
            constraints.add(da - dp, -1. - difference, 'jerk_lower_clamped')
            da = dp
        else:
            constraints.add(da - dp, 1. - difference, 'jerk_upper_free')
            constraints.add(dp - da, 1. + difference, 'jerk_lower_free')
        next_speed = speed + .1 * float(rows[index + 1, 5])
        dnext = dv + .1 * da
        constraints.add(-dnext if next_speed > 0. else dnext,
                        next_speed if next_speed > 0. else -next_speed, 'speed_floor_branch')
        rotation = source.rot_config
        if rotation not in (None, 'off', {}):
            constraints.add(-dv if speed > .1 else dv,
                            speed - .1 if speed > .1 else .1 - speed, 'rotation_speed_branch')
            if speed > .1:
                coefficient = float(rotation.get('a_rot', .38)) * source.vehicle_length / 1.7
                cap = math.atan(coefficient / speed ** 2)
                dcap = (-2. * coefficient / speed ** 3 / (1. + (coefficient / speed ** 2) ** 2)) * dv
                if abs(raw) > cap:
                    sign = math.copysign(1., raw)
                    constraints.add(dcap - sign * dr, abs(raw) - cap, 'rotation_clamped')
                else:
                    constraints.add(dr - dcap, cap - raw, 'rotation_upper_free')
                    constraints.add(-dr - dcap, cap + raw, 'rotation_lower_free')
        count = len(source.commands)
        u = index / (count - 1.) if count > 1 else 0.
        phase = .5 * u * (1. - u) * (count - 1)
        row = np.array([0., 0., 0., 0., phase])
        constraints.add(row, 1., 'phase_right_cell_upper')
        constraints.add(-row, 0., 'phase_right_cell_lower')
        dyaw = sensitivity[index + 1, 3] - sensitivity[index, 3]
        difference = float(yaw[index + 1] - yaw[index])
        constraints.add(dyaw, math.pi - 2. * QUANTIZATION_EPSILON - difference, 'yaw_unwrap_upper_branch')
        constraints.add(-dyaw, math.pi - 2. * QUANTIZATION_EPSILON + difference, 'yaw_unwrap_lower_branch')
    matrix, bounds = constraints.arrays()
    _finite(matrix, bounds)
    audit.update(qp_constraint_count=len(bounds), qp_trust_radius=radius,
        qp_trust_restricted_by_quantization=radius < QP_TRUST_RADIUS,
        qp_source_quantization_max_upper=max_quantized_upper,
        qp_quantization_bounds_are_local_affine_only=True,
        qp_source_rows_zero_step_feasible=True,
        qp_prospective_terminal_zero_step_required=False,
        qp_phase_right_cell_only=True, qp_constraint_labels=list(constraints.labels),
        qp_bounded_array_copies_in_per_row_formation_charge=True)
    return matrix, bounds


def _solve_qp(quadratic, gradient, matrix, bounds, budget, audit):
    """Bounded dual active-set solve; dimension stays five even with negative b.

    Start at the unconstrained minimizer, add one most violated halfspace,
    and remove a face when its nonnegative dual reaches zero. A prospective
    earlier terminal need not admit the identity zero step. No face subsets
    or alternative branches are enumerated.
    """
    budget.charge('qp_invocations')
    audit['qp_solve_count'] = 1
    budget.charge('qp_matrix_factorizations')
    inverse = np.linalg.solve(quadratic, np.eye(5))
    point = -inverse @ gradient
    working, multipliers = [], np.empty(0)
    selected, selected_multiplier = None, 0.
    audit['qp_iterations'] = 0
    audit['qp_active_faces_peak'] = 0
    def remember():
        labels = audit.get('qp_constraint_labels', [])
        audit.update(qp_active_face_indices=list(working),
            qp_active_constraint_labels=[labels[i] if labels else str(i) for i in working],
            qp_dual_multipliers=multipliers.tolist(),
            qp_pending_constraint_index=selected,
            qp_pending_dual_multiplier=float(selected_multiplier))

    def finish(violation):
        budget.charge('qp_KKT_verification', len(working) + 1)
        dual_gradient = matrix[working].T @ multipliers if working else np.zeros(5)
        stationarity = quadratic @ point + gradient + dual_gradient
        scale = 1. + np.linalg.norm(quadratic @ point, np.inf) + np.linalg.norm(gradient, np.inf) + np.linalg.norm(dual_gradient, np.inf)
        active_error = float(np.max(np.abs(matrix[working] @ point - bounds[working]))) if working else 0.
        dual_minimum = float(np.min(multipliers)) if working else 0.
        residual = float(np.linalg.norm(stationarity, np.inf))
        _finite(violation, residual, scale, active_error, dual_minimum)
        audit.update(qp_KKT_stationarity_inf=residual, qp_KKT_stationarity_scale=float(scale),
            qp_KKT_active_residual_inf=active_error, qp_KKT_dual_minimum=dual_minimum)
        require(dual_minimum >= -QP_NUMERIC_TOLERANCE
            and active_error <= 10. * QP_NUMERIC_TOLERANCE
            and residual <= 10. * QP_NUMERIC_TOLERANCE * scale,
            'qp_local_KKT_numeric_verification_failed')
        remember()
        audit.update(qp_status='local_convex_solution', qp_direction=point.tolist(),
            qp_max_halfspace_violation=float(np.max(violation)),
            qp_objective_change=float(gradient @ point + .5 * point @ quadratic @ point))
        return point

    for iteration in range(QP_ITERATION_LIMIT):
        if selected is None:
            budget.charge('qp_constraint_rows_scanned', len(bounds))
            violation = matrix @ point - bounds
            _finite(point, violation)
            if float(np.max(violation)) <= QP_NUMERIC_TOLERANCE:
                return finish(violation)
            selected = int(np.argmax(violation))
            selected_multiplier = 0.
        budget.charge('qp_iterations')
        audit['qp_iterations'] = iteration + 1
        remember()
        row = matrix[selected]
        inverse_row = inverse @ row
        if working:
            active = matrix[working]
            gram = active @ inverse @ active.T
            budget.charge('qp_matrix_factorizations')
            dual = np.linalg.solve(gram, active @ inverse_row)
            projected = inverse_row - inverse @ active.T @ dual
        else:
            dual, projected = np.empty(0), inverse_row
        denominator = float(row @ projected)
        primal_step = (float(row @ point - bounds[selected]) / denominator
                       if denominator > QP_NUMERIC_TOLERANCE else math.inf)
        positive = np.flatnonzero(dual > QP_NUMERIC_TOLERANCE)
        dual_step, leaving = math.inf, None
        if len(positive):
            values = multipliers[positive] / dual[positive]
            position = int(np.argmin(values))
            dual_step, leaving = float(values[position]), int(positive[position])
        step = min(primal_step, dual_step)
        require(math.isfinite(step) and step >= -QP_NUMERIC_TOLERANCE,
                'qp_restricted_halfspaces_inconsistent_or_numeric')
        step = max(0., step)  # Roundoff in a dual pivot, never a halfspace slack.
        point = point - step * projected
        multipliers = multipliers - step * dual
        selected_multiplier += step
        _finite(point, multipliers, selected_multiplier)
        if primal_step <= dual_step:
            require(len(working) < QP_ACTIVE_FACE_LIMIT, 'qp_independent_active_face_cap')
            working.append(selected)
            multipliers = np.append(multipliers, selected_multiplier)
            selected, selected_multiplier = None, 0.
        else:
            require(leaving is not None, 'qp_dual_leaving_face_required')
            working.pop(leaving)
            multipliers = np.delete(multipliers, leaving)
        audit['qp_active_faces_peak'] = max(audit['qp_active_faces_peak'], len(working))
        remember()
    if selected is None:
        budget.charge('qp_constraint_rows_scanned', len(bounds))
        violation = matrix @ point - bounds
        if float(np.max(violation)) <= QP_NUMERIC_TOLERANCE:
            return finish(violation)
    raise FullTraceRefusal('qp_iteration_limit_without_certified_local_solution')


def build_candidate(source, identity_receipt, priors, backend, *, clock=time.perf_counter):
    """One local QP, at most three fixed-scale whole static trials, no rollout."""
    meter = bank.Meter(clock)
    candidate = None
    audit = dict(mechanism='identity_seeded_single_5D_QP', accepted=False, candidate_count=0,
        qp_direction_dimension=QP_DIRECTION_DIMENSION, qp_solve_limit=QP_SOLVE_LIMIT,
        qp_iteration_limit=QP_ITERATION_LIMIT, qp_active_face_limit=QP_ACTIVE_FACE_LIMIT,
        qp_trial_multipliers=list(QP_TRIAL_MULTIPLIERS), qp_solve_count=0, qp_iterations=0,
        maximum_model_trials=3, max_scalar_steps=30000, max_chain_steps=430,
        static_budget_s=8., final_reserve_s=1.5, new_identity_complete_rollout_required=1,
        maximum_family_new_complete_rollouts=2, complete_policy_predictions=0, Env_runs=0,
        original_process_executed_by_helper=False, graph_searches=0, parameter_grid=False,
        branch_grid=False, trials=[], external_csv_used=False, physical_infeasibility_claimed=False)
    try:
        original, threshold, signature, goal, scale = _admit(source, identity_receipt, priors, backend, meter)
        best = original
        audit.update(identity_receipt_verified=True, identity_complete_rollouts=1,
            highest_physically_safe_prior_plus_hyst=threshold, source_admission_scalar=meter.scalar_steps,
            source_frames=len(source.commands), source_static_score=best.static_score,
            retained_complete_identity_incumbent=True, source_sensitivity_rows=len(original.sensitivities),
            source_linearization_diagnostics=original.diagnostics)
        stage_start = meter.scalar_steps
        meter.search = True
        maximum_final = _final_bound(backend, len(source.commands))
        reserve = max(MIN_FINAL_SCALARS, maximum_final + len(source.commands) + 4)
        worst_trial = 15 * len(source.commands) + 1
        qp_stop = min(30000 - reserve - worst_trial, stage_start + TRIAL_SCALAR_LIMIT - worst_trial)
        budget = _QPBudget(meter, qp_stop)
        audit.update(qp_stop_scalar=qp_stop, qp_reserved_full_trial_scalar=worst_trial,
                     qp_reserved_final_scalar=reserve, optimization_stage_scalar_start=stage_start)
        earlier, target, duration_limit = _earlier_target(source, threshold, goal, budget)
        budget.charge('qp_objective_build')
        jacobian = original.sensitivities[earlier, 1:3]
        residual = original.rows6[earlier, 1:3] - target
        quadratic = jacobian.T @ jacobian + REGULARIZATION * np.eye(5)
        gradient = jacobian.T @ residual
        audit.update(qp_fixed_earlier_goal_index=earlier, qp_earlier_duration_limit_s=duration_limit,
            qp_target_xy=target.tolist(), qp_target_bbox_inset_m=QP_TARGET_INSET_M,
            qp_fixed_target=True, qp_target_is_actual_goal_certificate=False,
            qp_goal_jacobian_xy=jacobian.tolist(), qp_goal_residual_xy=residual.tolist(),
            qp_target_source_distance_m=float(np.linalg.norm(residual)))
        matrix, bounds = _qp_constraints(source, original, earlier, budget, audit)
        direction = _solve_qp(quadratic, gradient, matrix, bounds, budget, audit)
        require(float(np.linalg.norm(direction)) > 1e-12 and audit['qp_objective_change'] < 0.,
                'qp_no_nonzero_local_merit_improvement')
        for index, multiplier in enumerate(QP_TRIAL_MULTIPLIERS):
            try:
                meter.check('before_full_trace_trial')
                maximum_final = _final_bound(backend, len(source.commands))
                reserve = max(MIN_FINAL_SCALARS, maximum_final + len(source.commands) + 4)
                if (meter.scalar_steps + worst_trial + reserve > 30000
                        or meter.scalar_steps - stage_start + worst_trial > TRIAL_SCALAR_LIMIT):
                    audit['optimizer_stop'] = 'full_trace_trial_scalar_reserve_reached'
                    break
                parameters = multiplier * direction
                report = dict(index=index, multiplier=multiplier, parameters=parameters.tolist(),
                    attempted_frames=0, model_advanced_frames=0, complete=False, feasible=False,
                    became_best=False, scalar_before=meter.scalar_steps)
                audit['trials'].append(report)
                try:
                    trial = _trial(source, parameters, backend, meter, goal, scale, report)
                    report.update(complete=True, feasible=True,
                        actual_model_first_goal_frame=len(trial.commands),
                        model_arrival_s=float(trial.rows6[-1, 0]), static_score=trial.static_score,
                        merit=trial.merit)
                    if trial.static_score > best.static_score:
                        best, report['became_best'] = trial, True
                except (FullTraceRefusal, bank.NativeBankPhaseRefusal) as error:
                    report['reason'] = str(error)
                except bank.SearchStop as error:
                    report['reason'] = audit['optimizer_stop'] = str(error)
                    break
                finally:
                    report['scalar_after'] = meter.scalar_steps
            except bank.SearchStop as error:
                audit['optimizer_stop'] = str(error)
                break
        meter.search = False
        meter.check('before_final_full_trace')
        require(best.static_score > threshold, 'no_complete_QP_trial_above_original_prior_hyst')
        reserve = _final_bound(backend, len(best.commands)) + len(best.commands) + 4
        require(meter.scalar_steps + reserve <= 30000, 'final_full_trace_scalar_reserve_missing')
        gate = backend.final_gate(best.rows6,
            np.column_stack((best.rows6[:-1, 0], best.commands, best.raw_delta)), best.final_delta, meter)
        require(isinstance(gate, dict) and all(gate.get(key) is True for key in (
            'complete_first_goal_chain', 'actual_bitmap_polygon_all_rows', 'absolute_GT_all_rows',
            'raw_and_recorder_ALAT_all_rows', 'TTC_and_lateral_all_rows', 'static_only'))
            and gate.get('original_process_actions_verified', False) is False
            and gate.get('original_bank_process_actions_verified', False) is False,
            'complete_original_static_gate_required_without_process_claim')
        meter.charge('frozen_candidate_frame_checks', len(best.commands))
        candidate = FrozenFullTraceTableCand(source, best.rows6, best.commands,
            best.raw_delta, best.final_delta, role='optimized')
        meter.charge('final_source_signature_reads')
        require(source.signature() == signature, 'source_mutated_by_QP')
        audit.update(candidate_count=1, model_frames=len(best.commands),
            model_arrival_s=float(best.rows6[-1, 0]), static_score=best.static_score,
            static_gain=best.static_score - float(source.result['score']),
            model_time_is_actual_Env_arrival=False, final_gate=gate,
            parameters=best.parameters.tolist(), retained_complete_identity_incumbent=True)
        meter.check('last_publication')
        audit['accepted'] = True
    except (FullTraceRefusal, bank.NativeBankPhaseRefusal, bank.SearchStop) as error:
        candidate = None
        audit.update(accepted=False, candidate_count=0, reason=str(error))
    except Exception as error:
        candidate = None
        audit.update(accepted=False, candidate_count=0, reason='qp_backend_or_numeric_exception',
                     error=f'{type(error).__name__}: {error}')
    audit.update(scalar_steps=meter.scalar_steps, counters=dict(meter.counts))
    audit.setdefault('reason', 'accepted' if candidate is not None else audit.get('optimizer_stop', 'not_accepted'))
    elapsed = clock() - meter.started
    audit['elapsed_s'] = elapsed if math.isfinite(elapsed) and elapsed >= 0. else None
    if not math.isfinite(elapsed) or not 0. <= elapsed < 8.:
        candidate = None
        audit.update(accepted=False, candidate_count=0, reason='total_soft_budget_final_metadata')
    return candidate, audit
