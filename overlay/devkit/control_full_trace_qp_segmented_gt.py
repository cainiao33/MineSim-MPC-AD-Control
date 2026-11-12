"""F38：11维局部分段转向/命令及非对称phase；原真实安全门保持。"""

import math
import time
from dataclasses import dataclass, replace

import numpy as np

import control_native_bank_phase as bank
from control_gt_work_meter import GTWorkMeter, MAX_SCALAR_STEPS
from control_qp_segmented_primal import solve_identity_start_qp
from control_qp_segmented_box import certify_direction_inside_exact_box
from control_qp_segmented_dominance import (prune_exact_redundancies as prune_box_implied_halfspaces,
                                      certify_original_removed_halfspaces)
from control_full_trace_qp_contract import (
    FullTraceSource, IdentityReceipt,
    FullTraceRefusal, FullTraceState, require, frozen_array,
)

from control_full_trace_native_domain_contract import FrozenNativeDomainTableCand as FrozenFullTraceTableCand

MAX_TRIALS = 3
MAX_STEPS = 430
SOURCE_SCALAR_LIMIT = 15000
TRIAL_SCALAR_LIMIT = 240000
MIN_FINAL_SCALARS = 5000
SCALES = np.array([2.] + [.12] * 4 + [4.] * 4 + [2.] * 2)
LOCAL_CENTERS = np.array([.15, .35, .65, .85])
LOCAL_HALF_WIDTH = .25
LOCAL_RAW_SCALE = .12
PHASE_SLOT = 10
GLOBAL_COMMAND_SLOT = 9
PARAMETER_ORDER = (['raw_relative_shape'] +
    ['local_raw_%d' % i for i in range(4)] +
    ['local_signed_command_%d' % i for i in range(4)] +
    ['global_nonnegative_command_lift', 'asymmetric_phase_advance'])
RELATIVE_RAW_STEERING_SCALE = 2.
NATIVE_STEERING_LIMIT = 1.
PHASE_ADVANCE_SCALE = 2.
TURN_CURVATURE_CUTOFF = .005
TURN_COMMAND_SCALE = 4.
LOWER = np.array([-1.] * 9 + [0., 0.])
UPPER = np.array([1.] * 10 + [.25])
REGULARIZATION = 1e-6
QP_DIRECTION_DIMENSION = 11
QP_SOLVE_LIMIT = 1
QP_ITERATION_LIMIT = 96
QP_ACTIVE_FACE_LIMIT = 11
QP_TRIAL_MULTIPLIERS = (1., .5, .25)
QP_TRUST_RADIUS = .25
QP_NUMERIC_TOLERANCE = 1e-9
QP_TARGET_INSET_M = 1e-6
QUANTIZATION_EPSILON = 5e-5 + 1e-12
EXCLUDED_NOMINAL_BRANCH_PREFIXES = ('P2_', 'jerk_', 'command_', 'raw_upper_free', 'raw_lower_free', 'speed_floor_branch', 'rotation_', 'phase_right_cell_', 'yaw_unwrap_')


@dataclass(frozen=True)
class FullTraceBackend:
    selected: object
    pose: object
    final_gate: object
    final_charge_bound: object
    original_process: object
    source_body_margins: object
    source_traffic_planes: object


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
    turn_basis: object = None
    body_margins: object = None
    traffic_planes: object = ()


def _finite(*values):
    require(all(np.isfinite(value).all() for value in values),
            'finite_continuation_numeric_required')


def _state(row):
    return FullTraceState(*map(float, row))


def _clip(value, derivative, lower, upper):
    # Strictly saturated branches have zero derivative. Equality uses the
    # inward one-sided derivative only as a proposal, never a gate certificate.
    if value < lower:
        return lower, np.zeros(QP_DIRECTION_DIMENSION)
    if value > upper:
        return upper, np.zeros(QP_DIRECTION_DIMENSION)
    return value, derivative


def _sample(array, phase):
    require(0. <= phase <= len(array) - 1., 'source_phase_outside_full_table')
    if len(array) == 1:
        return float(array[0]), 0.
    left = min(int(math.floor(phase)), len(array) - 2)
    slope = float(array[left + 1] - array[left])
    return float(array[left]) + (phase - left) * slope, slope


def _turn_basis(source, meter):
    meter.charge('source_turn_basis_rows', len(source.commands))
    curvature = np.abs(1.7 * np.tan(source.final_delta) / source.vehicle_length)
    peak = float(np.max(curvature))
    if peak <= TURN_CURVATURE_CUTOFF:
        values = np.zeros(len(curvature))
    else:
        normalized = np.clip((curvature - TURN_CURVATURE_CUTOFF) /
                             (peak - TURN_CURVATURE_CUTOFF), 0., 1.)
        values = normalized * normalized * (3. - 2. * normalized)
    return frozen_array(values, (len(source.commands),))


def _local_weights(phase, count):
    """Continuous local hats in reference phase; values and d/d(phase)."""
    if count <= 1:
        return np.zeros(4), np.zeros(4)
    u = phase / (count - 1.)
    envelope = 4. * u * (1. - u)
    envelope_slope = 4. * (1. - 2. * u)
    distance = u - LOCAL_CENTERS
    hats = np.maximum(0., 1. - np.abs(distance) / LOCAL_HALF_WIDTH)
    # At a hat knot this is a proposal derivative only. Full nonlinear gates
    # remain mandatory, and finite-difference tests use points off all knots.
    slopes = np.where(hats > 0., -np.sign(distance) / LOCAL_HALF_WIDTH, 0.)
    return envelope * hats, (envelope_slope * hats + envelope * slopes) / (count - 1.)


def _nominal_action_derivatives(source, index, turn_basis):
    # Preserve pre-clip derivatives for the proposal diagnostics only.
    return np.stack(_action_unclipped(source, index, np.zeros(QP_DIRECTION_DIMENSION), turn_basis)[2:])


def _action_unclipped(source, index, parameters, turn_basis):
    count = len(source.commands)
    u = index / (count - 1.) if count > 1 else 0.
    weight = 4. * u * (1. - u)
    phase_derivative = PHASE_ADVANCE_SCALE * u * u * (1. - u) * (count - 1)
    phase = index + parameters[PHASE_SLOT] * phase_derivative
    command, command_slope = _sample(source.commands, phase)
    raw, raw_slope = _sample(source.raw_delta, phase)
    local, local_slope = _local_weights(phase, count)
    dc, dr = np.zeros(QP_DIRECTION_DIMENSION), np.zeros(QP_DIRECTION_DIMENSION)
    dr[0] = RELATIVE_RAW_STEERING_SCALE * weight * raw
    dr[1:5] = LOCAL_RAW_SCALE * local
    dr[PHASE_SLOT] = (raw_slope * (1. + RELATIVE_RAW_STEERING_SCALE * parameters[0] * weight)
        + LOCAL_RAW_SCALE * float(parameters[1:5] @ local_slope)) * phase_derivative
    dc[5:9] = TURN_COMMAND_SCALE * local
    dc[GLOBAL_COMMAND_SLOT] = 2. * weight
    dc[PHASE_SLOT] = (command_slope + TURN_COMMAND_SCALE * float(parameters[5:9] @ local_slope)) * phase_derivative
    raw = raw * (1. + RELATIVE_RAW_STEERING_SCALE * parameters[0] * weight) + LOCAL_RAW_SCALE * float(parameters[1:5] @ local)
    command += TURN_COMMAND_SCALE * float(parameters[5:9] @ local) + 2. * parameters[GLOBAL_COMMAND_SLOT] * weight
    if index in (0, count - 1) or not np.any(parameters):
        command, raw = float(source.commands[index]), float(source.raw_delta[index])
    _finite(command, raw, dc, dr)
    return command, raw, dc, dr


def _action(source, index, parameters, turn_basis):
    command, raw, dc, dr = _action_unclipped(source, index, parameters, turn_basis)
    command, dc = _clip(command, dc, .1, 12.)
    if command == .1 or command == 12.:
        dc = np.zeros(QP_DIRECTION_DIMENSION)
    raw, dr = _clip(raw, dr, -NATIVE_STEERING_LIMIT, NATIVE_STEERING_LIMIT)
    _finite(command, raw, dc, dr)
    return command, raw, dc, dr


def _realize(row, command, raw, dc, dr, sensitivity, length, rotation):
    """Exact P2/jerk/original old-speed rot and plant plus 6x11 sensitivity."""
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
    derivative[4] = dspeed + .1 * da if after[4] > 0. else np.zeros(QP_DIRECTION_DIMENSION)
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
    zero = np.zeros(QP_DIRECTION_DIMENSION)
    scale = max(_goal_cost(source.rows6[0], np.zeros((6, QP_DIRECTION_DIMENSION)), goal)[0], 1.) * len(source.commands)
    first = _pose(backend, source.rows6[0], meter)
    require(first.in_goal is False, 'source_initial_already_in_goal')
    execution = source.execution
    require(execution['pass'] is True and execution['frame_count'] == len(source.commands),
            'complete_actual_source_audit_required')
    turn_basis = _turn_basis(source, meter)
    sensitivity, merit, gradient = np.zeros((6, QP_DIRECTION_DIMENSION)), 0., np.zeros(QP_DIRECTION_DIMENSION)
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
        command, raw, dc, dr = _action(source, index, zero, turn_basis)
        model, delta, acceleration, sensitivity = _realize(before, command, raw, dc, dr,
            sensitivity, source.vehicle_length, rotation)
        meter.charge('source_sensitivity_retention')
        sensitivities.append(sensitivity.copy())
        action_derivatives.append(_nominal_action_derivatives(source, index, turn_basis))
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
                       parameter_order=list(PARAMETER_ORDER),
                       action_derivative_order=['command_before_clip', 'raw_before_clip'],
                       six_state_sensitivities=diagnostic_sensitivity,
                       nominal_action_derivatives=diagnostic_actions,
                       native_export_counted_in_source_retention=True,
                       turn_basis=turn_basis.tolist(), turn_curvature_cutoff_per_m=TURN_CURVATURE_CUTOFF,
                       turn_command_scale_mps=TURN_COMMAND_SCALE), turn_basis), threshold, signature, goal, scale


def _trial(source, parameters, backend, meter, goal, scale, report, turn_basis):
    rows, commands, raws, deltas = [source.rows6[0].copy()], [], [], []
    sensitivity, gradient = np.zeros((6, QP_DIRECTION_DIMENSION)), np.zeros(QP_DIRECTION_DIMENSION)
    merit = REGULARIZATION * float(parameters @ parameters)
    gradient += 2. * REGULARIZATION * parameters
    for index in range(len(source.commands)):
        report['attempted_frames'] = index + 1
        report['stage'] = 'command_proposal'
        meter.charge('model_command_proposals')
        command, raw, dc, dr = _action(source, index, parameters, turn_basis)
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
        if index == 0:
            report['partial_rows6'] = [rows[0].tolist()]
            report['partial_actions'] = []
        report['partial_rows6'].append(after.tolist())
        report['partial_actions'].append([float(rows[-2][0]), float(command), float(raw), float(delta), float(acceleration)])
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
        if self.meter.scalar_steps + self.meter.work_units(kind, count) > self.stop_scalar:
            raise bank.SearchStop('qp_full_trial_and_final_scalar_reserve')
        self.meter.charge(kind, count)


class _Constraints:
    def __init__(self, budget):
        self.budget, self.rows, self.bounds, self.labels = budget, [], [], []
        self.indices = {}
        self.excluded_nominal_faces = []

    def add(self, row, bound, label):
        self.budget.charge('qp_constraint_rows_formed')
        row = np.asarray(row, dtype=float)
        _finite(row, bound)
        require(row.shape == (QP_DIRECTION_DIMENSION,), 'eleven_dimensional_halfspace_required')
        if label.startswith(EXCLUDED_NOMINAL_BRANCH_PREFIXES):
            # These only preserve the original affine derivative cell.
            # Actual command/raw clips, P2/jerk/rotation and phase sampling
            # remain authoritative in each complete nonlinear trial/process.
            self.excluded_nominal_faces.append(dict(row=row.tolist(), bound=float(bound), label=label))
            return
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


def _body_exit_corridor_rows(row, sensitivity, body_radius):
    lateral = (-math.sin(float(row[3])) * sensitivity[1]
               + math.cos(float(row[3])) * sensitivity[2])
    rotation = body_radius * sensitivity[3]
    return [a * lateral + b * rotation for a in (-1., 1.) for b in (-1., 1.)]


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
        constraints.add(unit, min(radius, UPPER[axis]), 'box_upper_%d' % axis)
        constraints.add(-unit, min(radius, -LOWER[axis]), 'box_lower_%d' % axis)
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
        delta_index = min(index, len(source.final_delta) - 1)
        curvature = abs(1.7 * math.tan(float(source.final_delta[delta_index])) / source.vehicle_length)
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


def _solve_qp(quadratic, gradient, matrix, bounds, budget, audit):
    return solve_identity_start_qp(quadratic, gradient, matrix, bounds, budget, audit,
        iteration_limit=QP_ITERATION_LIMIT, face_limit=QP_ACTIVE_FACE_LIMIT,
        tolerance=QP_NUMERIC_TOLERANCE)


def build_candidate(source, identity_receipt, priors, backend, *, clock=time.perf_counter):
    """One local QP, at most three fixed-scale whole static trials, no rollout."""
    meter = GTWorkMeter(clock)
    candidate = None
    audit = dict(mechanism='identity_seeded_segmented_11D_command_steering_QP', accepted=False, candidate_count=0,
        qp_direction_dimension=QP_DIRECTION_DIMENSION, qp_solve_limit=QP_SOLVE_LIMIT,
        qp_iteration_limit=QP_ITERATION_LIMIT, qp_active_face_limit=QP_ACTIVE_FACE_LIMIT,
        qp_trial_multipliers=list(QP_TRIAL_MULTIPLIERS), qp_solve_count=0, qp_iterations=0,
        maximum_model_trials=3, max_scalar_steps=MAX_SCALAR_STEPS, max_chain_steps=430,
        static_budget_s=8., final_reserve_s=1.5, new_identity_complete_rollout_required=1,
        maximum_family_new_complete_rollouts=2, complete_policy_predictions=0, Env_runs=0,
        original_process_executed_by_helper=False, graph_searches=0, parameter_grid=False,
        branch_grid=False, trials=[], external_csv_used=False, physical_infeasibility_claimed=False,
        physical_parameter_family_changed_from_parent=False,
        native_steering_limit_rad=NATIVE_STEERING_LIMIT,
        relative_raw_steering_scale=RELATIVE_RAW_STEERING_SCALE,
        source_admission_steering_limit_rad=.6,
        original_action_contract_unchanged=True,
        phase_advance_slot=PHASE_SLOT, parameter_order=list(PARAMETER_ORDER),
        local_centers=LOCAL_CENTERS.tolist(), local_half_width=LOCAL_HALF_WIDTH,
        vector_work_units=3, matrix_work_units=5, maximum_KKT_size=22, phase_advance_scale=PHASE_ADVANCE_SCALE,
        phase_is_asymmetric=True, phase_coordinate_upper=.25,
        policy_source_phase_is_identity=False)
    try:
        original, threshold, signature, goal, scale = _admit(source, identity_receipt, priors, backend, meter)
        margins = backend.source_body_margins(meter)
        original = replace(original, body_margins=frozen_array(margins, (len(source.rows6),)))
        require(np.all(original.body_margins > 0.) and meter.scalar_steps <= SOURCE_SCALAR_LIMIT,
                'positive_live_body_corridor_within_source_budget_required')
        planes=backend.source_traffic_planes(meter)
        original=replace(original,traffic_planes=tuple(planes))
        require(meter.scalar_steps<=SOURCE_SCALAR_LIMIT,'live_GT_plane_source_stage_cap')
        best = original
        audit.update(identity_receipt_verified=True, identity_complete_rollouts=1,
            highest_physically_safe_prior_plus_hyst=threshold, source_admission_scalar=meter.scalar_steps,
            source_frames=len(source.commands), source_static_score=best.static_score,
            retained_complete_identity_incumbent=True, source_sensitivity_rows=len(original.sensitivities),
            source_linearization_diagnostics=original.diagnostics,
            source_body_margins=original.body_margins.tolist(),
            source_body_margin_queries_in_same_static_meter=True)
        stage_start = meter.scalar_steps
        meter.search = True
        maximum_final = _final_bound(backend, len(source.commands))
        reserve = max(MIN_FINAL_SCALARS, maximum_final + len(source.commands) + 4)
        worst_trial = 19 * len(source.commands) + 1
        qp_stop = min(MAX_SCALAR_STEPS - reserve - worst_trial, stage_start + TRIAL_SCALAR_LIMIT - worst_trial)
        budget = _QPBudget(meter, qp_stop)
        audit.update(qp_stop_scalar=qp_stop, qp_reserved_full_trial_scalar=worst_trial,
                     qp_reserved_final_scalar=reserve, optimization_stage_scalar_start=stage_start)
        earlier, target, duration_limit = _earlier_target(source, threshold, goal, budget)
        budget.charge('qp_objective_build')
        jacobian = original.sensitivities[earlier, 1:3]
        residual = original.rows6[earlier, 1:3] - target
        violated = (original.rows6[earlier, 1:3] < goal[[0, 2]]) | (original.rows6[earlier, 1:3] > goal[[1, 3]])
        require(np.any(violated), 'earlier_goal_frame_must_have_unsatisfied_axis')
        objective_jacobian = jacobian[violated]
        objective_residual = residual[violated]
        quadratic = objective_jacobian.T @ objective_jacobian + REGULARIZATION * np.eye(QP_DIRECTION_DIMENSION)
        gradient = objective_jacobian.T @ objective_residual
        audit.update(qp_goal_objective_violated_axes=list(map(bool, violated)),
            qp_goal_objective_is_distance_to_box=True,
            qp_goal_free_axes_actual_affine_bounds_required=True)
        audit.update(qp_quadratic=quadratic.tolist(), qp_gradient=gradient.tolist())
        audit.update(qp_fixed_earlier_goal_index=earlier, qp_earlier_duration_limit_s=duration_limit,
            qp_target_xy=target.tolist(), qp_target_bbox_inset_m=QP_TARGET_INSET_M,
            qp_fixed_target=False, qp_fixed_earlier_frame=True,
            qp_target_xy_is_active_face_anchor=True, qp_target_is_actual_goal_certificate=False,
            qp_goal_jacobian_xy=jacobian.tolist(), qp_goal_residual_xy=residual.tolist(),
            qp_target_source_distance_m=float(np.linalg.norm(residual)))
        matrix, bounds = _qp_constraints(source, original, earlier, budget, audit)
        matrix, bounds = prune_box_implied_halfspaces(
            matrix, bounds, audit['qp_constraint_labels'],
            np.maximum(LOWER, -audit['qp_trust_radius']),
            np.minimum(UPPER, audit['qp_trust_radius']), budget, audit)
        direction = _solve_qp(quadratic, gradient, matrix, bounds, budget, audit)
        certify_direction_inside_exact_box(direction, budget, audit)
        certify_original_removed_halfspaces(direction, budget, audit)
        require(float(np.linalg.norm(direction)) > 1e-12 and audit['qp_objective_change'] < 0.,
                'qp_no_nonzero_local_merit_improvement')
        for index, multiplier in enumerate(QP_TRIAL_MULTIPLIERS):
            try:
                meter.check('before_full_trace_trial')
                maximum_final = _final_bound(backend, len(source.commands))
                reserve = max(MIN_FINAL_SCALARS, maximum_final + len(source.commands) + 4)
                if (meter.scalar_steps + worst_trial + reserve > MAX_SCALAR_STEPS
                        or meter.scalar_steps - stage_start + worst_trial > TRIAL_SCALAR_LIMIT):
                    audit['optimizer_stop'] = 'full_trace_trial_scalar_reserve_reached'
                    break
                parameters = multiplier * direction
                report = dict(index=index, multiplier=multiplier, parameters=parameters.tolist(),
                    attempted_frames=0, model_advanced_frames=0, complete=False, feasible=False,
                    became_best=False, scalar_before=meter.scalar_steps)
                audit['trials'].append(report)
                try:
                    trial = _trial(source, parameters, backend, meter, goal, scale, report, original.turn_basis)
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
        require(meter.scalar_steps + reserve <= MAX_SCALAR_STEPS, 'final_full_trace_scalar_reserve_missing')
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
