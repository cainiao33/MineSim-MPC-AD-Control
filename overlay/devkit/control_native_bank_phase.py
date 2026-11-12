"""F17: native IDM/F0 source, bank steering, executable traffic-phase speed.

The probe supplies current online source evidence and callbacks around the original
MotionController.process. This module never reads CSV, imports the simulator, or
overrides steering. Search changes only the speed command at the existing SAS
hook. Every proposal calls the original bank control path before advancing the
old-speed plant. A complete prediction and same-object Env remain probe gates.

State units are absolute seconds, map metres, radians, m/s and m/s**2. Callback
queries must charge the supplied Meter before each actual map/GT operation.
"""

from dataclasses import dataclass, field
import copy
import heapq
import math
import time
import types


DT = .1
STATIC_BUDGET_S = 8.
FINAL_RESERVE_S = 1.5
MAX_EXPANDED = 1200
MAX_EDGES = 6000
MAX_SCALAR_STEPS = 30000
MAX_CHAIN_STEPS = 430
EDGE_STEPS = 5
V_MIN, V_MAX = .1, 12.
P_GAIN = 2.
HYST = .15
STEERING_LIMIT = .6


class NativeBankPhaseRefusal(ValueError):
    pass


class SearchStop(Exception):
    pass


def require(condition, reason):
    if not condition:
        raise NativeBankPhaseRefusal(reason)


@dataclass(frozen=True)
class BankState:
    t: float
    x: float
    y: float
    yaw: float
    speed: float
    last_acc: float

    def values(self):
        return (self.t, self.x, self.y, self.yaw, self.speed, self.last_acc)


@dataclass(frozen=True)
class ProcessTrace:
    """Probe-observed counters from the original methods, never inferred from yaw."""
    process_identity: object
    path_identity: object
    process_calls: int
    bank_profile_stage_calls: int
    pursuit_calls: int
    geometry_overrides: int
    preview_failed: bool
    branch: str
    final_steering: float


@dataclass(frozen=True)
class BankFrame:
    before: BankState
    after: BankState
    command: float
    acceleration: float
    steering: float
    trace: ProcessTrace


@dataclass(frozen=True)
class NativeBankSource:
    """Identity-bound capture; only the probe may create this from native rollout."""
    ctx: object
    policy: object
    result: dict
    controller: object
    path: object
    initial: BankState
    frames: tuple
    process_identity: object
    vehicle_length: float
    online_same_object: bool
    external_csv_used: bool
    complete_native_capture_verified: bool


@dataclass(frozen=True)
class PhaseEvent:
    """Original-GT occupancy witness; used for proposals, never a safety proof."""
    x: float
    y: float
    blocked_start: float
    blocked_end: float
    actor_token: str
    source_policy: object
    source_result: object
    original_GT: bool


@dataclass(frozen=True)
class FrameInputs:
    vehicle_info: dict
    path: object
    traj_future: object
    observation: dict


@dataclass(frozen=True)
class PoseEvidence:
    t: float
    runtime_road_safe: bool
    bitmap_polygon_safe: bool
    GT_collision_free: bool
    ttc_safe: bool
    lateral_distance_safe: bool
    in_goal: bool
    goal_bbox_distance: float


@dataclass(frozen=True)
class BankPhaseBackend:
    """Adapters must not substitute pursuit, plant, GT schema or recorder semantics.

    selected(ctx) -> (actual selected policy, its complete online result).
    observe(ctx, state, meter) -> FrameInputs at the state's absolute time.
    process(ctrl, vi, path, future, observation) -> original (acc, delta).
    trace(ctrl) -> ProcessTrace observed around that exact process call.
    pose(ctx, state, meter) -> PoseEvidence; charge all three required queries.
    events(ctx, source, meter) -> tuple[PhaseEvent] from that online source/GT.
    final_gate(ctx, candidate, meter) -> complete static chain evidence dictionary.
    """
    selected: object
    observe: object
    process: object
    trace: object
    pose: object
    events: object
    final_gate: object
    process_identity: object


class Meter:
    """Soft time boundary; callbacks can overrun once, but no later work proceeds.

    scalar_steps counts every source frame check, process call, plant step, pose
    gate, adapter invocation and actual map/GT query. It is not an arithmetic-op
    count. native_process_calls is independently visible and included in it.
    """
    def __init__(self, clock):
        self.clock, self.started = clock, clock()
        self.search = False
        self.counts = {}
        self.scalar_steps = 0

    def check(self, stage):
        elapsed = self.clock() - self.started
        require(elapsed < STATIC_BUDGET_S, 'total_soft_budget_' + stage)
        if self.search and elapsed >= STATIC_BUDGET_S - FINAL_RESERVE_S:
            raise SearchStop('search_final_reserve_reached')

    def charge(self, kind, count=1):
        self.check('before_' + kind)
        require(isinstance(count, int) and count > 0, 'positive_scalar_charge_required')
        if self.scalar_steps + count > MAX_SCALAR_STEPS:
            raise SearchStop('scalar_step_cap')
        self.scalar_steps += count
        self.counts[kind] = self.counts.get(kind, 0) + count

    def call(self, kind, callback, *args):
        self.charge(kind)
        value = callback(*args)
        self.check('after_' + kind)
        return value

    def note(self, kind, count=1):
        """Memo hits and other non-work counters; never replaces a query charge."""
        require(isinstance(count, int) and count >= 0, 'nonnegative_note_required')
        self.counts[kind] = self.counts.get(kind, 0) + count


def _finite_state(state, track=True):
    require(isinstance(state, BankState), 'BankState_required')
    require(all(math.isfinite(float(v)) for v in state.values()), 'finite_state_required')
    require(state.speed >= 0., 'negative_speed')
    if track:
        require(-6. <= state.last_acc <= 2.5, 'initial_or_track_acceleration_outside_contract')


def _same_state(left, right, tolerance=1e-8):
    return all(abs(a - b) <= tolerance for a, b in zip(left.values(), right.values()))


def _physically_safe(result):
    if not isinstance(result, dict):
        return False
    detail = result.get('detail', {})
    score = result.get('score')
    arrival = result.get('arrival')
    return (result.get('end') == 4 and isinstance(score, (int, float))
            and math.isfinite(score) and isinstance(arrival, (int, float))
            and math.isfinite(arrival) and detail.get('vehicle_collision') is False
            and detail.get('road_boundary_collision') is False
            and detail.get('road_check') == 'bitmap_footprint')


def exact_bank_path_content(path):
    """Read-only shape/dtype/full-byte evidence for rebuilt native bank paths.

    Original planner lists remain the process argument; asarray is only a view
    for a finite numeric content signature. Opaque paths remain pointer-strict.
    """
    import numpy as np
    if not isinstance(path, (list, tuple, np.ndarray)):
        return None
    try:
        values = np.asarray(path)
    except (TypeError, ValueError):
        return None
    if (values.ndim != 2 or len(values) < 2 or values.shape[1] < 2
            or values.dtype.kind not in 'fiu' or not np.isfinite(values).all()):
        return None
    return (tuple(values.shape), values.dtype.str, values.tobytes(order='C'))


def _trace_contract(trace, source, branch, steering, *, execution_path_content=None):
    require(isinstance(trace, ProcessTrace), 'observed_original_process_trace_required')
    require(trace.process_identity is source.process_identity,
            'original_process_identity_changed')
    if execution_path_content is None:
        require(trace.path_identity is source.path, 'original_bank_path_changed')
    else:
        require(exact_bank_path_content(trace.path_identity) == execution_path_content,
                'original_bank_execution_path_bytes_changed')
    require((trace.process_calls, trace.bank_profile_stage_calls, trace.pursuit_calls,
             trace.geometry_overrides) == (1, 1, 1, 0),
            'one_original_bank_profile_pursuit_without_geometry_override_required')
    require(trace.preview_failed is False, 'native_bank_preview_failed')
    require(trace.branch == branch, 'native_bank_longitudinal_branch_mismatch')
    require(math.isfinite(trace.final_steering)
            and abs(trace.final_steering - steering) <= 1e-12,
            'returned_steering_differs_from_original_bank_process')
    require(abs(steering) <= STEERING_LIMIT + 1e-12,
            'original_bank_steering_domain_exceeded')


def _mutable_ids(value, seen=None):
    """Walk actual controller containers/custom state, including slots and arrays."""
    if seen is None:
        seen = set()
    ident = id(value)
    if ident in seen:
        return set()
    seen.add(ident)
    if value is None or isinstance(value, (str, bytes, int, float, bool, complex,
                                          type, types.FunctionType,
                                          types.BuiltinFunctionType, types.ModuleType)):
        return set()
    # Unknown opaque leaves are also treated as mutable, failing closed when a
    # custom deepcopy returns the original object. Only explicit immutable types
    # above and immutable tuple/frozenset containers may share identity.
    mutable = not isinstance(value, (tuple, frozenset))
    children = []
    if isinstance(value, dict):
        children = list(value.keys()) + list(value.values())
    elif isinstance(value, (tuple, list, set, frozenset)):
        children = list(value)
    if hasattr(value, '__array_interface__'):
        # A deepcopy must own its array storage; views with shared base are rejected.
        mutable = True
        if getattr(value, 'base', None) is not None:
            children.append(value.base)
    attributes = getattr(value, '__dict__', None)
    if isinstance(attributes, dict):
        mutable = True
        children.append(attributes)
    for cls in type(value).__mro__:
        slots = cls.__dict__.get('__slots__', ())
        if isinstance(slots, str):
            slots = (slots,)
        for slot in slots:
            if slot not in ('__dict__', '__weakref__') and hasattr(value, slot):
                mutable = True
                children.append(getattr(value, slot))
    require(not isinstance(value, memoryview), 'controller_memoryview_not_isolatable')
    result = {ident} if mutable else set()
    for child in children:
        result.update(_mutable_ids(child, seen))
    return result


def isolated_controller(controller, meter):
    meter.charge('controller_clones')
    original_ids = _mutable_ids(controller)
    replica = copy.deepcopy(controller)
    require(replica is not controller and not original_ids.intersection(_mutable_ids(replica)),
            'controller_mutable_state_alias_after_deepcopy')
    meter.check('after_controller_clone')
    return replica


def _source_contract(source, ctx, priors, backend, meter):
    require(isinstance(source, NativeBankSource), 'native_source_required')
    require(source.ctx is ctx, 'same_online_ctx_required')
    require(source.online_same_object is True and source.external_csv_used is False
            and source.complete_native_capture_verified is True,
            'verified_complete_online_native_capture_required')
    require(callable(source.policy) and getattr(source.policy, 'kind', None) == 'F0'
            and not getattr(source.policy, 'tracking', False)
            and not hasattr(source.policy, 'steering')
            and not getattr(source.policy, 'stpp_bank_shadow', False),
            'native_F0_IDM_source_required_without_fabricated_tracking')
    require(backend.process_identity is source.process_identity,
            'backend_original_process_identity_mismatch')
    selected_policy, selected_result = meter.call('selected_source_queries', backend.selected, ctx)
    require(selected_policy is source.policy and selected_result is source.result,
            'source_must_be_actual_selected_online_policy_and_result')
    require(_physically_safe(source.result), 'source_not_complete_physically_safe_bitmap_arrival')
    require(isinstance(priors, tuple) and any(r is source.result for r in priors),
            'all_online_priors_including_same_source_result_required')
    safe_priors = [r for r in priors if _physically_safe(r)]
    require(bool(safe_priors), 'no_complete_physically_safe_prior')
    threshold = max(r['score'] for r in safe_priors) + HYST
    require(threshold < 100., 'no_score_headroom_above_highest_prior_hyst')
    _finite_state(source.initial)
    require(abs(float(source.controller._last_acc) - source.initial.last_acc) <= 1e-12,
            'native_initial_controller_acceleration_mismatch')
    require(math.isfinite(source.vehicle_length) and source.vehicle_length > 0.,
            'actual_vehicle_length_required')
    require(isinstance(source.frames, tuple) and 1 <= len(source.frames) <= MAX_CHAIN_STEPS,
            'complete_bounded_native_frames_required')
    initial_evidence = _pose(ctx, source.initial, backend, meter)
    require(not initial_evidence.in_goal, 'native_source_initial_already_in_goal')
    previous, older = source.initial, None
    for index, frame in enumerate(source.frames):
        meter.charge('source_frame_checks')
        require(isinstance(frame, BankFrame), 'captured_native_bank_frame_required')
        _finite_state(frame.before, track=False)
        _finite_state(frame.after, track=False)
        require(_same_state(previous, frame.before), 'native_source_frame_gap')
        require(abs(frame.after.t - frame.before.t - DT) <= 1e-9,
                'native_source_dt_mismatch')
        require(all(math.isfinite(float(v)) for v in
                    (frame.command, frame.acceleration, frame.steering))
                and V_MIN <= frame.command <= V_MAX
                and abs(frame.after.last_acc - frame.acceleration) <= 1e-12,
                'native_source_command_or_acceleration_mismatch')
        _trace_contract(frame.trace, source, 'native_idm', frame.steering)
        # Algebraic consistency check on captured rows, not a source replay or
        # plant advance. Native IDM acceleration is not required to equal P2.
        meter.charge('source_old_speed_algebra_checks')
        before = frame.before
        expected = (before.t + DT,
            before.x + before.speed * math.cos(before.yaw) * DT,
            before.y + before.speed * math.sin(before.yaw) * DT,
            before.yaw + before.speed / source.vehicle_length
            * 1.7 * math.tan(frame.steering) * DT,
            max(0., before.speed + frame.acceleration * DT), frame.acceleration)
        require(all(abs(a - b) <= 1e-8 for a, b in zip(frame.after.values(), expected)),
                'native_source_old_speed_plant_algebra_mismatch')
        meter.charge('source_ALAT_checks')
        _alat_closed(older, before, frame.after)
        evidence = _pose(ctx, frame.after, backend, meter)
        require(evidence.in_goal is (index == len(source.frames) - 1),
                'native_source_first_goal_must_be_terminal')
        older = before
        previous = frame.after
    meter.charge('source_terminal_ALAT_checks')
    _terminal_alat(source.frames[-1].before, source.frames[-1].after)
    require(abs(previous.t - source.result['arrival']) <= 1e-8,
            'native_source_complete_arrival_time_mismatch')
    return threshold


def native_first_command(source):
    """P2 inverse keeps the first native actual acceleration, not its IDM command.

    Command bounds and both clips are checked after inversion. In particular a
    native zero-speed/zero-acceleration start may be unreachable with V_MIN=.1.
    """
    state, wanted = source.initial, source.frames[0].acceleration
    require(math.isfinite(wanted), 'finite_native_first_acceleration_required')
    command = min(V_MAX, max(V_MIN, state.speed + wanted / P_GAIN))
    requested = max(-6., min(2.5, P_GAIN * (command - state.speed)))
    actual = max(state.last_acc - 1., min(state.last_acc + 1., requested))
    require(abs(actual - wanted) <= 1e-10, 'native_first_acceleration_not_P2_reachable')
    return command


def _recorded(state):
    return (round(state.t, 1), round(state.yaw, 4), round(state.speed, 4))


def _yaw_step(start, finish):
    delta = finish - start
    require(abs(delta) < math.pi - 1e-8, 'ambiguous_yaw_unwrap_step')
    return delta


def _alat_closed(previous, current, following):
    """Append closes current sample. First sample uses original forward edge."""
    states = (current, following) if previous is None else (previous, current, following)
    for rounded in (False, True):
        data = [_recorded(s) if rounded else (s.t, s.yaw, s.speed) for s in states]
        if previous is None:
            omega = _yaw_step(data[0][1], data[1][1]) / (data[1][0] - data[0][0])
            speed = data[0][2]
        else:
            h0, h1 = data[1][0] - data[0][0], data[2][0] - data[1][0]
            require(h0 > 0. and h1 > 0., 'increasing_recorder_times_required')
            omega = (h1 / (h0 + h1) * _yaw_step(data[0][1], data[1][1]) / h0
                     + h0 / (h0 + h1) * _yaw_step(data[1][1], data[2][1]) / h1)
            speed = data[1][2]
        limit = .4 if rounded else .391
        require(abs(speed * omega) <= limit, 'raw_or_recorder_closed_ALAT')


def _terminal_alat(previous, current):
    for rounded in (False, True):
        a, b = (_recorded(previous), _recorded(current)) if rounded else (
            (previous.t, previous.yaw, previous.speed), (current.t, current.yaw, current.speed))
        require(abs(b[2] * _yaw_step(a[1], b[1]) / (b[0] - a[0]))
                <= (.4 if rounded else .391), 'raw_or_recorder_terminal_ALAT')


def _advance(state, acceleration, steering, vehicle_length):
    """Exact original scalar plant; only called after an actual process callback."""
    return BankState(state.t + DT,
        state.x + state.speed * math.cos(state.yaw) * DT,
        state.y + state.speed * math.sin(state.yaw) * DT,
        state.yaw + state.speed / vehicle_length * 1.7 * math.tan(steering) * DT,
        max(0., state.speed + acceleration * DT), acceleration)


def _controller_pose(ctrl):
    return (float(ctrl._ego_x), float(ctrl._ego_y), float(ctrl._ego_yaw),
            float(ctrl._ego_v), float(ctrl._last_acc))


def _consume_command(policy, ctrl, t, expected, command):
    require(math.isfinite(float(t)) and abs(t - expected.t) <= 1e-9,
            'same_absolute_command_time_required')
    pose = _controller_pose(ctrl)
    wanted = (expected.x, expected.y, expected.yaw, expected.speed, expected.last_acc)
    require(all(abs(a - b) <= 1e-8 for a, b in zip(pose, wanted)),
            'command_pose_speed_or_last_acc_mismatch')
    old = getattr(ctrl, '_native_bank_phase_token', None)
    require(old is None or old[-1] is True, 'duplicate_or_unconsumed_bank_command')
    if old is not None:
        require(abs(float(t) - old[2] - DT) <= 1e-9,
                'unique_consecutive_bank_command_time_required')
    ctrl._native_bank_phase_token = (policy, id(ctrl), float(t), pose, command, False)
    return command


def _complete_command(policy, ctrl, frame):
    token = getattr(ctrl, '_native_bank_phase_token', None)
    require(token is not None and token[0] is policy and token[1] == id(ctrl)
            and abs(token[2] - frame.before.t) <= 1e-9 and token[-1] is False,
            'same_controller_unique_pending_bank_command_required')
    require(abs(float(ctrl._last_acc) - frame.acceleration) <= 1e-10,
            'process_last_acc_mismatch')
    ctrl._native_bank_phase_token = token[:-1] + (True,)


@dataclass(frozen=True)
class _StepCommand:
    state: BankState
    command: float
    tracking = True
    kind = 'F17_NATIVE_BANK_PHASE_STEP'

    def __call__(self, ctrl, bank_v, t):
        require(math.isfinite(float(bank_v)), 'finite_original_bank_command_required')
        return _consume_command(self, ctrl, t, self.state, self.command)


@dataclass(frozen=True)
class BankPhaseCand:
    """Only speed is returned. Expected steering is an audit field, not an action."""
    source: NativeBankSource
    frames: tuple
    path_content: object = field(init=False, repr=False, compare=False)
    tracking = True
    kind = 'F17_NATIVE_BANK_PHASE'

    def __post_init__(self):
        require(isinstance(self.source, NativeBankSource) and isinstance(self.frames, tuple)
                and 1 <= len(self.frames) <= MAX_CHAIN_STEPS,
                'bounded_immutable_bank_phase_frames_required')
        object.__setattr__(self, 'path_content', exact_bank_path_content(self.source.path))
        previous = self.source.initial
        for frame in self.frames:
            require(isinstance(frame, BankFrame), 'bank_phase_frame_required')
            _finite_state(frame.before)
            _finite_state(frame.after)
            require(_same_state(previous, frame.before)
                    and abs(frame.after.t - frame.before.t - DT) <= 1e-9,
                    'bank_phase_complete_frame_chain_required')
            require(all(math.isfinite(v) for v in
                        (frame.command, frame.acceleration, frame.steering))
                    and V_MIN <= frame.command <= V_MAX,
                    'bank_phase_finite_bounded_commands_required')
            requested = max(-6., min(2.5, P_GAIN * (frame.command - frame.before.speed)))
            actual = max(frame.before.last_acc - 1.,
                         min(frame.before.last_acc + 1., requested))
            require(abs(frame.acceleration - actual) <= 1e-10
                    and abs(frame.after.last_acc - actual) <= 1e-10,
                    'bank_phase_P_clip_jerk_or_last_acc_table_mismatch')
            _trace_contract(frame.trace, self.source, 'track_P2', frame.steering)
            previous = frame.after
        require(abs(self.frames[0].acceleration - self.source.frames[0].acceleration) <= 1e-10
                and abs(self.frames[0].steering - self.source.frames[0].steering) <= 1e-10,
                'native_first_actual_acceleration_and_bank_steering_required')
        require(_same_state(self.frames[0].after, self.source.frames[0].after),
                'native_first_actual_state_table_mismatch')

    @property
    def label(self):
        return 'NATIVE_BANK_PHASE_I1_' + self.source.policy.label

    def __call__(self, ctrl, bank_v, t):
        require(math.isfinite(float(bank_v)), 'finite_original_bank_command_required')
        require(math.isfinite(float(t)), 'finite_bank_phase_command_time_required')
        index = int(round((float(t) - self.frames[0].before.t) / DT))
        require(0 <= index < len(self.frames), 'bank_phase_no_prefix_or_tail_fallback')
        frame = self.frames[index]
        return _consume_command(self, ctrl, float(t), frame.before, frame.command)

    def verify_execution_path(self, path):
        """Before complete prediction/Env process: verify actual path, keep its pointer."""
        if self.path_content is None:
            require(path is self.source.path, 'original_bank_path_changed')
        else:
            require(exact_bank_path_content(path) == self.path_content,
                    'original_bank_execution_path_bytes_changed')
        return True

    def execution_after_frame(self, ctrl, acceleration, steering, trace):
        token = getattr(ctrl, '_native_bank_phase_token', None)
        require(token is not None and token[0] is self, 'pending_candidate_command_required')
        index = int(round((token[2] - self.frames[0].before.t) / DT))
        require(0 <= index < len(self.frames), 'candidate_audit_index_outside_table')
        frame = self.frames[index]
        require(abs(acceleration - frame.acceleration) <= 1e-10
                and abs(steering - frame.steering) <= 1e-10,
                'same_native_process_action_chain_not_realized')
        _trace_contract(trace, self.source, 'track_P2', steering,
                        execution_path_content=self.path_content)
        _complete_command(self, ctrl, frame)


def _pose(ctx, state, backend, meter):
    before = {key: meter.counts.get(key, 0) for key in
              ('runtime_road_queries', 'bitmap_polygon_queries', 'GT_queries')}
    evidence = meter.call('pose_gates', backend.pose, ctx, state, meter)
    require(isinstance(evidence, PoseEvidence) and abs(evidence.t - state.t) <= 1e-9,
            'same_absolute_pose_evidence_required')
    require(all(getattr(evidence, key) is True for key in
                ('runtime_road_safe', 'bitmap_polygon_safe', 'GT_collision_free',
                 'ttc_safe', 'lateral_distance_safe')), 'pose_road_GT_or_score_safety_refused')
    require(all(meter.counts.get(key, 0) > value for key, value in before.items()),
            'actual_map_GT_queries_must_be_metered')
    require(isinstance(evidence.in_goal, bool)
            and math.isfinite(evidence.goal_bbox_distance)
            and evidence.goal_bbox_distance >= 0., 'goal_pose_evidence_required')
    return evidence


def _inputs_contract(inputs, source, state):
    require(isinstance(inputs, FrameInputs) and inputs.path is source.path,
            'original_same_bank_path_frame_inputs_required')
    ego = inputs.vehicle_info.get('ego', {})
    values = (ego.get('x'), ego.get('y'), ego.get('yaw_rad'), ego.get('v_mps'),
              ego.get('acc_mpss'))
    wanted = (state.x, state.y, state.yaw, state.speed, state.last_acc)
    require(all(isinstance(a, (int, float)) and math.isfinite(a)
                and abs(a - b) <= 1e-10 for a, b in zip(values, wanted)),
            'observed_pose_speed_last_acc_mismatch')
    test = inputs.observation.get('test_setting', {})
    require(isinstance(test.get('t'), (int, float))
            and abs(test['t'] - state.t) <= 1e-9
            and inputs.observation.get('vehicle_info') is inputs.vehicle_info,
            'original_absolute_observation_identity_required')


def _process_step(source, ctx, ctrl, state, command, backend, meter):
    require(abs(float(ctrl._last_acc) - state.last_acc) <= 1e-12,
            'node_controller_last_acc_mismatch')
    inputs = meter.call('observation_adapters', backend.observe, ctx, state, meter)
    _inputs_contract(inputs, source, state)
    policy = _StepCommand(state, command)
    ctrl._sas_cand = policy
    acceleration, steering = meter.call('native_process_calls', backend.process, ctrl,
        inputs.vehicle_info, inputs.path, inputs.traj_future, inputs.observation)
    require(all(math.isfinite(float(v)) for v in (acceleration, steering)),
            'nonfinite_original_process_action')
    expected = max(-6., min(2.5, P_GAIN * (command - state.speed)))
    expected = max(state.last_acc - 1., min(state.last_acc + 1., expected))
    require(abs(acceleration - expected) <= 1e-10,
            'original_P_clip_jerk_action_mismatch')
    require(ctrl._sas_exec_policy == 'track' and ctrl._sas_geometry_candidate is None
            and getattr(ctrl, '_stpp_exec_v', None) is None,
            'speed_only_native_bank_execution_branch_required')
    trace = meter.call('process_trace_reads', backend.trace, ctrl)
    _trace_contract(trace, source, 'track_P2', steering)
    meter.charge('plant_steps')
    after = _advance(state, acceleration, steering, source.vehicle_length)
    meter.check('after_plant_step')
    frame = BankFrame(state, after, command, acceleration, steering, trace)
    _complete_command(policy, ctrl, frame)
    return frame


def _commands(state, events):
    """Fixed physical primitives plus at most two traffic-window arrival commands."""
    commands = [V_MAX, V_MIN, min(V_MAX, max(V_MIN, state.speed))]
    witnesses = []
    for event in events:
        dx, dy = event.x - state.x, event.y - state.y
        if dx * math.cos(state.yaw) + dy * math.sin(state.yaw) <= 0.:
            continue
        # This is a proposal; actual process and every pose reject infeasible timing.
        deadline = (event.blocked_start - DT if state.t < event.blocked_start - DT
                    else event.blocked_end + DT)
        if deadline <= state.t + DT:
            continue
        witnesses.append((math.hypot(dx, dy), deadline))
    for distance, deadline in sorted(witnesses, key=lambda item: item[0])[:2]:
        commands.append(min(V_MAX, max(V_MIN, distance / (deadline - state.t))))
    return tuple(dict.fromkeys(commands))


@dataclass(frozen=True)
class _Node:
    state: BankState
    controller: object
    previous: BankState | None
    frames: tuple
    goal_distance: float


def build_candidate(source, ctx, priors, backend, *, clock=time.perf_counter):
    """One fixed-budget search and at most one speed-only static candidate.

    No node merge is performed: controller state, speed, last_acc and ALAT history
    cannot disappear inside a quantized key. Original IDM source is never relabelled
    TRACK. Search is a bounded proposal tree, with no optimality/completeness claim.
    """
    meter = Meter(clock)
    audit = dict(mechanism='native_IDM_source_bridge_and_GT_phase_longitudinal_control',
        accepted=False, candidate_count=0, complete_policy_predictions=0, Env_runs=0,
        external_csv_used=False, steering_override=False,
        native_IDM_source_is_TRACK=False, original_bank_steering_algorithm_preserved=True,
        spatial_trajectory_unchanged_claimed=False, parameter_grid=False,
        static_budget_s=STATIC_BUDGET_S, final_reserve_s=FINAL_RESERVE_S,
        max_expanded=MAX_EXPANDED, max_edges=MAX_EDGES,
        max_scalar_steps=MAX_SCALAR_STEPS, max_chain_steps=MAX_CHAIN_STEPS,
        one_search=True, graph_searches=0, actual_process_calls=0,
        expanded=0, edges=0, refusals={},
        scalar_unit='source check/process/plant/adapter/pose/actual map or GT query',
        all_actual_process_calls_in_scalar_cap=True)
    candidate = None
    try:
        require(isinstance(backend, BankPhaseBackend), 'original_bank_backend_required')
        for name in ('selected', 'observe', 'process', 'trace', 'pose', 'events', 'final_gate'):
            require(callable(getattr(backend, name)), 'missing_backend_' + name)
        threshold = _source_contract(source, ctx, priors, backend, meter)
        audit['highest_physically_safe_prior_plus_hyst'] = threshold
        audit['original_hyst'] = HYST
        events = meter.call('phase_event_adapters', backend.events, ctx, source, meter)
        require(isinstance(events, tuple) and 0 < len(events) <= MAX_CHAIN_STEPS,
                'bounded_original_GT_phase_events_required')
        for event in events:
            require(isinstance(event, PhaseEvent) and event.source_policy is source.policy
                    and event.source_result is source.result and event.original_GT is True
                    and bool(event.actor_token)
                    and all(math.isfinite(v) for v in
                            (event.x, event.y, event.blocked_start, event.blocked_end))
                    and event.blocked_start <= event.blocked_end,
                    'identity_bound_original_GT_phase_event_required')
        audit['phase_event_count'] = len(events)
        initial_pose = _pose(ctx, source.initial, backend, meter)
        require(not initial_pose.in_goal, 'source_initial_already_in_goal')
        root_ctrl = isolated_controller(source.controller, meter)
        # Crossing IDM -> P2 keeps the first actual action and original bank delta.
        # The newly inverted speed command is deliberately not the IDM command.
        first_command = native_first_command(source)
        meter.search = True
        meter.check('before_graph_search')
        meter.note('graph_searches')
        first = _process_step(source, ctx, root_ctrl, source.initial, first_command,
                              backend, meter)
        require(abs(first.acceleration - source.frames[0].acceleration) <= 1e-10
                and abs(first.steering - source.frames[0].steering) <= 1e-10,
                'first_native_actual_action_bridge_mismatch')
        require(_same_state(first.after, source.frames[0].after),
                'first_native_actual_state_bridge_mismatch')
        _alat_closed(None, source.initial, first.after)
        first_pose = _pose(ctx, first.after, backend, meter)
        require(not first_pose.in_goal, 'native_first_step_already_completes_task')
        audit.update(first_command_mps=first_command,
                     native_first_command_mps=source.frames[0].command,
                     native_first_command_equality_claimed=False,
                     first_actual_action_and_state_preserved=True)
        root = _Node(first.after, root_ctrl, source.initial, (first,),
                     first_pose.goal_bbox_distance)
        heap, serial = [(source.initial.t + 4. * root.goal_distance / V_MAX, 0, root)], 0
        best = None
        meter.search = True
        stop = 'search_exhausted'
        try:
            while heap and audit['expanded'] < MAX_EXPANDED and audit['edges'] < MAX_EDGES:
                meter.check('search_loop')
                _, _, node = heapq.heappop(heap)
                audit['expanded'] += 1
                for command in _commands(node.state, events):
                    if audit['edges'] >= MAX_EDGES:
                        stop = 'edge_cap'
                        break
                    audit['edges'] += 1
                    controller = isolated_controller(node.controller, meter)
                    state, previous, frames = node.state, node.previous, node.frames
                    try:
                        for _ in range(min(EDGE_STEPS, MAX_CHAIN_STEPS - len(frames))):
                            frame = _process_step(source, ctx, controller, state, command,
                                                  backend, meter)
                            _alat_closed(previous, state, frame.after)
                            evidence = _pose(ctx, frame.after, backend, meter)
                            frames = frames + (frame,)
                            previous, state = state, frame.after
                            if evidence.in_goal:
                                _terminal_alat(previous, state)
                                if state.t < source.result['arrival'] - 1e-9:
                                    if best is None or state.t < best.state.t:
                                        best = _Node(state, controller, previous, frames, 0.)
                                break
                        else:
                            if len(frames) >= MAX_CHAIN_STEPS:
                                continue
                            if state.t >= source.result['arrival'] - 1e-9:
                                continue
                            serial += 1
                            child = _Node(state, controller, previous, frames,
                                          evidence.goal_bbox_distance)
                            priority = state.t + 4. * child.goal_distance / V_MAX
                            heapq.heappush(heap, (priority, serial, child))
                    except NativeBankPhaseRefusal as error:
                        reason = str(error)
                        if reason not in ('raw_or_recorder_closed_ALAT',
                                          'raw_or_recorder_terminal_ALAT',
                                          'pose_road_GT_or_score_safety_refused',
                                          'native_bank_preview_failed'):
                            raise
                        audit['refusals'][reason] = audit['refusals'].get(reason, 0) + 1
                if best is not None:
                    # One extracted full static chain. No alternate finalist retries.
                    stop = 'one_complete_static_goal_chain'
                    break
            if audit['expanded'] >= MAX_EXPANDED:
                stop = 'expansion_cap'
        except SearchStop as error:
            stop = str(error)
        meter.search = False
        audit['search_stop'] = stop
        require(best is not None, 'no_executable_native_bank_phase_chain')
        candidate = meter.call('candidate_freezes', BankPhaseCand, source, best.frames)
        require(not hasattr(candidate, 'steering'), 'speed_candidate_must_not_override_steering')
        gate = meter.call('final_chain_gates', backend.final_gate, ctx, candidate, meter)
        require(isinstance(gate, dict) and all(gate.get(key) is True for key in
                    ('complete_first_goal_chain', 'original_bank_process_actions_verified',
                     'actual_bitmap_polygon_all_rows', 'absolute_GT_all_rows',
                     'raw_and_recorder_ALAT_all_rows', 'TTC_and_lateral_all_rows',
                     'static_only'))
                and gate.get('complete_policy_predictions') == 0 and gate.get('Env_runs') == 0,
                'complete_static_chain_gate_refused')
        audit.update(accepted=True, reason='one_native_bank_phase_static_candidate',
                     candidate_count=1, model_arrival_s=best.state.t,
                     model_time_is_actual_arrival=False, command_frames=len(best.frames),
                     final_gate=gate)
    except (NativeBankPhaseRefusal, SearchStop) as error:
        candidate = None
        audit.update(accepted=False, reason=str(error), candidate_count=0)
    except Exception as error:
        candidate = None
        audit.update(accepted=False, reason='backend_or_proposal_exception', candidate_count=0,
                     error=dict(type=type(error).__name__, message=str(error)))
    audit.update(graph_searches=meter.counts.get('graph_searches', 0),
                 actual_process_calls=meter.counts.get('native_process_calls', 0),
                 scalar_steps=meter.scalar_steps, counters=dict(meter.counts),
                 elapsed_s=clock() - meter.started)
    if candidate is not None:
        try:
            # Publication comes after all counter/metadata bookkeeping. No later
            # clock read or callback may return an accepted policy after expiry.
            meter.check('before_candidate_return')
        except (NativeBankPhaseRefusal, SearchStop) as error:
            candidate = None
            audit.update(accepted=False, reason=str(error), candidate_count=0)
    return candidate, audit
