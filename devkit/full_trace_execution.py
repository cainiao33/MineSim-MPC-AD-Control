"""原控制帧的全链观察与动作表执行保护；不替换控制动作。"""

import copy
import math

import numpy as np

from control_full_trace_qp_contract import (
    FrozenFullTraceTableCand, path_signature, raise_saved_policy_error, require,
)
from executed_braking_reachability import speed_step


_TABLE_BINDINGS = {}
_BOUND_CTX = None


def register_table(candidate):
    """在原生预测前冻结执行挂点的路径/车型绑定；不改政策对象。"""
    global _BOUND_CTX
    require(isinstance(candidate, FrozenFullTraceTableCand), 'full_trace_table_registration_required')
    if _BOUND_CTX is not candidate.source.ctx:
        _TABLE_BINDINGS.clear()
        _BOUND_CTX = candidate.source.ctx
    expected = candidate.source.execution['frames'][0]['actual_path_signature_before']
    require(path_signature(candidate.source.path) == expected, 'full_trace_registered_path_not_original')
    _TABLE_BINDINGS[id(candidate)] = dict(candidate=candidate, path=expected,
        shape=copy.deepcopy(candidate.source.ego_shape), original=candidate.source.original_process)


class ExecutionCapture:
    def __init__(self, candidate, phase, dt):
        require(float(dt) == .1, 'full_trace_capture_point_one_dt')
        self.candidate, self.phase, self.dt = candidate, phase, float(dt)
        self.frames, self.controller = [], None

    def append(self, ctrl, frame):
        self.frames.append(frame)
        if self.controller is None:
            anchor = copy.copy(ctrl)
            anchor._last_acc = frame['previous_acceleration_mps2']
            for key in ('_local_exit_frame', '_local_exit_capture_state',
                        '_full_trace_capture', '_full_trace_runtime_frame',
                        '_full_trace_error', '_full_trace_live_state'):
                setattr(anchor, key, None)
            self.controller = anchor

    def finish(self, expected_frames):
        frames = self.frames
        acceleration_bad = sum(abs(b['previous_acceleration_mps2'] -
                                   a['actual_acceleration_mps2']) > 1e-12
                               for a, b in zip(frames, frames[1:]))
        time_bad = sum(abs(b['t'] - a['t'] - self.dt) > 1e-7
                       for a, b in zip(frames, frames[1:]))
        failed = sum(f['pass_frame'] is not True for f in frames)
        return dict(phase=self.phase, frames=frames, frame_count=len(frames),
                    original_process_calls=len(frames),
                    command_count=sum(f['command_calls'] for f in frames),
                    expected_frames=int(expected_frames), failed_frames=failed,
                    acceleration_chain_failures=acceleration_bad,
                    time_chain_failures=time_bad,
                    **{'pass': bool(frames) and len(frames) == expected_frames
                       and failed == acceleration_bad == time_bad == 0})


def record_command(ctrl, candidate, command, t):
    frame = getattr(ctrl, '_full_trace_runtime_frame', None)
    if frame is None:
        return
    capture = frame['capture']
    if capture is None and not isinstance(candidate, FrozenFullTraceTableCand):
        return
    frame['commands'].append(dict(candidate=candidate, controller_id=id(ctrl),
        t=float(t), command_mps=float(command),
        same_policy=(getattr(ctrl, '_sas_cand', None) is candidate
                     or getattr(ctrl, '_sas_cand', None) is None),
        same_geometry=getattr(ctrl, '_sas_geometry_candidate', None) is candidate,
        track_branch=getattr(ctrl, '_sas_exec_policy', None) == 'track',
        stpp_clear=getattr(ctrl, '_stpp_exec_v', None) is None))


def record_steering(ctrl, candidate, raw):
    frame = getattr(ctrl, '_full_trace_runtime_frame', None)
    if frame is None:
        return
    if frame['capture'] is None and not isinstance(candidate, FrozenFullTraceTableCand):
        return
    frame['steering'].append(dict(controller_id=id(ctrl), policy_identity=id(candidate),
        t=frame['t'], command_calls=len(frame['commands']),
        pose=[float(getattr(ctrl, key)) for key in
              ('_ego_x', '_ego_y', '_ego_yaw', '_ego_v')],
        raw_steering_rad=float(raw)))


def _observe(ctrl, original, vehicles, path, observation, result, frame):
    commands, steering = frame['commands'], frame['steering']
    capture = frame['capture']
    candidate = capture.candidate if capture is not None else (
        commands[0]['candidate'] if commands else frame['expected_table'])
    if candidate is None:
        geometry = getattr(ctrl, '_sas_geometry_candidate', None)
        if isinstance(geometry, FrozenFullTraceTableCand):
            candidate = geometry
        elif (not frame['entered_replica'] and _BOUND_CTX is not None
              and _BOUND_CTX.name == observation['test_setting'].get('scenario_name')
              and _BOUND_CTX.plan is not None
              and isinstance(_BOUND_CTX.plan[0], FrozenFullTraceTableCand)):
            candidate = _BOUND_CTX.plan[0]
    saved = getattr(ctrl, '_full_trace_error', None)
    if saved is not None:
        raise_saved_policy_error(ctrl, saved[0])
    if candidate is None:
        return
    acceleration, delta = map(float, result)
    speed, previous, pose = frame['pose'][3], frame['previous'], frame['pose']
    event = dict(t=frame['t'], input_pose=pose, command_calls=len(commands),
        steering_calls=len(steering), steering=steering,
        actual_acceleration_mps2=acceleration, final_steering_rad=delta,
        previous_acceleration_mps2=previous, speed_mps=speed,
        actual_last_acceleration_match=float(ctrl._last_acc) == acceleration,
        original_process_identity=id(original), actual_path_identity=id(path),
        actual_path_signature_before=frame['path_before'],
        actual_path_signature_after=path_signature(path), pass_frame=False)
    valid = (len(commands) == len(steering) == 1
             and frame['path_before'] == event['actual_path_signature_after']
             and all(math.isfinite(x) for x in (*pose, previous, acceleration, delta))
             and speed >= 0. and -6. <= previous <= 2.5
             and abs(delta) <= 1. and event['actual_last_acceleration_match'])
    if len(commands) == len(steering) == 1:
        command, raw_packet = commands[0], steering[0]
        value, raw = command['command_mps'], raw_packet['raw_steering_rad']
        valid = (valid and command['candidate'] is candidate
            and command['controller_id'] == id(ctrl) and command['t'] == frame['t']
            and all(command[k] for k in ('same_policy', 'same_geometry', 'track_branch', 'stpp_clear'))
            and raw_packet['controller_id'] == id(ctrl)
            and raw_packet['policy_identity'] == id(candidate)
            and raw_packet['command_calls'] == 1 and raw_packet['pose'] == pose
            and math.isfinite(value) and .1 <= value <= 12.
            and math.isfinite(raw) and abs(raw) <= 1.)
        _, expected_acc, _ = speed_step(speed, previous, value, .1)
        rotations = getattr(ctrl, '_rot_scenes', {})
        rotation = rotations.get(observation['test_setting'].get('scenario_name'))
        if rotation is None:
            rotation = rotations.get('_default')
        expected_delta = raw
        if rotation not in (None, 'off', {}) and speed > .1:
            limit = math.atan(float(rotation.get('a_rot', .38)) *
                float(vehicles['ego']['shape']['length']) / (1.7 * speed * speed))
            expected_delta = max(-limit, min(limit, raw))
        event.update(command_mps=value, expected_acceleration_mps2=expected_acc,
            acceleration_delta=abs(expected_acc - acceleration),
            expected_final_steering_rad=expected_delta,
            final_steering_delta=abs(expected_delta - delta))
        valid = valid and abs(expected_acc - acceleration) <= 1e-12 and abs(expected_delta - delta) <= 1e-12
    required = bool(getattr(candidate, '_local_exit_policy', False))
    token = getattr(ctrl, '_local_exit_frame', None)
    consumed = not required or (isinstance(token, tuple) and len(token) == 7
        and token[0] is candidate and token[1] == id(ctrl) and token[2] == frame['t']
        and token[4] == tuple(pose) and token[5] is True and token[6] is True)
    event.update(steering_packet_required=required, steering_packet_consumed=bool(consumed),
                 action_domain_valid=math.isfinite(delta) and abs(delta) <= 1.,
                 pass_frame=bool(valid and consumed))
    if isinstance(candidate, FrozenFullTraceTableCand):
        source = candidate.source
        binding = _TABLE_BINDINGS.get(id(candidate))
        require(binding is not None and binding['candidate'] is candidate,
                'full_trace_table_not_registered_before_plant')
        require(original is source.original_process is binding['original'],
                'full_trace_original_process_changed_before_plant')
        require(path is source.path and vehicles['ego']['shape'] == binding['shape'],
                'full_trace_path_or_shape_changed_before_plant')
        require(frame['path_before'] == event['actual_path_signature_after'] == binding['path'],
                'full_trace_path_bytes_changed_before_plant')
        index = candidate._index(frame['t'])
        pre = np.asarray([frame['t'], *pose, previous])
        pre_error = float(np.max(np.abs(pre - candidate.rows6[index])))
        require(event['pass_frame'] is True and pre_error <= 1e-8
                and event['command_mps'] == candidate.commands[index]
                and steering[0]['raw_steering_rad'] == candidate.raw_delta[index]
                and abs(delta - candidate.final_delta[index]) <= 1e-10
                and abs(acceleration - candidate.rows6[index + 1, 5]) <= 1e-10,
                'full_trace_action_or_six_state_failed_before_plant')
        event['pre_six_state_delta'] = pre_error
        if getattr(ctrl, '_sas_cand', None) is None:
            require(source.ctx.plan is not None and source.ctx.plan[0] is candidate,
                    'full_trace_live_selected_object_changed_before_plant')
            state = getattr(ctrl, '_full_trace_live_state', None)
            if state is None or state['candidate'] is not candidate:
                require(index == 0, 'full_trace_live_first_frame_missing')
                state = dict(candidate=candidate, frames=0, max_pre_six_state_delta=0.)
                ctrl._full_trace_live_state = state
            require(index == state['frames'], 'full_trace_live_duplicate_or_missing_frame')
            state['frames'] += 1
            state['max_pre_six_state_delta'] = max(state['max_pre_six_state_delta'], pre_error)
            if index == len(candidate.commands) - 1:
                source.ctx._log(dict(ev='full_trace_live_complete', t=frame['t'],
                    candidate=candidate.label, candidate_identity=id(candidate),
                    frames=state['frames'], expected_frames=len(candidate.commands),
                    max_pre_six_state_delta=state['max_pre_six_state_delta'],
                    original_process_identity=id(original), pass_all=True,
                    action_checked_before_plant=True))
    if capture is not None:
        capture.append(ctrl, event)


def process_with_guard(original, ctrl, vehicles, path, future, observation):
    capture = getattr(ctrl, '_full_trace_capture', None)
    candidate = getattr(ctrl, '_sas_cand', None)
    table = isinstance(candidate, FrozenFullTraceTableCand)
    enabled = bool(getattr(ctrl, '_sas_cfg', {}).get('full_trace_sqp_enabled', False))
    if capture is None and not table and (candidate is not None or not enabled):
        return original(ctrl, vehicles, path, future, observation)
    previous_frame = getattr(ctrl, '_full_trace_runtime_frame', None)
    require(previous_frame is None or previous_frame['controller_id'] != id(ctrl),
            'full_trace_nested_control_frame')
    ego = vehicles['ego']
    frame = dict(controller_id=id(ctrl), capture=capture, commands=[], steering=[],
        expected_table=candidate if table else None, entered_replica=candidate is not None,
        t=float(observation['test_setting']['t']), previous=float(ctrl._last_acc),
        pose=[float(ego[key]) for key in ('x', 'y', 'yaw_rad', 'v_mps')],
        path_before=path_signature(path))
    ctrl._full_trace_runtime_frame = frame
    try:
        result = original(ctrl, vehicles, path, future, observation)
        _observe(ctrl, original, vehicles, path, observation, result, frame)
        return result
    finally:
        ctrl._full_trace_runtime_frame = previous_frame
