"""局部退弯副本执行审计；只观察原命令/P/clip/jerk，不改变动作。"""

import math

from executed_braking_reachability import speed_step


def begin(ctrl, candidate, dt):
    ctrl._local_exit_capture_state = dict(candidate=candidate, controller_id=id(ctrl),
        dt=float(dt), frame_active=False, commands=[], frames=[])


def before_frame(ctrl, t, speed, previous_acceleration):
    state = ctrl._local_exit_capture_state
    if state['controller_id'] != id(ctrl) or state['frame_active']:
        raise RuntimeError('local_exit_nested_or_foreign_frame')
    state.update(frame_active=True, t=float(t), speed=float(speed),
                 previous=float(previous_acceleration), command_start=len(state['commands']))


def record_command(ctrl, candidate, command, t):
    state = getattr(ctrl, '_local_exit_capture_state', None)
    if state is None:
        return
    if not state['frame_active']:
        raise RuntimeError('local_exit_command_outside_frame')
    state['commands'].append(dict(candidate=candidate, controller_id=id(ctrl),
        t=float(t), command=float(command), speed=float(ctrl._ego_v),
        previous=float(ctrl._last_acc)))


def after_frame(ctrl, acceleration, steering):
    state = ctrl._local_exit_capture_state
    commands = state['commands'][state['command_start']:]
    candidate = state['candidate']
    if not state['frame_active'] or len(commands) != 1:
        raise RuntimeError('local_exit_missing_or_duplicate_command')
    packet = commands[0]
    command, actual = packet['command'], float(acceleration)
    _, expected, _ = speed_step(state['speed'], state['previous'], command, state['dt'])
    packet_required = bool(getattr(candidate, '_local_exit_policy', False))
    geometry_packet = getattr(ctrl, '_local_exit_frame', None)
    packet_verified = not packet_required
    if packet_required and isinstance(geometry_packet, tuple) and len(geometry_packet) == 7:
        pose = tuple(float(getattr(ctrl, key)) for key in
                     ('_ego_x', '_ego_y', '_ego_yaw', '_ego_v'))
        packet_verified = (geometry_packet[0] is candidate
            and geometry_packet[1] == id(ctrl) and geometry_packet[2] == state['t']
            and geometry_packet[4] == pose
            and geometry_packet[5] is True and geometry_packet[6] is True)
    valid = (packet['candidate'] is candidate and packet['controller_id'] == id(ctrl)
        and packet['t'] == state['t'] and packet['speed'] == state['speed']
        and packet['previous'] == state['previous']
        and getattr(ctrl, '_sas_cand', None) is candidate
        and getattr(ctrl, '_sas_geometry_candidate', None) is candidate
        and getattr(ctrl, '_sas_exec_policy', None) == 'track'
        and getattr(ctrl, '_stpp_exec_v', None) is None
        and math.isfinite(command) and .1 <= command <= 12.
        and math.isfinite(actual) and abs(expected-actual) <= 1e-12
        and float(ctrl._last_acc) == actual and math.isfinite(float(steering))
        and packet_verified)
    if state['frames']:
        valid = valid and abs(state['previous']-
            state['frames'][-1]['actual_acceleration_mps2']) <= 1e-12
    frame = dict(t=state['t'], command_mps=command,
        speed_mps=state['speed'], previous_acceleration_mps2=state['previous'],
        actual_acceleration_mps2=actual, final_steering_rad=float(steering),
        expected_acceleration_mps2=expected, pass_frame=bool(valid),
        steering_packet_required=packet_required, steering_packet_verified=bool(packet_verified))
    state['frames'].append(frame)
    state['frame_active'] = False
    if not valid:
        raise RuntimeError('local_exit_original_execution_chain_failed')


def finish(ctrl, candidate, expected_frames):
    state = ctrl._local_exit_capture_state
    audit = dict(contract_version='local_exit_original_track_chain_v1',
        candidate=candidate.label, frame_count=len(state['frames']),
        command_count=len(state['commands']), expected_frames=int(expected_frames),
        local_policy=bool(getattr(candidate, '_local_exit_policy', False)),
        dt_s=state['dt'], frames=state['frames'],
        pass_all=not state['frame_active'])
    audit['pass'] = True
    audit['pass'] = valid({'local_exit_execution_audit': audit}, candidate, expected_frames)
    if not audit['pass']:
        raise RuntimeError('local_exit_incomplete_execution_audit')
    return audit


def valid(result, candidate=None, expected_frames=None):
    if not isinstance(result, dict):
        return False
    audit = result.get('local_exit_execution_audit', {})
    if not isinstance(audit, dict):
        return False
    frames = audit.get('frames', [])
    if (audit.get('contract_version') != 'local_exit_original_track_chain_v1'
            or audit.get('pass_all') is not True or audit.get('pass') is not True or not frames
            or audit.get('frame_count') != len(frames)
            or audit.get('command_count') != len(frames)
            or audit.get('expected_frames') != len(frames)
            or expected_frames is not None and expected_frames != len(frames)
            or candidate is not None and (audit.get('candidate') != candidate.label
                or audit.get('local_policy') != bool(getattr(candidate, '_local_exit_policy', False)))):
        return False
    try:
        dt = float(audit['dt_s'])
        if not math.isfinite(dt) or abs(dt-.1) > 1e-12:
            return False
        for frame in frames:
            command, speed, previous = (float(frame[key]) for key in
                ('command_mps', 'speed_mps', 'previous_acceleration_mps2'))
            actual = float(frame['actual_acceleration_mps2'])
            _, expected, _ = speed_step(speed, previous, command, dt)
            if (frame.get('pass_frame') is not True
                    or frame.get('steering_packet_verified') is not True
                    or frame.get('steering_packet_required') != audit.get('local_policy')
                    or not all(math.isfinite(float(value)) for value in frame.values())
                    or not .1 <= command <= 12. or speed < 0.
                    or abs(actual-expected) > 1e-12
                    or abs(float(frame['expected_acceleration_mps2'])-expected) > 1e-12):
                return False
        return all(abs(after['previous_acceleration_mps2']-
                       before['actual_acceleration_mps2']) <= 1e-12
                   and abs(after['t']-before['t']-dt) <= 1e-7
                   for before, after in zip(frames, frames[1:]))
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def reset(ctrl):
    ctrl._local_exit_capture_state = None
