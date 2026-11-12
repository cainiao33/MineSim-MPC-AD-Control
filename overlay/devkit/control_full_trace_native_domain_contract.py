"""F33独立原生±1转向动作表；不修改原±.6契约、source或车辆模型。"""
from dataclasses import dataclass
import numpy as np
from control_full_trace_qp_contract import (
    DT, MAX_CHAIN_STEPS, FullTraceSource, FrozenFullTraceTableCand,
    require, frozen_array,
)

STEERING_LIMIT = 1.


def _state_chain(rows, commands, raw_delta, final_delta, length, dt):
    """只检查已有动作代数；不调用 process 或推进一个新政策。"""
    require(np.all(rows[:, 4] >= 0.) and np.all((-6. <= rows[:, 5]) &
            (rows[:, 5] <= 2.5)), 'full_trace_state_domain')
    require(np.all((.1 <= commands) & (commands <= 12.)) and
            np.all(np.abs(raw_delta) <= STEERING_LIMIT) and
            np.all(np.abs(final_delta) <= STEERING_LIMIT),
            'full_trace_action_domain')
    require(np.all(np.abs(np.diff(rows[:, 0]) - dt) <= 1e-9),
            'full_trace_time_lattice')
    for i, command in enumerate(commands):
        t, x, y, yaw, speed, previous = rows[i]
        requested = max(-6., min(2.5, 2. * (float(command) - speed)))
        actual = max(previous - 1., min(previous + 1., requested))
        after = (t + dt, x + speed * np.cos(yaw) * dt,
                 y + speed * np.sin(yaw) * dt,
                 yaw + speed / length * 1.7 * np.tan(final_delta[i]) * dt,
                 max(0., speed + dt * actual), actual)
        require(np.max(np.abs(rows[i + 1] - after)) <= 1e-8,
                'full_trace_old_speed_P2_six_state_mismatch')


@dataclass(frozen=True, init=False)
class FrozenNativeDomainTableCand(FrozenFullTraceTableCand):
    def __init__(self, source, rows6=None, commands=None, raw_delta=None,
                 final_delta=None, *, role='identity'):
        require(isinstance(source, FullTraceSource), 'full_trace_source_required')
        require(role in ('identity', 'optimized'), 'full_trace_role_required')
        rows = frozen_array(source.rows6 if rows6 is None else rows6)
        require(rows.ndim == 2 and rows.shape[1] == 6 and
                2 <= len(rows) <= MAX_CHAIN_STEPS + 1, 'complete_candidate_rows6_required')
        count = len(rows) - 1
        arrays = dict(rows6=rows,
            commands=frozen_array(source.commands if commands is None else commands, (count,)),
            raw_delta=frozen_array(source.raw_delta if raw_delta is None else raw_delta, (count,)),
            final_delta=frozen_array(source.final_delta if final_delta is None else final_delta, (count,)))
        _state_chain(rows, arrays['commands'], arrays['raw_delta'], arrays['final_delta'],
                     source.vehicle_length, source.dt)
        if role == 'identity':
            require(all(getattr(source, key).tobytes() == value.tobytes()
                        for key, value in arrays.items()), 'identity_table_must_equal_source_bytes')
        values = dict(source=source, role=role, dt=source.dt,
            actions=frozen_array(np.column_stack((rows[:-1, 0], arrays['commands'], arrays['raw_delta']))),
            model_rows=frozen_array(rows[:, :5]), cut_time=float(rows[0, 0]),
            end_time=float(rows[-1, 0]), label='NATIVE_TRACE_' + role.upper() + '_I1_' + source.policy.label,
            tracking=True, _local_exit_policy=True, kind='F33_NATIVE_TRACE_' + role.upper(), **arrays)
        for key, value in values.items():
            object.__setattr__(self, key, value)
