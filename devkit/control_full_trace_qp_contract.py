"""F20 全链动作表及真实恒等重演合同；无 simulator 或评分调用。"""

from dataclasses import dataclass
import hashlib
import json
import math

import numpy as np


DT = .1
MAX_CHAIN_STEPS = 430
STEERING_LIMIT = .6


class FullTraceRefusal(RuntimeError):
    pass


def require(condition, reason):
    if not condition:
        raise FullTraceRefusal(reason)


def frozen_array(value, shape=None):
    """真正不可回写的 float64 C 数组，底层是 immutable bytes。"""
    array = np.asarray(value, dtype=np.float64, order='C')
    require(np.isfinite(array).all(), 'finite_full_trace_array_required')
    if shape is not None:
        require(array.shape == shape, 'full_trace_array_shape_mismatch')
    return np.frombuffer(array.tobytes(order='C'), dtype=np.float64).reshape(array.shape)


def array_signature(array):
    return dict(shape=list(array.shape), dtype=array.dtype.str,
                sha256=hashlib.sha256(array.tobytes(order='C')).hexdigest())


def path_signature(path):
    if isinstance(path, np.ndarray):
        require(path.ndim==2 and len(path)>=2 and path.shape[1]>=2
                and path.dtype.kind in 'fiu' and np.isfinite(path).all(),
                'finite_numeric_native_path_required')
        return dict(type=type(path).__name__, identity=id(path),
                    shape=list(path.shape), dtype=path.dtype.str,
                    strides=list(path.strides),
                    sha256=hashlib.sha256(path.tobytes(order='C')).hexdigest())
    require(isinstance(path, (list, tuple)), 'native_path_container_required')
    serialized = json.dumps(json_native(path), allow_nan=False, separators=(',', ':'))
    canonical = np.asarray(path)
    require(canonical.ndim == 2 and canonical.shape[1] >= 2 and
            canonical.dtype.kind in 'fiu' and np.isfinite(canonical).all(),
            'finite_numeric_native_path_required')
    return dict(type=type(path).__name__, identity=id(path),
                shape=list(canonical.shape), dtype=canonical.dtype.str,
                sha256=hashlib.sha256(canonical.tobytes(order='C')).hexdigest(),
                serialized_sha256=hashlib.sha256(serialized.encode()).hexdigest())


def json_native(value, *, _path='$', _stack=None):
    """严格 JSON wire 值；不修改物理数组、对象或冻结证据。"""
    if isinstance(value,np.generic):
        native=value.item()
        require(not isinstance(native,np.generic),
                'unsupported_numpy_scalar_at_'+_path+':'+type(value).__name__)
        return json_native(native,_path=_path,_stack=_stack)
    if value is None or type(value) in (bool,int,str):
        return value
    if type(value) is float:
        require(math.isfinite(value),'nonfinite_json_number_at_'+_path)
        return value
    require(isinstance(value,(list,tuple,dict)),
            'unsupported_json_type_at_'+_path+':'+type(value).__name__)
    stack=set() if _stack is None else _stack
    require(id(value) not in stack,'cyclic_json_value_at_'+_path)
    stack.add(id(value))
    try:
        if isinstance(value,dict):
            result={}
            for key,item in value.items():
                if isinstance(key,np.generic):
                    key=key.item()
                require(type(key) is str,'json_str_key_required_at_'+_path)
                result[key]=json_native(item,_path=_path+'.'+key,_stack=stack)
            return result
        return [json_native(item,_path=_path+'['+str(index)+']',_stack=stack)
                for index,item in enumerate(value)]
    finally:
        stack.remove(id(value))


def _json_bytes(value):
    return json.dumps(json_native(value), allow_nan=False, sort_keys=True,
                      separators=(',', ':')).encode()


def object_signature(value, seen=None):
    """只读对象内容指纹；循环用身份收束，不执行政策。"""
    seen = set() if seen is None else seen
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value,float) and not math.isfinite(value):
        return dict(nonfinite=repr(value))
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value,bytes):
        return dict(type='bytes',size=len(value),sha256=hashlib.sha256(value).hexdigest())
    if id(value) in seen:
        return {'reference_identity': id(value)}
    seen.add(id(value))
    if isinstance(value, np.ndarray):
        return dict(identity=id(value), **array_signature(value))
    if isinstance(value, dict):
        return {str(k): object_signature(v, seen) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [object_signature(v, seen) for v in value]
    fields = dict(getattr(value, '__dict__', {}))
    for cls in type(value).__mro__:
        slots = cls.__dict__.get('__slots__', ())
        if isinstance(slots, str):
            slots = (slots,)
        for key in slots:
            if key not in ('__dict__', '__weakref__') and hasattr(value, key):
                fields[key] = getattr(value, key)
    return dict(type=type(value).__name__, identity=id(value),
                values={k: object_signature(v, seen) for k, v in fields.items()})


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


@dataclass(frozen=True)
class FullTraceState:
    t: float
    x: float
    y: float
    yaw: float
    speed: float
    last_acc: float

    def values(self):
        return (self.t, self.x, self.y, self.yaw, self.speed, self.last_acc)


@dataclass(frozen=True, init=False)
class FullTraceSource:
    ctx: object
    policy: object
    result: object
    path: object
    original_process: object
    controller: object
    rows6: np.ndarray
    commands: np.ndarray
    raw_delta: np.ndarray
    final_delta: np.ndarray
    ego_shape: object
    rot_config: object
    dt: float
    online_same_object: bool
    external_csv_used: bool
    complete_actual_verified: bool
    _execution_bytes: bytes

    def __init__(self, *, ctx, policy, result, path, original_process, controller,
                 rows6, commands, raw_delta, final_delta, ego_shape, rot_config,
                 execution, online_same_object, external_csv_used,
                 complete_actual_verified, dt=DT):
        require(float(dt) == DT, 'point_one_second_source_required')
        rows = frozen_array(rows6)
        require(rows.ndim == 2 and rows.shape[1] == 6 and
                2 <= len(rows) <= MAX_CHAIN_STEPS + 1, 'complete_source_rows6_required')
        count = len(rows) - 1
        arrays = dict(rows6=rows, commands=frozen_array(commands, (count,)),
                      raw_delta=frozen_array(raw_delta, (count,)),
                      final_delta=frozen_array(final_delta, (count,)))
        require(online_same_object is True and external_csv_used is False and
                complete_actual_verified is True, 'actual_online_source_qualification_required')
        require(getattr(policy, 'tracking', False) is True and
                callable(getattr(policy, 'steering', None)), 'actual_tracking_steering_source_required')
        require(callable(original_process) and isinstance(result, dict) and
                result.get('end') == 4 and result.get('arrival') == rows[-1, 0],
                'complete_actual_source_result_required')
        require(isinstance(ego_shape, dict) and
                math.isfinite(float(ego_shape['length'])) and float(ego_shape['length']) > 0.,
                'actual_source_vehicle_shape_required')
        require(execution.get('pass') is True and execution.get('frame_count') == count
                and len(execution.get('frames', ())) == count,
                'complete_actual_source_execution_required')
        require(getattr(ctx, 'path', None) is path and getattr(ctx, 'ego_shape', None) is ego_shape
                and getattr(ctx, 'dt', None) == dt, 'source_actual_ctx_path_shape_dt_required')
        plan = getattr(ctx, 'plan', None)
        require(isinstance(plan, (list, tuple)) and len(plan) == 3 and plan[0] is policy
                and plan[1] == result.get('score') and plan[2] is result.get('recs'),
                'source_actual_selected_plan_pointer_required')
        rotations = getattr(controller, '_rot_scenes', {})
        selected_rotation = rotations.get(getattr(ctx, 'name', None))
        if selected_rotation is None:
            selected_rotation = rotations.get('_default')
        require(selected_rotation is rot_config, 'source_actual_rotation_object_required')
        for i, packet in enumerate(execution['frames']):
            steering = packet.get('steering', ())
            require(packet.get('pass_frame') is True and
                    packet.get('command_calls') == packet.get('steering_calls') == 1
                    and packet.get('original_process_identity') == id(original_process)
                    and packet.get('actual_path_identity') == id(path)
                    and packet.get('t') == rows[i, 0]
                    and packet.get('input_pose') == rows[i, 1:5].tolist()
                    and packet.get('previous_acceleration_mps2') == rows[i, 5]
                    and packet.get('actual_acceleration_mps2') == rows[i+1, 5]
                    and packet.get('command_mps') == arrays['commands'][i]
                    and packet.get('final_steering_rad') == arrays['final_delta'][i]
                    and len(steering) == 1 and
                    steering[0].get('raw_steering_rad') == arrays['raw_delta'][i],
                    'source_complete_actual_frame_binding_required')
        _state_chain(rows, arrays['commands'], arrays['raw_delta'], arrays['final_delta'],
                     float(ego_shape['length']), float(dt))
        path_signature(path)
        values = dict(ctx=ctx, policy=policy, result=result, path=path,
                      original_process=original_process, controller=controller,
                      ego_shape=ego_shape, rot_config=rot_config, dt=float(dt),
                      online_same_object=True, external_csv_used=False,
                      complete_actual_verified=True, _execution_bytes=_json_bytes(execution),
                      **arrays)
        for key, value in values.items():
            object.__setattr__(self, key, value)

    @property
    def execution(self):
        return json.loads(self._execution_bytes)

    @property
    def vehicle_length(self):
        return float(self.ego_shape['length'])

    def signature(self):
        plan=getattr(self.ctx,'plan',None)
        actual_ctx_binding=dict(path_identity=id(getattr(self.ctx,'path',None)),
            ego_shape_identity=id(getattr(self.ctx,'ego_shape',None)),dt=getattr(self.ctx,'dt',None),
            plan_policy_identity=id(plan[0]) if isinstance(plan,(list,tuple)) and len(plan)==3 else None,
            plan_score=plan[1] if isinstance(plan,(list,tuple)) and len(plan)==3 else None,
            plan_recs_identity=id(plan[2]) if isinstance(plan,(list,tuple)) and len(plan)==3 else None)
        return dict(ids={key: id(getattr(self, key)) for key in
                         ('ctx', 'policy', 'result', 'path', 'original_process',
                          'controller', 'ego_shape', 'rot_config')},
                    arrays={key: array_signature(getattr(self, key)) for key in
                            ('rows6', 'commands', 'raw_delta', 'final_delta')},
                    path=path_signature(self.path), shape=json.loads(_json_bytes(self.ego_shape)),
                    rotation=json.loads(_json_bytes(self.rot_config)),
                    source_result=dict(end=self.result.get('end'),
                        arrival=self.result.get('arrival'), score=self.result.get('score'),
                        recs_identity=id(self.result.get('recs'))),
                    execution_sha256=hashlib.sha256(self._execution_bytes).hexdigest(),
                    policy_content=object_signature(self.policy),
                    result_content=object_signature(self.result),
                    controller_content=object_signature(self.controller),
                    actual_ctx_binding=actual_ctx_binding,
                    dt=self.dt, online_same_object=self.online_same_object,
                    external_csv_used=self.external_csv_used,
                    complete_actual_verified=self.complete_actual_verified)


@dataclass(frozen=True, init=False)
class FrozenFullTraceTableCand:
    source: FullTraceSource
    rows6: np.ndarray
    commands: np.ndarray
    raw_delta: np.ndarray
    final_delta: np.ndarray
    actions: np.ndarray
    model_rows: np.ndarray
    role: str
    label: str
    cut_time: float
    end_time: float
    dt: float
    tracking: bool
    _local_exit_policy: bool
    kind: str

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
            end_time=float(rows[-1, 0]), label='FULL_TRACE_' + role.upper() + '_I1_' + source.policy.label,
            tracking=True, _local_exit_policy=True, kind='F20_FULL_TRACE_' + role.upper(), **arrays)
        for key, value in values.items():
            object.__setattr__(self, key, value)

    def signature(self):
        return dict(identity=id(self), source_identity=id(self.source), role=self.role,
                    arrays={key: array_signature(getattr(self, key)) for key in
                            ('rows6', 'commands', 'raw_delta', 'final_delta', 'actions', 'model_rows')})

    def _index(self, t):
        require(math.isfinite(float(t)), 'finite_table_time_required')
        index = int(round((float(t) - self.cut_time) / self.dt))
        require(0 <= index < len(self.commands) and
                abs(float(t) - self.rows6[index, 0]) <= 1e-8,
                'full_trace_time_outside_complete_table')
        return index

    def __call__(self, ctrl, bank_v, t):
        try:
            error = getattr(ctrl, '_full_trace_error', None)
            require(error is None, 'full_trace_prior_error_not_cleared')
            index = self._index(t)
            pose = tuple(float(getattr(ctrl, key)) for key in
                         ('_ego_x', '_ego_y', '_ego_yaw', '_ego_v'))
            require(all(math.isfinite(v) for v in (*pose, float(ctrl._last_acc))) and
                    np.max(np.abs(np.asarray((*pose, float(ctrl._last_acc))) -
                                  self.rows6[index, 1:])) <= 1e-8,
                    'full_trace_command_pre_six_state_mismatch')
            packet = getattr(ctrl, '_local_exit_frame', None)
            require(not (isinstance(packet, tuple) and len(packet) == 7 and
                packet[0] is self and (not packet[5] or abs(packet[2] - t) <= 1e-8)),
                'full_trace_duplicate_or_unconsumed_command')
            ctrl._local_exit_frame = (self, id(ctrl), float(t), True, pose, False, True)
            return float(self.commands[index])
        except Exception as error:
            if getattr(ctrl,'_full_trace_error',None) is None:
                ctrl._full_trace_error = (self, error)
            raise

    def steering(self, ctrl):
        try:
            require(getattr(ctrl,'_full_trace_error',None) is None,
                    'full_trace_prior_error_not_cleared')
            packet = getattr(ctrl, '_local_exit_frame', None)
            pose = tuple(float(getattr(ctrl, key)) for key in
                         ('_ego_x', '_ego_y', '_ego_yaw', '_ego_v'))
            require(isinstance(packet, tuple) and len(packet) == 7 and
                    packet[0] is self and packet[1] == id(ctrl) and packet[4] == pose
                    and packet[5] is False and packet[6] is True,
                    'full_trace_steering_requires_same_unique_command')
            index = self._index(packet[2])
            ctrl._local_exit_frame = packet[:5] + (True, True)
            return float(self.raw_delta[index])
        except Exception as error:
            if getattr(ctrl,'_full_trace_error',None) is None:
                ctrl._full_trace_error = (self, error)
            raise


def raise_saved_policy_error(ctrl, candidate):
    """原 process 返回后、plant 前调用；不能把吞异常 fallback 当成政策。"""
    error = getattr(ctrl, '_full_trace_error', None)
    if error is not None:
        require(isinstance(error, tuple) and len(error) == 2 and error[0] is candidate,
                'foreign_full_trace_policy_error')
        raise error[1]


@dataclass(frozen=True, init=False)
class IdentityReceipt:
    candidate: FrozenFullTraceTableCand
    actual_result: dict
    new_complete_rollouts: int
    _source_signature_bytes: bytes
    _verification_bytes: bytes
    _actual_result_signature_bytes: bytes

    def __init__(self, source_signature, candidate, actual_result, verification,
                 new_complete_rollouts=1):
        object.__setattr__(self, 'candidate', candidate)
        object.__setattr__(self, 'actual_result', actual_result)
        object.__setattr__(self, 'new_complete_rollouts', new_complete_rollouts)
        object.__setattr__(self, '_source_signature_bytes', _json_bytes(source_signature))
        object.__setattr__(self, '_verification_bytes', _json_bytes(verification))
        object.__setattr__(self, '_actual_result_signature_bytes',
                           _json_bytes(object_signature(actual_result)))

    @property
    def source_signature(self):
        return json.loads(self._source_signature_bytes)

    @property
    def verification(self):
        return json.loads(self._verification_bytes)

    def valid_for(self, source):
        required = ('raw_steering', 'final_steering', 'commands', 'P2_last_acc',
                    'six_state', 'path_bytes', 'first_goal', 'terminal', 'fullsafe',
                    'full_ALAT', 'original_process', 'identity_policy_execution')
        try:
            return bool(isinstance(source,FullTraceSource)
                    and isinstance(self.candidate,FrozenFullTraceTableCand)
                    and isinstance(self.actual_result,dict)
                    and self.new_complete_rollouts == 1 and self.candidate.source is source
                    and self.candidate.role == 'identity'
                    and self.source_signature == source.signature()
                    and all(self.verification.get(key) is True for key in required)
                    and self.verification.get('candidate_signature') == self.candidate.signature()
                    and self.verification.get('frame_count') == len(source.commands)
                    and self.actual_result is not source.result
                    and self._actual_result_signature_bytes == _json_bytes(object_signature(self.actual_result))
                    and self.actual_result.get('end') == 4
                    and self.actual_result.get('arrival') == source.rows6[-1, 0])
        except (FullTraceRefusal,ValueError,TypeError,AttributeError):
            return False
