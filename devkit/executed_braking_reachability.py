"""当前在线P/jerk最低命令制动尾的有界提案；只生成一个完整政策。

状态为(s,v,上一帧a)，位移使用旧v*dt。速度命令下限.1使低速制动
弱于常量-6；不把A_DEC=6当可达性证明。内存路线/曲率只作提案，
转向委托原base，最终仍须完整同码、足迹、原HYST和真实Env验收。
"""

import math
import time

import numpy as np

from executed_retiming import (
    ALAT_LIMIT, SPACING_M, V_MAX, V_MIN, ExecutedRetimingCand, build_profile,
)


A_MIN, A_MAX = -6., 2.5
JERK_STEP = 1.
P_GAIN = 2.
DT = .1
MAX_BRAKE_STEPS = 160
MAX_COMMAND_ROOT_STEPS = 14
MAX_SHADOW_STEPS = 660
MAX_STATIC_SCALAR_STEPS = 500000


class BrakingRefusal(Exception):
    pass


class BrakingExecutionViolation(RuntimeError):
    pass


def execution_audit_valid(result, candidate=None, expected_frames=None):
    """父/子进程均须验证完整执行合同；缺少审计直接撤销资格。"""
    if not isinstance(result, dict):
        return False
    audit = result.get('braking_execution_audit')
    if not isinstance(audit, dict):
        return False
    if (audit.get('contract_version') != 'original_track_p_clip_jerk_v1'
            or audit.get('pass') is not True):
        return False
    if candidate is not None and audit.get('candidate') != candidate.label:
        return False
    frames = audit.get('frame_count')
    if type(frames) is not int or frames <= 0:
        return False
    if expected_frames is not None and frames != expected_frames:
        return False
    if type(audit.get('expected_frames')) is not int or audit['expected_frames'] != frames:
        return False
    for name in ('command_count', 'unique_command_verified_frames',
                 'original_p_clip_jerk_verified_frames', 'track_geometry_verified_frames',
                 'stpp_clear_verified_frames'):
        if type(audit.get(name)) is not int or audit[name] != frames:
            return False
    for name in ('command_refusals', 'failed_frames', 'fallback_frames'):
        if type(audit.get(name)) is not int or audit[name] != 0:
            return False
    try:
        delta = float(audit['max_acceleration_delta'])
        return math.isfinite(delta) and 0. <= delta <= 1e-12
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def speed_step(speed, previous_acceleration, command, dt=DT):
    """实际track P/acc/每帧jerk及植物旧速度位移的完整标量合同。"""
    requested = max(A_MIN, min(A_MAX, P_GAIN * (command - speed)))
    acceleration = max(previous_acceleration - JERK_STEP,
                       min(previous_acceleration + JERK_STEP, requested))
    return max(0., speed + acceleration * dt), acceleration, speed * dt


class _Budget:
    def __init__(self, seconds):
        self.started, self.seconds, self.steps = time.perf_counter(), seconds, 0

    def tick(self):
        self.steps += 1
        if self.steps > MAX_STATIC_SCALAR_STEPS:
            raise BrakingRefusal('static_scalar_step_cap')
        if time.perf_counter() - self.started >= self.seconds:
            raise BrakingRefusal('static_budget_insufficient')


def _segment_limit(stations, limits, begin, end):
    """保守覆盖每次旧速位移跨过的所有空间采样点及端点。"""
    first = int(np.searchsorted(stations, begin, side='left'))
    last = int(np.searchsorted(stations, end, side='right'))
    value = min(float(np.interp(begin, stations, limits)),
                float(np.interp(end, stations, limits)))
    if last > first:
        value = min(value, float(np.min(limits[first:last])))
    return value


def minimum_command_tail(station, speed, previous_acceleration, first_command,
                         stations, limits, dt=DT, budget=None):
    """核验一个首命令，之后固定最低命令；不是全局可达集合或最优证明。

当v已不高于全路线最小限速且a<=0，最低命令P尾不会再越过该限速，
可有限停止检查。空间末端本步合格可停止，步数耗尽则拒绝。
"""
    global_minimum = float(np.min(limits))
    for step in range(MAX_BRAKE_STEPS):
        if budget is not None:
            budget.tick()
        if speed > float(np.interp(station, stations, limits)) + 1e-9:
            return False, dict(reason='current_scalar_speed_over_curvature_limit',steps=step)
        if step > 0 and speed <= global_minimum + 1e-9 and previous_acceleration <= 0.:
            return True, dict(reason='minimum_command_tail_below_global_limit',steps=step,
                              terminal_station_m=station,terminal_speed_mps=speed,
                              terminal_acceleration_mps2=previous_acceleration)
        command = first_command if step == 0 else V_MIN
        next_speed, acceleration, displacement = speed_step(speed,previous_acceleration,command,dt)
        next_station = min(float(stations[-1]), station + displacement)
        if max(speed,next_speed) > _segment_limit(stations,limits,station,next_station) + 1e-9:
            return False, dict(reason='scalar_displacement_crosses_speed_limit',steps=step+1)
        station,speed,previous_acceleration = next_station,next_speed,acceleration
        if station >= float(stations[-1]) - 1e-9:
            # 路线末端没有更多约束；末端速度仍须在当前限速内。
            return True, dict(reason='memory_route_end_within_scalar_limit',steps=step+1,
                              terminal_station_m=station,terminal_speed_mps=speed,
                              terminal_acceleration_mps2=previous_acceleration)
    return False, dict(reason='minimum_command_tail_finite_step_cap',steps=MAX_BRAKE_STEPS)


class ExecutedBrakingReachabilityCand(ExecutedRetimingCand):
    kind = 'F10_EXECUTED_BRAKING_REACHABILITY'
    _braking_execution_guard = True

    def __init__(self,base,profile,iteration=1,dt=DT):
        super().__init__(base,profile,iteration)
        self.dt = float(dt)
        self.label = f'EXEC_BRAKE_REACH_I{iteration}_' + base.label
        self._reach_acceleration = np.clip(
            .5*np.gradient(self._retime_caps**2,self._retime_s),A_MIN,A_MAX)
        self._reach_acceleration.setflags(write=False)

    def execution_audit_begin(self, ctrl):
        # 副本从真实ctrl浅拷贝而来；每次rollout新建dict，绝不写candidate。
        ctrl._braking_execution_state = dict(
            candidate=self, controller_id=id(ctrl), frame_count=0, command_count=0,
            command_refusals=0, failed_frames=0, fallback_frames=0,
            unique_command_verified_frames=0, original_p_clip_jerk_verified_frames=0,
            track_geometry_verified_frames=0, stpp_clear_verified_frames=0,
            max_acceleration_delta=0., frame_active=False, last_command=None)

    def execution_audit_before_frame(self, ctrl, t, speed, previous_acceleration):
        state = ctrl._braking_execution_state
        if (not isinstance(state, dict) or state.get('candidate') is not self
                or state.get('controller_id') != id(ctrl) or state.get('frame_active')):
            raise BrakingExecutionViolation('invalid_or_nested_execution_frame')
        state.update(frame_active=True, frame_t=float(t), frame_speed=float(speed),
                     frame_previous_acceleration=float(previous_acceleration),
                     frame_command_start=state['command_count'], last_command=None)

    def command(self, ctrl, bank_v, t, budget=None):
        state = getattr(ctrl, '_braking_execution_state', None)
        packet = None
        if isinstance(state, dict) and state.get('frame_active'):
            packet = dict(candidate=self, controller_id=id(ctrl), t=float(t),
                          speed=float(ctrl._ego_v),
                          previous_acceleration=float(ctrl._last_acc), accepted=False)
            state['command_count'] += 1
            state['last_command'] = packet
        try:
            value = self._command_impl(ctrl, bank_v, t, budget)
            if packet is not None:
                packet.update(accepted=True, command=float(value))
            return value
        except Exception:
            if packet is not None:
                state['command_refusals'] += 1
            raise

    def execution_audit_after_frame(self, ctrl, acceleration):
        state = ctrl._braking_execution_state
        packet = state.get('last_command')
        state['frame_count'] += 1
        unique = (state['command_count'] - state['frame_command_start'] == 1
                  and isinstance(packet, dict) and packet.get('accepted') is True
                  and packet.get('candidate') is self and packet.get('controller_id') == id(ctrl)
                  and packet.get('t') == state['frame_t']
                  and packet.get('speed') == state['frame_speed']
                  and packet.get('previous_acceleration') == state['frame_previous_acceleration'])
        track = (getattr(ctrl, '_sas_cand', None) is self
                 and getattr(ctrl, '_sas_geometry_candidate', None) is self
                 and getattr(ctrl, '_sas_exec_policy', None) == 'track')
        stpp_clear = getattr(ctrl, '_stpp_exec_v', None) is None
        scalar = False
        if unique:
            command = packet['command']
            _, expected, _ = speed_step(state['frame_speed'],
                                       state['frame_previous_acceleration'], command, self.dt)
            actual = float(acceleration)
            delta = abs(expected - actual)
            scalar = (math.isfinite(command) and V_MIN <= command <= V_MAX
                      and math.isfinite(actual) and delta <= 1e-12
                      and float(ctrl._last_acc) == actual)
            if math.isfinite(delta):
                state['max_acceleration_delta'] = max(state['max_acceleration_delta'], delta)
        for name, passed in (('unique_command_verified_frames', unique),
                             ('original_p_clip_jerk_verified_frames', scalar),
                             ('track_geometry_verified_frames', track),
                             ('stpp_clear_verified_frames', stpp_clear)):
            state[name] += int(passed)
        state['frame_active'] = False
        if not (unique and scalar and track and stpp_clear):
            state['failed_frames'] += 1
            state['fallback_frames'] += int(not track or not stpp_clear
                                           or not isinstance(packet, dict)
                                           or packet.get('accepted') is not True)
            raise BrakingExecutionViolation('command_or_original_track_chain_contract_failed')

    def execution_audit_finish(self, ctrl, expected_frames):
        state = ctrl._braking_execution_state
        if state.get('frame_active'):
            raise BrakingExecutionViolation('unfinished_execution_frame')
        keys = ('frame_count', 'command_count', 'command_refusals', 'failed_frames',
                'fallback_frames', 'unique_command_verified_frames',
                'original_p_clip_jerk_verified_frames', 'track_geometry_verified_frames',
                'stpp_clear_verified_frames', 'max_acceleration_delta')
        audit = {name: state[name] for name in keys}
        audit.update(contract_version='original_track_p_clip_jerk_v1', candidate=self.label,
                     expected_frames=expected_frames,
                     execution_only_not_safety_certificate=True)
        audit['pass'] = True
        if not execution_audit_valid({'braking_execution_audit': audit}, self, expected_frames):
            raise BrakingExecutionViolation('incomplete_execution_contract_aggregate')
        return audit

    def execution_audit_reset(self, ctrl):
        ctrl._braking_execution_state = None

    def _command_impl(self,ctrl,bank_v,t,budget=None):
        speed,previous_acceleration = float(ctrl._ego_v),float(ctrl._last_acc)
        if (not math.isfinite(speed) or not math.isfinite(previous_acceleration)
                or speed < 0. or not A_MIN <= previous_acceleration <= A_MAX):
            raise BrakingRefusal('current_track_scalar_state_outside_contract')
        if not all(math.isfinite(float(value)) for value in
                   (ctrl._ego_x,ctrl._ego_y,ctrl._ego_yaw,t)):
            raise BrakingRefusal('current_track_geometry_or_time_nonfinite')
        station = self._progress(ctrl)
        ahead = min(float(self._retime_s[-1]),station+min(3.,.4*speed))
        current_cap = float(np.interp(station,self._retime_s,self._retime_caps))
        ahead_cap = float(np.interp(ahead,self._retime_s,self._retime_caps))
        selected,cap = (station,current_cap) if current_cap<=ahead_cap else (ahead,ahead_cap)
        acceleration = float(np.interp(selected,self._retime_s,self._reach_acceleration))
        nominal = float(np.clip(cap+acceleration/P_GAIN,V_MIN,V_MAX))
        if t < float(getattr(self.base,'delay',0.)):
            nominal = V_MIN
        feasible,_ = minimum_command_tail(station,speed,previous_acceleration,nominal,
                                          self._retime_s,self._retime_caps,self.dt,budget)
        if feasible:
            return nominal
        feasible,reason = minimum_command_tail(station,speed,previous_acceleration,V_MIN,
                                               self._retime_s,self._retime_caps,self.dt,budget)
        if not feasible:
            raise BrakingRefusal('minimum_command_not_viable:'+reason['reason'])
        # 每个被保留的low命令都经过逐步验证。非单调时可能遗漏更高可行命令；
        # 不宣称求得全局最大命令，也不按搜索点生成多条完整候选政策。
        low,high = V_MIN,nominal
        for _ in range(MAX_COMMAND_ROOT_STEPS):
            middle = .5*(low+high)
            feasible,_ = minimum_command_tail(station,speed,previous_acceleration,middle,
                                              self._retime_s,self._retime_caps,self.dt,budget)
            if feasible:
                low = middle
            else:
                high = middle
        return low

    def __call__(self,ctrl,bank_v,t):
        return self.command(ctrl,bank_v,t)


def _raw_profile(rows):
    """继承原米制曲率和几何采样，仅去除固定2.5空间减速假设。"""
    profile,baseline = build_profile(rows)
    stations = profile['stations']
    curvature = np.asarray(baseline['actual_curvature_1pm'],dtype=float)
    desired = np.clip(np.sqrt(ALAT_LIMIT/np.maximum(curvature,1e-9)),V_MIN,V_MAX)
    # 仍保持原正向2.5空间加速包络；它只生成nominal，实际jerk由状态门消费。
    squared = desired**2
    squared[0] = min(squared[0],max(V_MIN,float(rows[0][4]))**2)
    for index in range(1,len(stations)):
        squared[index] = min(squared[index],squared[index-1]
                             +2.*A_MAX*(stations[index]-stations[index-1]))
    profile['caps'] = np.sqrt(np.maximum(squared,V_MIN**2))
    return profile,baseline


def build_candidate(base,executed_rows,iteration=1,*,initial_acceleration=None,
                    dt=DT,max_duration_s=66.,budget_s=8.):
    audit = dict(mechanism='state_dependent_minimum_command_braking_tail',policy_count=0,
                 accepted=False,reference_uses_external_csv=False,
                 memory_source='current_state_plus_strict_safe_online_future_suffix',parameter_grid=False,
                 complete_candidate_limit=1,constant_a_dec_6_envelope_used=False,
                 shadow_is_physical_rollout=False,shadow_is_safety_certificate=False,
                 global_optimality_claimed=False,predicate_monotonicity_claimed=False,
                 speed_command_floor_mps=V_MIN,track_acceleration_bounds_mps2=[A_MIN,A_MAX],
                 jerk_per_frame_mps2=JERK_STEP,controller_p_gain=P_GAIN,
                 max_brake_tail_steps=MAX_BRAKE_STEPS,command_root_steps=MAX_COMMAND_ROOT_STEPS,
                 max_static_scalar_steps=MAX_STATIC_SCALAR_STEPS,static_budget_s=budget_s,
                 full_control_chain_rollouts=0,production_integration_verified=False)
    budget = _Budget(budget_s)
    try:
        if (iteration!=1 or not math.isfinite(float(dt)) or abs(dt-DT)>1e-12
                or not 0.<budget_s<=8.):
            raise BrakingRefusal('one_policy_dt_0p1_and_bounded_budget_required')
        if not math.isfinite(float(max_duration_s)) or not 0.<max_duration_s<=66.:
            raise BrakingRefusal('bounded_scalar_shadow_duration_required')
        if initial_acceleration is None or not math.isfinite(float(initial_acceleration)):
            raise BrakingRefusal('missing_actual_initial_controller_acceleration')
        if not A_MIN<=float(initial_acceleration)<=A_MAX:
            raise BrakingRefusal('actual_initial_acceleration_outside_track_contract')
        rows = np.asarray(executed_rows,dtype=float)
        profile,baseline = _raw_profile(rows)
        if float(np.min(profile['caps']))<=V_MIN+1e-6:
            raise BrakingRefusal('speed_floor_or_stop_constraints_outside_finite_tail_scope')
        improvement = profile['caps']-np.asarray(baseline['caps_mps'])
        audit.update(raw_curvature_proposal=dict(
            reference_length_m=baseline['reference_length_m'],input_rows=baseline['input_rows'],
            minimum_cap_mps=float(np.min(profile['caps'])),maximum_cap_mps=float(np.max(profile['caps'])),
            stations_above_original_2p5_envelope=int(np.sum(improvement>.002)),
            maximum_nominal_increase_mps=float(np.max(improvement))))
        if not np.any(improvement>.002):
            raise BrakingRefusal('no_larger_nominal_braking_envelope_in_this_memory')
        candidate = ExecutedBrakingReachabilityCand(base,profile,iteration,dt)
        replica = type('ScalarState',(),{})()
        station,speed,acceleration,t = 0.,float(rows[0,4]),float(initial_acceleration),float(rows[0,0])
        shadow,commands = [rows[0].tolist()],[]
        for _ in range(MAX_SHADOW_STEPS):
            budget.tick()
            if station>=float(candidate._retime_s[-1])-1e-9:
                break
            if t-float(rows[0,0])>=max_duration_s-1e-9:
                raise BrakingRefusal('scalar_shadow_duration_cap')
            replica._ego_x = float(np.interp(station,candidate._retime_s,candidate._retime_x))
            replica._ego_y = float(np.interp(station,candidate._retime_s,candidate._retime_y))
            replica._ego_yaw = float(np.interp(station,candidate._retime_s,candidate._retime_heading))
            replica._ego_v,replica._last_acc = speed,acceleration
            command = candidate.command(replica,V_MAX,t,budget)
            speed,acceleration,displacement = speed_step(speed,acceleration,command,dt)
            station = min(float(candidate._retime_s[-1]),station+displacement)
            t += dt
            commands.append(command)
            shadow.append([t,float(np.interp(station,candidate._retime_s,candidate._retime_x)),
                           float(np.interp(station,candidate._retime_s,candidate._retime_y)),
                           float(np.interp(station,candidate._retime_s,candidate._retime_heading)),speed])
        else:
            if station<float(candidate._retime_s[-1])-1e-9:
                raise BrakingRefusal('scalar_shadow_step_cap')
        # 这只是有限族的diagnostic；不以影子到达时间代替HYST/实际评分。
        audit.update(accepted=True,reason='bounded_scalar_braking_tail_proposal',policy_count=1,
                     candidate=candidate.label,initial_acceleration_mps2=float(initial_acceleration),
                     dt_s=dt,steering_source=base.label,steering_delegated=True,
                     acceleration_feedforward='positive_and_negative_plus_state_viability_filter',
                     reference_arrival_s=float(rows[-1,0]),shadow_arrival_s=t,
                     shadow_rows=shadow,shadow_commands_mps=commands,
                     stations_m=candidate._retime_s.tolist(),nominal_caps_mps=candidate._retime_caps.tolist(),
                     nominal_feedforward_mps2=candidate._reach_acceleration.tolist(),
                     scalar_steps=budget.steps,elapsed_s=time.perf_counter()-budget.started)
        return candidate,audit
    except BrakingRefusal as error:
        audit.update(reason=str(error),scalar_steps=budget.steps,
                     elapsed_s=time.perf_counter()-budget.started)
        return None,audit
    except Exception as error:
        audit.update(reason='proposal_exception',error=dict(type=type(error).__name__,message=str(error)),
                     scalar_steps=budget.steps,elapsed_s=time.perf_counter()-budget.started)
        return None,audit
