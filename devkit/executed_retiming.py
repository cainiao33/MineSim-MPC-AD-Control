"""按真实执行曲率重定时；不改变在位政策转向、不读取外部轨迹。

输入行(t,x,y,yaw,v)使用地图米制坐标、弧度及m/s。实际曲率来自
|dyaw/dt|/v，限速sqrt(.39/k)，空间前后2.5m/s²包络。没有速度倍率网格，
不要求原效率满分或已经产生侧加速度扣分。安全及得分仍由完整闭环验证。
"""

import math

import numpy as np


ALAT_LIMIT = .39
A_ACC = 2.5
A_DEC = 2.5
V_MAX = 12.
V_MIN = .1
SPACING_M = .5


def build_profile(executed_rows):
    """从完整本次执行轨迹生成只读空间速度表及可核验审计。"""
    rows = np.asarray(executed_rows, dtype=float)
    if rows.ndim != 2 or rows.shape[1] != 5 or len(rows) < 5 or not np.isfinite(rows).all():
        raise ValueError('实际执行参考须为至少5行有限(t,x,y,yaw,v)')
    # 评分使用recorder精度，曲率估计也直接采用这组实际被评分的状态。
    rows = rows.copy()
    rows[:, 0] = np.round(rows[:, 0], 1)
    rows[:, 1:] = np.round(rows[:, 1:], 4)
    times, xy = rows[:, 0], rows[:, 1:3]
    heading, speed = np.unwrap(rows[:, 3]), rows[:, 4]
    if np.any(np.diff(times) <= 0.) or np.any(speed < 0.):
        raise ValueError('实际执行时间须严格递增，速度非负')
    yaw_rate = np.gradient(heading, times)
    curvature = np.abs(yaw_rate) / np.maximum(speed, 1e-6)
    distances = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    stations = np.r_[0., np.cumsum(distances)]
    if stations[-1] < 1.:
        raise ValueError('参考实际行驶距离不足1米')
    groups = np.r_[0, np.flatnonzero(np.diff(stations) > 1e-6) + 1]
    unique_s = stations[groups]
    # 原停车速度不复制到空间表；同位置多帧取最大实际曲率，避免隐藏风险。
    unique_k = np.maximum.reduceat(curvature, groups)
    grid = np.arange(0., unique_s[-1], SPACING_M)
    grid = np.r_[grid, unique_s[-1]]
    kappa = np.interp(grid, unique_s, unique_k)
    desired = np.clip(np.sqrt(ALAT_LIMIT / np.maximum(kappa, 1e-9)), V_MIN, V_MAX)
    squared = desired ** 2
    squared[0] = min(squared[0], max(V_MIN, float(speed[0])) ** 2)
    for index in range(len(grid) - 2, -1, -1):
        squared[index] = min(squared[index], squared[index + 1]
                             + 2. * A_DEC * (grid[index + 1] - grid[index]))
    for index in range(1, len(grid)):
        squared[index] = min(squared[index], squared[index - 1]
                             + 2. * A_ACC * (grid[index] - grid[index - 1]))
    caps = np.sqrt(np.maximum(squared, V_MIN ** 2))
    profile_xy = np.column_stack([np.interp(grid, unique_s, xy[groups, axis])
                                  for axis in (0, 1)])
    profile_heading = np.interp(grid, unique_s, heading[groups])
    reference_speed = np.interp(grid, unique_s, speed[groups])
    profile_time = float(np.sum(np.diff(grid) / np.maximum(
        V_MIN, np.minimum(caps[:-1], caps[1:]))))
    actual_alat = np.abs(speed * yaw_rate)
    audit = dict(curvature_source='abs_gradient_actual_recorder_yaw_per_time_div_actual_speed',
                 input_rows=len(rows), reference_arrival_s=float(times[-1]),
                 reference_length_m=float(stations[-1]), max_actual_curvature_1pm=float(curvature.max()),
                 max_actual_alat_mps2=float(actual_alat.max()),
                 reference_start_speed_mps=float(speed[0]),
                 speed_replacement=True, min_with_base_speed=False, geometry_changed=False,
                 alat_limit_mps2=ALAT_LIMIT, acceleration_mps2=A_ACC,
                 deceleration_mps2=A_DEC, spacing_m=SPACING_M,
                 minimum_cap_mps=float(caps.min()), max_cap_mps=float(caps.max()),
                 geometric_profile_time_s=profile_time,
                 faster_station_count=int(np.sum(caps > reference_speed + .01)),
                 slower_station_count=int(np.sum(caps < reference_speed - .01)),
                 max_increase_over_executed_speed_mps=float(np.max(caps - reference_speed)),
                 stations_m=grid.tolist(), caps_mps=caps.tolist(),
                 actual_curvature_1pm=kappa.tolist())
    return dict(stations=grid, xy=profile_xy, heading=profile_heading, caps=caps), audit


class ExecutedRetimingCand:
    """原几何转向对象保持不变，速度直接消费实际曲率空间包络。"""

    kind = 'F9_EXECUTED_RETIMING'
    tracking = True

    def __init__(self, base, profile, iteration):
        if not getattr(base, 'tracking', False) or not callable(getattr(base, 'steering', None)):
            raise ValueError('执行重定时只适用于在位tracking几何政策')
        self.base = base
        self.iteration = iteration
        self.label = f'EXEC_RETIME_I{iteration}_' + base.label
        self._retime_s = np.array(profile['stations'], dtype=float, copy=True)
        self._retime_x = np.array(profile['xy'][:, 0], dtype=float, copy=True)
        self._retime_y = np.array(profile['xy'][:, 1], dtype=float, copy=True)
        self._retime_heading = np.array(profile['heading'], dtype=float, copy=True)
        self._retime_caps = np.array(profile['caps'], dtype=float, copy=True)
        for value in (self._retime_s, self._retime_x, self._retime_y,
                      self._retime_heading, self._retime_caps):
            value.setflags(write=False)

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        # steering直接委托原对象；副本中的原几何progress状态不被包装投影覆盖。
        return getattr(object.__getattribute__(self, 'base'), name)

    def _progress(self, ctrl):
        if getattr(ctrl, '_executed_retime_token', None) is not self:
            ctrl._executed_retime_token = self
            ctrl._executed_retime_s = None
        previous = ctrl._executed_retime_s
        if previous is None:
            indices = np.arange(len(self._retime_s))
        else:
            indices = np.flatnonzero((self._retime_s >= previous - 3.)
                                     & (self._retime_s <= previous + 25.))
        if not len(indices):
            indices = np.arange(len(self._retime_s))
        distance = ((self._retime_x[indices] - ctrl._ego_x) ** 2
                    + (self._retime_y[indices] - ctrl._ego_y) ** 2)
        if previous is None:
            angle = (self._retime_heading[indices] - ctrl._ego_yaw + np.pi) % (2. * np.pi) - np.pi
            distance += 9. * angle ** 2
        near = indices[int(np.argmin(distance))]
        heading = self._retime_heading[near]
        along = ((ctrl._ego_x - self._retime_x[near]) * math.cos(heading)
                 + (ctrl._ego_y - self._retime_y[near]) * math.sin(heading))
        station = float(np.clip(self._retime_s[near] + along, 0., self._retime_s[-1]))
        ctrl._executed_retime_s = station
        return station

    def __call__(self, ctrl, bank_v, t):
        if t < float(getattr(self.base, 'delay', 0.)):
            return V_MIN
        station = self._progress(ctrl)
        ahead = min(3., .4 * max(0., float(ctrl._ego_v)))
        # 本地与真实Env共用当前位置及前方制动约束。这里没有base_v上界，
        # 因此在实际曲率小于原保守几何曲率的位置允许提速。
        cap = min(float(np.interp(station, self._retime_s, self._retime_caps)),
                  float(np.interp(station + ahead, self._retime_s, self._retime_caps)))
        return float(np.clip(cap, V_MIN, V_MAX))


def build_candidate(base, executed_rows, iteration=1):
    profile, audit = build_profile(executed_rows)
    candidate = ExecutedRetimingCand(base, profile, iteration)
    audit.update(candidate=candidate.label, iteration=iteration,
                 steering_source=base.label, steering_delegated=True,
                 tracking_preserved=True, trigger='safe_tracking_incumbent_any_efficiency',
                 reference_uses_external_csv=False)
    return candidate, audit


def profile_delta(previous, current):
    """在两次真实轨迹共同空间范围比较速度包络，仅用于收敛停止。"""
    maximum = min(float(previous._retime_s[-1]), float(current._retime_s[-1]))
    grid = np.arange(0., maximum, SPACING_M)
    if not len(grid):
        raise ValueError('迭代空间交集为空')
    delta = (np.interp(grid, current._retime_s, current._retime_caps)
             - np.interp(grid, previous._retime_s, previous._retime_caps))
    return dict(max_abs_speed_change_mps=float(np.max(np.abs(delta))),
                rms_speed_change_mps=float(np.sqrt(np.mean(delta ** 2))),
                common_distance_m=maximum)
