"""实际曲率重定时的负加速度前馈；速度表及转向沿用原重定时机制。

原真实控制链加速度为clip(2*(v_cmd-v), -6, 2.5)。空间速度表的
a_ref=.5*d(v_cap²)/ds属于植物需要兑现的加速度。因此减速段消费
v_cmd=v_cap+min(0,a_ref)/2，使原P链同时消费负前馈，避免速度追踪滞后。
正前馈为零；这是一种控制执行机制，不搜索额外速度倍率。
"""

import numpy as np

from executed_retiming import (
    A_ACC, A_DEC, ExecutedRetimingCand, V_MAX, V_MIN, build_profile,
)


class ExecutedRetimingFeedforwardCand(ExecutedRetimingCand):
    """在原同码P速度链中兑现选中空间限速位置的负加速度斜率。"""

    kind = 'F9_EXECUTED_RETIMING_NEGATIVE_FF'

    def __init__(self, base, profile, iteration):
        super().__init__(base, profile, iteration)
        self.label = f'EXEC_RETIME_NEGFF_I{iteration}_' + base.label
        self._retime_acceleration = np.clip(
            .5 * np.gradient(self._retime_caps ** 2, self._retime_s), -A_DEC, A_ACC)
        self._retime_acceleration.setflags(write=False)

    def __call__(self, ctrl, bank_v, t):
        if t < float(getattr(self.base, 'delay', 0.)):
            return V_MIN
        station = self._progress(ctrl)
        ahead = min(3., .4 * max(0., float(ctrl._ego_v)))
        future_station = min(float(self._retime_s[-1]), station + ahead)
        current_cap = float(np.interp(station, self._retime_s, self._retime_caps))
        ahead_cap = float(np.interp(future_station, self._retime_s, self._retime_caps))
        selected_station, cap = ((station, current_cap) if current_cap <= ahead_cap
                                 else (future_station, ahead_cap))
        acceleration = float(np.interp(selected_station, self._retime_s,
                                       self._retime_acceleration))
        # 原控制器P增益固定2；只补偿负斜率，原jerk和加速度钳链继续同码消费。
        command = cap + min(0., acceleration) / 2.
        return float(np.clip(command, V_MIN, V_MAX))


def build_candidate(base, executed_rows, iteration=1):
    profile, audit = build_profile(executed_rows)
    candidate = ExecutedRetimingFeedforwardCand(base, profile, iteration)
    audit.update(candidate=candidate.label, iteration=iteration,
                 steering_source=base.label, steering_delegated=True,
                 tracking_preserved=True, trigger='safe_tracking_incumbent_any_efficiency',
                 reference_uses_external_csv=False,
                 acceleration_feedforward='negative_only', controller_p_gain=2.,
                 feedforward_formula='selected_cap + min(0, .5*d(cap**2)/ds) / 2',
                 acceleration_clip_mps2=[-A_DEC, A_ACC],
                 selected_station='current_or_ahead_whichever_cap_is_smaller',
                 reference_acceleration_mps2=candidate._retime_acceleration.tolist())
    return candidate, audit
