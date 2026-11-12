"""在同一空间包络上补偿正、负加速度的研究政策；不改变转向。

空间速度需要 a = .5*d(v**2)/ds，原控制器的 P 增益为 2。
研究只把 v_cmd = v_cap + a/2 交给原加速度、jerk 与转向链，
没有速度倍率或参数网格。完整同码预测和真实 Env 仍是晋级门。
"""

import numpy as np

from executed_retiming import V_MAX, V_MIN, build_profile
from executed_retiming_ff import ExecutedRetimingFeedforwardCand


class ExecutedRetimingFullFeedforwardCand(ExecutedRetimingFeedforwardCand):
    """保持原投影及几何对象，允许加速段消费同一包络的斜率。"""

    kind = 'F9_EXECUTED_RETIMING_FULL_FF'

    def __init__(self, base, profile, iteration):
        super().__init__(base, profile, iteration)
        self.label = f'EXEC_RETIME_FULLFF_I{iteration}_' + base.label

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
        return float(np.clip(cap + acceleration / 2., V_MIN, V_MAX))


def build_candidate(base, executed_rows, iteration=1):
    profile, audit = build_profile(executed_rows)
    candidate = ExecutedRetimingFullFeedforwardCand(base, profile, iteration)
    audit.update(candidate=candidate.label, iteration=iteration,
                 steering_source=base.label, steering_delegated=True,
                 tracking_preserved=True, reference_uses_external_csv=False,
                 acceleration_feedforward='positive_and_negative',
                 controller_p_gain=2.,
                 feedforward_formula='selected_cap + .5*d(cap**2)/ds / 2',
                 selected_station='current_or_ahead_whichever_cap_is_smaller',
                 reference_acceleration_mps2=candidate._retime_acceleration.tolist(),
                 production_integration_verified=False)
    return candidate, audit
