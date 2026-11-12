"""F39 bounded three-pass SQP work, without changing earlier meters."""

from control_gt_work_meter import GTWorkMeter
from control_native_bank_phase import SearchStop


MAX_SCALAR_STEPS = 864000


class SQPWorkMeter(GTWorkMeter):
    @staticmethod
    def work_units(kind, count=1):
        if kind in ('sqp_recenter_algebra_checks', 'sqp_recenter_sensitivity_updates',
                    'sqp_recenter_sensitivity_retention', 'sqp_center_diagnostic_rows'):
            return GTWorkMeter.work_units('source_sensitivity_retention', count)
        return GTWorkMeter.work_units(kind, count)

    def charge(self, kind, count=1):
        self.check('before_' + kind)
        units = self.work_units(kind, count)
        if self.scalar_steps + units > MAX_SCALAR_STEPS:
            raise SearchStop('sqp_work_step_cap')
        self.scalar_steps += units
        self.counts[kind] = self.counts.get(kind, 0) + units
