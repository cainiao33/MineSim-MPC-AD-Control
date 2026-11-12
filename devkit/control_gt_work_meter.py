"""F38 traffic proposal rows retain11D vector charges and the288000 cap."""

from control_segmented_work_meter import SegmentedWorkMeter, MAX_SCALAR_STEPS


class GTWorkMeter(SegmentedWorkMeter):
    @staticmethod
    def work_units(kind,count=1):
        if kind=='qp_GT_projection_rows':
            return SegmentedWorkMeter.work_units('qp_constraint_rows_formed',count)
        return SegmentedWorkMeter.work_units(kind,count)
