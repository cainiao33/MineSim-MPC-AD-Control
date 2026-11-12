"""F35独立工作预算：11维向量按三个原5维单位计，矩阵按五个。

单次QP从16增至48轮，声明整体288000/优化180000/source9000单位。
8秒总时钟及1.5秒最终检查预留继承原Meter；旧计量器不改。
每次KKT分解另外按ceil((11+active)^3/10^3)原10阶单位计费。
"""

from control_native_bank_phase import Meter, SearchStop, require

MAX_SCALAR_STEPS = 288000
VECTOR_WORK_UNITS = 3
MATRIX_WORK_UNITS = 5
VECTOR_KINDS = frozenset((
    'source_old_speed_P2_rot_algebra_checks',
    'source_sensitivity_frame_updates', 'source_sensitivity_retention',
    'model_command_proposals', 'model_sensitivity_frame_updates',
    'qp_constraint_rows_formed', 'qp_quantization_row_bounds',
    'qp_ALAT_stencils_formed', 'qp_prospective_terminal_stencil',
    'qp_exit_corridor_stencils_formed', 'qp_exact_box_setup',
    'qp_exact_box_support_rows', 'qp_exact_box_direction_membership',
    'qp_exact_dominance_setup', 'qp_exact_dominance_group_rows',
    'qp_exact_dominance_support_rows', 'qp_constraint_rows_scanned',
    'qp_primal_direction_rows_scanned', 'qp_original_removed_direction_rows',
))
MATRIX_KINDS = frozenset(('qp_objective_build', 'qp_KKT_verification'))


class SegmentedWorkMeter(Meter):
    @staticmethod
    def work_units(kind, count=1):
        require(type(count) is int and count > 0, 'positive_segmented_work_count_required')
        multiplier = (VECTOR_WORK_UNITS if kind in VECTOR_KINDS else
                      MATRIX_WORK_UNITS if kind in MATRIX_KINDS else 1)
        return count * multiplier

    def charge(self, kind, count=1):
        self.check('before_' + kind)
        units = self.work_units(kind, count)
        if self.scalar_steps + units > MAX_SCALAR_STEPS:
            raise SearchStop('segmented_work_step_cap')
        self.scalar_steps += units
        self.counts[kind] = self.counts.get(kind, 0) + units
