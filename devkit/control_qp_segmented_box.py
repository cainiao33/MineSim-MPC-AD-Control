"""精确有理数box支撑剪枝；只处理已形成的11维线性半空间。

每行在计算前独立计量一次，固定最多11个有理数乘积，以及本行原A/b、
标签和证明的原生诊断导出、保留行复制均在该单位内。原形成、后续QP
扫描、KKT、非线性和执行安全门继续各自计量。没有浮点容差删行。
"""

from fractions import Fraction

import numpy as np

from control_full_trace_qp_contract import require

DIMENSION = 11


def prune_box_implied_halfspaces(matrix, bounds, labels, lower, upper, budget, audit):
    """Retain coordinate bounds and remove only exactly box-implied rows.

    A: (N, 11), b: (N,); every coefficient is interpreted as its exact binary64
    rational value. The first twenty-two retained rows must imply lower <= d <= upper.
    Deduplication may have strengthened a coordinate bound and changed its label.
    """
    budget.charge('qp_exact_box_setup')
    matrix, bounds = np.asarray(matrix), np.asarray(bounds)
    lower, upper = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
    require(matrix.dtype == bounds.dtype == np.dtype('float64')
            and matrix.ndim == 2 and matrix.shape[1] == DIMENSION
            and bounds.shape == (len(matrix),) and len(labels) == len(matrix)
            and len(matrix) >= 2 * DIMENSION, 'exact_box_halfspace_shape_or_dtype')
    require(lower.shape == upper.shape == (DIMENSION,) and np.isfinite(lower).all()
            and np.isfinite(upper).all() and np.all(lower <= upper), 'exact_box_domain')
    lower_exact = [Fraction.from_float(float(x)) for x in lower]
    upper_exact = [Fraction.from_float(float(x)) for x in upper]
    eye = np.eye(DIMENSION)
    for axis in range(DIMENSION):
        high, low = 2 * axis, 2 * axis + 1
        require(np.array_equal(matrix[high], eye[axis])
                and np.array_equal(matrix[low], -eye[axis])
                and np.isfinite(bounds[[high, low]]).all()
                and float(bounds[high]) <= float(upper[axis])
                and float(bounds[low]) <= -float(lower[axis]),
                'retained_coordinate_faces_must_imply_declared_box')
    diagnostic = dict(method='exact_binary64_rational_box_support_no_tolerance',
        lower=lower.tolist(), upper=upper.tolist(), coordinate_indices=list(range(2 * DIMENSION)),
        original_matrix=[], original_bounds=[], original_labels=[], proofs=[],
        retained_original_indices=[], removed_original_indices=[], complete=False,
        independent_row_charge=True,
        row_charge_scope='ceil(11/5)=3 work units for11D rational support plus native row/proof export and retained row copy')
    audit['qp_exact_box_pruning'] = diagnostic
    retained_matrix, retained_bounds, retained_labels = [], [], []
    for index, (row, bound, label) in enumerate(zip(matrix, bounds, labels)):
        budget.charge('qp_exact_box_support_rows')
        require(np.isfinite(row).all() and np.isfinite(bound), 'finite_exact_box_halfspace')
        support = sum((Fraction.from_float(float(coefficient)) *
            (upper_exact[axis] if coefficient >= 0. else lower_exact[axis])
            for axis, coefficient in enumerate(row)), Fraction(0))
        rhs = Fraction.from_float(float(bound))
        removed = index >= 2 * DIMENSION and support <= rhs
        diagnostic['original_matrix'].append(row.tolist())
        diagnostic['original_bounds'].append(float(bound))
        diagnostic['original_labels'].append(label)
        diagnostic['proofs'].append(dict(index=index,
            support_numerator=support.numerator, support_denominator=support.denominator,
            removed=bool(removed)))
        if removed:
            diagnostic['removed_original_indices'].append(index)
        else:
            diagnostic['retained_original_indices'].append(index)
            retained_matrix.append(row.copy())
            retained_bounds.append(float(bound))
            retained_labels.append(label)
    diagnostic.update(complete=True, original_count=len(bounds),
        removed_count=len(diagnostic['removed_original_indices']),
        retained_count=len(retained_bounds))
    audit.update(qp_original_constraint_count=len(bounds),
        qp_constraint_count=len(retained_bounds), qp_constraint_labels=retained_labels,
        qp_box_implied_removed_count=diagnostic['removed_count'])
    return np.asarray(retained_matrix), np.asarray(retained_bounds)


def certify_direction_inside_exact_box(direction, budget, audit):
    """Verify the solver's binary64 point meets the deletion premise exactly.

    Solver feasibility tolerances cannot enlarge the box used by the proof.
    We reject an outward rounded point without clipping or changing direction.
    """
    budget.charge('qp_exact_box_direction_membership')
    proof = audit['qp_exact_box_pruning']
    direction = np.asarray(direction)
    require(proof['complete'] is True and direction.shape == (DIMENSION,)
            and np.isfinite(direction).all(), 'pruned_QP_direction_proof_missing')
    inside = bool(np.all(direction >= np.asarray(proof['lower']))
                  and np.all(direction <= np.asarray(proof['upper'])))
    audit['qp_direction_inside_exact_pruning_box'] = inside
    require(inside, 'pruned_QP_direction_outside_exact_box')
