"""固定11维box内的单anchor精确半空间蕴含；完整实际轨迹门保持。"""

from fractions import Fraction

import numpy as np

from control_full_trace_qp_contract import require

DIMENSION = 11
from control_qp_segmented_box import prune_box_implied_halfspaces


def prune_exact_redundancies(matrix, bounds, labels, lower, upper, budget, audit):
    """Remove only rows implied by a retained anchor and the original box.

    For positive rational lambda, A_i d <= lambda*b_j + support_box(A_i-lambda*A_j).
    Every noncoordinate row has at most one comparison. Anchors are the minimum
    RHS row in each label family / dominant coefficient axis / sign group.
    Row order and the first twenty-two coordinate faces remain unchanged.
    """
    matrix, bounds = prune_box_implied_halfspaces(
        matrix, bounds, labels, lower, upper, budget, audit)
    labels = list(audit['qp_constraint_labels'])
    budget.charge('qp_exact_dominance_setup')
    lo = [Fraction.from_float(float(value)) for value in lower]
    hi = [Fraction.from_float(float(value)) for value in upper]
    proof = dict(method='retained_single_anchor_plus_exact_binary64_rational_box_support',
        complete=False, input_matrix=matrix.tolist(), input_bounds=bounds.tolist(),
        input_labels=labels, lower=list(map(float, lower)), upper=list(map(float, upper)),
        comparisons=[], retained_input_indices=[], removed_input_indices=[],
        comparison_limit_per_row=1,
        grouping_charge_scope='one per input row: ceil(11/5)=3 work units for11D key, native grouping/order/proof bookkeeping and final retained-row copy; not an arithmetic operation count',
        support_charge_scope='one per executed ceil(11/5)=3 work units for11D rational implication comparison and native witness export')
    audit['qp_exact_dominance_pruning'] = proof
    groups = {}
    for index, row in enumerate(matrix):
        budget.charge('qp_exact_dominance_group_rows')
        if index < 2 * DIMENSION:
            continue
        axis = int(np.argmax(np.abs(row)))
        require(float(row[axis]) != 0., 'nonzero_dominance_normal_required')
        key = (labels[index].rsplit('_', 1)[0], axis, float(row[axis]) > 0.)
        groups.setdefault(key, []).append(index)
    removed = set()
    for (_, axis, _), indices in groups.items():
        # The grouping work and bounded native ordering are covered by the
        # per-row charge above. No solver, face subset or adaptive retry occurs.
        ordered = sorted(indices, key=lambda index: (float(bounds[index]), index))
        anchor = ordered[0]
        for index in ordered[1:]:
            budget.charge('qp_exact_dominance_support_rows')
            multiplier = (Fraction.from_float(float(matrix[index, axis]))
                          / Fraction.from_float(float(matrix[anchor, axis])))
            require(multiplier > 0., 'positive_dominance_multiplier_required')
            difference = [Fraction.from_float(float(value))
                          - multiplier * Fraction.from_float(float(reference))
                          for value, reference in zip(matrix[index], matrix[anchor])]
            support = sum((value * (hi[k] if value >= 0 else lo[k])
                           for k, value in enumerate(difference)), Fraction(0))
            implied_upper = multiplier * Fraction.from_float(float(bounds[anchor])) + support
            deletion = implied_upper <= Fraction.from_float(float(bounds[index]))
            proof['comparisons'].append(dict(index=index, anchor=anchor,
                multiplier_numerator=multiplier.numerator,
                multiplier_denominator=multiplier.denominator,
                support_numerator=support.numerator, support_denominator=support.denominator,
                implied_upper_numerator=implied_upper.numerator,
                implied_upper_denominator=implied_upper.denominator, removed=bool(deletion)))
            if deletion:
                removed.add(index)
    retained = [index for index in range(len(matrix)) if index not in removed]
    proof.update(complete=True, retained_input_indices=retained,
        removed_input_indices=sorted(removed), input_count=len(matrix),
        retained_count=len(retained), removed_count=len(removed),
        actual_comparisons=len(proof['comparisons']),
        anchors_are_retained=all(item['anchor'] in retained for item in proof['comparisons']))
    require(proof['anchors_are_retained'], 'dominance_anchor_must_be_retained')
    audit.update(qp_constraint_count=len(retained),
        qp_constraint_labels=[labels[index] for index in retained],
        qp_dominance_removed_count=len(removed),
        qp_dominance_proof_is_exact_domain_equivalence=True)
    return matrix[retained].copy(), bounds[retained].copy()


def certify_original_removed_halfspaces(direction, budget, audit, tolerance=1e-9):
    """Recheck removed rows at the solver's original feasibility tolerance.

    Exact implication concerns the exact system; rounded solver points also
    receive this explicit metered check, before a model trial can start.
    """
    proof = audit['qp_exact_dominance_pruning']
    require(proof['complete'] is True and proof['anchors_are_retained'] is True,
            'complete_dominance_proof_required')
    indices = proof['removed_input_indices']
    if indices:
        budget.charge('qp_original_removed_direction_rows', len(indices))
        rows = np.asarray(proof['input_matrix'])[indices]
        bounds = np.asarray(proof['input_bounds'])[indices]
        maximum = float(np.max(rows @ np.asarray(direction) - bounds))
        audit['qp_original_removed_direction_max_violation'] = maximum
        require(np.isfinite(maximum) and maximum <= tolerance,
                'QP_direction_violates_original_removed_halfspace')
    audit['qp_original_removed_direction_check_pass'] = True
