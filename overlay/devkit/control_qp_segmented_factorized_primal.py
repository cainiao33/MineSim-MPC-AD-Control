"""F40 有界可行起点 active-set QP；不运行政策或改变11D动作家族。"""

import numpy as np

from control_full_trace_qp_contract import require

DIMENSION = 11


def solve_factorized_identity_start_qp(quadratic, gradient, matrix, bounds, budget, audit,
                           *, objective_jacobian, objective_residual, regularization_center,
                           regularization=1e-6, iteration_limit=96, face_limit=11, tolerance=1e-9):
    """Start at exact feasible d=0 and take blocking-face Newton steps.

    Every matrix-row dot product is charged independently. Linear systems are
    at most 22x22, with at most eleven independent active rows. A prospective
    earlier-terminal row excluding zero causes refusal before any iteration.
    """
    require(quadratic.shape == (DIMENSION, DIMENSION) and gradient.shape == (DIMENSION,)
            and matrix.ndim == 2 and matrix.shape[1] == DIMENSION
            and bounds.shape == (len(matrix),) and len(matrix) > 0
            and 0 < face_limit <= DIMENSION, 'segmented_QP_shapes_or_face_cap')
    budget.charge('qp_invocations')
    audit.update(qp_solve_count=1, qp_solver='factorized_nullspace_primal_feasible_identity_start',
        qp_iterations=0, qp_active_faces_peak=0, qp_primal_trace=[],
        qp_row_dot_products_separately_metered=True, qp_duals_are_solution_certificate=False,
        qp_active_coordinate_faces_enforced_exactly=True)
    budget.charge('qp_primal_initial_bound_rows', len(bounds))
    require(np.isfinite(quadratic).all() and np.isfinite(gradient).all()
            and np.isfinite(matrix).all() and np.isfinite(bounds).all(),
            'finite_primal_QP_required')
    require(np.all(bounds >= 0.), 'primal_identity_start_not_feasible')
    factor=np.asarray(objective_jacobian,dtype=float)
    residual=np.asarray(objective_residual,dtype=float)
    center=np.asarray(regularization_center,dtype=float)
    require(factor.ndim==2 and factor.shape[1]==DIMENSION and 1<=len(factor)<=2
            and residual.shape==(len(factor),) and center.shape==(DIMENSION,)
            and np.isfinite(factor).all() and np.isfinite(residual).all()
            and np.isfinite(center).all() and np.isfinite(regularization) and regularization>0.,
            'finite_low_rank_objective_factor_required')
    budget.charge('qp_factor_binding', 5)
    require(np.array_equal(quadratic,factor.T@factor+regularization*np.eye(DIMENSION))
            and np.array_equal(gradient,factor.T@residual+regularization*center),
            'factor_must_bind_the_unchanged_quadratic_and_gradient')
    root_regularization=float(np.sqrt(regularization))
    audit.update(qp_objective_factor_bound=True,qp_objective_factor=factor.tolist(),
                 qp_objective_residual=residual.tolist(),qp_regularization_center=center.tolist(),
                 qp_regularization=float(regularization),qp_direction_tolerance_unchanged=True)
    point, working = np.zeros(DIMENSION), []
    labels = audit.get('qp_constraint_labels', [])

    def remember(multipliers):
        audit.update(qp_active_face_indices=list(working),
            qp_active_constraint_labels=[labels[i] if labels else str(i) for i in working],
            qp_dual_multipliers=multipliers.tolist(), qp_pending_constraint_index=None,
            qp_pending_dual_multiplier=0., qp_last_primal_point=point.tolist())

    def restore_active_coordinate_faces():
        # Build exact active coordinate equalities instead of accumulating
        # roundoff normal to an already active face. The final full row check
        # still certifies every coupled halfspace; no bound is relaxed.
        for face in working:
            budget.charge('qp_active_coordinate_face_checks')
            row = matrix[face]
            axes = np.flatnonzero(row)
            if len(axes) == 1 and abs(float(row[axes[0]])) == 1.:
                point[axes[0]] = float(bounds[face]) / float(row[axes[0]])

    def finish(multipliers):
        budget.charge('qp_constraint_rows_scanned', len(bounds))
        violation = matrix @ point - bounds
        budget.charge('qp_KKT_verification', len(working) + 1)
        dual_gradient = matrix[working].T @ multipliers if working else np.zeros(DIMENSION)
        stationarity = quadratic @ point + gradient + dual_gradient
        scale = 1. + np.linalg.norm(quadratic @ point, np.inf) + np.linalg.norm(gradient, np.inf) + np.linalg.norm(dual_gradient, np.inf)
        active_error = float(np.max(np.abs(violation[working]))) if working else 0.
        dual_minimum = float(np.min(multipliers)) if working else 0.
        residual = float(np.linalg.norm(stationarity, np.inf))
        require(np.isfinite(violation).all() and np.isfinite(residual), 'finite_primal_KKT_required')
        audit.update(qp_KKT_stationarity_inf=residual, qp_KKT_stationarity_scale=float(scale),
            qp_KKT_active_residual_inf=active_error, qp_KKT_dual_minimum=dual_minimum)
        require(float(np.max(violation)) <= tolerance and dual_minimum >= -tolerance
                and active_error <= 10. * tolerance and residual <= 10. * tolerance * scale,
                'qp_local_KKT_numeric_verification_failed')
        remember(multipliers)
        audit.update(qp_status='local_convex_solution', qp_direction=point.tolist(),
            qp_max_halfspace_violation=float(np.max(violation)),
            qp_objective_change=float(gradient @ point + .5 * point @ quadratic @ point),
            qp_duals_are_solution_certificate=True)
        return point

    for iteration in range(iteration_limit):
        budget.charge('qp_iterations')
        audit['qp_iterations'] = iteration + 1
        active = matrix[working]
        if working:
            budget.charge('qp_nullspace_QR', (DIMENSION**3+999)//1000)
            orthogonal,_=np.linalg.qr(active.T,mode='complete')
            nullspace=orthogonal[:,len(working):]
        else:
            nullspace=np.eye(DIMENSION)
        if nullspace.shape[1]:
            budget.charge('qp_reduced_objective_least_squares',((DIMENSION+len(factor))**3+999)//1000)
            augmented=np.vstack((factor@nullspace,root_regularization*nullspace))
            target=-np.r_[factor@point+residual,root_regularization*(point+center)]
            coefficients,_,rank,_=np.linalg.lstsq(augmented,target,rcond=None)
            require(rank==nullspace.shape[1],'full_rank_regularized_nullspace_objective')
            direction=nullspace@coefficients
        else:
            direction=np.zeros(DIMENSION)
        if working:
            budget.charge('qp_active_dual_least_squares',(DIMENSION**3+999)//1000)
            next_gradient=factor.T@(factor@(point+direction)+residual)+regularization*(point+direction+center)
            multipliers,_,rank,_=np.linalg.lstsq(active.T,-next_gradient,rcond=None)
            require(rank==len(working),'independent_active_rows_required')
        else:
            multipliers=np.zeros(0)
        require(np.isfinite(direction).all() and np.isfinite(multipliers).all(),
                'finite_factorized_primal_KKT_solution')
        remember(multipliers)
        trace = dict(iteration=iteration + 1, before=point.tolist(), direction=direction.tolist(),
            active_indices=list(working), multipliers=multipliers.tolist())
        audit['qp_primal_trace'].append(trace)
        if float(np.linalg.norm(direction, np.inf)) <= tolerance:
            if not working or float(np.min(multipliers)) >= -tolerance:
                trace['operation'] = 'certify'
                return finish(multipliers)
            leaving = int(np.argmin(multipliers))
            trace.update(operation='release_face', released_index=working[leaving])
            working.pop(leaving)
            remember(np.delete(multipliers, leaving))
            continue
        if np.all(point == 0.):
            # Exact zero dot products are not executed. This can persist
            # through several zero-length boundary steps at the identity.
            slack = bounds
            audit['qp_zero_point_row_dots_elided'] = audit.get('qp_zero_point_row_dots_elided', 0) + len(bounds)
        else:
            budget.charge('qp_constraint_rows_scanned', len(bounds))
            slack = bounds - matrix @ point
        require(float(np.min(slack)) >= -tolerance, 'primal_iterate_outside_halfspaces')
        budget.charge('qp_primal_direction_rows_scanned', len(bounds))
        rates = matrix @ direction
        eligible = rates > tolerance
        eligible[working] = False
        indices = np.flatnonzero(eligible)
        step, blocking = 1., None
        if len(indices):
            # A roundoff-sized negative current slack gives a zero step; no
            # constraint bound, actual parameter or final proof is relaxed.
            ratios = np.maximum(slack[indices], 0.) / rates[indices]
            minimum = float(np.min(ratios))
            if minimum < 1.:
                ties = indices[ratios == minimum]
                blocking = int(ties[np.argmax(rates[ties])])
                step = max(0., minimum)
        point = point + step * direction
        trace.update(operation='feasible_step', step=step, blocking_index=blocking,
            after=point.tolist())
        if blocking is not None:
            require(len(working) < face_limit, 'qp_independent_active_face_cap')
            working.append(blocking)
        audit['qp_active_faces_peak'] = max(audit['qp_active_faces_peak'], len(working))
        restore_active_coordinate_faces()
        trace['after'] = point.tolist()
        remember(np.r_[multipliers, 0.] if blocking is not None else multipliers)
    require(False, 'qp_iteration_limit_without_certified_local_solution')
