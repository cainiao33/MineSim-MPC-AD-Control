"""当前真实凸目标的半空间提案；原Env的polygon到达判断仍为唯一门。"""

from dataclasses import dataclass
import math

import numpy as np

from control_full_trace_qp_contract import require, frozen_array


@dataclass(frozen=True)
class GoalPolygon:
    vertices: np.ndarray
    origin: np.ndarray
    normals: np.ndarray
    bounds: np.ndarray


def goal_polygon(source, meter):
    x = np.asarray(source.ctx.goal['x'], dtype=float)
    y = np.asarray(source.ctx.goal['y'], dtype=float)
    require(x.ndim == y.ndim == 1 and x.shape == y.shape and 3 <= len(x) <= 8,
            'bounded_actual_polygon_vertices_required')
    meter.charge('polygon_goal_geometry_vertex_reads', len(x))
    require(np.isfinite(x).all() and np.isfinite(y).all(), 'finite_actual_goal_vertices_required')
    vertices = np.column_stack((x, y))
    origin = vertices[0]
    local = vertices - origin
    edges = np.roll(local, -1, axis=0) - local
    lengths = np.linalg.norm(edges, axis=1)
    meter.charge('polygon_goal_edge_geometry', len(x))
    require(np.all(lengths > 1e-10), 'nondegenerate_actual_goal_edges_required')
    following = np.roll(local, -1, axis=0)
    area2 = float(np.sum(local[:, 0] * following[:, 1] - local[:, 1] * following[:, 0]))
    require(math.isfinite(area2) and abs(area2) > 1e-10, 'nondegenerate_actual_goal_area_required')
    normals = np.column_stack((edges[:, 1], -edges[:, 0])) / lengths[:, None]
    normals *= 1. if area2 > 0. else -1.
    bounds = np.sum(normals * local, axis=1)
    meter.charge('polygon_goal_convexity_pair_checks', len(x) * len(x))
    require(np.all(normals @ local.T - bounds[:, None] <= 1e-9),
            'actual_goal_must_be_convex_without_hull_expansion')
    return GoalPolygon(frozen_array(vertices, vertices.shape), frozen_array(origin, (2,)),
                       frozen_array(normals, normals.shape), frozen_array(bounds, bounds.shape))


def face_distances(goal, point, meter):
    point = np.asarray(point, dtype=float)
    require(point.shape == (2,) and np.isfinite(point).all(), 'finite_actual_goal_query_point')
    meter.charge('polygon_goal_halfspace_queries', len(goal.bounds))
    return goal.normals @ (point - goal.origin) - goal.bounds


def violation_cost(goal, point, jacobian, meter):
    """Squared positive normalized face distances, with its exact local gradient."""
    distances = face_distances(goal, point, meter)
    jacobian = np.asarray(jacobian, dtype=float)
    require(jacobian.ndim == 2 and jacobian.shape[0] == 2 and np.isfinite(jacobian).all(),
            'finite_actual_goal_xy_sensitivities')
    meter.charge('polygon_goal_gradient_vectors', len(goal.bounds) * math.ceil(jacobian.shape[1] / 5))
    positive = np.maximum(distances, 0.)
    cost = float(positive @ positive)
    gradient = 2. * (positive @ goal.normals @ jacobian)
    require(math.isfinite(cost) and np.isfinite(gradient).all(), 'finite_polygon_goal_merit')
    return cost, gradient


def projection_target(goal, point, meter, inset=1e-6):
    point = np.asarray(point, dtype=float)
    if np.all(face_distances(goal, point, meter) <= 0.):
        return point.copy()
    meter.charge('polygon_goal_projection_edge_queries', len(goal.bounds))
    following = np.roll(goal.vertices, -1, axis=0)
    edges = following - goal.vertices
    parameter = np.clip(np.sum((point - goal.vertices) * edges, axis=1)
                        / np.sum(edges * edges, axis=1), 0., 1.)
    projected = goal.vertices + parameter[:, None] * edges
    target = projected[int(np.argmin(np.sum((projected - point) ** 2, axis=1)))].copy()
    toward_center = np.mean(goal.vertices, axis=0) - target
    length = float(np.linalg.norm(toward_center))
    if length > 0.:
        target += toward_center * min(1., inset / length)
    return target


def objective_factors(goal, point, jacobian, meter, inset=1e-6):
    distances = face_distances(goal, point, meter)
    violated = distances > 0.
    require(np.any(violated), 'earlier_goal_frame_must_violate_actual_polygon_face')
    count = int(np.sum(violated))
    meter.charge('polygon_goal_objective_vector_projections', 3 * count)
    factor = goal.normals[violated] @ jacobian
    residual = distances[violated] + inset
    return factor, residual, violated
