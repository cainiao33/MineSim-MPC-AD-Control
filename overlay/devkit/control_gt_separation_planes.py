"""Current native-GT separating planes for QP proposals, never a safety gate.

An oriented normal joins the nearest points of two disjoint truck polygons.
Each ego corner receives a frozen source-pose affine projection inequality.
Nonlinear whole-polygon collision/TTC/side-distance gates remain mandatory.
"""

from dataclasses import dataclass
import math

import numpy as np
from shapely.geometry import Polygon
from shapely.ops import nearest_points

from control_full_trace_qp_contract import require
from control_pose_time_corridor import _corners

NEAR_DISTANCE_M = 20.
PROPOSAL_GAP_M = .801


@dataclass(frozen=True)
class GTPlane:
    index: int
    actor: str
    recorded: bool
    corner: int
    pose_normal: tuple
    bound: float
    source_projected_gap: float


def source_pair_planes(row, actor, shape, index, actor_id, recorded, meter):
    """Return at most four metered source affine faces for one current actor."""
    meter.charge('source_GT_plane_polygon_pairs')
    ego = _corners(row[1], row[2], row[3], shape['width'], shape['head'], shape['rear'])
    other_shape = actor['shape']
    other = _corners(float(actor['x']), float(actor['y']), float(actor['yaw_rad']),
                     float(other_shape['width']), float(other_shape['locationPoint2Head']),
                     float(other_shape['locationPoint2Rear']))
    require(np.isfinite(ego).all() and np.isfinite(other).all(), 'finite_live_GT_plane_polygons')
    ego_polygon, other_polygon = Polygon(ego).convex_hull, Polygon(other).convex_hull
    distance = float(ego_polygon.distance(other_polygon))
    if distance <= PROPOSAL_GAP_M or distance > NEAR_DISTANCE_M:
        return ()
    meter.charge('source_GT_plane_nearest_pairs')
    left, right = nearest_points(ego_polygon, other_polygon)
    vector = np.array([right.x-left.x, right.y-left.y])
    norm = float(np.linalg.norm(vector))
    require(math.isfinite(norm) and norm > 0., 'positive_GT_plane_normal_required')
    normal = vector / norm
    projected = float(np.min(other @ normal) - np.max(ego @ normal))
    # Nearest-point normals are only proposed at strictly feasible source
    # support gaps. No tiny negative slack is clipped or certified.
    if projected <= PROPOSAL_GAP_M:
        return ()
    target = float(np.min(other @ normal)) - PROPOSAL_GAP_M
    forward = np.array([math.cos(row[3]), math.sin(row[3])])
    lateral = np.array([-forward[1], forward[0]])
    result = []
    offsets = ((shape['head'], shape['width']/2.),
               (shape['head'], -shape['width']/2.),
               (-shape['rear'], -shape['width']/2.),
               (-shape['rear'], shape['width']/2.))
    # _corners order is front+lateral/front-lateral/back-lateral/back+lateral.
    for corner, (longitudinal, transverse) in enumerate(offsets):
        meter.charge('source_GT_plane_corner_derivatives')
        dyaw = longitudinal*lateral - transverse*forward
        pose_normal = (float(normal[0]), float(normal[1]), float(normal @ dyaw))
        bound = target - float(normal @ ego[corner])
        require(math.isfinite(bound) and bound >= 0., 'source_GT_plane_zero_step_feasible')
        result.append(GTPlane(int(index), str(actor_id), bool(recorded), corner,
                              pose_normal, bound, projected))
    return tuple(result)
