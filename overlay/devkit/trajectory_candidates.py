"""安全执行参考的空间限速及转向反馈候选；XY单位米、曲率单位1/m。"""

import math
from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter


class TraceCand:
    """所有状态放在控制器副本内，候选几何只读并在rollout/执行共享。"""

    kind = "F7"
    tracking = True

    def __init__(self, xy, coefficient, look, t0, delay=0., source=""):
        xy = np.asarray(xy, dtype=float)
        keep = np.r_[True, np.linalg.norm(np.diff(xy, axis=0), axis=1) > 0.03]
        xy = xy[keep]
        if len(xy) < 5:
            raise ValueError("参考轨迹有效点不足")
        distance = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        ss = np.r_[0., np.cumsum(distance)]
        direction = (xy[-1] - xy[-2]) / distance[-1]
        xy = np.vstack([xy, xy[-1] + 10. * direction])
        ss = np.r_[ss, ss[-1] + 10.]
        self.ss = np.arange(0., ss[-1], .5)
        self.rx = np.interp(self.ss, ss, xy[:, 0])
        self.ry = np.interp(self.ss, ss, xy[:, 1])
        window = min(15, (len(self.ss) // 2) * 2 - 1)
        dx = savgol_filter(self.rx, window, 3, deriv=1, delta=.5)
        dy = savgol_filter(self.ry, window, 3, deriv=1, delta=.5)
        self.heading = np.unwrap(np.arctan2(dy, dx))
        self.curvature = savgol_filter(self.heading, window, 3, deriv=1, delta=.5)
        desired = np.minimum(12., coefficient * np.sqrt(
            .39 / np.maximum(np.abs(self.curvature), 1e-5)))
        envelope = np.minimum.accumulate((desired ** 2 + 5. * self.ss)[::-1])[::-1]
        self.speeds = np.sqrt(np.maximum(envelope - 5. * self.ss, .01))
        self.look = look
        self.delay = t0 + delay
        self.label = f"TRACE_{source}_T{t0:g}_C{coefficient:g}_L{look:g}_W{delay:g}"
        for array in (self.ss, self.rx, self.ry, self.heading, self.curvature, self.speeds):
            array.setflags(write=False)

    def progress(self, ctrl):
        if getattr(ctrl, "_sas_geometry_token", None) is not self:
            ctrl._sas_geometry_token = self
            ctrl._sas_geometry_s = None
        previous = ctrl._sas_geometry_s
        if previous is None:
            indices = np.arange(len(self.ss))
        else:
            indices = np.flatnonzero((self.ss >= previous - 3.) &
                                     (self.ss <= previous + 25.))
        distance = ((self.rx[indices] - ctrl._ego_x) ** 2 +
                    (self.ry[indices] - ctrl._ego_y) ** 2)
        if previous is None:
            # 再次接管旧参考时，以朝向区分回头弯相邻的来/回程分支。
            angle = (self.heading[indices] - ctrl._ego_yaw + np.pi) % (2. * np.pi) - np.pi
            distance += 9. * angle ** 2
        near = indices[int(np.argmin(distance))]
        heading = self.heading[near]
        along = ((ctrl._ego_x - self.rx[near]) * math.cos(heading) +
                 (ctrl._ego_y - self.ry[near]) * math.sin(heading))
        progress = float(np.clip(self.ss[near] + along, 0., self.ss[-1]))
        ctrl._sas_geometry_s = progress
        return progress

    def __call__(self, ctrl, bank_v, t):
        if t < self.delay:
            return .1
        return float(np.interp(self.progress(ctrl) + min(3., .4 * ctrl._ego_v),
                               self.ss, self.speeds))

    def steering(self, ctrl):
        s = self.progress(ctrl)
        x, y = np.interp(s, self.ss, self.rx), np.interp(s, self.ss, self.ry)
        heading = float(np.interp(s, self.ss, self.heading))
        error_y = -(ctrl._ego_x - x) * math.sin(heading) + (ctrl._ego_y - y) * math.cos(heading)
        error_yaw = (heading - ctrl._ego_yaw + np.pi) % (2. * np.pi) - np.pi
        curvature = float(np.interp(s, self.ss, self.curvature))
        command = curvature + 2. * error_yaw / self.look - 2. * error_y / self.look ** 2
        length = float(ctrl._ego_shape["length"])
        return float(np.clip(math.atan(length / 1.7 * command), -.6, .6))


def hairpin_candidates(ctx, t, datasets_dir):
    """由起点朝向和目标区域构建短回头弯，官方真实足迹先静态预筛。

    网格有界：10条路径，每条最多4个时序/速度政策。新路仍须经过
    与真实执行同码的全程动态碰撞、终局和官方评分，静态通过不代表安全。
    """
    start = np.asarray(ctx.prefix[-1][1:3], dtype=float)
    yaw = ctx.prefix[-1][3]
    forward = np.array([math.cos(yaw), math.sin(yaw)])
    lateral = np.array([-forward[1], forward[0]])
    target = np.array([np.mean(ctx.goal["x"]), np.mean(ctx.goal["y"])])
    f, lat = float((target - start) @ forward), float((target - start) @ lateral)
    if not (12. <= abs(lat) <= 65. and abs(f) <= 40.):
        return []
    import score_embed
    from score_result import road_boundary_collision
    score_embed.install_map_cache()
    shape = ctx.ego_shape
    actual_shape = dict(width=shape["width"], length=shape["length"],
                        head=shape["locationPoint2Head"], rear=shape["locationPoint2Rear"])
    out = []
    for depth in (30., 40., 50., 60., 70.):
        if depth <= f + 5.:
            continue
        for shift in (-5. * np.sign(lat), 0.):
            remaining = lat + shift
            if remaining * lat <= 0. or abs(remaining) < 10.:
                continue
            radius, side = abs(remaining) / 2., np.sign(remaining)
            line = start + np.arange(0., depth, .5)[:, None] * forward
            theta = np.linspace(0., np.pi, max(20, int(np.pi * radius / .5)))
            arc = (start + depth * forward + radius * np.sin(theta)[:, None] * forward
                   + side * radius * (1. - np.cos(theta))[:, None] * lateral)
            end = start + depth * forward + remaining * lateral
            back = end - np.arange(.5, depth - f + 15., .5)[:, None] * forward
            xy = np.vstack([line, arc, back])
            reached = np.flatnonzero([ctx._in_goal(x, y) for x, y in xy])
            if not len(reached):
                continue
            limit = int(reached[0]) + 1
            tangent = np.unwrap(np.arctan2(*np.diff(xy, axis=0).T[::-1]))
            geometry = xy[:limit:2]
            frame = score_embed.build_frame(
                np.arange(len(geometry)) * ctx.dt,
                dict(x=geometry[:, 0], y=geometry[:, 1], yaw=tangent[:limit:2],
                     v=np.ones(len(geometry))), terminal=4, ego_shape=actual_shape)
            hit, _ = road_boundary_collision(frame, Path(f"{ctx.name}_result.csv"), datasets_dir)
            if hit:
                continue
            source = f"HP_D{depth:g}_G{shift:g}"
            out.extend(TraceCand(xy, coefficient, 6., t, delay=delay, source=source)
                       for coefficient in (.85, 1.) for delay in (0., 6.))
    return out
