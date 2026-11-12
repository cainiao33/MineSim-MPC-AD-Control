"""在线安全路线的局部退弯提案：保留原政策前缀，连续曲率连接自由门户。

输入五状态为(t秒,x米,y米,yaw弧度,v米/秒)，仅接受调用方的完整同码
在线安全预测。静态曲率/时间不证明车辆可执行；最终Trace几何逐估值做
官方bitmap足迹过滤，只返回一个政策，仍需完整预测、原HYST及同对象Env。
"""

import copy
import math
import time
from pathlib import Path

import numpy as np
from scipy.interpolate import BPoly
from scipy.optimize import minimize

MAX_EVALUATIONS = 120
MAX_ITERATIONS = 16
JOIN_HEADING_TOLERANCE = .04
JOIN_CURVATURE_TOLERANCE = .015
MAX_PROPOSAL_CURVATURE = .12


def wrap_angle(angle):
    return (angle + math.pi) % (2. * math.pi) - math.pi


def quintic_curve(position, heading, curvature, target, end_heading, length):
    """米制参数的五次Hermite：接头位置、一阶单位切向、二阶法向匹配。"""
    normal = np.array([-math.sin(heading), math.cos(heading)])
    start_direction = np.array([math.cos(heading), math.sin(heading)])
    end_direction = np.array([math.cos(end_heading), math.sin(end_heading)])
    if not math.isfinite(length) or length <= 1.:
        raise ValueError('曲线参数长度非法')
    return BPoly.from_derivatives([0., float(length)], [
        [np.asarray(position, dtype=float), start_direction, float(curvature) * normal],
        [np.asarray(target, dtype=float), end_direction, np.zeros(2)],
    ])


def alignment_anchor(rows, vertices):
    """显著转弯之后第一次与目标方向对齐的事件，无场名或时间网格。"""
    rows = np.asarray(rows, dtype=float)
    if rows.ndim != 2 or rows.shape[1] != 5 or len(rows) < 12 or not np.isfinite(rows).all():
        raise ValueError('在线参考行不足或非有限')
    if np.any(np.diff(rows[:, 0]) <= 0.) or np.any(rows[:, 4] < 0.):
        raise ValueError('在线参考时间/速度非法')
    ss = np.r_[0., np.cumsum(np.linalg.norm(np.diff(rows[:, 1:3], axis=0), axis=1))]
    yaw = np.unwrap(rows[:, 3])
    curvature = np.gradient(yaw, rows[:, 0]) / np.maximum(rows[:, 4], 1e-6)
    centroid = np.asarray(vertices, dtype=float).mean(axis=0)
    bearing = np.arctan2(centroid[1]-rows[:, 2], centroid[0]-rows[:, 1])
    error = np.abs((yaw-bearing+math.pi) % (2.*math.pi)-math.pi)
    eligible = np.flatnonzero((ss >= 15.) & (ss[-1]-ss >= 20.)
        & (np.abs(yaw-yaw[0]) >= .7) & (np.abs(curvature) >= .01)
        & (np.abs(curvature) <= MAX_PROPOSAL_CURVATURE) & (error <= .12)
        & (rows[:, 4] >= .5))
    if not len(eligible):
        return None, dict(reason='no_goal_alignment_turn_event')
    index = int(eligible[0])
    return index, dict(reason='first_online_goal_alignment_after_significant_turn',
        index=index, cut_time_s=float(rows[index, 0]), station_m=float(ss[index]),
        xy=rows[index, 1:3].tolist(), heading_rad=float(yaw[index]),
        curvature_1pm=float(curvature[index]), alignment_error_rad=float(error[index]),
        original_suffix_time_s=float(rows[-1, 0]-rows[index, 0]))


class LocalCurvatureExitCand:
    kind = 'F11_LOCAL_CURVATURE_EXIT'
    tracking = True

    def __init__(self, base, suffix, cut_time, dt):
        if not getattr(base, 'tracking', False) or not callable(getattr(base, 'steering', None)):
            raise ValueError('原政策必须提供tracking转向')
        self.base, self.suffix = base, suffix
        self.cut_time, self.dt = float(cut_time), float(dt)
        self.label = 'LOCAL_CURVATURE_EXIT_I1_' + base.label

    def command(self, ctrl, bank_v, t, budget=None):
        # 是否切换与时间只放当前控制器，候选本体不存可变执行状态。
        t = float(t)
        state = getattr(ctrl, '_local_exit_frame', None)
        if (state is not None and state[0] is self and state[1] == id(ctrl)
                and (not state[5] or abs(state[2]-t) < 1e-8)):
            raise RuntimeError('每帧只允许一次完整速度命令和转向')
        suffix = t + 1e-7 >= self.cut_time
        pose = tuple(float(getattr(ctrl, key)) for key in
                     ('_ego_x', '_ego_y', '_ego_yaw', '_ego_v'))
        ctrl._local_exit_frame = (self, id(ctrl), t, suffix, pose, False, False)
        policy = self.suffix if suffix else self.base
        value = float(policy(ctrl, bank_v, t))
        if not math.isfinite(value) or not .1 <= value <= 12.:
            raise ValueError('原政策或suffix速度命令非法')
        ctrl._local_exit_frame = (self, id(ctrl), t, suffix, pose, False, True)
        return value

    def __call__(self, ctrl, bank_v, t):
        return self.command(ctrl, bank_v, t)

    def steering(self, ctrl):
        state = getattr(ctrl, '_local_exit_frame', None)
        pose = tuple(float(getattr(ctrl, key)) for key in
                     ('_ego_x', '_ego_y', '_ego_yaw', '_ego_v'))
        if (state is None or state[0] is not self or state[1] != id(ctrl)
                or state[5] or not state[6] or state[4] != pose):
            raise RuntimeError('新几何转向必须在同帧唯一速度命令之后')
        policy = self.suffix if state[3] else self.base
        value = float(policy.steering(ctrl))
        if not math.isfinite(value):
            raise ValueError('几何转向非有限')
        ctrl._local_exit_frame = state[:5] + (True, True)
        return value


def build_candidate(base, online_rows, ctx, ctrl_at_anchor, datasets_dir, budget_s=8.):
    """单事件、单门户边的有界连续估值；失败不生成政策，不改原对象。"""
    from trajectory_candidates import TraceCand
    import score_embed
    from score_result import road_boundary_collision
    started = time.perf_counter()
    audit = dict(mechanism='prefix_preserving_local_quintic_free_portal',
        reference_uses_external_csv=False, source='complete_online_same_code_safe_policy',
        policy_count=0, accepted=False, parameter_grid=False,
        static_budget_s=float(budget_s), budget_checked_between_evaluations=True,
        static_wall_hard_cap_claimed=False, geometric_time_is_safety_certificate=False,
        complete_dynamic_rollouts=0, evaluations=0, official_checks=0, official_accepts=0)
    if not math.isfinite(budget_s) or not 0. < budget_s <= 15.:
        raise ValueError('静态预算须在(0,15]秒')
    if not getattr(base, 'tracking', False) or not callable(getattr(base, 'steering', None)):
        audit['reason']='source_not_tracking_with_steering';return None, audit
    vertices = np.column_stack([ctx.goal['x'], ctx.goal['y']]).astype(float)
    if vertices.ndim != 2 or len(vertices) < 3 or not np.isfinite(vertices).all():
        raise ValueError('目标门户须为至少三个有限二维顶点')
    if np.linalg.norm(vertices[0]-vertices[-1]) < 1e-6:
        vertices=vertices[:-1]
    index, anchor = alignment_anchor(online_rows, vertices)
    audit['anchor']=anchor
    if index is None:
        audit['reason']=anchor['reason'];return None, audit
    rows=np.asarray(online_rows, dtype=float)
    if (np.max(np.abs(rows[0]-np.asarray(ctx.prefix[-1],dtype=float))) > 1e-7
            or np.max(np.abs(np.diff(rows[:,0])-ctx.dt)) > 1e-7):
        raise ValueError('参考须始于当前状态，不能包含历史prefix')
    xy=rows[index, 1:3];yaw=anchor['heading_rad'];kappa=anchor['curvature_1pm']
    # 使用完整固定原政策在同一帧的控制器状态；不以首帧状态冒充接头状态。
    expected = rows[index, 1:]
    actual = np.array([ctrl_at_anchor._ego_x, ctrl_at_anchor._ego_y,
                       ctrl_at_anchor._ego_yaw, ctrl_at_anchor._ego_v])
    if not np.isfinite(actual).all() or np.max(np.abs(actual-expected)) > 1e-8:
        raise ValueError('接头控制器必须来自原政策同码执行的对齐帧')
    centroid=vertices.mean(axis=0)
    edges=[]
    for edge in range(len(vertices)):
        begin=vertices[edge];direction=vertices[(edge+1)%len(vertices)]-begin
        if np.dot(direction,direction) <= 1e-12:
            raise ValueError('目标门户有退化边')
        parameter=float(np.clip(np.dot(xy-begin,direction)/np.dot(direction,direction),0.,1.))
        point=begin+parameter*direction
        edges.append((float(np.linalg.norm(point-xy)),edge,parameter))
    _, edge, parameter=min(edges)
    audit.update(portal_edge=edge, portal_seed_parameter=parameter,
                 initial_join_curvature_1pm=kappa, prefix_switch_time_s=anchor['cut_time_s'])
    shape=ctx.ego_shape
    actual_shape=dict(width=shape['width'],length=shape['length'],
                      head=shape['locationPoint2Head'],rear=shape['locationPoint2Rear'])
    score_embed.install_map_cache()
    best=None
    refused=dict(nonregular=0,curvature=0,join=0,goal=0,lookup=0,official_footprint=0,steering=0)

    def portal(parameter):
        point=vertices[edge]*(1.-parameter)+vertices[(edge+1)%len(vertices)]*parameter
        inward=centroid-point
        return point+min(.6,float(np.linalg.norm(inward)))*inward/max(1e-9,float(np.linalg.norm(inward)))

    seed=portal(parameter);bearing=math.atan2(*(seed-xy)[::-1])
    seed_heading=yaw+wrap_angle(bearing-yaw)

    def evaluate(variables):
        nonlocal best
        if audit['evaluations'] >= MAX_EVALUATIONS or time.perf_counter()-started >= budget_s:
            audit['truncated']=True;return 1e9
        audit['evaluations']+=1
        target=portal(float(variables[0]));length=float(np.linalg.norm(target-xy)*variables[2])
        curve=quintic_curve(xy,yaw,kappa,target,float(variables[1]),length)
        samples=np.linspace(0.,length,max(80,int(math.ceil(length*4.))+1))
        points=curve(samples);first=curve(samples,1);second=curve(samples,2)
        norm=np.linalg.norm(first,axis=1)
        if not np.isfinite(points).all() or np.any(norm<.2):
            refused['nonregular']+=1;return 1e8
        raw_curvature=(first[:,0]*second[:,1]-first[:,1]*second[:,0])/norm**3
        if float(np.max(np.abs(raw_curvature)))>MAX_PROPOSAL_CURVATURE:
            refused['curvature']+=1;return 1e7
        # Trace重采样/SG改变原样条；只对最终对象的几何做资格筛选。
        suffix=TraceCand(points,1.,float(getattr(base,'look',6.)),anchor['cut_time_s'],source='LOCAL_C2_PORTAL')
        if (abs(wrap_angle(float(suffix.heading[0])-yaw))>JOIN_HEADING_TOLERANCE
                or abs(float(suffix.curvature[0])-kappa)>JOIN_CURVATURE_TOLERANCE):
            refused['join']+=1;return 1e6
        hits=np.flatnonzero([ctx._in_goal(x,y) for x,y in zip(suffix.rx,suffix.ry)])
        if not len(hits):refused['goal']+=1;return 1e6
        limit=int(hits[0])+1
        if limit<5 or np.max(np.abs(suffix.curvature[:limit]))>MAX_PROPOSAL_CURVATURE:
            refused['curvature']+=1;return 1e6
        positions=list(range(0,limit,2))
        if positions[-1]!=limit-1:positions.append(limit-1)
        if any(ctx._road_hit(float(suffix.rx[i]),float(suffix.ry[i]),float(suffix.heading[i])) for i in positions):
            refused['lookup']+=1;return 1e5
        frame=score_embed.build_frame(np.arange(limit)*ctx.dt,
            dict(x=suffix.rx[:limit],y=suffix.ry[:limit],yaw=suffix.heading[:limit],v=np.ones(limit)),
            terminal=4,ego_shape=actual_shape)
        collision,check=road_boundary_collision(frame,Path(f'{ctx.name}_result.csv'),datasets_dir)
        audit['official_checks']+=1
        if collision or check!='bitmap_footprint':
            refused['official_footprint']+=1;return 1e5
        audit['official_accepts']+=1
        # 接管pose下比较原base与最终suffix反馈命令；仅作静态拒绝门。
        # 浅拷贝保留几何token本体身份；steering仅改各自控制器标量状态。
        original_steer=float(base.steering(copy.copy(ctrl_at_anchor)))
        proposed_steer=float(suffix.steering(copy.copy(ctrl_at_anchor)))
        if not math.isfinite(original_steer) or abs(proposed_steer-original_steer)>.05:
            refused['steering']+=1;return 1e5
        speed=np.minimum(suffix.speeds[:limit-1],suffix.speeds[1:limit])
        geometric_time=float(np.sum(np.diff(suffix.ss[:limit])/np.maximum(speed,.1)))
        if best is None or geometric_time<best['geometric_suffix_time_s']:
            best=dict(suffix=suffix,geometric_suffix_time_s=geometric_time,
                variables=list(map(float,variables)),portal_xy=target.tolist(),
                final_join_heading_rad=float(suffix.heading[0]),final_join_curvature_1pm=float(suffix.curvature[0]),
                static_feedback_steering_delta_rad=abs(proposed_steer-original_steer),
                official_footprint_check=check)
        return geometric_time

    minimize(evaluate,np.array([parameter,seed_heading,1.]),method='Powell',
        bounds=[(0.,1.),(seed_heading-.7,seed_heading+.7),(.8,1.4)],
        options=dict(maxiter=MAX_ITERATIONS,maxfev=MAX_EVALUATIONS,xtol=.02,ftol=.001))
    audit.update(refused=refused,elapsed_s=time.perf_counter()-started)
    if best is None:
        audit['reason']='no_final_trace_footprint_and_join_feasible_suffix';return None,audit
    candidate=LocalCurvatureExitCand(base,best.pop('suffix'),anchor['cut_time_s'],ctx.dt)
    audit.update(best,accepted=True,reason='one_static_proposal_requires_full_execution',
                 policy_count=1,candidate=candidate.label,prefix_control_delegated_to_original_base=True,
                 raw_quintic_C2=True,final_Trace_exact_C2_claimed=False,
                 suffix_speed_policy_changed=True,suffix_speed_source='TraceCand_coefficient_1',
                 gain_attributable_to_geometry_alone=False)
    return candidate,audit
