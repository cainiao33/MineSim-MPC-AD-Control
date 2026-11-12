"""生产只读校验；从冻结F40提取原函数，保留动作和安全阈值。"""

import hashlib
import math
from control_full_trace_qp_contract import require, json_native, path_signature


def policy_tree_signature(policy):
    """记录政策与base/suffix闭包的身份、标量和只读数组，不推进模型。"""
    import numpy as np
    seen = {}

    def visit(owner):
        if id(owner) in seen:
            return dict(reference_identity=id(owner))
        values = {}
        seen[id(owner)] = values
        for key, value in vars(owner).items():
            if isinstance(value, np.ndarray):
                values[key] = dict(shape=list(value.shape), dtype=str(value.dtype),
                    readonly=not value.flags.writeable,
                    sha256=hashlib.sha256(value.tobytes()).hexdigest())
            elif value is None or isinstance(value, (str, int, float, bool)):
                values[key] = value
            elif key in ('base', 'suffix'):
                values[key] = visit(value)
            else:
                values[key] = dict(type=type(value).__name__, identity=id(value))
        return dict(type=type(owner).__name__, identity=id(owner), values=values)

    return visit(policy)


def readonly_policy_tree(signature):
    arrays = []

    def visit(value):
        if isinstance(value, dict):
            if 'readonly' in value:
                arrays.append(value['readonly'])
            for item in value.values():
                visit(item)
    visit(signature)
    return bool(arrays) and all(arrays)


def complete_alat_check(rows):
    """只读完整行的原精度及recorder四位精度侧加速度，含切点两邻帧。"""
    import numpy as np
    result = dict(pass_alat=False, unrounded_limit_mps2=.391,
                  recorder_limit_mps2=.4, model_step_executions=0)
    try:
        values = np.asarray(rows, dtype=float)
        if (values.ndim != 2 or values.shape[1] != 5 or len(values) < 3
                or not np.isfinite(values).all() or np.any(np.diff(values[:, 0]) <= 0.)):
            raise ValueError('complete_finite_increasing_five_state_rows_required')
        unrounded = np.abs(values[:, 4] * np.gradient(np.unwrap(values[:, 3]), values[:, 0]))
        recorded = np.asarray([[round(float(row[0]), 1)] +
                               [round(float(x), 4) for x in row[1:]] for row in values])
        alat = np.abs(recorded[:, 4] * np.gradient(np.unwrap(recorded[:, 3]), recorded[:, 0]))
        result.update(max_unrounded_alat_mps2=float(np.max(unrounded)),
            max_recorder_alat_mps2=float(np.max(alat)),
            unrounded_bad_frames=int(np.sum(unrounded > .391)),
            recorder_bad_frames=int(np.sum(alat > .4)),
            pass_alat=bool(np.all(unrounded <= .391) and np.all(alat <= .4)))
    except (ValueError, TypeError) as error:
        result['reason'] = str(error)
    return result


def zero_safety_violations(result):
    try:
        ratios = result['detail']['ratios']
        return all(float(ratios[key]) == 0. for key in ('ttc', 'lat', 'alat'))
    except (KeyError, TypeError, ValueError):
        return False


def execution_rows6(rows, execution):
    import numpy as np
    values=np.asarray(rows,dtype=float)
    frames=execution.get('frames',())
    require(values.shape==(len(frames)+1,5) and bool(frames), 'complete_execution_rows_required')
    previous=[frames[0]['previous_acceleration_mps2']]+[x['actual_acceleration_mps2'] for x in frames]
    return np.column_stack((values,previous))


def execution_actions(execution):
    import numpy as np
    frames=execution['frames']
    require(all(len(x.get('steering',()))==1 for x in frames),'one_raw_steering_packet_per_frame')
    return tuple(np.asarray([getter(x) for x in frames],dtype=float) for getter in (
        lambda x:x['command_mps'],lambda x:x['steering'][0]['raw_steering_rad'],
        lambda x:x['final_steering_rad']))


def table_execution_check(candidate, rows, execution):
    import numpy as np
    result=dict(pass_model=False, model_step_executions=0)
    try:
        actual=execution_rows6(rows,execution)
        commands,raw,final=execution_actions(execution)
        state=float(np.max(np.abs(actual-candidate.rows6))) if actual.shape==candidate.rows6.shape else None
        errors=[float(np.max(np.abs(a-b))) if a.shape==b.shape else None
                for a,b in zip((commands,raw,final),(candidate.commands,candidate.raw_delta,candidate.final_delta))]
        result.update(max_six_state_delta=state,max_command_delta=errors[0],
            max_raw_steering_delta=errors[1],max_final_steering_delta=errors[2],
            pass_model=execution.get('pass') is True and state is not None and state<=1e-8
                and all(x is not None and x<=1e-10 for x in errors))
    except (KeyError,ValueError,TypeError,RuntimeError):
        pass
    return result


def first_actual_action_bridge(candidate, source):
    import numpy as np
    error=float(np.max(np.abs(candidate.rows6[:2]-source.rows6[:2])))
    return json_native(dict(max_first_two_six_state_delta=error, six_state_tolerance=1e-8,
        first_command_exact=candidate.commands[0]==source.commands[0],
        first_raw_steering_exact=candidate.raw_delta[0]==source.raw_delta[0],
        first_final_steering_exact=candidate.final_delta[0]==source.final_delta[0],
        pass_bridge=error<=1e-8 and candidate.commands[0]==source.commands[0]
            and candidate.raw_delta[0]==source.raw_delta[0]
            and candidate.final_delta[0]==source.final_delta[0]))


def identity_verification(source,candidate,result,rows,execution,model,alat,safe,zero,
                          *, native_execution_valid=False):
    import numpy as np
    try:
        actual=execution_rows6(rows,execution)
        commands,raw,final=execution_actions(execution)
        frames=execution['frames']
        expected_path=path_signature(source.path)
        valid=dict(raw_steering=np.array_equal(raw,source.raw_delta),
            final_steering=np.array_equal(final,source.final_delta),
            commands=np.array_equal(commands,source.commands),
            P2_last_acc=execution.get('pass') is True and np.array_equal(actual[:,5],source.rows6[:,5]),
            six_state=np.array_equal(actual,source.rows6),
            path_bytes=bool(frames) and all(x.get('actual_path_identity')==id(source.path)
                and x.get('actual_path_signature_before')==expected_path
                and x.get('actual_path_signature_after')==expected_path for x in frames),
            first_goal=result is not None and result.get('end')==4 and len(frames)==len(source.commands),
            terminal=result is not None and result.get('arrival')==source.rows6[-1,0],
            fullsafe=safe(result) and zero(result),full_ALAT=alat.get('pass_alat') is True,
            original_process=all(x.get('original_process_identity')==id(source.original_process) for x in frames),
            identity_policy_execution=native_execution_valid is True
                and model.get('pass_model') is True and candidate.role=='identity'
                and all(x['steering'][0].get('policy_identity')==id(candidate) for x in frames),
            frame_count=len(frames),candidate_signature=candidate.signature())
        return json_native(valid)
    except (KeyError,ValueError,TypeError,RuntimeError):
        return dict(frame_count=0)
