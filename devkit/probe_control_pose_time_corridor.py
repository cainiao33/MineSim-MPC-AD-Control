"""仅提供冻结数学函数的兼容导入；不包含研究启动器。"""

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
