"""Robust AXC=Z hand-eye fit with untouched validation poses and explicit quality gates.

Both camera mounts reduce to A X C = Z with C = T_camera_target:
- wrist (eye in hand, target fixed in the scene): A = T_base_eef, X = T_eef_camera, Z = T_base_target;
- fixed (eye to hand, target held by the gripper): A = inv(T_base_eef), X = T_base_camera, Z = T_eef_target.
"""

import cv2
import numpy as np

from embodied.core.transforms import from_rt, inverse, mean_pose, pose_error, transform

TRANSLATION_LIMIT = 0.008
ROTATION_LIMIT_DEG = 2.0
MOUNTS = {  # mount: (calibration type, equation, X name, Z name)
    "wrist": ("eye_in_hand", "T_base_eef @ T_eef_camera @ T_camera_target = T_base_target", "T_eef_camera", "T_base_target"),
    "fixed": ("eye_to_hand", "T_base_camera @ T_camera_target = T_base_eef @ T_eef_target", "T_base_camera", "T_eef_target"),
}


def diversity(poses, minimum=10):
    poses = [transform(p) for p in poses]
    if len(poses) < minimum:
        raise ValueError(f"Need at least {minimum} distinct poses")
    rotations = [cv2.Rodrigues(poses[0][:3, :3].T @ p[:3, :3])[0].ravel() for p in poses[1:]]
    singular = np.linalg.svd(rotations, compute_uv=False)
    errors = [pose_error(a, b) for i, a in enumerate(poses) for b in poses[i + 1:]]
    span_m, span_deg = np.max(errors, axis=0)
    rank = int(np.count_nonzero(singular > np.radians(4)))
    if span_m < 0.04 or span_deg < 20 or rank < 2:
        raise ValueError(f"Insufficient motion diversity: {span_m=:.3f}, {span_deg=:.1f}, {rank=}")
    return {"translation_span_m": float(span_m), "rotation_span_deg": float(span_deg),
            "rotation_axis_rank": rank, "rotation_singular_values_rad": singular.tolist()}


def _fit(a, c, indices, method):
    if method == "relative_svd":
        # P X = X Q, P=inv(A_j) A_i, Q=C_j inv(C_i).
        pairs = [(inverse(a[j]) @ a[i], c[j] @ inverse(c[i]))
                 for offset, i in enumerate(indices) for j in indices[offset + 1:]]
        system = np.vstack([np.kron(np.eye(3), p[:3, :3]) - np.kron(q[:3, :3].T, np.eye(3))
                            for p, q in pairs])
        _, singular, vt = np.linalg.svd(system, full_matrices=False)
        if singular[-2] < 1e-5:
            raise ValueError("Unobservable hand-eye rotation")
        raw = vt[-1].reshape(3, 3, order="F")
        if np.linalg.det(raw) < 0:
            raw = -raw
        u, _, rv = np.linalg.svd(raw)
        rotation = u @ np.diag([1, 1, np.linalg.det(u @ rv)]) @ rv
        lhs = np.vstack([p[:3, :3] - np.eye(3) for p, _ in pairs])
        rhs = np.concatenate([rotation @ q[:3, 3] - p[:3, 3] for p, q in pairs])
        translation, _, rank, _ = np.linalg.lstsq(lhs, rhs, rcond=None)
        if rank < 3:
            raise ValueError("Unobservable hand-eye translation")
        x = from_rt(rotation, translation)
        return x, mean_pose([a[i] @ x @ c[i] for i in indices])
    # OpenCV solves A X = Z B. For eye-in-hand B=inv(T_camera_target).
    b = [inverse(c[i]) for i in indices]
    aa = [a[i] for i in indices]
    rx, tx, rz, tz = cv2.calibrateRobotWorldHandEye(
        [p[:3, :3] for p in aa], [p[:3, 3] for p in aa],
        [p[:3, :3] for p in b], [p[:3, 3] for p in b], method=method)
    return from_rt(rx, tx), from_rt(rz, tz)


def residuals(a, c, x, z):
    return np.array([pose_error(aa @ x @ cc, z) for aa, cc in zip(a, c)])


def metrics(errors):
    return {"count": len(errors), "translation_rms_m": float(np.sqrt(np.mean(errors[:, 0]**2))),
            "translation_max_m": float(np.max(errors[:, 0])),
            "rotation_rms_deg": float(np.sqrt(np.mean(errors[:, 1]**2))),
            "rotation_max_deg": float(np.max(errors[:, 1]))}


def solve(samples, mount="wrist"):
    """Rows contain id, split=fit/validation, T_base_eef and T_camera_target; mount is wrist or fixed.

    Thresholds are initial engineering gates, not a guarantee of task accuracy.
    Validation samples never participate in fitting, trimming or model selection.
    """
    ids = [row["id"] for row in samples]
    if len(set(ids)) != len(ids):
        raise ValueError("Duplicate sample IDs")
    if any(row["split"] not in {"fit", "validation"} for row in samples):
        raise ValueError("Sample split must be fit or validation")
    if mount not in MOUNTS:
        raise ValueError("Mount must be wrist or fixed")
    kind, equation, x_name, z_name = MOUNTS[mount]
    side = (lambda pose: pose) if mount == "wrist" else inverse
    fit = [row for row in samples if row["split"] == "fit"]
    val = [row for row in samples if row["split"] == "validation"]
    robot = [transform(row["T_base_eef"]) for row in fit]
    a = [side(pose) for pose in robot]
    c = [transform(row["T_camera_target"]) for row in fit]
    # Repeated burst frames are one pose, not independent calibration samples.
    for i, first in enumerate(samples):
        for second in samples[i + 1:]:
            distance, angle = pose_error(first["T_base_eef"], second["T_base_eef"])
            if distance < 0.001 and angle < 0.5:
                raise ValueError("Duplicate/near-identical robot poses; use one burst per pose")
    info = diversity(robot)
    rng = np.random.default_rng(20260908)
    subsets = [np.arange(len(a))] + [rng.choice(len(a), 6, replace=False) for _ in range(96)]
    candidates = []
    limits = np.array([TRANSLATION_LIMIT, ROTATION_LIMIT_DEG])
    methods = [("relative_svd", "relative_svd")]
    if hasattr(cv2, "calibrateRobotWorldHandEye"):
        methods += [("shah", cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH),
                    ("li", cv2.CALIB_ROBOT_WORLD_HAND_EYE_LI)]
    for name, method in methods:
        for indices in subsets:
            try:
                x, z = _fit(a, c, indices, method)
                errors = residuals(a, c, x, z)
                inliers = np.flatnonzero(np.all(errors <= limits, axis=1))
                if len(inliers) < max(10, int(np.ceil(0.8 * len(a)))):
                    continue
                # Recheck diversity AFTER outlier rejection (missing in old robust fit).
                diversity([robot[i] for i in inliers])
                x, z = _fit(a, c, inliers, method)
                errors = residuals(a, c, x, z)
                if not np.all(errors[inliers] <= limits):
                    continue
                score = float(np.mean(np.sum((errors[inliers] / limits)**2, axis=1)))
                candidates.append((-len(inliers), score, name, method, x, z, inliers, errors))
            except (cv2.error, ValueError, np.linalg.LinAlgError):
                continue
    if not candidates:
        raise ValueError("No fit passed residual/diversity gates with <=20% excluded samples")
    _, _, name, method, x, z, inliers, errors = min(candidates, key=lambda item: item[:2])
    # Leave-one-out stability of the mounting transform, not only board location.
    stability = []
    for omitted in inliers:
        fold = [i for i in inliers if i != omitted]
        diversity([robot[i] for i in fold], minimum=9)
        xx, _ = _fit(a, c, fold, method)
        stability.append(pose_error(x, xx))
    stability = np.asarray(stability)
    stable = bool(np.all(stability <= limits))
    validation = None
    validation_pass = False
    if val:
        vr = [transform(row["T_base_eef"]) for row in val]
        vc = [transform(row["T_camera_target"]) for row in val]
        ve = residuals([side(pose) for pose in vr], vc, x, z)
        validation = {**metrics(ve), "sample_ids": [row["id"] for row in val],
                      "per_sample_m_deg": ve.tolist(), "refitted": False}
        try:
            validation["motion_diversity"] = diversity(vr, minimum=4)
            validation_pass = bool(np.all(ve <= limits))
        except ValueError as exc:
            validation["failure"] = str(exc)
    return {"schema_version": 2, "calibration_type": kind, "mount": mount,
            "status": "validated" if stable and validation_pass else "candidate",
            "equation": equation, "camera_frame": "RGB optical (+X right, +Y down, +Z forward)",
            x_name: x.tolist(), z_name: z.tolist(), "method": name,
            "fit": metrics(errors[inliers]), "motion_diversity": diversity([robot[i] for i in inliers]),
            "input_motion_diversity": info,
            "inlier_ids": [fit[i]["id"] for i in inliers],
            "excluded_ids": [fit[i]["id"] for i in range(len(a)) if i not in inliers],
            "fit_residuals_m_deg": errors.tolist(),
            "leave_one_out": {**metrics(stability), "passed": stable},
            "validation": validation,
            "quality_limits": {"translation_m": TRANSLATION_LIMIT,
                               "rotation_deg": ROTATION_LIMIT_DEG,
                               "max_outlier_fraction": 0.2}}
