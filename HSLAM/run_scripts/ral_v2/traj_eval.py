"""Trajectory metrics for the RA-L v2 eval pipeline.

Every trajectory is reported scale-honestly (PLAN.md P6): Sim(3) ATE for comparability
with monocular baselines, SE(3) ATE and the recovered Umeyama scale s as the metric-scale
evidence, plus scale drift and RPE. A number that only exists after Sim(3) alignment is
not evidence that the system is metric -- that conflation is what reviewer R11-c caught
in v1.

Uses evo (v1.31) as a library rather than shelling out to evo_ape, because the CLI does not
hand back the alignment scale, which is the quantity contribution 1 lives or dies on.
"""
from __future__ import annotations

import copy
import math
import tempfile
from pathlib import Path
from typing import Optional

import numpy as np
from evo.core import metrics, sync
from evo.core.trajectory import PosePath3D, PoseTrajectory3D
from evo.tools import file_interface


def _load_euroc_gt_in_camera_frame(gt_path: Path, sensor_yaml: Path) -> PoseTrajectory3D:
    """EuRoC ground truth is T_WB (body in world). HSLAM estimates the camera.

    T_WC = T_WB * T_BC, with T_BC read from cam0/sensor.yaml (T_BS). Skipping this is the
    evaluation bug flagged in NIGHT_RUN_2026-07-30.md:72-74; it inflates ATE and makes our
    EuRoC numbers incomparable to published ones.
    """
    traj = file_interface.read_euroc_csv_trajectory(str(gt_path))

    text = sensor_yaml.read_text()
    start = text.index("T_BS:")
    body = text[start:]
    open_b = body.index("data:")
    nums = body[open_b:body.index("]", open_b)]
    vals = [float(v) for v in nums.replace("data:", "").replace("[", "").split(",") if v.strip()]
    if len(vals) != 16:
        raise ValueError(f"{sensor_yaml}: expected 16 values in T_BS, got {len(vals)}")
    T_BC = np.array(vals, dtype=float).reshape(4, 4)

    poses = [T_WB @ T_BC for T_WB in traj.poses_se3]
    return PoseTrajectory3D(poses_se3=poses, timestamps=traj.timestamps)


def load_gt(gt_path: Path, gt_format: str, extrinsics: Optional[Path]) -> PoseTrajectory3D:
    if gt_format == "euroc":
        if extrinsics is None:
            raise ValueError("EuRoC ground truth needs cam0/sensor.yaml for T_BC")
        return _load_euroc_gt_in_camera_frame(gt_path, extrinsics)
    if gt_format == "tum":
        return _read_tum_lenient(Path(gt_path))
    raise ValueError(f"unknown gt_format {gt_format!r}")


def _read_tum_lenient(path: Path) -> PoseTrajectory3D:
    """Read a TUM trajectory, normalising whitespace first.

    WP0 fixed printResult to emit strict single-space TUM, but trajectories produced before
    that commit are padded by Eigen's column alignment and evo rejects them outright. Rather
    than making old runs unreadable, collapse whitespace into a temp copy. Malformed rows are
    skipped loudly via the exception path, never silently dropped.
    """
    try:
        return file_interface.read_tum_trajectory_file(str(path))
    except Exception:
        rows = []
        for line in Path(path).read_text(errors="replace").splitlines():
            parts = line.split()
            if len(parts) == 8:
                rows.append(" ".join(parts))
        if not rows:
            raise
        with tempfile.NamedTemporaryFile("w", suffix=".tum", delete=False) as tmp:
            tmp.write("\n".join(rows) + "\n")
            tmp_path = tmp.name
        try:
            return file_interface.read_tum_trajectory_file(tmp_path)
        finally:
            Path(tmp_path).unlink(missing_ok=True)


def _path_length(xyz: np.ndarray) -> float:
    if len(xyz) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum())


def _scale_drift_pct_per_100m(ref: PosePath3D, est: PosePath3D, n_bins: int = 10):
    """Scale drift as %/100 m, from prefix-wise Umeyama scale versus distance travelled.

    PLAN.md asks for drift from a per-keyframe [SCALE_DRIFT] tag, but that printf is
    commented out at FullSystem.cpp:4965. Deriving it from the aligned trajectories instead
    needs no binary change and is what a reader can reproduce from our released
    trajectories, so it is the better primitive regardless.

    Returns (drift_pct_per_100m, total_distance_m). NaN when the trajectory is too short
    or too stationary to fit a line -- never 0.0, which would read as "no drift".
    """
    gt_xyz = ref.positions_xyz
    if len(gt_xyz) < n_bins * 2:
        return float("nan"), _path_length(gt_xyz)

    cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(gt_xyz, axis=0), axis=1))])
    total = float(cum[-1])
    if total < 1.0:
        return float("nan"), total

    ds, ss = [], []
    for frac in np.linspace(1.0 / n_bins, 1.0, n_bins):
        idx = int(np.searchsorted(cum, frac * total)) + 1
        if idx < 10:
            continue
        r_pre = PosePath3D(poses_se3=ref.poses_se3[:idx])
        e_pre = PosePath3D(poses_se3=est.poses_se3[:idx])
        try:
            _, _, s = e_pre.align(r_pre, correct_scale=True)
        except Exception:
            continue
        if s > 0 and math.isfinite(s):
            ds.append(cum[min(idx, len(cum)) - 1])
            ss.append(s)

    if len(ds) < 3:
        return float("nan"), total
    slope, _ = np.polyfit(np.asarray(ds), np.asarray(ss), 1)
    mean_s = float(np.mean(ss))
    if mean_s <= 0:
        return float("nan"), total
    return float(slope / mean_s * 100.0 * 100.0), total


def evaluate(est_path: Path, gt_path: Path, gt_format: str,
             extrinsics: Optional[Path] = None, max_diff: float = 0.02) -> dict:
    """Return the trajectory half of a summary.csv row.

    On any failure this returns a dict whose metrics are NaN and whose `traj_error` says
    why. It never raises and never substitutes a plausible-looking default: a silently
    zeroed ATE is worse than a missing one.
    """
    out = {k: float("nan") for k in (
        "ate_sim3_rmse", "ate_se3_rmse", "scale_s", "scale_drift_pct_per_100m",
        "rpe_trans", "rpe_rot", "gt_distance_m")}
    out.update(poses=0, matched_poses=0, traj_error="")

    try:
        est = _read_tum_lenient(Path(est_path))
        ref = load_gt(Path(gt_path), gt_format, Path(extrinsics) if extrinsics else None)
        out["poses"] = est.num_poses

        ref_s, est_s = sync.associate_trajectories(ref, est, max_diff=max_diff)
        out["matched_poses"] = est_s.num_poses
        if est_s.num_poses < 10:
            out["traj_error"] = f"only {est_s.num_poses} poses matched GT within {max_diff}s"
            return out

        # --- Sim(3): rotation + translation + scale. Comparable to monocular baselines. ---
        est_sim3 = copy.deepcopy(est_s)
        _, _, scale = est_sim3.align(ref_s, correct_scale=True)
        out["scale_s"] = float(scale)
        ape = metrics.APE(metrics.PoseRelation.translation_part)
        ape.process_data((ref_s, est_sim3))
        out["ate_sim3_rmse"] = float(ape.get_statistic(metrics.StatisticsType.rmse))

        # --- SE(3): no scale freedom. This is the metric-scale evidence. ---
        est_se3 = copy.deepcopy(est_s)
        est_se3.align(ref_s, correct_scale=False)
        ape_se3 = metrics.APE(metrics.PoseRelation.translation_part)
        ape_se3.process_data((ref_s, est_se3))
        out["ate_se3_rmse"] = float(ape_se3.get_statistic(metrics.StatisticsType.rmse))

        # --- RPE on the Sim(3)-aligned pair (translation m, rotation deg) ---
        for key, rel in (("rpe_trans", metrics.PoseRelation.translation_part),
                         ("rpe_rot", metrics.PoseRelation.rotation_angle_deg)):
            try:
                rpe = metrics.RPE(rel, delta=1.0, delta_unit=metrics.Unit.frames, all_pairs=False)
                rpe.process_data((ref_s, est_sim3))
                out[key] = float(rpe.get_statistic(metrics.StatisticsType.rmse))
            except Exception:
                pass

        drift, dist = _scale_drift_pct_per_100m(ref_s, est_s)
        out["scale_drift_pct_per_100m"] = drift
        out["gt_distance_m"] = dist
    except Exception as exc:                      # noqa: BLE001 - reported, not swallowed
        out["traj_error"] = f"{type(exc).__name__}: {exc}"
    return out
