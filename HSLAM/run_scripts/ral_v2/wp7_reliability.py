#!/usr/bin/env python3
"""WP7 as-run table: coverage, run success, the ATE cell, reliability metrics R1-R4 and the
sensitivity analyses S1-S5, for every system, from one manifest.

Implements DECISIONS.md "WP7 as-run table -- coverage, run success and reliability metrics"
(PRE-REGISTERED 2026-09-25). Research record: docs/ral_v2_resubmission/wp7_baselines/
COVERAGE_AND_RELIABILITY_LITERATURE.md. Per-run coverage is coverage.py; ATE, SE(3) ATE and scale come
from traj_eval.evaluate(), the code that scores HSLAM's summary.csv rows, so baselines and HSLAM are
scored by one implementation.

usage:
  wp7_reliability.py --manifest M.csv --out DIR [--root DIR] [--ref-system NAME]
                     [--rejected-init {ignore,fail}] [--plot]
  wp7_reliability.py --manifest wp7_b6_manifest.csv --validate [--root DIR]
  wp7_reliability.py --selftest

Before first use, --validate must reproduce the pre-registration's validation table (section 6).

Manifest: a CSV with one row per run; lines starting with '#' are comments. Columns:
  system, dataset, sequence, rep   identify the run (unique together)
  traj            trajectory file (TUM format), relative to --root unless absolute
  output          every_frame | keyframe
  time_map        native | index | index/<K>      (coverage.map_times)
  frames_given    frames the system was given after its stride; required for every_frame
  outcome         ran (default) | harness_failure. A harness failure is re-run and logged, and never
                  counts as a system failure (section 2). A missing or empty traj IS a system failure.
  rejected_init_accepted   HSLAM only, 0/1 (eval_run.py's init_rejected_accepted). It is used only
                  with --rejected-init fail, because whether such a run counts is an open user decision.

Outputs in --out: runs.csv, cells.csv, auc.csv, curves.csv, pairs.csv, summary.json (+ PNGs with --plot).
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import socket
import subprocess
import sys
import tempfile
import time
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import coverage as cv            # noqa: E402
import datasets as ds            # noqa: E402
import traj_eval                 # noqa: E402
from evo.core.trajectory import PoseTrajectory3D   # noqa: E402
from evo.tools import file_interface               # noqa: E402

HSLAM_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_ROOT = Path(os.environ.get("WP7_BASELINES", Path.home() / "Dev" / "baselines"))

BAR = 0.9                                 # section 2: success iff C >= BAR * C_ref
BARS_S1 = (0.8, 0.95)                     # S1
AUC_PRIMARY = (-3.0, -1.0)                # R4: log10(ATE / GT path length), i.e. 0.1-10 %
AUC_S5 = ((-3.0, -2.0), (-3.0, 0.0))      # S5: 0.1-1 % and 0.1-100 %
GRID = np.round(np.linspace(-3.0, 0.0, 301), 6)
GT_MAX_DIFF = 0.02                        # s; traj_eval.evaluate's association tolerance
ALIGNS = ("sim3", "se3")
ATE = {"sim3": "ate_sim3_rmse", "se3": "ate_se3_rmse"}
REQUIRED = ("system", "dataset", "sequence", "rep", "traj", "output", "time_map")

# Section 6's validation table: (C_span without tail credit, C_span, F_own or None, C), one run each,
# binary 692adec6 on ws. --validate must reproduce every row to within VALIDATE_TOL.
EXPECTED = {
    ("HSLAM2", "tum", "freiburg1_desk", "1"): (0.948, 0.966, None, 0.966),
    ("HSLAM mono (A0)", "tum", "freiburg1_desk", "1"): (0.539, 0.554, None, 0.554),
    ("HSLAM mono (A0)", "tum", "freiburg1_desk", "2"): (0.503, 0.518, None, 0.518),
    ("ORB-SLAM3", "tum", "freiburg1_desk", "2"): (0.995, 0.995, 0.995, 0.995),
    ("DPVO", "tum", "freiburg1_desk", "1"): (1.000, 1.000, 1.000, 1.000),
    ("DROID-SLAM", "tum", "freiburg1_desk", "1"): (1.000, 1.000, 1.000, 1.000),
    ("MASt3R-SLAM", "tum", "freiburg1_desk", "1"): (0.863, 1.000, None, 1.000),
    ("DropD-SLAM", "tum", "freiburg1_desk", "2"): (0.972, 0.972, 1.000, 0.972),
    ("HSLAM2", "kitti", "07", "1"): (0.979, 1.000, None, 1.000),
    ("HSLAM mono (A0)", "kitti", "07", "1"): (0.981, 1.000, None, 1.000),
    ("ORB-SLAM3 (KF)", "kitti", "07", "2"): (0.993, 1.000, None, 1.000),
    ("DROID-SLAM", "kitti", "07", "1"): (0.998, 1.000, 1.000, 1.000),
    ("DropD-SLAM", "kitti", "07", "1"): (1.000, 1.000, 1.000, 1.000),
}
VALIDATE_TOL = 0.0015          # the table is printed to 3 decimals

RUN_COLUMNS = [
    "system", "dataset", "sequence", "rep", "status", "output", "time_map", "frames_given",
    "n_poses", "n_own", "n_dup", "t_first_s", "t_last_s", "gap_max_s",
    "c_span_raw", "c_span", "f_own", "c", "c_ref", "success", "fail_reason",
    "success_bar0.8", "success_bar0.95", "rejected_init_accepted",
    "matched_poses", "ate_sim3_rmse", "ate_se3_rmse", "scale_s", "drift_local_pct_per_100m",
    "fill_ate_sim3", "fill_ate_se3", "gt_path_m", "traj_error", "traj",
]


# ------------------------------------------------------------------------------------------------
# per run
# ------------------------------------------------------------------------------------------------

def read_manifest(path: Path) -> list[dict]:
    lines = [l for l in Path(path).read_text().splitlines()
             if l.strip() and not l.lstrip().startswith("#")]
    rows = list(csv.DictReader(lines))
    if not rows:
        raise SystemExit(f"{path}: no runs")
    missing = [c for c in REQUIRED if c not in rows[0]]
    if missing:
        raise SystemExit(f"{path}: missing columns {missing}")
    seen = set()
    for r in rows:
        for k in list(r):
            r[k] = (r[k] or "").strip()
        r["dataset"] = r["dataset"].lower()
        key = (r["system"], r["dataset"], r["sequence"], r["rep"])
        if key in seen:
            raise SystemExit(f"{path}: duplicate run {key}")
        seen.add(key)
        r["outcome"] = r.get("outcome") or "ran"
        if r["outcome"] not in ("ran", "harness_failure"):
            raise SystemExit(f"{path}: {key}: outcome must be ran or harness_failure")
        r["frames_given"] = int(r["frames_given"]) if r.get("frames_given") else None
        ria = r.get("rejected_init_accepted", "")
        r["rejected_init_accepted"] = int(ria) if ria != "" else None
    return rows


def _traj(poses, stamps) -> PoseTrajectory3D:
    return PoseTrajectory3D(poses_se3=list(poses), timestamps=np.asarray(stamps, dtype=float))


def _evaluate(traj: PoseTrajectory3D, spec, tmpdir: Path, tag: str) -> dict:
    path = tmpdir / f"{hashlib.md5(tag.encode()).hexdigest()}.tum"
    file_interface.write_tum_trajectory_file(str(path), traj)
    return traj_eval.evaluate(path, spec.gt, spec.gt_format, spec.extrinsics, max_diff=GT_MAX_DIFF)


def fill_to_timeline(traj: PoseTrajectory3D, timeline: np.ndarray) -> PoseTrajectory3D:
    """S3, ETH3D-style fill: a pose at every image timestamp. Poses are interpolated inside the tracked
    span (linear translation, slerp rotation) and held constant before the first and after the last."""
    from scipy.spatial.transform import Rotation, Slerp
    t = np.asarray(traj.timestamps, dtype=float)
    tt = np.clip(timeline, t[0], t[-1])
    xyz = traj.positions_xyz
    pos = np.column_stack([np.interp(tt, t, xyz[:, i]) for i in range(3)])
    q = traj.orientations_quat_wxyz
    rot = Slerp(t, Rotation.from_quat(q[:, [1, 2, 3, 0]]))(tt).as_matrix()
    poses = []
    for R, p in zip(rot, pos):
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, p
        poses.append(T)
    return _traj(poses, timeline)


def gt_path_length(spec, timeline: np.ndarray) -> float:
    """R4's denominator: GT path length over the WHOLE image timeline, GT sampled at the image
    timestamps (within GT_MAX_DIFF), so mocap jitter between frames does not inflate it."""
    gt = traj_eval.load_gt(spec.gt, spec.gt_format, spec.extrinsics)
    tg = np.asarray(gt.timestamps, dtype=float)
    j = np.clip(np.searchsorted(tg, timeline), 1, len(tg) - 1)
    k = np.where(np.abs(timeline - tg[j - 1]) <= np.abs(timeline - tg[j]), j - 1, j)
    ok = np.abs(tg[k] - timeline) <= GT_MAX_DIFF
    pts = gt.positions_xyz[k[ok]]
    return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())


def score_run(row: dict, root: Path, tmpdir: Path, seqcache: dict) -> tuple[dict, object]:
    """Coverage, ATE and the S3 fill for one run. Returns (runs.csv row, own-pose trajectory or None).

    TimelineError / TimeMappingError propagate: an input defect stops the run, it is never scored."""
    nan = float("nan")
    key = (row["dataset"], row["sequence"])
    if key not in seqcache:
        spec = ds.resolve(*key)
        tl = cv.load_timeline(*key)
        seqcache[key] = (spec, tl, gt_path_length(spec, tl))
    spec, timeline, gtlen = seqcache[key]

    out = {k: nan for k in RUN_COLUMNS}
    out.update({k: row[k] for k in ("system", "dataset", "sequence", "rep", "output", "time_map",
                                    "frames_given", "rejected_init_accepted", "traj")})
    out.update(gt_path_m=gtlen, traj_error="", fail_reason="", n_poses=0, n_own=0, n_dup=0)
    if row["outcome"] == "harness_failure":
        out["status"] = "harness_failure"
        return out, None

    path = Path(os.path.expanduser(row["traj"]))
    path = path if path.is_absolute() else root / path
    if not path.exists() or path.stat().st_size == 0:
        out.update(status="no_trajectory", c=0.0, c_span=0.0, c_span_raw=0.0)
        return out, None

    raw = traj_eval._read_tum_lenient(path)
    t = cv.map_times(raw.timestamps, row["time_map"], timeline)
    try:
        cov = cv.coverage(t, timeline, row["output"], row["frames_given"])
    except cv.TimeMappingError as exc:
        raise cv.TimeMappingError(f"{row['system']} {key} rep {row['rep']} ({path}): {exc}") from None
    out.update(n_poses=cov.n_poses, n_own=cov.n_own, n_dup=cov.n_dup, t_first_s=cov.t_first_s,
               t_last_s=cov.t_last - timeline[0] if cov.n_own else nan, gap_max_s=cov.gap_max_s,
               c_span_raw=cov.c_span_raw, c_span=cov.c_span, f_own=cov.f_own, c=cov.c)
    if cov.n_own < 2:
        out["status"] = "no_trajectory"
        return out, None

    keep = cv.own_pose_index(t, timeline)
    own = _traj([raw.poses_se3[i] for i in keep], t[keep])
    ev = _evaluate(own, spec, tmpdir, "|".join((*key, row["system"], row["rep"], "own")))
    out.update({k: ev[k] for k in ("matched_poses", "ate_sim3_rmse", "ate_se3_rmse", "scale_s",
                                   "drift_local_pct_per_100m", "traj_error")})
    if ev["matched_poses"] < 2:
        out.update(status="no_associable_poses", c=0.0)
        return out, None
    out["status"] = "ok"

    fill = _evaluate(fill_to_timeline(own, timeline), spec, tmpdir,
                     "|".join((*key, row["system"], row["rep"], "fill")))
    out["fill_ate_sim3"], out["fill_ate_se3"] = fill["ate_sim3_rmse"], fill["ate_se3_rmse"]
    return out, own


# ------------------------------------------------------------------------------------------------
# the table
# ------------------------------------------------------------------------------------------------

def _finite(x) -> bool:
    return isinstance(x, (int, float)) and math.isfinite(x)


def counted(runs):
    """Runs that count toward n: everything but harness failures."""
    return [r for r in runs if r["status"] != "harness_failure"]


def c_ref_table(runs) -> dict:
    """C_ref(seq) = the largest per-system median C on that sequence, over every as-run row."""
    per = defaultdict(lambda: defaultdict(list))
    for r in counted(runs):
        per[(r["dataset"], r["sequence"])][r["system"]].append(r["c"])
    return {seq: max(float(np.median(v)) for v in systems.values()) for seq, systems in per.items()}


def success(r: dict, c_ref: float, bar: float, rejected_init: str) -> tuple[bool, str]:
    if r["status"] != "ok":
        return False, r["status"]
    if rejected_init == "fail" and r["rejected_init_accepted"] == 1:
        return False, "rejected_init_accepted"
    if r["c"] < bar * c_ref - 1e-12:
        return False, f"coverage {r['c']:.3f} < {bar:g} x {c_ref:.3f}"
    if not (_finite(r["ate_sim3_rmse"]) and _finite(r["ate_se3_rmse"])):
        return False, "no ATE: " + (r["traj_error"] or "unknown")
    return True, ""


def _med_iqr(vals):
    if not vals:
        return float("nan"), float("nan"), float("nan")
    a = np.asarray(vals, dtype=float)
    return float(np.median(a)), float(np.percentile(a, 25)), float(np.percentile(a, 75))


def cell_text(med: float, k: int, n: int) -> str:
    """Section 3: 'X (k/n)' when k <= n/2, else the median over the successful runs with k/n."""
    return f"X ({k}/{n})" if 2 * k <= n else f"{med:.4g} ({k}/{n})"


def build_cells(runs, cref, rejected_init) -> list[dict]:
    groups = defaultdict(list)
    for r in counted(runs):
        groups[(r["system"], r["dataset"], r["sequence"])].append(r)
    cells = []
    for (system, dset, seq), rs in groups.items():
        n = len(rs)
        c_ref = cref[(dset, seq)]
        ok = [r for r in rs if success(r, c_ref, BAR, rejected_init)[0]]
        k = len(ok)
        cell = dict(system=system, dataset=dset, sequence=seq, n=n, k=k, c_ref=c_ref,
                    harness_failures=sum(1 for r in runs if r["system"] == system
                                         and r["dataset"] == dset and r["sequence"] == seq
                                         and r["status"] == "harness_failure"),
                    c_median=float(np.median([r["c"] for r in rs])),
                    t_first_median_s=float(np.median([r["t_first_s"] for r in rs
                                                      if _finite(r["t_first_s"])]))
                    if any(_finite(r["t_first_s"]) for r in rs) else float("nan"),
                    scale_median=_med_iqr([r["scale_s"] for r in ok])[0])
        for al in ALIGNS:
            med, q1, q3 = _med_iqr([r[ATE[al]] for r in ok])
            cell.update({f"{al}_median": med, f"{al}_q1": q1, f"{al}_q3": q3,
                         f"{al}_cell": cell_text(med, k, n)})
            # S2: median over all n runs with every failure as +inf
            allv = [r[ATE[al]] if success(r, c_ref, BAR, rejected_init)[0] else math.inf for r in rs]
            cell[f"s2_{al}_inf_median"] = float(np.median(allv))
            # S3: ETH3D-style fill over the whole sequence, every run (no trajectory = +inf)
            fv = [r[f"fill_ate_{al}"] if _finite(r[f"fill_ate_{al}"]) else math.inf for r in rs]
            cell[f"s3_{al}_fill_median"] = float(np.median(fv))
        # S1: the same cell at the other coverage bars
        for bar in BARS_S1:
            okb = [r for r in rs if success(r, c_ref, bar, rejected_init)[0]]
            cell[f"s1_k_bar{bar:g}"] = len(okb)
            for al in ALIGNS:
                cell[f"s1_{al}_cell_bar{bar:g}"] = cell_text(
                    _med_iqr([r[ATE[al]] for r in okb])[0], len(okb), n)
        cells.append(cell)
    return cells


def _trapezoid(y, x) -> float:
    y, x = np.asarray(y, dtype=float), np.asarray(x, dtype=float)
    return float(np.sum((y[1:] + y[:-1]) * np.diff(x)) / 2.0) if len(x) > 1 else 0.0


def auc(frac: np.ndarray, lo: float, hi: float) -> float:
    m = (GRID >= lo - 1e-9) & (GRID <= hi + 1e-9)
    return _trapezoid(frac[m], GRID[m]) / (hi - lo)


def build_curves(runs, cref, rejected_init, warnings: list) -> tuple[list[dict], list[dict]]:
    """R4: per system x dataset x alignment, the fraction of ALL counted runs whose ATE / GT path length
    is <= x. A failed run is +inf and never succeeds."""
    groups = defaultdict(list)
    for r in counted(runs):
        groups[(r["system"], r["dataset"])].append(r)
    seqsets = defaultdict(dict)
    curves, aucs = [], []
    for (system, dset), rs in groups.items():
        nper = defaultdict(int)
        for r in rs:
            nper[r["sequence"]] += 1
        seqsets[dset][system] = tuple(sorted(nper))
        if len(set(nper.values())) > 1:
            warnings.append(f"R4 {system}/{dset}: n differs across sequences {dict(nper)}; the pooled "
                            f"fraction weights sequences unequally")
        for al in ALIGNS:
            x = np.array([r[ATE[al]] / r["gt_path_m"]
                          if success(r, cref[(r["dataset"], r["sequence"])], BAR, rejected_init)[0]
                          else math.inf for r in rs])
            frac = np.array([(x <= 10.0 ** g).mean() for g in GRID])
            for g, f in zip(GRID, frac):
                curves.append(dict(system=system, dataset=dset, align=al, log10_x=g, frac=f))
            a = dict(system=system, dataset=dset, align=al, n_runs=len(rs),
                     sequences=" ".join(sorted(nper)),
                     auc_primary=auc(frac, *AUC_PRIMARY))
            for lo, hi in AUC_S5:
                a[f"s5_auc_{lo:g}_{hi:g}"] = auc(frac, lo, hi)
            aucs.append(a)
    for dset, per in seqsets.items():
        if len(set(per.values())) > 1:
            warnings.append(f"R4 {dset}: systems were run on different sequence sets; their curves "
                            f"are not comparable: {per}")
    return curves, aucs


def _crop(traj: PoseTrajectory3D, t0: float, t1: float):
    t = np.asarray(traj.timestamps)
    idx = np.where((t >= t0) & (t <= t1))[0]
    return _traj([traj.poses_se3[i] for i in idx], t[idx]) if len(idx) >= 10 else None


def build_pairs(runs, owns, ref_system, cref, tmpdir, warnings) -> list[dict]:
    """S4: HSLAM-vs-baseline ATE over the COMMON time window (evo's --t_start/--t_end), per sequence.
    The window is [max of the two systems' median first-pose times, min of their median last-pose
    times]; every run of both systems with >= 10 poses inside it is scored on it."""
    if not any(r["system"] == ref_system for r in runs):
        warnings.append(f"S4: reference system {ref_system!r} not in the manifest; no pairs computed")
        return []
    by = defaultdict(lambda: defaultdict(list))
    for r in counted(runs):
        by[(r["dataset"], r["sequence"])][r["system"]].append(r)
    pairs = []
    for (dset, seq), systems in by.items():
        if ref_system not in systems:
            continue
        spec = ds.resolve(dset, seq)
        T0 = float(cv.load_timeline(dset, seq)[0])
        def span(rs):
            f = [r["t_first_s"] for r in rs if _finite(r["t_first_s"])]
            l = [r["t_last_s"] for r in rs if _finite(r["t_last_s"])]
            return (float(np.median(f)), float(np.median(l))) if f and l else None
        sa = span(systems[ref_system])
        for other, rb in systems.items():
            if other == ref_system:
                continue
            sb = span(rb)
            row = dict(dataset=dset, sequence=seq, ref=ref_system, other=other,
                       c_ref_median=float(np.median([r["c"] for r in systems[ref_system]])),
                       c_other_median=float(np.median([r["c"] for r in rb])))
            row["coverage_parity"] = (min(row["c_ref_median"], row["c_other_median"])
                                      / max(row["c_ref_median"], row["c_other_median"], 1e-12))
            if sa is None or sb is None or min(sa[1], sb[1]) <= max(sa[0], sb[0]):
                row["note"] = "no common window"
                pairs.append(row)
                continue
            w0, w1 = max(sa[0], sb[0]), min(sa[1], sb[1])
            row.update(window_start_s=w0, window_end_s=w1)
            for tag, rs in (("ref", systems[ref_system]), ("other", rb)):
                vals = {al: [] for al in ALIGNS}
                for r in rs:
                    own = owns.get((r["system"], dset, seq, r["rep"]))
                    c = _crop(own, T0 + w0, T0 + w1) if own is not None else None
                    if c is None:
                        continue
                    ev = _evaluate(c, spec, tmpdir, "|".join((dset, seq, r["system"], r["rep"], "s4")))
                    for al in ALIGNS:
                        if _finite(ev[ATE[al]]):
                            vals[al].append(ev[ATE[al]])
                for al in ALIGNS:
                    row[f"{tag}_{al}_median"] = _med_iqr(vals[al])[0]
                    row[f"{tag}_n"] = len(vals["sim3"])
            pairs.append(row)
    return pairs


# ------------------------------------------------------------------------------------------------
# output
# ------------------------------------------------------------------------------------------------

def _write_csv(path: Path, rows: list[dict], columns=None) -> None:
    columns = columns or list(dict.fromkeys(k for r in rows for k in r))
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=columns, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({k: (f"{v:.6g}" if isinstance(v, float) else v) for k, v in r.items()})


def _sha(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()[:16]


def provenance(args) -> dict:
    def g(*a):
        return subprocess.run(["git", *a], cwd=HSLAM_ROOT, capture_output=True, text=True).stdout.strip()
    import evo
    return dict(timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"), host=socket.gethostname(),
                commit=g("rev-parse", "--short", "HEAD"), dirty=bool(g("status", "--porcelain")),
                script_sha=_sha(Path(__file__)), coverage_sha=_sha(Path(cv.__file__)),
                manifest=str(args.manifest), manifest_sha=_sha(args.manifest), root=str(args.root),
                evo_version=evo.__version__, bar=BAR, bars_s1=BARS_S1, auc_primary=AUC_PRIMARY,
                auc_s5=AUC_S5, rejected_init=args.rejected_init, ref_system=args.ref_system)


def plot_curves(curves, out: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    by = defaultdict(lambda: defaultdict(list))
    for c in curves:
        by[(c["dataset"], c["align"])][c["system"]].append((c["log10_x"], c["frac"]))
    for (dset, al), systems in by.items():
        fig, ax = plt.subplots(figsize=(6, 4))
        colours = plt.cm.tab20(np.linspace(0, 1, 20))
        for i, (system, pts) in enumerate(systems.items()):
            x, y = zip(*sorted(pts))
            ax.step(100.0 * 10.0 ** np.asarray(x), y, where="post", label=system,
                    color=colours[(2 * i) % 20 + (i // 10) % 2], linestyle=("-", "--", ":")[i % 3])
        ax.set_xscale("log")
        ax.set_xlabel(f"{'Sim(3)' if al == 'sim3' else 'SE(3)'} ATE / GT path length (%)")
        ax.set_ylabel("fraction of runs")
        ax.set_ylim(0, 1.02)
        ax.axvspan(100 * 10 ** AUC_PRIMARY[0], 100 * 10 ** AUC_PRIMARY[1], color="0.93", zorder=0)
        ax.set_title(f"{dset}: success curve ({al})")
        ax.legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(out / f"success_curve_{dset}_{al}.png", dpi=150)
        plt.close(fig)


def run(args) -> tuple[list[dict], dict]:
    manifest = read_manifest(args.manifest)
    warnings: list[str] = []
    seqcache: dict = {}
    runs, owns = [], {}
    with tempfile.TemporaryDirectory(prefix="wp7rel_") as td:
        tmp = Path(td)
        for row in manifest:
            r, own = score_run(row, args.root, tmp, seqcache)
            runs.append(r)
            if own is not None:
                owns[(r["system"], r["dataset"], r["sequence"], r["rep"])] = own
        cref = c_ref_table(runs)
        for r in runs:
            r["c_ref"] = cref.get((r["dataset"], r["sequence"]), float("nan"))
            if r["status"] == "harness_failure":
                r["success"] = ""
                continue
            ok, why = success(r, r["c_ref"], BAR, args.rejected_init)
            r["success"], r["fail_reason"] = int(ok), why
            for bar in BARS_S1:
                r[f"success_bar{bar:g}"] = int(success(r, r["c_ref"], bar, args.rejected_init)[0])
            if r["rejected_init_accepted"] == 1 and args.rejected_init == "ignore":
                warnings.append(f"{r['system']} {r['dataset']}/{r['sequence']} rep {r['rep']}: continued "
                                f"on a rejected initialisation; counted as usable (--rejected-init ignore)")
            if r["status"] == "ok" and r["c"] >= BAR * r["c_ref"] and not ok:
                warnings.append(f"{r['system']} {r['dataset']}/{r['sequence']} rep {r['rep']}: covered "
                                f"the sequence but has no ATE ({r['traj_error']})")
        cells = build_cells(runs, cref, args.rejected_init)
        curves, aucs = build_curves(runs, cref, args.rejected_init, warnings)
        pairs = build_pairs(runs, owns, args.ref_system, cref, tmp, warnings) if args.out else []
    summary = dict(provenance=provenance(args), c_ref={f"{d}/{s}": v for (d, s), v in cref.items()},
                   warnings=warnings, cells=cells, auc=aucs)
    if args.out:
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        _write_csv(out / "runs.csv", runs, RUN_COLUMNS)
        _write_csv(out / "cells.csv", cells)
        _write_csv(out / "auc.csv", aucs)
        _write_csv(out / "curves.csv", curves)
        _write_csv(out / "pairs.csv", pairs)
        (out / "summary.json").write_text(json.dumps(summary, indent=1, default=str))
        if args.plot:
            plot_curves(curves, out)
    return runs, summary


def print_report(runs, summary) -> None:
    print(f"{'system':24s} {'seq':18s} {'n':>2s} {'k':>2s} {'C med':>6s} {'R3 s':>6s} "
          f"{'Sim(3) cell':>16s} {'SE(3) cell':>16s} {'scale':>6s}")
    for c in summary["cells"]:
        print(f"{c['system']:24s} {c['dataset'] + '/' + c['sequence']:18s} {c['n']:2d} {c['k']:2d} "
              f"{c['c_median']:6.3f} {c['t_first_median_s']:6.2f} {c['sim3_cell']:>16s} "
              f"{c['se3_cell']:>16s} {c['scale_median']:6.3f}")
    print("\nR4 AUC over 0.1-10 % (log axis):")
    for a in summary["auc"]:
        print(f"  {a['system']:24s} {a['dataset']:6s} {a['align']:5s} {a['auc_primary']:.3f} "
              f"(n={a['n_runs']}, {a['sequences']})")
    for w in summary["warnings"]:
        print("WARNING:", w)


def validate(runs) -> bool:
    got = {(r["system"], r["dataset"], r["sequence"], r["rep"]): r for r in runs}
    ok = True
    print(f"{'run':44s} {'expected raw/span/F/C':>26s} {'computed':>26s}")
    for key, exp in EXPECTED.items():
        r = got.get(key)
        if r is None:
            print(f"{' '.join(key):44s} MISSING from the manifest")
            ok = False
            continue
        comp = (r["c_span_raw"], r["c_span"], None if r["output"] == "keyframe" else r["f_own"], r["c"])
        good = all((e is None and c is None) or (e is not None and c is not None
                                                 and abs(e - c) <= VALIDATE_TOL)
                   for e, c in zip(exp, comp))
        ok &= good
        f = lambda v: "   -  " if v is None else f"{v:6.3f}"          # noqa: E731
        print(f"{' '.join(key):44s} {' '.join(f(v) for v in exp):>26s} "
              f"{' '.join(f(v) for v in comp):>26s}  {'OK' if good else 'MISMATCH'}")
    print("\nnot in the pre-registered table (reported, not checked):")
    for key, r in got.items():
        if key not in EXPECTED:
            print(f"  {' '.join(key):44s} C {r['c']:.3f}  (span {r['c_span']:.3f}, F_own "
                  f"{r['f_own']:.3f}, status {r['status']})")
    return ok


def selftest() -> None:
    cv.selftest()
    mk = lambda s, c, ate=0.1, st="ok", ria=None: dict(                        # noqa: E731
        system=s, dataset="tum", sequence="x", rep="1", status=st, c=c, ate_sim3_rmse=ate,
        ate_se3_rmse=ate, rejected_init_accepted=ria, traj_error="")
    runs = [mk("A", 0.97), mk("A", 0.95), mk("B", 1.0), mk("B", 0.5), mk("B", 1.0),
            mk("C", 0.0, st="harness_failure")]
    cref = c_ref_table(runs)
    assert cref[("tum", "x")] == 1.0, cref                  # B's median 1.0; C's harness run excluded
    assert success(runs[0], 1.0, BAR, "ignore")[0]
    assert not success(runs[3], 1.0, BAR, "ignore")[0]
    assert success(runs[1], 1.0, 0.95, "ignore")[0] and not success(runs[1], 1.0, 0.96, "ignore")[0]
    r = mk("H", 1.0, ria=1)
    assert success(r, 1.0, BAR, "ignore")[0] and not success(r, 1.0, BAR, "fail")[0]
    assert not success(mk("D", 1.0, ate=float("nan")), 1.0, BAR, "ignore")[0]
    assert cell_text(0.1, 3, 5) == "0.1 (3/5)" and cell_text(0.1, 2, 4) == "X (2/4)"
    assert cell_text(0.1, 1, 1) == "0.1 (1/1)" and cell_text(float("nan"), 0, 1) == "X (0/1)"
    # AUC: all runs at x = 1e-2 succeed from the grid point 10^-2 on, i.e. half the primary range.
    frac = np.array([1.0 if g >= -2.0 else 0.0 for g in GRID])
    assert abs(auc(frac, *AUC_PRIMARY) - 0.5) < 0.01, auc(frac, *AUC_PRIMARY)
    assert auc(np.ones_like(GRID), *AUC_PRIMARY) == 1.0 and auc(np.zeros_like(GRID), *AUC_PRIMARY) == 0.0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--manifest", type=Path)
    ap.add_argument("--out", help="output directory (required unless --validate)")
    ap.add_argument("--root", type=Path, default=DEFAULT_ROOT,
                    help=f"base for relative traj paths (default $WP7_BASELINES or {DEFAULT_ROOT})")
    ap.add_argument("--ref-system", default="HSLAM2", help="S4's reference system (default HSLAM2)")
    ap.add_argument("--rejected-init", choices=("ignore", "fail"), default="ignore",
                    help="HSLAM runs that continue on a rejected initialisation: open user decision; "
                         "'ignore' counts them as usable and lists them as warnings")
    ap.add_argument("--plot", action="store_true", help="write success-curve PNGs")
    ap.add_argument("--validate", action="store_true",
                    help="check coverage against the pre-registration's validation table")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        selftest()
        print("wp7_reliability.py selftest: OK")
        return 0
    if not a.manifest:
        ap.error("--manifest is required")
    if not a.out and not a.validate:
        ap.error("--out is required unless --validate")
    a.root = a.root.expanduser()
    runs, summary = run(a)
    if a.validate:
        good = validate(runs)
        print("\nVALIDATION:", "PASS" if good else "FAIL")
        return 0 if good else 1
    print_report(runs, summary)
    return 0


if __name__ == "__main__":
    sys.exit(main())
