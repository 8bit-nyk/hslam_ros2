#!/usr/bin/env python3
"""Drift-conditional gate, PHASE 0 extractor (FUTURE_DIRECTIONS.md section 11; DECISIONS.md 2026-09-23).

Run ON THE EVAL SERVER with ~/Dev/evo/evo_env/bin/python (needs evo + numpy), e.g.
  python driftgate_p0_extract.py --root ../../runs/wp2ii_eval-server --arms full L_w1000_k1 L_w10000_k3 --out p0_data.json
then analyse with driftgate_p0_analyse.py (numpy + scipy only).

READ-ONLY over <root>/<arm>/<dataset>_<seq>_rep<k>/ (run.log + result.txt) and <arm>/summary.csv.
No new runs, no binary. Emits one JSON record per rep with:
  - the [PRIOR_ALIGN] series (kf, s, n, iqr)            -- proxy (a)
  - every [INDIRECT.P2_GATE] evaluation with its degenerate flag  -- proxy (c)
  - the estimated keyframe trajectory from result.txt (row k == keyframe k, verified 23 Sep)
  - the GT side through the SAME evo path eval_run.py uses (association max_diff 0.02, Umeyama
    Sim(3)), plus the 10 prefix-wise (distance, scale) pairs that scale_drift_pct_per_100m is the
    slope of, and the per-step local scale ratio log(step_est_aligned / step_gt).
  - the summary.csv row's own metrics, to cross-check the re-derivation.
"""
import argparse, copy, csv, glob, json, math, os, re, sys, time
from pathlib import Path
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import traj_eval                      # noqa: E402
import datasets as ds                 # noqa: E402
from evo.core import sync            # noqa: E402
from evo.core.trajectory import PosePath3D  # noqa: E402

PA = re.compile(r"\[PRIOR_ALIGN\] kf=(\d+) s=(-?[\d.]+|-?nan) n=(\d+) iqr=(-?[\d.]+|-?nan)")
GATE = re.compile(r"\[INDIRECT\.P2_GATE\] cur=(\d+) cand=(\d+) s_ransac=([-\d.]+) s_ml=([-\d.]+) "
                  r"disagreement=([-\d.]+)% n=(\d+) source=(\w+) thresh=([\d.]+) decision=(\w+)")
SML = re.compile(r"\[INDIRECT\.SML_COMPARE\] cur=(\d+) cand=(\d+) s_ransac=([-\d.]+) [^\n]*?degenerate=(\w+)")
RUNDIR = re.compile(r"^(tum|kitti|euroc)_(.+)_rep(\d+)$")


def fnum(x):
    try:
        v = float(x)
    except Exception:
        return None
    return v if math.isfinite(v) else None


def rl(v, nd=5):
    return [None if (x is None or not math.isfinite(x)) else round(float(x), nd) for x in v]


def prefix_scales(ref, est, n_bins=10):
    """Same loop as traj_eval._scale_drift_pct_per_100m, returning its (d, s) pairs + its slope."""
    gt_xyz = ref.positions_xyz
    if len(gt_xyz) < n_bins * 2:
        return [], [], None
    cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(gt_xyz, axis=0), axis=1))])
    total = float(cum[-1])
    if total < 1.0:
        return [], [], None
    dd, ss = [], []
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
            dd.append(float(cum[min(idx, len(cum)) - 1]))
            ss.append(float(s))
    if len(dd) < 3:
        return dd, ss, None
    slope, _ = np.polyfit(np.asarray(dd), np.asarray(ss), 1)
    return dd, ss, float(slope / np.mean(ss) * 1e4)


def one_rep(arm, rundir, row):
    m = RUNDIR.match(rundir.name)
    dataset, seq, rep = m.group(1), m.group(2), int(m.group(3))
    spec = ds.resolve(dataset, seq)
    text = (rundir / "run.log").read_text(errors="replace")

    pa = [(int(a), fnum(b), int(c), fnum(d)) for a, b, c, d in PA.findall(text)]
    degen = {(int(a), int(b)): (d.upper() == "YES") for a, b, _, d in SML.findall(text)}
    gates = []
    for cur, cand, sr, sm, dis, n, src, thr, dec in GATE.findall(text):
        gates.append({"cur": int(cur), "cand": int(cand), "s_ransac": fnum(sr), "s_ml": fnum(sm),
                      "disagreement_pct": fnum(dis), "n": int(n), "source": src, "decision": dec,
                      "degenerate": degen.get((int(cur), int(cand)))})
    kfs = None
    if (mm := re.search(r"\[RUN_SUMMARY\][^\n]*?kfs=(\d+)", text)):
        kfs = int(mm.group(1))

    rec = {"arm": arm, "dataset": dataset, "sequence": seq, "rep": rep,
           "status": row.get("status"), "track_success": row.get("track_success"),
           "csv": {k: fnum(row.get(k)) for k in ("ate_sim3_rmse", "ate_se3_rmse", "scale_s",
                                                  "scale_drift_pct_per_100m", "gt_distance_m",
                                                  "keyframes", "lc_gate_evals", "lc_gate_fires",
                                                  "prior_align_s_med", "prior_align_s_iqr")},
           "kfs_log": kfs,
           "pa_kf": [p[0] for p in pa], "pa_s": rl([p[1] for p in pa], 4),
           "pa_n": [p[2] for p in pa], "pa_iqr": rl([p[3] for p in pa], 4),
           "gates": gates, "traj_error": ""}

    est_path = rundir / "result.txt"
    if not est_path.exists() or est_path.stat().st_size == 0:
        rec["traj_error"] = "no result.txt"
        return rec
    try:
        est = traj_eval._read_tum_lenient(est_path)
        ref = traj_eval.load_gt(spec.gt, spec.gt_format, spec.extrinsics)
        rec["est_rows"] = int(est.num_poses)
        rec["est_t"] = rl(est.timestamps, 6)
        rec["est_xyz"] = [rl(p, 4) for p in est.positions_xyz]
        ref_s, est_s = sync.associate_trajectories(ref, est, max_diff=0.02)
        # which result.txt rows were matched (row index == keyframe index)
        midx = np.searchsorted(est.timestamps, est_s.timestamps)
        ok = (midx < est.num_poses) & (np.abs(est.timestamps[np.minimum(midx, est.num_poses - 1)]
                                              - est_s.timestamps) < 1e-6)
        rec["matched_idx"] = [int(i) for i in midx[ok]]
        rec["matched_all_exact"] = bool(ok.all())
        gt_xyz = ref_s.positions_xyz
        rec["gt_cumdist"] = rl(np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(gt_xyz, axis=0), axis=1))]), 4)
        est_al = copy.deepcopy(est_s)
        _, _, s = est_al.align(ref_s, correct_scale=True)
        rec["sim3_scale"] = float(s)
        step_e = np.linalg.norm(np.diff(est_al.positions_xyz, axis=0), axis=1)
        step_g = np.linalg.norm(np.diff(gt_xyz, axis=0), axis=1)
        lr = np.full(len(step_e), np.nan)
        good = step_g > 1e-3
        lr[good] = np.log(step_e[good] / step_g[good])
        rec["local_logratio"] = rl(lr, 4)
        dd, ss, drift = prefix_scales(ref_s, est_s)
        rec["prefix_d"], rec["prefix_s"], rec["drift_rederived"] = rl(dd, 3), rl(ss, 5), drift
    except Exception as exc:  # noqa: BLE001
        rec["traj_error"] = f"{type(exc).__name__}: {exc}"
    return rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--arms", nargs="+", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    out, t0 = [], time.time()
    for arm in a.arms:
        rows = {}
        with open(os.path.join(a.root, arm, "summary.csv")) as fh:
            for r in csv.DictReader(fh):
                rows.setdefault((r["dataset"], r["sequence"], int(r["rep"])), []).append(r)
        dup = {k: len(v) for k, v in rows.items() if len(v) > 1}
        if dup:
            print(f"WARNING {arm}: duplicate summary rows for {dup}", file=sys.stderr)
        dirs = sorted(Path(a.root, arm).glob("*_rep*"))
        for i, d in enumerate(dirs):
            m = RUNDIR.match(d.name)
            if not m:
                continue
            key = (m.group(1), m.group(2), int(m.group(3)))
            row = rows.get(key, [{}])[-1]
            if not row:
                print(f"NOTE {arm}/{d.name}: rep dir without a summary row (interrupted run) -- skipped",
                      file=sys.stderr)
                continue
            rec = one_rep(arm, d, row)
            out.append(rec)
            if i % 25 == 0:
                print(f"{arm} {i+1}/{len(dirs)} {d.name} {time.time()-t0:.0f}s", file=sys.stderr)
    with open(a.out, "w") as fh:
        json.dump(out, fh, separators=(",", ":"))
    print(f"wrote {len(out)} records to {a.out} in {time.time()-t0:.0f}s", file=sys.stderr)


if __name__ == "__main__":
    main()
