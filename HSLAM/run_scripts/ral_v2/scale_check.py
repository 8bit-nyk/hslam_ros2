#!/usr/bin/env python3
"""Post-run sanity check for a plain run-script run: is this the metric-scale configuration, and did it work?

    python3 scale_check.py <dataset> <sequence> <trajectory.txt> [--log run.log]

Two layers, so it is useful even without evo installed:

  1. LOG (stdlib only). Reads [RUN_SUMMARY] / [TIMESTAMPS] / [ML_GEOM] and fails loudly if the run was not
     the frozen paper configuration (canonical scale off, K14/K15 off, isotropic off where fx != fy, ...).
     This is the check that catches "ran the stock script, got bad scale" without needing ground truth.
  2. TRAJECTORY (needs numpy + evo, the same stack as eval_run.py). Sim(3) and SE(3) ATE and the Umeyama
     scale against the dataset's ground truth, via traj_eval.evaluate -- the evaluator the paper rows use.

Exit status: 0 = config OK and scale in the expected band, 1 = config wrong or run failed, 2 = config OK but
scale outside the band (a result, not a bug: look at it). Dataset keys are datasets.py's: tum, kitti, euroc.
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

# What the frozen config must read back from [RUN_SUMMARY]. Mirrors arms.PAPER_CONFIG (+ K14/K15); if
# arms.py changes, this table is what trips -- on purpose, a changed paper config deserves a look here.
_EXPECT = {"depth_src": "ml", "geom": "metric3d", "canon": "on", "init_scale": "median", "blend_fix": "on",
           "p2": "off", "p3": "off", "idepth_prior": "box", "lc_scale_gate": "on", "ml_gpu": "on",
           "status": "OK"}
_NON_SQUARE = {"kitti", "euroc", "tummonovo"}

# Sim(3) scale of a trajectory vs ground truth, measured with this configuration (see CLAUDE.md "Critical
# Configuration"; n=5 on the eval server). A band, not a pass/fail threshold: ±0.15 is ~3x the spread seen.
_SCALE_SEEN = {("tum", "freiburg1_room"): 1.13, ("kitti", "07"): 0.92, ("kitti", "00"): 0.96}
_SCALE_BAND = 0.15


def _kv(line: str) -> dict:
    return dict(re.findall(r"(\w+)=(\S+)", line))


def check_log(log: Path, dataset: str) -> list[str]:
    """Return a list of problems (empty = the log says this was the paper configuration)."""
    text = log.read_text(errors="replace")
    summ = [l for l in text.splitlines() if l.startswith("[RUN_SUMMARY]")]
    if not summ:
        return ["no [RUN_SUMMARY] line: the run did not finish, or this is not an HSLAM log"]
    kv = _kv(summ[-1])
    problems = []
    expect = dict(_EXPECT)
    expect["iso"] = "on" if dataset in _NON_SQUARE else kv.get("iso", "off")
    for k, want in expect.items():
        got = kv.get(k)
        if got != want:
            problems.append(f"[RUN_SUMMARY] {k}={got}, expected {want}")
    for l in text.splitlines():
        if l.startswith("[ML_GEOM]") and "WARNING" in l:
            problems.append(l.strip())
        if l.startswith("[TIMESTAMPS]") and re.search(r"plausible=no", l, re.I):
            problems.append(l.strip())
    if not any(l.startswith("[TIMESTAMPS]") and "source=" in l for l in text.splitlines()):
        problems.append("no [TIMESTAMPS] source= line: cannot confirm the sampling rate")
    return problems


def check_traj(dataset: str, sequence: str, traj: Path) -> dict:
    import datasets
    import traj_eval            # imports numpy + evo; ImportError is handled by the caller
    spec = datasets.resolve(dataset, sequence)
    return traj_eval.evaluate(traj, spec.gt, spec.gt_format, spec.extrinsics)


def _partial_note(dataset: str, sequence: str, log) -> str:
    """Non-empty when the run processed clearly fewer frames than the sequence holds (--endindex)."""
    try:
        import datasets
        frames = int(_kv([l for l in Path(log).read_text(errors="replace").splitlines()
                          if l.startswith("[RUN_SUMMARY]")][-1])["frames"])
        total = len(list(datasets.resolve(dataset, sequence).images.glob("*.png")))
    except Exception:
        return ""
    if total and frames < 0.9 * total:
        return f"partial run ({frames} of {total} frames)."
    return ""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset")
    ap.add_argument("sequence")
    ap.add_argument("trajectory", type=Path)
    ap.add_argument("--log", type=Path, default=None,
                    help="run log; default: run_log_ml_depth_0.txt or run.log next to the trajectory")
    a = ap.parse_args()

    log = a.log
    if log is None:
        for name in ("run_log_ml_depth_0.txt", "run.log"):
            if (a.trajectory.parent / name).exists():
                log = a.trajectory.parent / name
                break
    rc = 0
    print(f"== scale check: {a.dataset} {a.sequence}")
    if log is None or not log.exists():
        print("LOG   : not found (pass --log) -- cannot confirm which configuration ran")
        rc = 1
    else:
        problems = check_log(log, a.dataset)
        if problems:
            rc = 1
            print("LOG   : NOT the metric-scale paper configuration, or the run failed:")
            for p in problems:
                print(f"          - {p}")
            print("        Run via the hslam_run_*_ml_depth.sh scripts (they apply the paper flags), "
                  "and check HSLAM_LEGACY_FLAGS is unset.")
        else:
            print("LOG   : OK -- paper configuration, status=OK, timestamps plausible")

    try:
        r = check_traj(a.dataset, a.sequence, a.trajectory)
    except ImportError as e:
        print(f"TRAJ  : skipped ({e}). Needs numpy + evo (pip install evo); set HSLAM_PYTHON to that python.")
        return rc
    except Exception as e:      # missing GT, bad path: say so, never invent a number
        print(f"TRAJ  : skipped ({type(e).__name__}: {e})")
        return rc
    if r.get("traj_error"):
        print(f"TRAJ  : failed -- {r['traj_error']}")
        return 1
    s = r["scale_s"]
    print(f"TRAJ  : scale s = {s:.3f}   ATE Sim(3) = {r['ate_sim3_rmse']:.3f} m   "
          f"ATE SE(3) = {r['ate_se3_rmse']:.3f} m")
    print("        (s ~ 1 and SE(3) ATE close to Sim(3) ATE = metric scale. s is Umeyama: est * s ~ GT.)")
    partial = _partial_note(a.dataset, a.sequence, log)
    if partial:
        print(f"        NOTE: {partial} A short-baseline scale is not comparable to the full-sequence numbers.")
    seen = _SCALE_SEEN.get((a.dataset, a.sequence))
    if seen is not None and not partial:
        ok = abs(s - seen) <= _SCALE_BAND
        print(f"        paper config measured s = {seen:.2f} on this sequence -> {'within' if ok else 'OUTSIDE'} "
              f"±{_SCALE_BAND} of that")
        if not ok and rc == 0:
            rc = 2
    elif seen is None and abs(s - 1.0) > 0.25 and rc == 0:
        print("        s is far from 1 and there is no recorded reference for this sequence: look at it "
              "(EuRoC is known to be the weakest dataset: scale self-consistent but ~45 % residual).")
        rc = 2
    return rc


if __name__ == "__main__":
    sys.exit(main())
