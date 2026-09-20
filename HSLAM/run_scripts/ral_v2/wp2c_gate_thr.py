#!/usr/bin/env python3
"""WP2c phase ii: the keyframe-gate threshold from the healthy [PRIOR_ALIGN] distribution, and the G2-d check.

Reads every rep's run.log under <root>/<arm>/<dataset>_<seq>_rep<k>/ for the ABL-10 sequences, collects
s_k from the `[PRIOR_ALIGN] kf=… s=… n=… iqr=…` lines (n >= --min-n), and prints:
  * per sequence: number of keyframes, median s_k, IQR of s_k across the run (median over reps) -- G2-d
    requires IQR < 0.3 on every healthy sequence;
  * pooled over all keyframes of all ten sequences: the 95th percentile of |log s_k| = thr (DECISIONS.md,
    WP2c phase ii), plus p90 / p99 for context and the fraction of keyframes that a gate at thr would fire on
    (= 5 % by construction on this data).
Usage: wp2c_gate_thr.py --root runs/wp2c_eval-server --arm C_w10_k1 [--min-n 50]
"""
import argparse
import glob
import os
import re
import statistics as st

import numpy as np

ABL = [("tum", "freiburg1_desk"), ("tum", "freiburg1_room"), ("tum", "freiburg2_desk"),
       ("tum", "freiburg2_large_no_loop"), ("tum", "freiburg3_long_office_household"),
       ("kitti", "00"), ("kitti", "05"), ("kitti", "06"), ("kitti", "07"), ("kitti", "10")]
PAT = re.compile(r"\[PRIOR_ALIGN\] kf=(\d+) s=(-?[\d.]+|nan) n=(\d+) iqr=(-?[\d.]+|nan)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--arm", required=True)
    ap.add_argument("--min-n", type=int, default=50)
    ap.add_argument("--skip-first", type=int, default=0, help="ignore the first K keyframes of each run")
    a = ap.parse_args()

    pooled = []
    print(f"{'sequence':32s} {'reps':>4s} {'kfs/run':>7s} {'med s':>7s} {'IQR s':>7s} {'p95|log s|':>10s}  G2-d")
    for ds, sq in ABL:
        per_rep_iqr, per_rep_med, per_rep_n, seq_vals = [], [], [], []
        for log in sorted(glob.glob(os.path.join(a.root, a.arm, f"{ds}_{sq}_rep*", "run.log"))):
            vals = []
            for line in open(log, errors="replace"):
                m = PAT.match(line)
                if not m:
                    continue
                kf, s, n = int(m.group(1)), float(m.group(2)), int(m.group(3))
                if s != s or n < a.min_n or kf < a.skip_first:
                    continue
                vals.append(s)
            if len(vals) < 5:
                continue
            v = np.array(vals)
            per_rep_iqr.append(float(np.percentile(v, 75) - np.percentile(v, 25)))
            per_rep_med.append(float(np.median(v)))
            per_rep_n.append(len(v))
            seq_vals.extend(vals)
        if not per_rep_n:
            print(f"{sq:32s}  -- no [PRIOR_ALIGN] lines --")
            continue
        iqr = st.median(per_rep_iqr)
        lg = np.abs(np.log(np.array(seq_vals)))
        print(f"{sq:32s} {len(per_rep_n):4d} {st.median(per_rep_n):7.0f} {st.median(per_rep_med):7.3f} {iqr:7.3f} "
              f"{np.percentile(lg, 95):10.3f}  {'ok' if iqr < 0.3 else 'FAIL'}")
        pooled.extend(seq_vals)
    if pooled:
        lg = np.abs(np.log(np.array(pooled)))
        p90, p95, p99 = (float(np.percentile(lg, q)) for q in (90, 95, 99))
        print(f"\npooled keyframes: {len(pooled)}; |log s_k| p90 {p90:.4f}  **p95 {p95:.4f} = thr**  p99 {p99:.4f}; "
              f"fire rate at thr on this data {100 * float(np.mean(lg > p95)):.1f} %")


if __name__ == "__main__":
    main()
