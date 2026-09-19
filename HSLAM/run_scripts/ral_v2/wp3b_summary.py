#!/usr/bin/env python3
"""WP3b summary: per arm x sequence medians from runs/wp3b_<host>/<arm>/summary.csv, the ratio
to the monocular comparator, and the pre-registered "rescue" verdict (DECISIONS.md, WP3b):
rescue = median Sim(3) ATE <= 1.5 x mono AND track >= 2/3 on the sequence.

Usage: wp3b_summary.py --root runs/wp3b_eval-server [--wp3 runs/wp3_eval-server/summary.csv]
The WP3 A0/full rows (same binary) are pooled with the WP3b A0 rows for the comparator.
"""
import argparse
import csv
import glob
import os
import statistics as st
from collections import defaultdict

FIELDS = ["ate_sim3_rmse", "ate_se3_rmse", "scale_s", "rpe_trans", "rpe_rot", "scale_drift_pct_per_100m", "keyframes", "pipeline_fps"]


def load(path, arm_override=None):
    rows = []
    for r in csv.DictReader(open(path)):
        if arm_override:
            r["arm"] = arm_override
        rows.append(r)
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--wp3", default=None)
    ap.add_argument("--arms", nargs="*", default=None)
    a = ap.parse_args()

    rows = []
    for f in sorted(glob.glob(os.path.join(a.root, "*", "summary.csv"))):
        rows += load(f, arm_override=os.path.basename(os.path.dirname(f)))
    if a.wp3 and os.path.exists(a.wp3):
        rows += [r for r in load(a.wp3) if r["arm"] in ("A0", "full")]

    groups, attempts, commits = defaultdict(list), defaultdict(int), defaultdict(set)
    for r in rows:
        k = (r["arm"], r["dataset"], r["sequence"])
        attempts[k] += 1
        commits[k].add(r["commit"][:7])
        if r["status"] == "OK" and r["track_success"] == "1":
            groups[k].append(r)

    seqs = sorted({(r["dataset"], r["sequence"]) for r in rows})
    arms = a.arms or sorted({r["arm"] for r in rows}, key=lambda x: (x != "A0", x != "full", x))
    med = lambda g, f: st.median(float(r[f]) for r in g if r[f] not in ("", "nan"))

    print(f"{'arm':14s} {'seq':18s} {'ok/n':>5s} {'ATE_s3':>8s} {'IQR':>7s} {'x mono':>7s} {'ATE_se3':>8s} {'scale':>6s} "
          f"{'rpe_t':>7s} {'rpe_r':>6s} {'drift':>7s} {'kfs':>5s} {'fps':>5s}  verdict   commits")
    for ds, sq in seqs:
        mono = groups.get(("A0", ds, sq))
        mono_ate = med(mono, "ate_sim3_rmse") if mono else float("nan")
        for arm in arms:
            k = (arm, ds, sq)
            g = groups.get(k)
            n = attempts.get(k, 0)
            if not n:
                continue
            if not g:
                print(f"{arm:14s} {sq:18s} {'0/' + str(n):>5s}  -- no usable run --")
                continue
            ate = med(g, "ate_sim3_rmse")
            s = sorted(float(r["ate_sim3_rmse"]) for r in g)
            iqr = (s[3 * len(s) // 4] - s[len(s) // 4]) if len(s) > 3 else (s[-1] - s[0])
            ratio = ate / mono_ate if mono_ate == mono_ate else float("nan")
            track_ok = len(g) >= 2 if n >= 3 else len(g) >= 1
            verdict = "-" if arm == "A0" else ("RESCUE" if (ratio <= 1.5 and track_ok) else "no")
            print(f"{arm:14s} {sq:18s} {str(len(g)) + '/' + str(n):>5s} {ate:8.4f} {iqr:7.4f} {ratio:7.2f} "
                  f"{med(g, 'ate_se3_rmse'):8.4f} {med(g, 'scale_s'):6.3f} {med(g, 'rpe_trans'):7.4f} "
                  f"{med(g, 'rpe_rot'):6.3f} {med(g, 'scale_drift_pct_per_100m'):7.1f} {med(g, 'keyframes'):5.0f} "
                  f"{med(g, 'pipeline_fps'):5.1f}  {verdict:8s} {','.join(sorted(commits[k]))}")
        print()


if __name__ == "__main__":
    main()
