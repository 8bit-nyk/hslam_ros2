#!/usr/bin/env python3
"""WP2a summary: each arm against a reference arm, per sequence, with the pre-registered
no-regression rule (DECISIONS.md, WP2a R1):

  a fix passes on a sequence iff  median ATE_fix <= median ATE_ref + IQR_ref  and  track_fix >= track_ref

IQR at n=5 is s[3]-s[1] of the sorted usable ATEs (the same estimator wp3b_summary.py uses; for
n<=3 it is max-min). Also prints the per-arm sign count across sequences, the [INIT_FOUNDING_DIAG]
and [INIT_SCALE_DIAG] medians read from each rep's run.log, and (if --mono is given) the ratio to
the monocular arm for the information-only sequences.

Usage: wp2a_summary.py --root runs/wp2a_eval-server --ref full
         [--mono runs/wp6_eval-server/A0/summary.csv runs/wp3_eval-server/summary.csv] [--arms ...]
"""
import argparse
import csv
import glob
import os
import re
import statistics as st
from collections import defaultdict

SEQ_ORDER = ["freiburg1_desk", "freiburg1_room", "freiburg2_desk", "freiburg2_large_no_loop",
             "freiburg3_long_office_household", "00", "05", "06", "07", "10",
             "MH_01_easy", "V1_01_easy", "V2_02_medium", "sequence_31"]


def load(path, arm=None):
    out = []
    for r in csv.DictReader(open(path)):
        if arm:
            r["arm"] = arm
        r["_dir"] = os.path.dirname(path)
        out.append(r)
    return out


def iqr(vals):
    s = sorted(vals)
    return (s[3 * len(s) // 4] - s[len(s) // 4]) if len(s) > 3 else (s[-1] - s[0])


def med(rows, f):
    v = [float(r[f]) for r in rows if r[f] not in ("", "nan")]
    return st.median(v) if v else float("nan")


def log_diag(row, tag, field):
    """Read `field=` from the first `[tag]` line of the rep's run.log; nan if absent."""
    p = os.path.join(row["_dir"], f"{row['dataset']}_{row['sequence']}_rep{row['rep']}", "run.log")
    try:
        for line in open(p, errors="replace"):
            if line.startswith(f"[{tag}]"):
                m = re.search(rf"{field}=(-?[\d.]+|nan)", line)
                return float(m.group(1)) if m else float("nan")
    except OSError:
        pass
    return float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--ref", default="full")
    ap.add_argument("--mono", nargs="*", default=[])
    ap.add_argument("--arms", nargs="*", default=None)
    a = ap.parse_args()

    rows = []
    for f in sorted(glob.glob(os.path.join(a.root, "*", "summary.csv"))):
        arm = os.path.basename(os.path.dirname(f))
        rows += load(f, arm=arm.replace("_r0ext", ""))     # R0 extension rows pool with `full`
    mono = []
    for f in a.mono:
        if os.path.exists(f):
            mono += [r for r in load(f) if r["arm"] == "A0"]

    ok_rows, attempts = defaultdict(list), defaultdict(int)
    for r in rows:
        k = (r["arm"], r["dataset"], r["sequence"])
        attempts[k] += 1
        if r["status"] == "OK" and r["track_success"] == "1":
            ok_rows[k].append(r)
    mono_ok = defaultdict(list)
    for r in mono:
        if r["status"] == "OK" and r["track_success"] == "1":
            mono_ok[(r["dataset"], r["sequence"])].append(r)

    seqs = sorted({(r["dataset"], r["sequence"]) for r in rows},
                  key=lambda x: SEQ_ORDER.index(x[1]) if x[1] in SEQ_ORDER else 99)
    arms = a.arms or sorted({r["arm"] for r in rows}, key=lambda x: (x != a.ref, x))
    tally = defaultdict(lambda: [0, 0, 0, 0])     # better, worse-within-IQR, REGRESSION, n_seq

    print(f"{'arm':14s} {'seq':22s} {'ok/n':>5s} {'ATE_s3':>8s} {'IQR':>7s} {'d%ref':>7s} {'x mono':>7s} "
          f"{'ATE_se3':>8s} {'scale':>6s} {'kfs':>5s} {'found':>7s} {'fIQR':>6s}  rule")
    for ds_, sq in seqs:
        ref = ok_rows.get((a.ref, ds_, sq), [])
        ref_ate = med(ref, "ate_sim3_rmse") if ref else float("nan")
        ref_iqr = iqr([float(r["ate_sim3_rmse"]) for r in ref]) if len(ref) > 1 else float("nan")
        ref_n = len(ref)
        m = mono_ok.get((ds_, sq), [])
        mono_ate = med(m, "ate_sim3_rmse") if m else float("nan")
        for arm in arms:
            k = (arm, ds_, sq)
            n = attempts.get(k, 0)
            if not n:
                continue
            g = ok_rows.get(k, [])
            if not g:
                print(f"{arm:14s} {sq:22s} {'0/' + str(n):>5s}  -- no usable run --")
                if arm != a.ref:
                    tally[arm][2] += 1; tally[arm][3] += 1
                continue
            ate = med(g, "ate_sim3_rmse")
            d = 100 * (ate / ref_ate - 1) if ref_ate == ref_ate else float("nan")
            xm = ate / mono_ate if mono_ate == mono_ate else float("nan")
            fnd = st.median([log_diag(r, "INIT_FOUNDING_DIAG", "med_log_ratio_good") for r in g])
            fiq = st.median([log_diag(r, "INIT_FOUNDING_DIAG", "iqr") for r in g])
            if arm == a.ref or ref_ate != ref_ate:
                rule = "(ref)" if arm == a.ref else "no ref"
            else:
                within = ate <= ref_ate + ref_iqr and len(g) >= ref_n
                rule = "pass" if within else "REGRESSION"
                if ate < ref_ate:
                    tally[arm][0] += 1
                elif within:
                    tally[arm][1] += 1
                else:
                    tally[arm][2] += 1
                tally[arm][3] += 1
            print(f"{arm:14s} {sq:22s} {str(len(g)) + '/' + str(n):>5s} {ate:8.4f} "
                  f"{iqr([float(r['ate_sim3_rmse']) for r in g]) if len(g) > 1 else float('nan'):7.4f} "
                  f"{d:+7.1f} {xm:7.2f} {med(g, 'ate_se3_rmse'):8.4f} {med(g, 'scale_s'):6.3f} "
                  f"{med(g, 'keyframes'):5.0f} {fnd:+7.3f} {fiq:6.3f}  {rule}")
        print()
    print("per-arm tally over sequences with a reference: better / worse-within-IQR / REGRESSION / total")
    for arm, (b, w, r, n) in tally.items():
        print(f"  {arm:14s} {b:2d} / {w:2d} / {r:2d} / {n:2d}" + ("   <-- fails R1" if r else "   passes R1"))


if __name__ == "__main__":
    main()
