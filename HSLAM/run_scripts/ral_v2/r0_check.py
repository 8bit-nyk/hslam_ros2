#!/usr/bin/env python3
"""R0 reproduction check for a new binary epoch (PLAN.md P4b as amended 2026-09-19).

Reads one or more summary.csv files (rows are pooled), takes the `full` arm's usable rows per
reference sequence, and compares the median of each reference field with the frozen reference
point estimate at the per-dataset tolerance:

  TUM fr1_room : ATE Sim(3) 0.3072 +/-5 %, scale 1.1282 +/-5 %     (WP1, 6799be6, n=5, eval-server)
  KITTI 07     : ATE Sim(3) 5.583  +/-20 %, scale 0.9217 +/-5 %    (same; KITTI 07's between-session
                 spread is 5.58 / 6.31 / 6.63, so +/-5 % fails a legitimately identical binary --
                 DECISIONS.md "USER DECISIONS -- 2026-09-19" item 5)

Exit 0 if every present reference sequence is within tolerance, 1 otherwise. A sequence with no
usable rows is skipped with a warning unless --require lists it.
Usage: r0_check.py --csv runs/x/full/summary.csv [more.csv ...] [--require freiburg1_room 07]
"""
import argparse
import csv
import statistics as st
import sys

REF = {
    "freiburg1_room": {"ate_sim3_rmse": (0.3072, 0.05), "scale_s": (1.1282, 0.05)},
    "07":             {"ate_sim3_rmse": (5.583, 0.20),  "scale_s": (0.9217, 0.05)},
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", nargs="+", required=True)
    ap.add_argument("--arm", default="full")
    ap.add_argument("--require", nargs="*", default=[])
    a = ap.parse_args()
    rows = []
    for f in a.csv:
        rows += [r for r in csv.DictReader(open(f))
                 if r["arm"] == a.arm and r["status"] == "OK" and r["track_success"] == "1"]
    ok = True
    for sq, fields in REF.items():
        g = [r for r in rows if r["sequence"] == sq]
        if not g:
            print(f"  R0 {sq}: no usable rows" + (" -- REQUIRED" if sq in a.require else " (skipped)"))
            ok &= sq not in a.require
            continue
        for f, (ref, tol) in fields.items():
            vals = [float(r[f]) for r in g if r[f] not in ("", "nan")]
            got = st.median(vals)
            dev = got / ref - 1
            hit = abs(dev) <= tol
            ok &= hit
            print(f"  R0 {sq:16s} {f:14s} {got:.4f} ref {ref:.4f} {100*dev:+.2f}% "
                  f"{'within' if hit else 'OUTSIDE'} +/-{int(100*tol)}%  (n={len(vals)})")
    print("  R0 " + ("PASSED" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
