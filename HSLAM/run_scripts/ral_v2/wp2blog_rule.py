#!/usr/bin/env python3
"""WP2b-log phase i: apply the four pre-registered bars and make the pick.

DECISIONS.md "WP2b-log (card b2) -- PRE-REGISTERED 2026-09-21":
  1. KITTI 07 median scale_s in [0.85, 1.15]   <- the bar the inverse-depth design failed
  2. fr1_room median scale_s in [0.85, 1.15]
  3. median Sim(3) ATE <= full median + full IQR on BOTH sequences (the G2-b parity floor)
  4. 3/3 reps usable on both sequences
Pick: smallest w; tie-break k1, then k3, then knone; then sigma_log nearest 0.30.
If no arm clears bar 1 the card is NEGATIVE and closes.

Usage: wp2blog_rule.py --root runs/wp2blog_eval-server [--ref full]
"""
import argparse
import csv
import glob
import os
import re
import statistics as st

SEQS = [("tum", "freiburg1_room"), ("kitti", "07")]
SCALE_LO, SCALE_HI = 0.85, 1.15


def iqr(v):
    v = sorted(v)
    n = len(v)
    return v[(3 * n) // 4] - v[n // 4] if n >= 4 else (max(v) - min(v) if v else float("nan"))


def load(root):
    """{arm: {sequence: (med_ate, iqr_ate, med_scale, n_ok, n_att)}}"""
    out = {}
    for f in sorted(glob.glob(os.path.join(root, "*", "summary.csv"))):
        arm_dir = os.path.basename(os.path.dirname(f))
        arm = re.sub(r"_(r0ext|esc)$", "", arm_dir)
        per = out.setdefault(arm, {})
        with open(f) as fh:
            for r in csv.DictReader(fh):
                per.setdefault(r["sequence"], []).append(r)
    res = {}
    for arm, per in out.items():
        res[arm] = {}
        for sq, rs in per.items():
            ok = [r for r in rs if r.get("status") == "OK"]
            if not ok:
                res[arm][sq] = (float("nan"), float("nan"), float("nan"), 0, len(rs))
                continue
            a = [float(r["ate_sim3_rmse"]) for r in ok]
            s = [float(r["scale_s"]) for r in ok]
            res[arm][sq] = (st.median(a), iqr(a), st.median(s), len(ok), len(rs))
    return res


def sort_key(arm):
    m = re.match(r"L_w(\d+)_(k1|k3|knone)(?:_s(\d+))?$", arm)
    if not m:
        return (10**9, 9, 9.9)
    w = int(m.group(1))
    k = {"k1": 0, "k3": 1, "knone": 2}[m.group(2)]
    slog = 0.30 if not m.group(3) else float("0." + m.group(3)[1:]) if m.group(3).startswith("0") else float(m.group(3)) / 100
    return (w, k, abs(slog - 0.30))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--ref", default="full")
    a = ap.parse_args()
    d = load(a.root)
    if a.ref not in d:
        raise SystemExit(f"no reference arm '{a.ref}' under {a.root}")
    ref = d[a.ref]

    print(f"reference {a.ref}: " + "; ".join(
        f"{sq} ATE {ref[sq][0]:.3f} (IQR {ref[sq][1]:.3f}) s {ref[sq][2]:.3f} n={ref[sq][3]}"
        for _, sq in SEQS if sq in ref))
    print(f"\n{'arm':18s} {'fr1_room ATE (IQR) s  n':>30s} {'KITTI07 ATE (IQR) s  n':>30s}  bars 1234  verdict")
    passing = []
    for arm in sorted([k for k in d if k.startswith("L_")], key=sort_key):
        cells, bars, line = [], [], ""
        for _, sq in SEQS:
            if sq not in d[arm]:
                cells.append(None)
                continue
            cells.append(d[arm][sq])
        if any(c is None for c in cells):
            print(f"{arm:18s} -- incomplete --")
            continue
        (ra, ri, rs_, rn, _), (ka, ki, ks, kn, _) = cells
        b1 = SCALE_LO <= ks <= SCALE_HI
        b2 = SCALE_LO <= rs_ <= SCALE_HI
        b3 = (ra <= ref["freiburg1_room"][0] + ref["freiburg1_room"][1]) and (ka <= ref["07"][0] + ref["07"][1])
        b4 = rn >= 3 and kn >= 3
        ok = b1 and b2 and b3 and b4
        if ok:
            passing.append(arm)
        line = "".join("+" if b else "-" for b in (b1, b2, b3, b4))
        print(f"{arm:18s} {ra:9.3f} ({ri:.3f}) {rs_:5.3f} {rn:2d} {ka:9.3f} ({ki:.3f}) {ks:5.3f} {kn:2d}  "
              f"{line}      {'PASS' if ok else ''}")

    any_b1 = any(SCALE_LO <= d[arm]["07"][2] <= SCALE_HI for arm in d
                 if arm.startswith("L_") and "07" in d[arm])
    print()
    if not any_b1:
        print("VERDICT: no arm clears bar 1 (KITTI 07 scale in [0.85, 1.15]) => WP2b-log NEGATIVE, card closes.")
    elif passing:
        print(f"VERDICT: pick = {passing[0]} (bars 1-4 clear; ordered smallest w, then k1/k3/knone, "
              f"then sigma_log nearest 0.30). Other passing arms: {passing[1:] or 'none'}")
    else:
        print("VERDICT: bar 1 is cleared by at least one arm but no arm clears all four -- "
              "check the bounded escalation clause in DECISIONS.md before concluding.")


if __name__ == "__main__":
    main()
