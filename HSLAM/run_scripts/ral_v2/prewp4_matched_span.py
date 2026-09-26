#!/usr/bin/env python3
"""Pre-WP4 stage 2, (d5): matched-span ATE for P5a coverage mismatches (reviewer R10.6), as pre-registered in
DECISIONS.md "PRE-WP4 STAGE 0 -- PRE-REGISTRATIONS" (d5), with the implementation fixed in "PRE-WP4 STAGE 2 --
launch notes and scorer choices" before any stage-2 row was read. Computed only after (d).

For each (full, A0) and (full, A0_tol) pair -- and (shipped, ...) if a feature was adopted -- on every sequence
whose P5a coverage ratio (make_tables.coverage_parity: min/max of the arms' median frame coverage over usable
reps) is < 0.90:
  * every usable rep of both arms (status OK, track_success 1, P5a floor -- prewp4_stage2_score.load) gives the
    first and last timestamps its result.txt associates with GT (traj_eval.associated_span; evaluate()'s own
    association, max_diff 0.02 s);
  * the common interval is [max of firsts, min of lasts] over ALL of them;
  * shorter than 25 % of the GT file's duration -> "no common span";
  * otherwise each rep is re-scored on it with traj_eval.evaluate(t_range=...) -- the associated pair is cropped
    before alignment, so Sim(3) ATE, SE(3) ATE and s are recomputed on the interval.
Medians (IQR, n) per arm go to a supplementary table labelled "matched span", never merged into T1. The five
pairs P5a discarded in WP6 (fr1_360, fr1_desk, fr1_desk2, KITTI 01, KITTI 06) are listed whatever their ratio now.

Usage: prewp4_matched_span.py --root runs/prewp4_s2_eval-server [--shipped full_K13]
"""
import argparse
import math
import statistics as st
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import adoption_rule as R                                      # noqa: E402
import datasets as D                                           # noqa: E402
import make_tables as MT                                       # noqa: E402
import traj_eval as TE                                         # noqa: E402
from prewp4_stage2_score import SEQS, load, num, f             # noqa: E402

WP6_DISCARDED = {("tum", "freiburg1_360"), ("tum", "freiburg1_desk"), ("tum", "freiburg1_desk2"),
                 ("kitti", "01"), ("kitti", "06")}
MIN_SPAN = 0.25


def med_iqr(v):
    v = [x for x in v if x == x]
    return (st.median(v), R.iqr(v), len(v)) if v else (float("nan"), float("nan"), 0)


def pair(root, ref_arm, mono, table):
    rows = MT.load_rows([p for p in (Path(root) / ref_arm / "summary.csv", Path(root) / mono / "summary.csv")
                         if p.exists()])
    parity = MT.coverage_parity(MT.aggregate(rows), [ref_arm, mono])
    groups = {a: load(str(Path(root) / a)) for a in (ref_arm, mono)}
    print(f"\n==== matched span: {ref_arm} vs {mono} ====")
    for k in SEQS:
        pv = parity.get(k)
        named = " [WP6-discarded pair]" if k in WP6_DISCARDED else ""
        if pv is None:
            if named:
                print(f"  {k[0]} {k[1]:32s} not run by both arms{named}")
            continue
        ratio = pv["ratio"]
        if pv["valid"]:
            if named:
                print(f"  {k[0]} {k[1]:32s} ratio {f(ratio, 2)} >= {MT.COVERAGE_PARITY} -- parity holds, "
                      f"not a (d5) pair{named}")
            continue
        spec = D.resolve(*k)
        reps = {a: [r for r in groups[a].get(k, []) if r["_usable"]] for a in (ref_arm, mono)}
        spans, missing = {}, []
        for a, rr in reps.items():
            for r in rr:
                est = Path(root) / a / f"{k[0]}_{k[1]}_rep{r['rep']}" / "result.txt"
                if not est.exists():
                    missing.append(f"{a} rep{r['rep']}")
                    continue
                spans[(a, r["rep"])] = (est, TE.associated_span(est, spec.gt, spec.gt_format, spec.extrinsics))
        head = f"  {k[0]} {k[1]:32s} ratio {f(ratio, 2)}  usable {len(reps[ref_arm])}/{len(reps[mono])}{named}"
        if missing:
            print(f"{head}\n      MISSING result.txt for {', '.join(missing)} -- pair not computed")
            continue
        if not reps[ref_arm] or not reps[mono]:
            print(f"{head}\n      an arm has no usable rep -- no matched span")
            continue
        t0 = max(s["first"] for _, s in spans.values())
        t1 = min(s["last"] for _, s in spans.values())
        dur = next(iter(spans.values()))[1]["gt_duration"]
        frac = (t1 - t0) / dur if dur > 0 else float("nan")
        if not (t1 > t0) or frac < MIN_SPAN:
            print(f"{head}\n      NO COMMON SPAN: [{t0:.2f}, {t1:.2f}] = {frac:.1%} of GT duration {dur:.1f} s (< 25 %)")
            table.append((ref_arm, mono, k, ratio, "no common span", frac, None))
            continue
        res = {}
        for a in (ref_arm, mono):
            m = [TE.evaluate(spans[(a, r["rep"])][0], spec.gt, spec.gt_format, spec.extrinsics, t_range=(t0, t1))
                 for r in reps[a]]
            res[a] = dict(sim3=med_iqr([x["ate_sim3_rmse"] for x in m]), se3=med_iqr([x["ate_se3_rmse"] for x in m]),
                          s=med_iqr([x["scale_s"] for x in m]),
                          full=med_iqr([num(r["ate_sim3_rmse"]) for r in reps[a]]),
                          err=[x["traj_error"] for x in m if x["traj_error"]])
        print(f"{head}\n      common span [{t0:.2f}, {t1:.2f}] = {frac:.1%} of GT duration {dur:.1f} s")
        for a in (ref_arm, mono):
            v = res[a]
            print(f"      {a:16s} matched: Sim3 {f(v['sim3'][0])} (IQR {f(v['sim3'][1])}, n={v['sim3'][2]})  "
                  f"SE3 {f(v['se3'][0])}  s {f(v['s'][0])}   |  whole-run Sim3 {f(v['full'][0])}"
                  f"{'   errors: ' + '; '.join(v['err']) if v['err'] else ''}")
        table.append((ref_arm, mono, k, ratio, "matched", frac, res))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", required=True)
    ap.add_argument("--shipped", default="full")
    a = ap.parse_args()
    table = []
    for ref_arm in ["full"] + ([a.shipped] if a.shipped != "full" else []):
        for mono in ("A0", "A0_tol"):
            pair(a.root, ref_arm, mono, table)
    print("\n==== supplementary table: \"matched span\" (never merged into T1) ====")
    print(f"  {'pair':22s} {'sequence':34s} {'cov ratio':>9s} {'span':>6s}  Sim3 ref / mono   SE3 ref / mono   s ref / mono")
    for ref_arm, mono, k, ratio, state, frac, res in table:
        if state != "matched":
            print(f"  {ref_arm + '|' + mono:22s} {k[0] + ' ' + k[1]:34s} {f(ratio, 2):>9s} {frac:6.1%}  no common span")
            continue
        r, m = res[ref_arm], res[mono]
        print(f"  {ref_arm + '|' + mono:22s} {k[0] + ' ' + k[1]:34s} {f(ratio, 2):>9s} {frac:6.1%}  "
              f"{f(r['sim3'][0])} / {f(m['sim3'][0])}   {f(r['se3'][0])} / {f(m['se3'][0])}   "
              f"{f(r['s'][0])} / {f(m['s'][0])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
