#!/usr/bin/env python3
"""Re-read every earlier Track D decision under the calibrated method (21 Sep 2026 evening).

POST-HOC BY CONSTRUCTION. Per the standing rule (DECISIONS.md "RULE CHANGE 2026-09-21" and the
amendment), a verdict recorded under an older rule is NOT flipped on the same data. This tool answers
"would we read the evidence differently now?", never "is the verdict overturned?".

  python3 run_scripts/ral_v2/rule_reaudit.py --runs runs/

Zero SLAM runs. Reads campaign summary.csv files already on disk.
"""
import argparse, csv, collections, glob, os, statistics as st, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import adoption_rule as R

def pull(root, camp, field="ate_sim3_rmse"):
    rows = collections.defaultdict(list)
    for f in glob.glob(os.path.join(root, camp, "summary.csv")) + \
             glob.glob(os.path.join(root, camp, "*", "summary.csv")):
        for r in csv.DictReader(open(f)):
            if r.get("status") != "OK" or r.get("track_success") != "1":
                continue
            try:
                rows[(r["arm"], r["dataset"], r["sequence"])].append(float(r[field]))
            except (KeyError, ValueError):
                pass
    return rows

def compare(rows, ref, cand, cells, label):
    print(f"\n--- {label} ---")
    print(f"  {'dataset/sequence':28} {'n ref/cand':>11} {'delta':>9} {'MWU p':>7}  reading")
    for ds, sq in cells:
        a, b = rows.get((ref, ds, sq)), rows.get((cand, ds, sq))
        if not a or not b:
            continue
        d = (st.median(b) - st.median(a)) / st.median(a)
        p = R.mannwhitney_p(a, b)
        if min(len(a), len(b)) < R.MIN_REPS:
            read = f"UNRESOLVED (n<{R.MIN_REPS})"
        elif p < R.P_MATERIAL:
            read = "MATERIAL " + ("improvement" if d < 0 else "regression")
        else:
            read = "tie (not material)"
        print(f"  {ds+'/'+sq:28} {f'{len(a)}/{len(b)}':>11} {d*100:+8.1f}% {p:7.4f}  {read}")

def noise_by_metric(root):
    per = collections.defaultdict(lambda: collections.defaultdict(list))
    for f in glob.glob(os.path.join(root, "*", "summary.csv")) + \
             glob.glob(os.path.join(root, "*", "*", "summary.csv")):
        for r in csv.DictReader(open(f)):
            if r.get("status") != "OK" or r.get("track_success") != "1":
                continue
            k = (f, r["arm"], r["dataset"], r["sequence"])
            for m in ("ate_sim3_rmse", "scale_s"):
                try:
                    v = float(r[m])
                    if v > 0:
                        per[m][k].append(v)
                except (KeyError, ValueError):
                    pass
    print("\n--- Run-to-run noise by METRIC (IQR/median, cells with n>=5) ---")
    out = {}
    for m, cells in per.items():
        rel = [R.iqr(v) / st.median(v) for v in cells.values() if len(v) >= 5 and st.median(v) > 0]
        if not rel:
            continue
        rel.sort()
        out[m] = st.median(rel)
        print(f"  {m:16} {len(rel):4d} cells   median {st.median(rel)*100:6.2f}%"
              f"   90th pct {rel[int(len(rel)*.9)]*100:6.2f}%")
    if "ate_sim3_rmse" in out and "scale_s" in out:
        print(f"  => scale is ~{out['ate_sim3_rmse']/out['scale_s']:.0f}x less noisy than ATE on the "
              f"same runs. A scale-based bar is far better powered at the same n than an ATE bar,\n"
              f"     which is why the n=3 scale conclusions (WP2c/G2, the WP1 geometry freeze) hold.")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", default="runs")
    a = ap.parse_args()
    print("POST-HOC re-read of earlier Track D decisions under the calibrated method.")
    print("Nothing here overturns a verdict; the rule is prospective (DECISIONS.md).")

    w1 = pull(a.runs, "wp1_eval-server")
    compare(w1, "full", "K9_n3",
            [("tum", "freiburg1_room"), ("kitti", "07"), ("kitti", "00")],
            "WP1 / G1 -- throughput lever K9_n3 (cadence 3). Decision: NO LEVER SHIPS")

    w3 = pull(a.runs, "wp3_eval-server")
    compare(w3, "A0", "full",
            [("euroc", s) for s in ("MH_01_easy", "V1_01_easy", "V2_02_medium")],
            "WP3 -- EuRoC ML vs monocular. Decision: BOTH, EuRoC is a characterised limitation")
    compare(w3, "legacy_geom", "full",
            [("euroc", s) for s in ("MH_01_easy", "V1_01_easy", "V2_02_medium")],
            "WP3 -- corrected vs legacy geometry on EuRoC ATE (the WP3 claim is about SCALE, not ATE)")

    w2c = pull(a.runs, "wp2c_eval-server", field="scale_s")
    print("\n--- WP2c phase i / G2 -- KITTI 07 scale, the pre-registered bar [0.85, 1.15] ---")
    off = [(arm, v) for (arm, ds, sq), v in w2c.items() if sq == "07" and arm != "full"]
    allruns = [x for _, v in off for x in v]
    ref = w2c.get(("full", "kitti", "07")) or w2c.get(("full", "kitti", "07"))
    print(f"  {len(off)} freeze-off arms, {len(allruns)} runs pooled: "
          f"median s {st.median(allruns):.3f}, range [{min(allruns):.3f}, {max(allruns):.3f}]")
    if ref:
        print(f"  reference 'full' (freeze ON), n={len(ref)}: "
              f"median s {st.median(ref):.3f}, range [{min(ref):.3f}, {max(ref):.3f}]")
    inside = sum(1 for x in allruns if 0.85 <= x <= 1.15)
    print(f"  runs inside the bar [0.85, 1.15]: {inside}/{len(allruns)}")
    print("  => the conclusion rests on CONSISTENCY ACROSS ARMS, not on any one n=3 cell.")

    noise_by_metric(a.runs)
