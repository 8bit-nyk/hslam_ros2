#!/usr/bin/env python3
"""Pre-WP4 stage 1, the WP2a-R3b founding-segment screen, as pre-registered in DECISIONS.md
"PRE-WP4 STAGE 0 -- PRE-REGISTRATIONS" (c). Each mode m in {relin, anchor} against `full`, same epoch.

DVs per (arm, sequence), medians over usable reps (status OK and track_success; n=3, one invocation):
  fo      = median founding_offset_log            (bars use |fo|)
  |ln s|  = median over reps of |ln scale_s|      (the DV as listed; |ln median s| is printed beside it)
Bars:
  F  each D sequence: |fo_m| <= max(0.223, 0.5*|fo_full|);  passes on >= 4 of 5 D sequences
  S  all 9 sequences: |ln s|_m <= |ln s|_full + ln 1.10
  K  each C sequence: |fo_m| <= max(0.223, |fo_full| + ln 1.10)
  T  no sequence where m has < 2 usable reps while full has 3/3
  A sequence where `full` has < 2 usable reps leaves F's denominator (the bar becomes all-but-one of
  the rest); two or more such D sequences => NOT DECIDED.
Mechanical choices the text leaves open, fixed here before any R3b row exists:
  * a sequence where `full` has < 2 usable reps is also left out of S and K (there is no reference);
  * a sequence where m has no usable rep fails F, S and K there (the burden is on the candidate).
Outcomes: relin passing F, S, K and T enters PAPER_CONFIG before stage 2; anchor passing is only
NOMINATED (stop and report; the user decides). ATE and [INIT_CONSISTENCY] are descriptive, never decisional.

Usage: prewp4_r3b_screen.py --root runs/prewp4_s1_eval-server/r3b
"""
import argparse
import csv
import math
import re
import statistics as st
import sys
from pathlib import Path

D = [("kitti", "03"), ("kitti", "04"), ("kitti", "06"),
     ("tum", "freiburg3_long_office_household"), ("tum", "freiburg2_desk")]
C = [("kitti", "05"), ("kitti", "02"), ("kitti", "07"), ("tum", "freiburg1_room")]
MODES = ["full_R3b_relin", "full_R3b_anchor"]
FO_FLOOR = math.log(1.25)      # 0.223, the 20 Sep bar [0.8, 1.25]
TOL = math.log(1.10)           # P4c's scale tolerance


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def stats(root, arm, ds, sq):
    p = Path(root) / arm / "summary.csv"
    rows = [r for r in csv.DictReader(open(p)) if r["dataset"] == ds and r["sequence"] == sq] if p.exists() else []
    u = [r for r in rows if r["status"] == "OK" and r["track_success"] == "1"]
    fo = [num(r["founding_offset_log"]) for r in u]
    fo = [x for x in fo if x == x]
    s = [num(r["scale_s"]) for r in u]
    s = [x for x in s if x == x and x > 0]
    ate = [num(r["ate_sim3_rmse"]) for r in u]
    ate = [x for x in ate if x == x]
    return dict(n=len(rows), u=len(u),
                fo=st.median(fo) if fo else float("nan"),
                lns=st.median(abs(math.log(x)) for x in s) if s else float("nan"),
                lns_of_med=abs(math.log(st.median(s))) if s else float("nan"),
                ate=st.median(ate) if ate else float("nan"),
                rej=sum(int(num(r.get("init_rejected_accepted")) or 0) for r in rows
                        if num(r.get("init_rejected_accepted")) == num(r.get("init_rejected_accepted"))),
                ic=init_consistency(Path(root) / arm, ds, sq, [r["rep"] for r in u]))


def init_consistency(armdir, ds, sq, reps):
    """Descriptive: mean_iR snap -> init at the initialisation that stuck, and the founding-map ratio at the
    last of the first 15 keyframes after it. Medians over usable reps."""
    snap, init, ratio = [], [], []
    for rep in reps:
        log = armdir / f"{ds}_{sq}_rep{rep}" / "run.log"
        if not log.exists():
            continue
        text = log.read_text(errors="replace")
        i = text.rfind("[INIT_CONSISTENCY] stage=init")
        if i < 0:
            continue
        m = re.search(r"mean_iR_snap=([-\d.naif]+) mean_iR_init=([-\d.naif]+)", text[i:])
        if m:
            snap.append(num(m.group(1))); init.append(num(m.group(2)))
        kf = re.findall(r"stage=kf kf=\d+ founding_active=\d+ founding_idepth_ratio_med=([-\d.naif]+)", text[i:])
        if kf:
            ratio.append(num(kf[-1]))
    med = lambda v: st.median([x for x in v if x == x]) if [x for x in v if x == x] else float("nan")
    return med(snap), med(init), med(ratio)


def screen(root, mode):
    print(f"\n=== {mode} vs full ===")
    print(f"  {'set':3s} {'sequence':32s} {'u f/m':>6s} {'fo_full':>8s} {'fo_m':>8s} {'F/K bar':>8s} "
          f"{'lns_f':>7s} {'lns_m':>7s} {'S':>2s} {'F/K':>3s} {'T':>2s}   ATE full / m   iR snap>init, fr15 (full | m)")
    f_pass = f_den = 0
    s_ok = k_ok = t_ok = True
    excluded_d = 0
    for grp, seqs in (("D", D), ("C", C)):
        for ds, sq in seqs:
            f = stats(root, "full", ds, sq)
            m = stats(root, mode, ds, sq)
            t_hit = not (m["u"] < 2 and f["u"] == 3)
            t_ok &= t_hit
            if f["u"] < 2:
                if grp == "D":
                    excluded_d += 1
                print(f"  {grp:3s} {ds + ' ' + sq:32s} {f['u']}/{m['u']:<4d} full has < 2 usable reps: out of "
                      f"{'F, ' if grp == 'D' else 'K, '}S   T {'ok' if t_hit else 'FAIL'}")
                continue
            has_m = m["u"] > 0 and m["fo"] == m["fo"] and m["lns"] == m["lns"]
            s_hit = has_m and m["lns"] <= f["lns"] + TOL
            s_ok &= s_hit
            if grp == "D":
                bar = max(FO_FLOOR, 0.5 * abs(f["fo"]))
                hit = has_m and abs(m["fo"]) <= bar
                f_den += 1
                f_pass += hit
            else:
                bar = max(FO_FLOOR, abs(f["fo"]) + TOL)
                hit = has_m and abs(m["fo"]) <= bar
                k_ok &= hit
            yn = lambda b: "ok" if b else "NO"
            print(f"  {grp:3s} {ds + ' ' + sq:32s} {f['u']}/{m['u']:<4d} {f['fo']:+8.3f} {m['fo']:+8.3f} {bar:8.3f} "
                  f"{f['lns']:7.3f} {m['lns']:7.3f} {yn(s_hit):>2s} {yn(hit):>3s} {yn(t_hit):>2s}   "
                  f"{f['ate']:.3f} / {m['ate']:.3f}   "
                  f"{f['ic'][0]:.2f}>{f['ic'][1]:.2f},{f['ic'][2]:.2f} | {m['ic'][0]:.2f}>{m['ic'][1]:.2f},{m['ic'][2]:.2f}"
                  f"{'   rejected-init runs full/m ' + str(f['rej']) + '/' + str(m['rej']) if f['rej'] or m['rej'] else ''}")
            if (m["lns_of_med"] <= f["lns_of_med"] + TOL) != s_hit and has_m:
                print(f"      note: S differs under |ln median s| ({f['lns_of_med']:.3f} vs {m['lns_of_med']:.3f})")
    if excluded_d >= 2:
        print(f"  {mode}: NOT DECIDED -- {excluded_d} D sequences where full has < 2 usable reps")
        return None
    need = 4 if excluded_d == 0 else f_den - 1
    f_ok = f_pass >= need
    ok = f_ok and s_ok and k_ok and t_ok
    print(f"  F {f_pass}/{f_den} (need {need}) {'PASS' if f_ok else 'FAIL'} | S {'PASS' if s_ok else 'FAIL'} | "
          f"K {'PASS' if k_ok else 'FAIL'} | T {'PASS' if t_ok else 'FAIL'}  =>  {mode}: {'PASS' if ok else 'FAIL'}")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    a = ap.parse_args()
    res = {m: screen(a.root, m) for m in MODES}
    relin, anchor = res["full_R3b_relin"], res["full_R3b_anchor"]
    print()
    if relin is None or anchor is None:
        print("  OUTCOME: NOT DECIDED for " + ", ".join(m for m, r in res.items() if r is None) + " -- report")
    if relin:
        print("  OUTCOME: relin PASSES (bug-fix class) -> --init-founding-fix=relin enters PAPER_CONFIG before stage 2")
    elif relin is False:
        print("  OUTCOME: relin FAILS -> stage 2 runs without it; recorded as a characterised limitation")
    if anchor:
        print("  OUTCOME: anchor PASSES (feature class) -> NOMINATED only: stop and report; the user decides")
    elif anchor is False:
        print("  OUTCOME: anchor FAILS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
