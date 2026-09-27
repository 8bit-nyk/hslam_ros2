#!/usr/bin/env python3
"""Pre-WP4 stage 2b, the softer-pin-weight screen for --init-founding-fix=anchor (DECISIONS.md
"PRE-WP4 STAGE 2b"). Generalises prewp4_r3b_screen.py (read that first -- this file repeats its
F/S/K/T bars almost verbatim) in three ways:

  * the reference is not a same-root `full` arm run alongside the candidates, but the FROZEN
    stage-2 `full` row set (--ref-root, default runs/prewp4_s2_eval-server; n=10 KITTI / n=5 TUM),
    read fresh every time this script runs;
  * T is made rep-aware, because stage 2b runs n=3 on 7 sequences but n=5 on the two named risks
    (KITTI 04, fr2_desk) -- a literal "< 2 usable reps while full has 3/3" would be wrong at n=5;
  * two bars are added beyond F/S/K/T, because a softer pin is a narrower fix than the R3b anchor
    itself and needs narrower evidence that it didn't buy its scale/founding-offset numbers by a
    failure mode invisible to F/S/K/T:
      U (fallback)  the anchor is a PIN; a softer pin can slip and let the founding segment fall
                    back to whatever [INIT_CONSISTENCY]'s stage=init line calls a non-metric
                    hand-over (metric=no). Scoped to KITTI 04 (the named risk with the shortest,
                    most fragile founding segment -- 271 frames total): usable >= 4/5 AND zero
                    runs (usable or not -- a fallback that then crashed is still a fallback) whose
                    LAST stage=init line reads metric=no.
      E (ATE cost)  the softer pin exists to trade founding-offset/scale correctness for cost
                    elsewhere; the adoption rule's own +25% "no non-exempt regression" band (C) is
                    the pre-existing bar for that trade, and anchor v1's fr2_desk failure was
                    exactly a C-band failure. On every sequence with >= 2 usable reps on both
                    sides: median Sim(3) ATE_mode <= 1.25 x median Sim(3) ATE_ref.
  U and E are reported for all 9 sequences (U's metric=no counts, E's per-sequence ratio) because
  the numbers are informative even off the sequence that gates the bar; only the gating sequence
  decides the bar itself.

DVs per (arm, sequence), medians over usable reps (status OK and track_success; n=3 or n=5, one
invocation each):
  fo      = median founding_offset_log            (bars use |fo|)
  |ln s|  = median over reps of |ln scale_s|       (the DV as listed; |ln median s| is printed beside it)
Bars (unchanged from R3b unless noted):
  F  each D sequence: |fo_m| <= max(0.223, 0.5*|fo_ref|);  passes on >= 4 of 5 D sequences
  S  all 9 sequences: |ln s|_m <= |ln s|_ref + ln 1.10
  K  each C sequence: |fo_m| <= max(0.223, |fo_ref| + ln 1.10)
  T  [rep-aware] a sequence fails T iff mode usable < ceil(2/3 * n_mode) while the reference's
     usable rate (usable / total rows) is >= 0.8.
  U  [new] KITTI 04 only: mode usable >= 4/5 AND zero metric=no hand-overs over every rep (any status).
  E  [new] every sequence with >= 2 usable reps on both mode and reference: median ATE_mode <=
     1.25 * median ATE_ref.
  A sequence where the reference has < 2 usable reps leaves F's denominator (the bar becomes
  all-but-one of the rest); two or more such D sequences => NOT DECIDED, exactly as R3b.
Mechanical choices carried over from R3b, unchanged:
  * a sequence where the reference has < 2 usable reps is also left out of S and K;
  * a sequence where m has no usable rep fails F, S and K there (the burden is on the candidate);
  * fo_ref undefined (windowed estimator skipped window 0) leaves F / K the same way as < 2 usable.
Verdict per mode: PASS iff F & S & K & T & U & E; NOT DECIDED as in R3b; else FAIL, listing the
bars that failed. --controls are printed beside the modes for context (their weight is the same
as a matching aw arm's) but are never scored by any bar. --bracket-root, if given, is a purely
descriptive n=1-per-sequence environment check against the reference's median scale, independent
of any mode.

Usage: prewp4_s2b_screen.py --root runs/prewp4_s2b_eval-server --ref-root runs/prewp4_s2_eval-server
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
FO_FLOOR = math.log(1.25)      # 0.223, the 20 Sep bar [0.8, 1.25]
TOL = math.log(1.10)           # P4c's scale tolerance / the env-drift flag threshold
ATE_TOL = 1.25                 # the adoption rule's C-band ceiling (+25%)
REPS = {("kitti", "04"): 5, ("tum", "freiburg2_desk"): 5}      # the two named risks; else n=3
DEFAULT_MODES = ["full_R3b_anchor_aw2500", "full_R3b_anchor_aw1000"]
DEFAULT_CONTROLS = ["S1_alphaw_lo", "S1_alphaw_1000"]


def n_for(ds, sq):
    return REPS.get((ds, sq), 3)


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
                s_med=st.median(s) if s else float("nan"),
                ate=st.median(ate) if ate else float("nan"),
                all_reps=[r["rep"] for r in rows],
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


def metric_no_stats(armdir, ds, sq, reps):
    """U bar support: over every rep (any status -- a fallback that then crashed is still a fallback),
    how many have a LAST [INIT_CONSISTENCY] stage=init line reading metric=no. Returns
    (n_metric_no, n_examined, n_missing_log)."""
    n_no = n_examined = n_missing = 0
    for rep in reps:
        log = armdir / f"{ds}_{sq}_rep{rep}" / "run.log"
        if not log.exists():
            n_missing += 1
            continue
        text = log.read_text(errors="replace")
        hits = re.findall(r"\[INIT_CONSISTENCY\] stage=init.*?metric=(\w+)", text)
        if not hits:
            n_missing += 1
            continue
        n_examined += 1
        if hits[-1] == "no":
            n_no += 1
    return n_no, n_examined, n_missing


def screen(root, ref_root, mode):
    print(f"\n=== {mode} vs ref ({ref_root}/full) ===")
    print(f"  {'set':3s} {'sequence':32s} {'u f/m':>7s} {'fo_ref':>8s} {'fo_m':>8s} {'F/K bar':>8s} "
          f"{'lns_f':>7s} {'lns_m':>7s} {'S':>2s} {'F/K':>3s} {'T':>2s}   ATE ref / m   iR snap>init, fr15 (ref | m)")
    f_pass = f_den = 0
    s_ok = k_ok = t_ok = True
    excluded_d = 0
    per_seq = {}
    for grp, seqs in (("D", D), ("C", C)):
        for ds, sq in seqs:
            f = stats(ref_root, "full", ds, sq)
            m = stats(root, mode, ds, sq)
            per_seq[(ds, sq)] = (f, m)
            n_mode = n_for(ds, sq)
            need_t = math.ceil(2 * n_mode / 3)
            ref_rate = (f["u"] / f["n"]) if f["n"] else 0.0
            t_hit = not (m["u"] < need_t and ref_rate >= 0.8)
            t_ok &= t_hit
            if f["u"] < 2:
                if grp == "D":
                    excluded_d += 1
                print(f"  {grp:3s} {ds + ' ' + sq:32s} {f['u']}/{m['u']:<5d} ref has < 2 usable reps: out of "
                      f"{'F, ' if grp == 'D' else 'K, '}S   T {'ok' if t_hit else 'FAIL'}")
                continue
            has_s = m["u"] > 0 and m["lns"] == m["lns"]
            has_m = m["u"] > 0 and m["fo"] == m["fo"]
            s_hit = has_s and m["lns"] <= f["lns"] + TOL
            s_ok &= s_hit
            if f["fo"] != f["fo"]:
                hit, bar = True, float("nan")
                if grp == "D":
                    excluded_d += 1
                print(f"  {grp:3s} {ds + ' ' + sq:32s} fo_ref undefined (window 0 skipped): out of "
                      f"{'F' if grp == 'D' else 'K'}; S {'ok' if s_hit else 'NO'} "
                      f"({f['lns']:.3f} vs {m['lns']:.3f}); fo_m {m['fo']:+.3f}")
            elif grp == "D":
                bar = max(FO_FLOOR, 0.5 * abs(f["fo"]))
                hit = has_m and abs(m["fo"]) <= bar
                f_den += 1
                f_pass += hit
            else:
                bar = max(FO_FLOOR, abs(f["fo"]) + TOL)
                hit = has_m and abs(m["fo"]) <= bar
                k_ok &= hit
            if f["fo"] != f["fo"]:
                continue
            yn = lambda b: "ok" if b else "NO"
            print(f"  {grp:3s} {ds + ' ' + sq:32s} {f['u']}/{m['u']:<5d} {f['fo']:+8.3f} {m['fo']:+8.3f} {bar:8.3f} "
                  f"{f['lns']:7.3f} {m['lns']:7.3f} {yn(s_hit):>2s} {yn(hit):>3s} {yn(t_hit):>2s}   "
                  f"{f['ate']:.3f} / {m['ate']:.3f}   "
                  f"{f['ic'][0]:.2f}>{f['ic'][1]:.2f},{f['ic'][2]:.2f} | {m['ic'][0]:.2f}>{m['ic'][1]:.2f},{m['ic'][2]:.2f}")
            if (m["lns_of_med"] <= f["lns_of_med"] + TOL) != s_hit and has_s:
                print(f"      note: S differs under |ln median s| ({f['lns_of_med']:.3f} vs {m['lns_of_med']:.3f})")

    if excluded_d >= 2:
        print(f"  {mode}: NOT DECIDED -- {excluded_d} D sequences where ref has < 2 usable reps")
        return None
    need = 4 if excluded_d == 0 else f_den - 1
    f_ok = f_pass >= need
    print(f"  F {f_pass}/{f_den} (need {need}) {'PASS' if f_ok else 'FAIL'} | S {'PASS' if s_ok else 'FAIL'} | "
          f"K {'PASS' if k_ok else 'FAIL'} | T {'PASS' if t_ok else 'FAIL'}")

    # --- U bar: fallback did not fire, scoped to KITTI 04 --------------------------------------
    print(f"\n  U (fallback) -- metric=no hand-overs (LAST [INIT_CONSISTENCY] stage=init line), all reps of {mode}:")
    print(f"    {'set':3s} {'sequence':32s} {'metric=no / examined':>22s}  {'missing log':>11s}")
    armdir = Path(root) / mode
    no_counts = {}
    for grp, seqs in (("D", D), ("C", C)):
        for ds, sq in seqs:
            _, m = per_seq[(ds, sq)]
            n_no, n_ex, n_miss = metric_no_stats(armdir, ds, sq, m["all_reps"])
            no_counts[(ds, sq)] = (n_no, n_ex, n_miss)
            flag = "  <- U bar sequence" if (ds, sq) == ("kitti", "04") else ""
            print(f"    {grp:3s} {ds + ' ' + sq:32s} {n_no}/{n_ex:<19d}  {n_miss:>11d}{flag}")
    k04_no, _, _ = no_counts[("kitti", "04")]
    k04_u = per_seq[("kitti", "04")][1]["u"]
    u_ok = (k04_u >= 4) and (k04_no == 0)
    print(f"  U: KITTI 04 usable {k04_u}/5 (need >=4), metric=no hand-overs {k04_no} (need 0) -> "
          f"{'PASS' if u_ok else 'FAIL'}")

    # --- E bar: ATE cost inside the adoption rule's C band (+25%) -------------------------------
    print(f"\n  E (ATE cost) -- median Sim(3) ATE ratio {mode}/ref, sequences with >= 2 usable reps on both sides:")
    e_ok = True
    for grp, seqs in (("D", D), ("C", C)):
        for ds, sq in seqs:
            f, m = per_seq[(ds, sq)]
            f_ate_ok = f["ate"] == f["ate"] and f["ate"] > 0
            if f["u"] >= 2 and m["u"] >= 2 and f_ate_ok and m["ate"] == m["ate"]:
                ratio = m["ate"] / f["ate"]
                hit = ratio <= ATE_TOL
                e_ok &= hit
                print(f"    {grp:3s} {ds + ' ' + sq:32s} ATE ref {f['ate']:.3f}  m {m['ate']:.3f}  "
                      f"ratio {ratio:.3f} (<= {ATE_TOL})  {'ok' if hit else 'NO'}")
            else:
                print(f"    {grp:3s} {ds + ' ' + sq:32s} skipped (usable ref {f['u']}, m {m['u']})")
    print(f"  E: {'PASS' if e_ok else 'FAIL'}")

    ok = f_ok and s_ok and k_ok and t_ok and u_ok and e_ok
    failing = [name for name, v in (("F", f_ok), ("S", s_ok), ("K", k_ok), ("T", t_ok),
                                     ("U", u_ok), ("E", e_ok)) if not v]
    print(f"\n  => {mode}: {'PASS' if ok else 'FAIL'}" + (f"  (failing: {', '.join(failing)})" if failing else ""))
    return ok


def describe_control(root, ref_root, name):
    print(f"\n--- control {name} (descriptive only, not scored by any bar) ---")
    print(f"  {'set':3s} {'sequence':32s} {'u ref/c':>8s} {'fo_ref':>8s} {'fo_c':>8s} "
          f"{'lns_ref':>7s} {'lns_c':>7s}   ATE ref / c")
    for grp, seqs in (("D", D), ("C", C)):
        for ds, sq in seqs:
            f = stats(ref_root, "full", ds, sq)
            c = stats(root, name, ds, sq)
            print(f"  {grp:3s} {ds + ' ' + sq:32s} {f['u']}/{c['u']:<6d} {f['fo']:+8.3f} {c['fo']:+8.3f} "
                  f"{f['lns']:7.3f} {c['lns']:7.3f}   {f['ate']:.3f} / {c['ate']:.3f}")


def describe_bracket(bracket_root, ref_root):
    print(f"\n=== environment bracket ({bracket_root}/full, n=1) vs ref median scale ===")
    print(f"  {'set':3s} {'sequence':32s} {'s_bracket':>10s} {'s_ref_med':>10s} {'|ln ratio|':>10s}  flag")
    for grp, seqs in (("D", D), ("C", C)):
        for ds, sq in seqs:
            f = stats(ref_root, "full", ds, sq)
            b = stats(bracket_root, "full", ds, sq)
            f_ok = f["s_med"] == f["s_med"] and f["s_med"] > 0
            b_ok = b["s_med"] == b["s_med"] and b["s_med"] > 0
            if not (f_ok and b_ok):
                print(f"  {grp:3s} {ds + ' ' + sq:32s} bracket or ref scale unavailable "
                      f"(bracket n={b['n']} u={b['u']})")
                continue
            ln_ratio = abs(math.log(b["s_med"] / f["s_med"]))
            flag = "ENV DRIFT" if ln_ratio > TOL else ""
            print(f"  {grp:3s} {ds + ' ' + sq:32s} {b['s_med']:10.4f} {f['s_med']:10.4f} {ln_ratio:10.3f}  {flag}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True, help="the 2b root, containing <arm>/summary.csv")
    ap.add_argument("--ref-root", default="runs/prewp4_s2_eval-server",
                     help="its full/summary.csv is the reference (medians over usable reps)")
    ap.add_argument("--modes", nargs="+", default=DEFAULT_MODES)
    ap.add_argument("--controls", nargs="+", default=DEFAULT_CONTROLS,
                     help="printed beside the modes, never scored by any bar")
    ap.add_argument("--bracket-root", default=None,
                     help="optional: a root whose full/summary.csv holds n=1-per-sequence "
                          "environment-bracket rows")
    a = ap.parse_args()

    res = {mode: screen(a.root, a.ref_root, mode) for mode in a.modes}

    for name in a.controls:
        describe_control(a.root, a.ref_root, name)

    if a.bracket_root:
        describe_bracket(a.bracket_root, a.ref_root)

    print("\nNOMINATION:")
    aw2500, aw1000 = "full_R3b_anchor_aw2500", "full_R3b_anchor_aw1000"
    passing = [m for m in a.modes if res.get(m) is True]
    not_decided = [m for m in a.modes if res.get(m) is None]
    if set(a.modes) == {aw2500, aw1000}:
        if len(passing) == 2:
            print(f"  both {aw2500} and {aw1000} PASS -> nominate {aw2500}")
        elif len(passing) == 1:
            nom = passing[0]
            print(f"  exactly one mode PASSES -> nominate {nom}")
            if nom == aw1000:
                print("  PROVISIONAL: control S1_alphaw_1000 must be read")
                describe_control(a.root, a.ref_root, "S1_alphaw_1000")
        else:
            print("  no nomination" + (" (NOT DECIDED: " + ", ".join(not_decided) + ")" if not_decided else ""))
    else:
        # Generic invocation (e.g. --modes not the {aw2500, aw1000} pair): report, don't force the
        # two-mode nomination wording onto a set it wasn't written for.
        if passing:
            print(f"  PASS: {', '.join(passing)}")
        else:
            print("  no nomination -- no mode PASSES" +
                  (" (NOT DECIDED: " + ", ".join(not_decided) + ")" if not_decided else "") +
                  "  [note: --modes is not the default {aw2500, aw1000} pair -- generic report]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
