#!/usr/bin/env python3
"""Calibration for the dataset-level adoption rule v2 -- the traceable basis for every constant.

Re-runnable: it reads the campaign summary.csv files already on disk and simulates. It runs ZERO
SLAM runs. Output is written up in docs/ral_v2_resubmission/wp/RULE_CALIBRATION.md.

  python3 run_scripts/ral_v2/rule_calibration.py --runs runs/ --selftest

Method. For each ABL-10 sequence, pool every full-track ATE recorded for one arm on it: that pool is
an empirical sample of the system's run-to-run distribution. Draw two independent samples of n reps
from the SAME pool and the measured difference between them is what "no difference at all" looks
like through our protocol (the null). Draw the candidate's reps from the pool scaled by (1+effect)
and it is what a true effect of that size looks like (the power). Everything else -- classification,
conditions A/B/C -- is imported from adoption_rule.py, so the calibration and the live evaluator
cannot drift apart.
"""
import argparse, csv, glob, math, os, random, statistics as st, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import adoption_rule as R

TUM = ["freiburg1_room", "freiburg1_desk", "freiburg2_desk",
       "freiburg2_large_no_loop", "freiburg3_long_office_household"]
KIT = ["00", "05", "06", "07", "10"]

def build_pools(root):  # noqa: C901
    """Pools are keyed by (campaign, arm, dataset, sequence).

    Reps are NEVER pooled across campaigns: a campaign is one binary epoch and the house rule
    forbids pooling arms from different binaries (PLAN.md section 5). Pooling across epochs would
    inflate the apparent run-to-run spread with between-binary differences and quietly overstate
    the noise this calibration exists to measure.
    """
    pool, chosen = {}, {}
    for f in glob.glob(os.path.join(root, "*", "*", "summary.csv")):
        camp = f.split(os.sep)[-3]
        for r in csv.DictReader(open(f)):
            if r.get("status") != "OK" or r.get("track_success") != "1":
                continue
            try:
                v = float(r["ate_sim3_rmse"])
            except (KeyError, ValueError):
                continue
            if v > 0:
                pool.setdefault((camp, r["arm"], r["dataset"], r["sequence"]), []).append(v)
    best = {}
    for ds, seqs in (("tum", TUM), ("kitti", KIT)):
        for s in seqs:
            cands = [(len(v), c, a, v) for (c, a, d, q), v in pool.items()
                     if d == ds and q == s and len(v) >= 5]
            if cands:
                n, c, a, v = max(cands, key=lambda x: x[0])
                best[(ds, s)] = v
                chosen[(ds, s)] = (c, a, n)
    return pool, best, chosen

def table_a(pool):
    print("\n=== Table A. Run-to-run noise per sequence (reference-style arms, n>=5 full-track reps) ===")
    print(f"{'dataset/sequence':38} {'arm':12} {'n':>3} {'median':>9} {'IQR':>8} {'IQR/med':>8}")
    for (camp, arm, ds, sq), v in sorted(pool.items(), key=lambda kv: (kv[0][2], kv[0][3], -len(kv[1]))):
        if len(v) < 5 or arm not in ("full", "K14_K15"):
            continue
        m = st.median(v); q = R.iqr(v)
        print(f"{ds+'/'+sq:38} {arm:12} {len(v):3d} {m:9.3f} {q:8.3f} {q/m*100:7.1f}%"
              f"   [{camp}]")

def table_b(best, n, trials):
    print(f"\n=== Table B. NULL: two IDENTICAL configs, n={n} reps each, {trials} draws ===")
    print("    |delta| = |median(A) - median(B)| / median(B), both drawn from the same pool")
    print(f"{'dataset/sequence':38} {'pool':>5} {'p50':>7} {'p90':>7} {'P|d|>10%':>9} {'P|d|>25%':>9}")
    for (ds, sq), p in sorted(best.items()):
        if len(p) < 8:
            continue
        d = []
        for _ in range(trials):
            A = [random.choice(p) for _ in range(n)]; B = [random.choice(p) for _ in range(n)]
            mb = st.median(B); d.append(abs(st.median(A) - mb) / mb)
        d.sort()
        f10 = sum(1 for x in d if x > .10) / len(d); f25 = sum(1 for x in d if x > .25) / len(d)
        print(f"{ds+'/'+sq:38} {len(p):5d} {d[len(d)//2]*100:6.1f}% {d[int(len(d)*.9)]*100:6.1f}%"
              f" {f10*100:8.1f}% {f25*100:8.1f}%")

def simulate(best, ds, seqs, effect, n, material, trials):
    """material: 'mwu' (the rule), 'iqr' (the rejected draft), 'pct10', 'none'."""
    npass = nnd = 0
    for _ in range(trials):
        per = {}
        for s in seqs:
            p = best[(ds, s)]
            ref = [random.choice(p) for _ in range(n)]
            cand = [random.choice(p) * (1 + effect) for _ in range(n)]
            c = R.classify({"ate": ref, "ok": n, "att": n}, {"ate": cand, "ok": n, "att": n})
            if material != "mwu":
                mr, mc = st.median(ref), st.median(cand)
                if material == "iqr":
                    mat = abs(mc - mr) > R.iqr(ref)
                elif material == "pct10":
                    mat = abs(c["delta"]) > 0.10
                else:
                    mat = True
                c["cls"] = ("improved" if c["delta"] < 0 else "regressed") if mat else "tie"
            per[s] = c
        v = R.judge(per)
        if v["verdict"] == "NOT DECIDED":
            nnd += 1
        elif v["verdict"] == "PASS":
            npass += 1
    return npass / trials, nnd / trials

def table_c(best, trials):
    print(f"\n=== Table C. Materiality test: null adoption vs power at a true -15%, {trials} sims ===")
    print("    (conditions A/B/C1 from adoption_rule.py; only the materiality test varies)")
    print(f"{'materiality':12} {'reps':>5} | {'TUM null':>9} {'TUM pwr':>8} {'TUM n/d':>8}"
          f" | {'KITTI null':>10} {'KIT pwr':>8} {'KIT n/d':>8}")
    for mat in ("iqr", "pct10", "none", "mwu"):
        for n in (5, 10):
            tn, _ = simulate(best, "tum", TUM, 0.0, n, mat, trials)
            tp, tnd = simulate(best, "tum", TUM, -0.15, n, mat, trials)
            kn, _ = simulate(best, "kitti", KIT, 0.0, n, mat, trials)
            kp, knd = simulate(best, "kitti", KIT, -0.15, n, mat, trials)
            star = " <-- the rule" if mat == "mwu" and n == 10 else ""
            print(f"{mat:12} {n:5d} | {tn*100:8.1f}% {tp*100:7.1f}% {tnd*100:7.1f}%"
                  f" | {kn*100:9.1f}% {kp*100:7.1f}% {knd*100:7.1f}%{star}")

def table_d(best, trials):
    print(f"\n=== Table D. Effect size vs power (materiality = MWU p<{R.P_MATERIAL}), {trials} sims ===")
    print(f"{'true effect':>12} {'reps':>5} | {'TUM adopt':>10} | {'KITTI adopt':>12}")
    for eff in (0.0, -0.05, -0.10, -0.15, -0.25, +0.15):
        for n in (5, 10):
            tp, _ = simulate(best, "tum", TUM, eff, n, "mwu", trials)
            kp, _ = simulate(best, "kitti", KIT, eff, n, "mwu", trials)
            print(f"{eff*100:+11.0f}% {n:5d} | {tp*100:9.1f}% | {kp*100:11.1f}%")

def table_e(best, trials):
    print(f"\n=== Table E. Sensitivity of the two constants, {trials} sims, n=10 ===")
    a0, c0 = R.A_MEDIAN_MAX, R.C1_REGRESSION
    print("  Condition A threshold:")
    for a in (-0.0, -0.05, -0.10, -0.15):
        R.A_MEDIAN_MAX = a
        tn, _ = simulate(best, "tum", TUM, 0.0, 10, "mwu", trials)
        tp, _ = simulate(best, "tum", TUM, -0.15, 10, "mwu", trials)
        kn, _ = simulate(best, "kitti", KIT, 0.0, 10, "mwu", trials)
        kp, _ = simulate(best, "kitti", KIT, -0.15, 10, "mwu", trials)
        print(f"    A<={a*100:+4.0f}% | TUM null {tn*100:5.1f}% pwr {tp*100:5.1f}%"
              f" | KITTI null {kn*100:5.1f}% pwr {kp*100:5.1f}%")
    R.A_MEDIAN_MAX = a0
    print("  Condition C1 threshold (an engineering veto -- expect it to be statistically inert):")
    for c in (0.15, 0.25, 0.50, 99.0):
        R.C1_REGRESSION = c
        tn, _ = simulate(best, "tum", TUM, 0.0, 10, "mwu", trials)
        tp, _ = simulate(best, "tum", TUM, -0.15, 10, "mwu", trials)
        kn, _ = simulate(best, "kitti", KIT, 0.0, 10, "mwu", trials)
        kp, _ = simulate(best, "kitti", KIT, -0.15, 10, "mwu", trials)
        lab = "none" if c > 9 else f"+{int(c*100)}%"
        print(f"    C1={lab:5} | TUM null {tn*100:5.1f}% pwr {tp*100:5.1f}%"
              f" | KITTI null {kn*100:5.1f}% pwr {kp*100:5.1f}%")
    R.C1_REGRESSION = c0

def selftest():
    try:
        from scipy import stats
    except ImportError:
        print("selftest: scipy unavailable here (expected on the eval server) -- skipped")
        return
    bad_m = bad_f = 0
    for _ in range(400):
        n1, n2 = random.choice([(5, 5), (5, 10), (10, 10), (3, 7), (8, 10), (10, 12)])
        a = [random.gauss(0, 1) for _ in range(n1)]; b = [random.gauss(0.3, 1) for _ in range(n2)]
        if abs(R.mannwhitney_p(a, b) - stats.mannwhitneyu(a, b, alternative="two-sided",
                                                          method="exact").pvalue) > 1e-9:
            bad_m += 1
    for _ in range(400):
        t = [random.randint(0, 12) for _ in range(4)]
        if sum(t) == 0:
            continue
        if abs(R.fisher_p(*t) - stats.fisher_exact([[t[0], t[1]], [t[2], t[3]]])[1]) > 1e-9:
            bad_f += 1
    print(f"selftest vs scipy: Mann-Whitney {bad_m}/400 mismatches, Fisher {bad_f}/400 mismatches")
    print(f"  spot check, the KITTI 07 track split 8/10 vs 10/10: Fisher p = {R.fisher_p(8,2,10,0):.4f}")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", default="runs")
    ap.add_argument("--trials", type=int, default=4000)
    ap.add_argument("--seed", type=int, default=20260921)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    random.seed(a.seed)
    if a.selftest:
        selftest()
    pool, best, chosen = build_pools(a.runs)
    missing = [k for k in [("tum", s) for s in TUM] + [("kitti", s) for s in KIT] if k not in best]
    if missing:
        print(f"WARNING: no n>=5 pool for {missing} -- those sequences are skipped")
    print(f"\nPools built from {a.runs}: {len(best)} sequences, "
          f"{sum(len(v) for v in best.values())} full-track reps. seed={a.seed}, trials={a.trials}")
    print("\n=== Table A0. The pool used as each sequence's noise model (largest single campaign x arm) ===")
    print(f"{'dataset/sequence':38} {'campaign':28} {'arm':16} {'n':>3}")
    for k in sorted(chosen):
        c, ar, n = chosen[k]
        print(f"{k[0]+'/'+k[1]:38} {c:28} {ar:16} {n:3d}")
    table_a(pool); table_b(best, 5, 20000); table_c(best, a.trials)
    table_d(best, a.trials); table_e(best, a.trials)
