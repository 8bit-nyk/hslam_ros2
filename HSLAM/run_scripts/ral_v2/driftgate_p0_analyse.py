#!/usr/bin/env python3
"""Drift-conditional gate, PHASE 0 analysis (FUTURE_DIRECTIONS.md section 11; DECISIONS.md 2026-09-23).

Input: the JSON written by driftgate_p0_extract.py (per-rep [PRIOR_ALIGN] series, H2-gate evaluations,
keyframe trajectory, GT-side prefix scales), from the three b2 phase-ii arms on 19 sequences.
Needs numpy + scipy only.  Usage: driftgate_p0_analyse.py p0_data.json

Question: does an ONLINE-observable quantity track evo's GT-derived scale_drift_pct_per_100m?
Primary test (fixed before any number was looked at):
  full arm, 19 sequences (TUM-10 + KITTI-9), per-sequence medians over usable reps,
  Spearman rho(proxy drift, GT drift), exact permutation p (n<=10) or 1e6-draw Monte-Carlo p (n>10).
Sign convention: the proxy is reported as the drift of ln(prior/map) = -slope of ln s_k, so that it
carries the same sign as evo's s = GT/est.  (T2 prints the raw -sign; O1/U use the aligned sign.)

Proxy (a) [PRIOR_ALIGN] s_k (FullSystem.cpp:4981), drift over ESTIMATED distance travelled:
  a_local  = OLS slope of ln s_k vs d_k over the run, x1e4  (%/100 m)
  a_prefix = the GT metric's own form: prefix-median of s_k at 10 distance fractions, slope/mean x1e4
  a_skip20 = a_local ignoring the first 20 keyframes (founding-segment sensitivity)
Proxy (b) [SCALE_DRIFT] live_ratio: the printf is commented out (FullSystem.cpp:5087); 0 lines on disk.
Proxy (c) H2 loop-closure gate: per-run median |ln(s_ransac/s_ml)| over non-degenerate evaluations.

Sections: T0 sanity; T1 reproduction of the recorded KITTI-9 test; T2 per-arm tables + correlations;
T4 gate relevance (does |X on full| order dATE the way |GT drift| does); T5 within-sequence tracking;
O1 per-sequence dATE with sensor ranks; O2 n=8 check; O3 ORACLE per-sequence gate through the real
adoption rule (one tau, both datasets); U alternative normalisations and level sensors.
The oracle (O3/U) was added AFTER the correlation tables were seen; it consumes no new data and is a
direct evaluation of the measured arms, not a statistical test.
"""
import importlib.util, itertools, json, math, os, sys
from collections import defaultdict
import numpy as np
from scipy.stats import rankdata

rng = np.random.default_rng(20260923)
HERE = os.path.dirname(os.path.abspath(__file__))


def usable(r):
    return r["status"] == "OK" and r["track_success"] == "1" and not r["traj_error"]


def cumdist(xyz):
    a = np.asarray(xyz, float)
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(a, axis=0), axis=1))])


def pa_series(r, min_n=50, skip=0):
    kf = np.asarray(r["pa_kf"], int)
    s = np.asarray([np.nan if v is None else v for v in r["pa_s"]], float)
    n = np.asarray(r["pa_n"], int)
    d = cumdist(r["est_xyz"])
    ok = (kf < r["est_rows"]) & (n >= min_n) & np.isfinite(s) & (s > 0) & (kf >= skip)
    return d[kf[ok]], s[ok]


def a_local(d, s):
    if len(d) < 10 or d[-1] - d[0] < 1.0:
        return np.nan
    b, _ = np.polyfit(d, np.log(s), 1)
    return b * 1e4


def a_prefix(d, s, n_bins=10):
    total = d[-1]
    if total < 1.0 or len(d) < 20:
        return np.nan
    dd, ss = [], []
    for frac in np.linspace(1.0 / n_bins, 1.0, n_bins):
        idx = int(np.searchsorted(d, frac * total)) + 1
        if idx < 10:
            continue
        dd.append(d[min(idx, len(d)) - 1])
        ss.append(np.median(s[:idx]))
    if len(dd) < 3:
        return np.nan
    b, _ = np.polyfit(dd, ss, 1)
    return b / np.mean(ss) * 1e4


def gate_stat(r):
    v = [abs(math.log(g["s_ransac"] / g["s_ml"])) for g in r["gates"]
         if g["s_ransac"] and g["s_ml"] and g["s_ransac"] > 0 and g["s_ml"] > 0 and not g["degenerate"]]
    return (float(np.median(v)) if v else np.nan), len(v)


def spearman(x, y):
    """Spearman rho with a TWO-SIDED permutation p: exact for n<=10, 1e6 Monte-Carlo draws above."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    ok = np.isfinite(x) & np.isfinite(y)
    x, y = x[ok], y[ok]
    n = len(x)
    if n < 4:
        return np.nan, np.nan, n, "n<4"
    rx, ry = rankdata(x), rankdata(y)
    rxc, ryc = rx - rx.mean(), ry - ry.mean()
    den = math.sqrt((rxc ** 2).sum() * (ryc ** 2).sum())
    if den == 0:
        return np.nan, np.nan, n, "constant"
    obs = float(rxc @ ryc / den)
    thr = abs(obs) - 1e-12
    if n <= 10:
        cnt = tot = 0
        it = itertools.permutations(range(n))
        while True:
            chunk = list(itertools.islice(it, 200000))
            if not chunk:
                break
            P = np.asarray(chunk, dtype=np.int8)
            r = (ryc[P] @ rxc) / den
            cnt += int((np.abs(r) >= thr).sum())
            tot += len(chunk)
        return obs, cnt / tot, n, "exact"
    m = 1_000_000
    cnt = 0
    for _ in range(10):
        P = rng.permuted(np.tile(np.arange(n), (m // 10, 1)), axis=1)
        r = (ryc[P] @ rxc) / den
        cnt += int((np.abs(r) >= thr).sum())
    return obs, cnt / m, n, "MC 1e6"


def fmt(rho, p, n, kind):
    if rho != rho:
        return f"rho=   nan  p=  nan  n={n:2d} ({kind})"
    return f"rho={rho:+.3f}  p={p:.4f}  n={n:2d} ({kind})"


def main():
    D = json.load(open(sys.argv[1]))
    spec = importlib.util.spec_from_file_location("adoption_rule", os.path.join(HERE, "adoption_rule.py"))
    AR = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(AR)

    ARMS = ["full", "L_w1000_k1", "L_w10000_k3"]
    per = defaultdict(list)                                  # (arm, ds, sq) -> per-rep dicts
    reps = defaultdict(lambda: {"ate": [], "ok": 0, "att": 0})   # adoption_rule input
    for r in D:
        key = (r["arm"], r["dataset"], r["sequence"])
        reps[key]["att"] += 1
        if r["status"] == "OK" and r["track_success"] == "1" and r["csv"]["ate_sim3_rmse"]:
            reps[key]["ate"].append(r["csv"]["ate_sim3_rmse"])
            reps[key]["ok"] += 1
        row = {"usable": usable(r), "rep": r["rep"], "ate": r["csv"]["ate_sim3_rmse"],
               "scale_s": r["csv"]["scale_s"], "gt_drift": r["csv"]["scale_drift_pct_per_100m"],
               "gt_dist": r["csv"]["gt_distance_m"], "rederived": r.get("drift_rederived"),
               "exact": r.get("matched_all_exact"), "rows_ok": (r.get("est_rows") == (r["kfs_log"] or 0) - 7)}
        if r.get("est_rows"):
            d, s = pa_series(r)
            row["a_local"] = a_local(d, s)
            row["a_prefix"] = a_prefix(d, s)
            d2, s2 = pa_series(r, skip=20)
            row["a_skip20"] = a_local(d2, s2)
            row["s_med"] = float(np.median(s)) if len(s) else np.nan
            row["s_first10"] = float(np.median(s[:10])) if len(s) >= 10 else np.nan
            row["est_dist"] = float(d[-1]) if len(d) else np.nan
            row["n_kf"] = len(s)
        row["gate_med"], row["gate_n"] = gate_stat(r)
        per[key].append(row)

    print("=== T0 sanity")
    diff = [abs(x["rederived"] - x["gt_drift"]) for v in per.values() for x in v
            if x["usable"] and x["rederived"] is not None and x["gt_drift"] is not None]
    print(f"  drift re-derived vs summary.csv: n={len(diff)} max|diff|={max(diff):.3e}")
    print(f"  matched_all_exact false: {sum(1 for v in per.values() for x in v if x['usable'] and not x['exact'])}")
    print(f"  est_rows != kfs-7:        {sum(1 for v in per.values() for x in v if x['usable'] and not x['rows_ok'])}"
          "  (KITTI 02 writes kfs-6; trailing window either way, ordered mapping holds)")
    for arm in ARMS:
        u = sum(1 for k, v in per.items() if k[0] == arm for x in v if x["usable"])
        t = sum(1 for k, v in per.items() if k[0] == arm for x in v)
        print(f"  {arm:12s} usable reps {u}/{t} over {sum(1 for k in per if k[0] == arm)} sequences")

    def med(arm, ds, sq, f):
        v = [x[f] for x in per[(arm, ds, sq)] if x["usable"] and x.get(f) is not None and x[f] == x[f]]
        return float(np.median(v)) if v else np.nan

    SEQS = sorted({(k[1], k[2]) for k in per})
    KITTI = [s for s in SEQS if s[0] == "kitti"]
    TUM = [s for s in SEQS if s[0] == "tum"]
    SETS = {"ALL-19": SEQS, "KITTI-9": KITTI, "TUM-10": TUM}

    def dATE(arm, ds, sq):
        a, b = med("full", ds, sq, "ate"), med(arm, ds, sq, "ate")
        return (b - a) / a

    print("\n=== T1 reproduce DECISIONS.md: rho(|frozen GT drift|, dATE) on KITTI-9  "
          "[recorded w10000: -0.700 p=0.0216; w1000: +0.200]  -- p here is TWO-sided; the record's is one-sided")
    for arm in ("L_w10000_k3", "L_w1000_k1"):
        print(f"  {arm:12s} " + fmt(*spearman([abs(med('full', *s, 'gt_drift')) for s in KITTI],
                                                [dATE(arm, *s) for s in KITTI])))

    for arm in ARMS:
        print(f"\n=== T2 per-sequence values, arm={arm}  (medians over usable reps; drift %/100 m; proxy sign = raw slope of ln s_k)")
        print(f"  {'sequence':36s} {'n':>2s} {'GTdist':>7s} {'ESTdist':>7s} {'scale_s':>7s} {'s_med':>6s} {'s_first10':>9s} "
              f"{'GT_drift':>9s} {'a_local':>9s} {'a_prefix':>9s} {'a_skip20':>9s} {'gate_med':>8s} {'gate_n':>6s}")
        for ds, sq in SEQS:
            n = sum(1 for x in per[(arm, ds, sq)] if x["usable"])
            gn = int(np.median([x["gate_n"] for x in per[(arm, ds, sq)] if x["usable"]] or [0]))
            print(f"  {ds+'/'+sq:36s} {n:2d} {med(arm,ds,sq,'gt_dist'):7.1f} {med(arm,ds,sq,'est_dist'):7.1f} "
                  f"{med(arm,ds,sq,'scale_s'):7.3f} {med(arm,ds,sq,'s_med'):6.3f} {med(arm,ds,sq,'s_first10'):9.3f} "
                  f"{med(arm,ds,sq,'gt_drift'):9.2f} {med(arm,ds,sq,'a_local'):9.2f} {med(arm,ds,sq,'a_prefix'):9.2f} "
                  f"{med(arm,ds,sq,'a_skip20'):9.2f} {med(arm,ds,sq,'gate_med'):8.3f} {gn:6d}")
        print(f"  --- Spearman rho(proxy drift, GT drift) on arm={arm}; 'signed' uses the raw slope (expected NEGATIVE = tracking)")
        for name, S in SETS.items():
            g = [med(arm, *s, "gt_drift") for s in S]
            for px in ("a_local", "a_prefix", "a_skip20"):
                x = [med(arm, *s, px) for s in S]
                print(f"    {name:8s} {px:9s} signed: {fmt(*spearman(x, g))}   |abs|: {fmt(*spearman(np.abs(x), np.abs(g)))}")
            x = [med(arm, *s, "gate_med") for s in S]
            print(f"    {name:8s} {'gate(c)':9s} |abs|:  {fmt(*spearman(x, np.abs(g)))}")
        print(f"  --- level check: rho(median s_k, Sim(3) scale_s) on arm={arm}  (expected NEGATIVE if the prior's own scale is constant)")
        for name, S in SETS.items():
            print(f"    {name:8s} " + fmt(*spearman([med(arm, *s, 's_med') for s in S], [med(arm, *s, 'scale_s') for s in S])))

    print("\n=== T4 gate relevance: rho(|X on full|, dATE(arm vs full))")
    for arm in ("L_w10000_k3", "L_w1000_k1"):
        for name, S in SETS.items():
            y = [dATE(arm, *s) for s in S]
            for px in ("gt_drift", "a_local", "a_prefix", "a_skip20"):
                x = [abs(med("full", *s, px)) for s in S]
                print(f"  {arm:12s} {name:8s} |{px:9s}| " + fmt(*spearman(x, y)))

    print("\n=== T5 within-sequence rep-level rho(a_local, GT drift), arm=full  (negative = tracking)")
    rhos = []
    for ds, sq in SEQS:
        v = [x for x in per[("full", ds, sq)] if x["usable"]]
        rho, p, n, kind = spearman([t["a_local"] for t in v], [t["gt_drift"] for t in v])
        rhos.append(rho)
        print(f"  {ds+'/'+sq:36s} " + fmt(rho, p, n, kind))
    rr = [r for r in rhos if r == r]
    print(f"  median rho = {np.median(rr):+.3f}; negative (tracking) {sum(1 for r in rr if r < 0)}/{len(rr)}")

    # ---- O: sensors read on full, sign aligned to GT/est --------------------------------------
    sensor = defaultdict(list)
    for r in D:
        if r["arm"] == "full" and usable(r) and r.get("est_rows"):
            d, s = pa_series(r)
            gd, gdist, al, nk = r["csv"]["scale_drift_pct_per_100m"], r["csv"]["gt_distance_m"], -a_local(d, s), len(s)
            sensor[(r["dataset"], r["sequence"])].append({
                "gt": gd, "a_local": al, "a_prefix": -a_prefix(d, s),
                "gt_total": gd * gdist / 100.0, "gt_perkf": gd * gdist / nk,
                "a_total": al * d[-1] / 100.0, "a_perkf": al * d[-1] / nk,
                "lvl_first10": abs(np.log(np.median(s[:10]))) if len(s) >= 10 else np.nan,
                "lvl_med": abs(np.log(np.median(s)))})

    def medsens(ds, sq, f):
        v = [x[f] for x in sensor[(ds, sq)] if x[f] == x[f]]
        return float(np.median(v)) if v else float("nan")

    print("\n=== O1 per-sequence: dATE of each arm vs full, with the sensors read on full (sign aligned to GT/est)")
    print(f"  {'sequence':36s} {'GTdrift':>9s} {'|rank|':>6s} {'proxy':>9s} {'|rank|':>6s} {'dATE w1000':>11s} {'dATE w10000':>12s}")
    for ds in ("kitti", "tum"):
        S = [s for s in SEQS if s[0] == ds]
        g = np.array([abs(medsens(*s, "gt")) for s in S]); p = np.array([abs(medsens(*s, "a_local")) for s in S])
        rg = {s: int(r) for s, r in zip(S, (-g).argsort().argsort() + 1)}
        rp = {s: int(r) for s, r in zip(S, (-p).argsort().argsort() + 1)}
        for s in sorted(S, key=lambda s: -abs(medsens(*s, "gt"))):
            print(f"  {s[0]+'/'+s[1]:36s} {medsens(*s,'gt'):9.2f} {rg[s]:6d} {medsens(*s,'a_local'):9.2f} {rp[s]:6d} "
                  f"{dATE('L_w1000_k1',*s)*100:+10.1f}% {dATE('L_w10000_k3',*s)*100:+11.1f}%")
        y = [dATE("L_w10000_k3", *s) for s in S]
        print(f"    rho(|GTdrift|, dATE w10000) {ds}: " + fmt(*spearman(g, y)))
        print(f"    rho(|proxy|,   dATE w10000) {ds}: " + fmt(*spearman(p, y)))

    print("\n=== O2 n=8 check (drop KITTI 01): recorded rho=-0.714 p=0.0288 (one-sided)")
    S8 = [s for s in KITTI if s[1] != "01"]
    print("  " + fmt(*spearman([abs(medsens(*s, "gt")) for s in S8], [dATE("L_w10000_k3", *s) for s in S8])))

    HAZ = [("kitti", "02"), ("kitti", "05")]

    def oracle(sens_name, off_arm, verbose):
        vals = sorted({abs(medsens(*s, sens_name)) for s in SEQS if medsens(*s, sens_name) == medsens(*s, sens_name)})
        taus = [0.0] + [(a + b) / 2 for a, b in zip(vals, vals[1:])] + [vals[-1] + 1]
        seen, both = set(), []
        if verbose:
            print(f"    {'tau':>8s}  {'KITTI':>10s} {'A':>7s} {'B':>5s} {'C1':>28s} | {'TUM':>10s} {'A':>7s} {'B':>5s} {'C1':>28s} | haz 02/05")
        for tau in taus:
            choice = {s: ("L_w10000_k3" if abs(medsens(*s, sens_name)) > tau else off_arm) for s in SEQS}
            sig = tuple(choice[s] for s in SEQS)
            if sig in seen:
                continue
            seen.add(sig)
            out = {}
            for ds in ("kitti", "tum"):
                per_ds = {s[1]: AR.classify(reps[("full",) + s], reps[(choice[s],) + s]) for s in SEQS if s[0] == ds}
                out[ds] = AR.judge(per_ds)
            k, t = out["kitti"], out["tum"]
            if k["verdict"] == "PASS" and t["verdict"] == "PASS":
                both.append(tau)

            def cell(v):
                a = f"{v.get('median_delta', float('nan'))*100:+6.1f}%" if v.get("median_delta") is not None else "   --"
                b = f"{v.get('improved',0)}/{v.get('n_eff',0)}"
                c1 = "ok" if v.get("C1") else ",".join(v.get("c1_fail") or ["?"])[:28]
                return f"{v['verdict']:>10s} {a:>7s} {b:>5s} {c1:>28s}"
            if verbose or k["verdict"] == "PASS" or t["verdict"] == "PASS":
                haz = "/".join("ON" if choice[h] == "L_w10000_k3" else "off" for h in HAZ)
                print(f"    {tau:8.2f}  {cell(k)} | {cell(t)} | {haz}")
        print(f"    taus where BOTH datasets pass: {both if both else 'NONE'}")

    print("\n=== O3 ORACLE per-sequence gate through adoption_rule: X > tau -> L_w10000_k3, else OFF arm; ONE tau for both datasets")
    for sens_name in ("gt", "a_local"):
        for off_arm in ("full", "L_w1000_k1"):
            print(f"\n  sensor=|{sens_name} on full|   ON=L_w10000_k3   OFF={off_arm}")
            oracle(sens_name, off_arm, verbose=True)

    print("\n=== U alternative normalisations (unit-free) and level sensors, OFF=full; only rows where a dataset passes")
    for f in ("gt_total", "gt_perkf", "a_total", "a_perkf", "lvl_first10", "lvl_med"):
        print(f"\n  sensor=|{f}| on full")
        for ds in ("kitti", "tum"):
            S = [s for s in SEQS if s[0] == ds]
            vals = sorted((abs(medsens(*s, f)), s[1]) for s in S)
            print(f"    {ds:5s} " + "  ".join(f"{q}:{v:.1f}" for v, q in vals))
            print(f"    rho(|{f}|, dATE w10000) {ds}: " + fmt(*spearman([abs(medsens(*s, f)) for s in S],
                                                                            [dATE('L_w10000_k3', *s) for s in S])))
        oracle(f, "full", verbose=False)


if __name__ == "__main__":
    main()
