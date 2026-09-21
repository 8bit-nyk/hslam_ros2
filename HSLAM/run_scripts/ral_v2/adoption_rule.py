#!/usr/bin/env python3
"""The dataset-level adoption rule, v2 -- ONE canonical implementation.

Every campaign evaluator that judges adoption imports this module. It exists because both of the
21 Sep 2026 evaluator bugs were ad-hoc reimplementations of a binding text drifting away from it
(DECISIONS.md, "WP2a-R5 -- the n=5 reading ..." and "... a second bug in my evaluator").

Rule text: DECISIONS.md, "RULE AMENDMENT -- 2026-09-21 (evening)".
Calibration that fixes every constant: docs/ral_v2_resubmission/wp/RULE_CALIBRATION.md,
reproduced by run_scripts/ral_v2/rule_calibration.py.

Stdlib only on purpose: the eval server's system python3 has no scipy, and this must run there.
The exact Mann-Whitney and Fisher tests below are verified against scipy in rule_calibration.py
(--selftest).
"""
import csv, math, os, statistics as st
from collections import defaultdict

# ---- constants, each with the calibration line that fixes it -------------------------------------
P_MATERIAL      = 0.20   # Mann-Whitney two-sided p below which an arm-vs-arm difference is "material".
                         # RULE_CALIBRATION.md Table C: at n=10 on KITTI this gives null 1.0 %, power 93.7 %.
P_TRACK         = 0.10   # Fisher two-sided p below which a track-success drop is "established".
                         # Below this threshold a drop is non-evidence and does not reclassify the sequence.
A_MEDIAN_MAX    = -0.05  # Condition A: dataset median delta must be <= this.
                         # Table E: -5 % and -10 % are within 2 points of each other on power; -15 % craters it.
C1_REGRESSION   = 0.25   # Condition C1: an engineering veto, NOT a statistical gate.
                         # Table E: +15/+25/+50/none move null adoption by <1 point. Chosen on judgement.
BREAKAGE_CAND   = 0.50   # C2 breakage band: candidate track-success below this ...
BREAKAGE_REF    = 0.80   # ... while the reference is at or above this -> dataset disqualified outright.
N_EFF_THIN      = 7      # below this many materially-changed sequences the verdict is THIN EVIDENCE
N_EFF_MIN       = 3      # below this the dataset is NOT DECIDED
MIN_REPS        = 5      # fewer usable reps than this on either arm -> the sequence is UNRESOLVED

# ---- exact tests, stdlib ------------------------------------------------------------------------
from functools import lru_cache

@lru_cache(maxsize=None)
def _u_ways(i, j, u):
    """Number of rank arrangements of i vs j items giving Mann-Whitney U == u (exact null, no ties).

    Standard recurrence: the last item in the merged ordering comes from sample 1 (adding j to U)
    or from sample 2 (adding nothing).
    """
    if u < 0:
        return 0
    if i == 0 or j == 0:
        return 1 if u == 0 else 0
    return _u_ways(i - 1, j, u - j) + _u_ways(i, j - 1, u)

def _u_counts(n1, n2):
    return [_u_ways(n1, n2, u) for u in range(n1 * n2 + 1)]

def mannwhitney_p(a, b):
    """Two-sided Mann-Whitney U p-value. Exact when there are no ties and n1*n2 is small."""
    n1, n2 = len(a), len(b)
    if n1 == 0 or n2 == 0:
        return 1.0
    merged = sorted([(v, 0) for v in a] + [(v, 1) for v in b])
    ties = len(merged) != len({v for v, _ in merged})
    ranks, i = {}, 0
    while i < len(merged):
        j = i
        while j + 1 < len(merged) and merged[j + 1][0] == merged[i][0]:
            j += 1
        r = (i + j) / 2 + 1
        for k in range(i, j + 1):
            ranks[k] = r
        i = j + 1
    r1 = sum(ranks[k] for k in range(len(merged)) if merged[k][1] == 0)
    u1 = r1 - n1 * (n1 + 1) / 2
    u2 = n1 * n2 - u1
    if not ties and n1 * n2 <= 400:
        counts = _u_counts(n1, n2)
        total = sum(counts)
        u = int(round(u1))
        cdf = sum(counts[: u + 1]) / total
        sf = sum(counts[u:]) / total
        return min(1.0, 2.0 * min(cdf, sf))
    mu = n1 * n2 / 2.0
    n = n1 + n2
    tie_term = 0.0
    seen = defaultdict(int)
    for v, _ in merged:
        seen[v] += 1
    for c in seen.values():
        tie_term += c ** 3 - c
    var = n1 * n2 / 12.0 * ((n + 1) - tie_term / (n * (n - 1))) if n > 1 else 0.0
    if var <= 0:
        return 1.0
    z = (abs(u1 - mu) - 0.5) / math.sqrt(var)
    return min(1.0, math.erfc(max(z, 0.0) / math.sqrt(2)))

def fisher_p(a, b, c, d):
    """Two-sided Fisher exact on [[a,b],[c,d]]."""
    n = a + b + c + d
    if n == 0:
        return 1.0
    def hyp(x):
        return math.comb(a + b, x) * math.comb(c + d, (a + c) - x) / math.comb(n, a + c)
    obs = hyp(a)
    lo = max(0, (a + c) - (c + d))
    hi = min(a + b, a + c)
    return min(1.0, sum(hyp(x) for x in range(lo, hi + 1) if hyp(x) <= obs * (1 + 1e-9)))

# ---- row loading: the locked protocol -----------------------------------------------------------
def load_reps(paths):
    """{(dataset, sequence): {"ate": [...full-track only...], "ok": n, "att": n}}.

    PLAN.md section 5 / HSLAM/CLAUDE.md: medians over FULL-TRACK runs only; track success separate.
    A run that loses tracking still exits status=OK with a short trajectory and a flatteringly small
    ATE -- filtering on status alone biases whichever arm loses tracking. This is the 16:20 bug.
    """
    out = defaultdict(lambda: {"ate": [], "ok": 0, "att": 0})
    for p in paths:
        if not os.path.exists(p):
            continue
        with open(p) as fh:
            for r in csv.DictReader(fh):
                key = (r["dataset"], r["sequence"])
                out[key]["att"] += 1
                if r.get("status") != "OK" or r.get("track_success") != "1":
                    continue
                try:
                    v = float(r["ate_sim3_rmse"])
                except (KeyError, ValueError):
                    continue
                if v > 0:
                    out[key]["ate"].append(v)
                    out[key]["ok"] += 1
    return out

def iqr(v):
    s = sorted(v)
    n = len(s)
    if n < 2:
        return 0.0
    def q(p):
        i = (n - 1) * p
        lo = int(i)
        hi = min(lo + 1, n - 1)
        return s[lo] + (s[hi] - s[lo]) * (i - lo)
    return q(0.75) - q(0.25)

# ---- the rule -----------------------------------------------------------------------------------
def classify(ref, cand, exempt=False):
    """One sequence -> dict with delta, class and the evidence behind it."""
    r, c = ref["ate"], cand["ate"]
    out = {"n_ref": len(r), "n_cand": len(c), "exempt": exempt,
           "track_ref": (ref["ok"], ref["att"]), "track_cand": (cand["ok"], cand["att"]),
           "delta": float("nan"), "p": float("nan"), "p_track": float("nan"),
           "cls": "unresolved", "note": "", "breakage": False}
    rr = ref["ok"] / ref["att"] if ref["att"] else 0.0
    cr = cand["ok"] / cand["att"] if cand["att"] else 0.0
    if cr < BREAKAGE_CAND <= 1.0 and rr >= BREAKAGE_REF:
        out["breakage"] = True
        out["cls"] = "breakage"
        out["note"] = f"track {cand['ok']}/{cand['att']} vs ref {ref['ok']}/{ref['att']}"
        return out
    if len(r) < MIN_REPS or len(c) < MIN_REPS:
        out["note"] = f"fewer than {MIN_REPS} usable reps (ref {len(r)}, cand {len(c)})"
        return out
    mr, mc = st.median(r), st.median(c)
    out["delta"] = (mc - mr) / mr
    out["p"] = mannwhitney_p(r, c)
    out["iqr_ref"] = iqr(r)
    if cand["ok"] < ref["ok"] or cr < rr:
        out["p_track"] = fisher_p(cand["ok"], cand["att"] - cand["ok"], ref["ok"], ref["att"] - ref["ok"])
    if exempt:
        out["cls"] = "exempt"
        return out
    # an ESTABLISHED track-success drop reclassifies the sequence as regressed whatever the ATE says
    if out["p_track"] == out["p_track"] and out["p_track"] < P_TRACK:
        out["cls"] = "regressed"
        out["note"] = f"established track drop (Fisher p={out['p_track']:.3f})"
        return out
    if out["p"] < P_MATERIAL:
        out["cls"] = "improved" if out["delta"] < 0 else "regressed"
    else:
        out["cls"] = "tie"
        out["note"] = f"not material (MWU p={out['p']:.3f})"
    # an UNESTABLISHED track drop is non-evidence: it is reported, it does not reclassify.
    if out["p_track"] == out["p_track"] and out["p_track"] >= P_TRACK:
        out["note"] = (out["note"] + "; " if out["note"] else "") + \
            f"track {cand['ok']}/{cand['att']} vs {ref['ok']}/{ref['att']}, not established (p={out['p_track']:.3f})"
    return out

def judge(per_seq):
    """per_seq: {sequence: classify(...)} for ONE dataset -> verdict dict."""
    if any(s["breakage"] for s in per_seq.values()):
        return {"verdict": "FAIL", "reason": "C2 breakage", "thin": False,
                "n_eff": 0, "improved": 0, "regressed": 0, "A": None, "C1": None, "B": None}
    deltas = [s["delta"] for s in per_seq.values() if s["delta"] == s["delta"]]
    med = st.median(deltas) if deltas else float("nan")
    cond_a = med <= A_MEDIAN_MAX
    c1_fail = [k for k, s in per_seq.items()
               if not s["exempt"] and s["cls"] == "regressed" and s["delta"] > C1_REGRESSION]
    cond_c1 = not c1_fail
    imp = sum(1 for s in per_seq.values() if s["cls"] == "improved")
    reg = sum(1 for s in per_seq.values() if s["cls"] == "regressed")
    n_eff = imp + reg
    thin = False
    if n_eff < N_EFF_MIN:
        return {"verdict": "NOT DECIDED", "reason": f"only {n_eff} materially-changed sequences "
                f"(need {N_EFF_MIN}); add sequences or reps, do not lower the bar", "thin": False,
                "n_eff": n_eff, "improved": imp, "regressed": reg, "median_delta": med,
                "A": cond_a, "C1": cond_c1, "B": None, "c1_fail": c1_fail}
    if n_eff >= N_EFF_THIN:
        need = math.ceil(0.7 * n_eff)
        cond_b = imp >= need
    else:
        need = reg + 1
        cond_b = imp > reg
        thin = True
    ok = cond_a and cond_b and cond_c1
    return {"verdict": ("PASS" if ok else "FAIL"), "thin": thin and ok, "n_eff": n_eff,
            "improved": imp, "regressed": reg, "need": need, "median_delta": med,
            "A": cond_a, "B": cond_b, "C1": cond_c1, "c1_fail": c1_fail,
            "reason": "" if ok else "; ".join(
                ([] if cond_a else [f"A: median delta {med*100:+.1f}% > {A_MEDIAN_MAX*100:+.0f}%"]) +
                ([] if cond_b else [f"B: {imp}/{n_eff} improved, need {need}"]) +
                ([] if cond_c1 else [f"C1: {', '.join(c1_fail)} regressed beyond +{C1_REGRESSION*100:.0f}%"]))}

def print_ledger(name, per_seq, v):
    """Every sequence, its number, its class and WHY -- including the ones the rule dropped."""
    print(f"\n=== {name} ===")
    print(f"  {'sequence':36} {'delta':>9} {'MWU p':>7} {'class':>10}  note")
    for sq in sorted(per_seq):
        s = per_seq[sq]
        d = f"{s['delta']*100:+8.1f}%" if s["delta"] == s["delta"] else "       --"
        p = f"{s['p']:7.3f}" if s["p"] == s["p"] else "     --"
        print(f"  {sq:36} {d} {p} {s['cls']:>10}  {s['note']}")
    md = v.get("median_delta", float("nan"))
    print(f"  -- median delta over ALL sequences (exempt included): {md*100:+.1f}%"
          f"   A(<= {A_MEDIAN_MAX*100:+.0f}%): {'pass' if v.get('A') else 'FAIL'}")
    if v.get("B") is not None:
        print(f"  -- breadth: {v['improved']} improved / {v['regressed']} regressed, "
              f"N_eff={v['n_eff']}, need {v.get('need')}: {'pass' if v['B'] else 'FAIL'}")
    print(f"  -- C1(no non-exempt regression beyond +{C1_REGRESSION*100:.0f}%): "
          f"{'pass' if v.get('C1') else 'FAIL ' + ','.join(v.get('c1_fail') or [])}")
    tag = "  [THIN EVIDENCE -- may not by itself decide anything that ships]" if v.get("thin") else ""
    print(f"  ==> {v['verdict']}{tag}" + (f"  ({v['reason']})" if v.get("reason") else ""))

# ---- CLI --------------------------------------------------------------------------------------
def evaluate(ref_csvs, cand_csvs, exempt=(), datasets=None):
    ref, cand = load_reps(ref_csvs), load_reps(cand_csvs)
    by_ds = defaultdict(dict)
    for (ds, sq) in sorted(set(ref) & set(cand)):
        if datasets and ds not in datasets:
            continue
        by_ds[ds][sq] = classify(ref[(ds, sq)], cand[(ds, sq)], exempt=(sq in exempt))
    return {ds: (per, judge(per)) for ds, per in by_ds.items()}

if __name__ == "__main__":
    import argparse, glob as _glob
    ap = argparse.ArgumentParser(description="Dataset-level adoption rule v2 (DECISIONS.md, "
                                             "RULE AMENDMENT 2026-09-21 evening)")
    ap.add_argument("--ref", required=True, help="reference arm run dir or summary.csv (globs ok)")
    ap.add_argument("--cand", required=True, help="candidate arm run dir or summary.csv (globs ok)")
    ap.add_argument("--exempt", nargs="*", default=[],
                    help="sequences exempt from condition B; must be named BEFORE the first run")
    ap.add_argument("--dataset", nargs="*", default=None)
    a = ap.parse_args()
    def expand(spec):
        out = []
        for s in spec if isinstance(spec, list) else [spec]:
            for p in _glob.glob(s):
                out.append(p if p.endswith(".csv") else os.path.join(p, "summary.csv"))
            if not _glob.glob(s):
                out.append(s if s.endswith(".csv") else os.path.join(s, "summary.csv"))
        return out
    res = evaluate(expand(a.ref), expand(a.cand), exempt=set(a.exempt), datasets=a.dataset)
    if a.exempt:
        print(f"exempt from condition B (must predate the run and be pre-registered): {', '.join(a.exempt)}")
    for ds, (per, v) in sorted(res.items()):
        print_ledger(ds, per, v)
    thin = [d for d, (_, v) in res.items() if v.get("thin")]
    passed = [d for d, (_, v) in res.items() if v["verdict"] == "PASS"]
    print(f"\n==== datasets passing: {len(passed)}/{len(res)} "
          f"({', '.join(sorted(passed)) or 'none'}) ====")
    if len(res):
        print("generalisability signal: " +
              ("POSITIVE (more than half)" if len(passed) * 2 > len(res) else "not positive"))
    if thin:
        print(f"THIN EVIDENCE on: {', '.join(sorted(thin))} -- may not by itself decide anything "
              f"that ships; re-run on the full sequence set.")
