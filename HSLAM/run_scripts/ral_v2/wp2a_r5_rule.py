#!/usr/bin/env python3
"""WP2a-R5: apply the dataset-level adoption rule to a campaign root, exactly as pre-registered.

Rule (DECISIONS.md "WP2a-R5 -- PRE-REGISTERED 2026-09-21"): the candidate arm is adopted as the hygiene
base if it improves the median Sim(3) ATE against the reference arm on >= 70 % of TUM's five sequences
AND >= 70 % of KITTI's five sequences, with (a) no sequence other than freiburg2_large_no_loop regressed
beyond the reference IQR and (b) track success not worse anywhere. EuRoC-3 and mono-VO are reported,
not judged. Escalation: a dataset landing exactly on the boundary whose deciding sequence is inside the
reference IQR escalates that sequence to n=10 (pooled with a `<arm>_esc` directory); at most two
sequences escalate, otherwise the result is NOT DECIDED at n=5.

Usage: wp2a_r5_rule.py --root runs/wp2a_r5_eval-server [--ref K14_K15] [--cand K13_K14_K15]
"""
import argparse
import csv
import glob
import os
import statistics as st

TUM = ["freiburg1_desk", "freiburg1_room", "freiburg2_desk", "freiburg2_large_no_loop",
       "freiburg3_long_office_household"]
KITTI = ["00", "05", "06", "07", "10"]
EUROC = ["MH_01_easy", "V1_01_easy", "V2_02_medium"]
MONO = ["sequence_31"]
DEFECTIVE = {"freiburg2_large_no_loop"}          # named in the pre-registration, evidence predates it
THRESH = 0.70


def iqr(v):
    v = sorted(v)
    n = len(v)
    return v[(3 * n) // 4] - v[n // 4] if n >= 4 else (max(v) - min(v) if v else float("nan"))


def load(root, arm):
    """{sequence: (median_ate, iqr, n_usable, n_attempted, median_scale)} pooling <arm> and <arm>_esc."""
    rows = {}
    for d in (arm, arm + "_esc", arm + "_r0ext"):
        for f in glob.glob(os.path.join(root, d, "summary.csv")):
            with open(f) as fh:
                for r in csv.DictReader(fh):
                    rows.setdefault(r.get("sequence", ""), []).append(r)
    out = {}
    for sq, rs in rows.items():
        # PLAN.md §5 / CLAUDE.md, LOCKED: medians are over FULL-TRACK runs only, and track success is
        # reported separately. A run that loses tracking still exits with status=OK and a SHORT trajectory,
        # whose ATE is small because it covers less of the sequence -- folding those in biases an arm that
        # loses tracking to look better. (Bug found and fixed 21 Sep 16:20, before the R5 verdict: the
        # first version filtered on status alone and reported KITTI 07 as 10/10 with a median that included
        # two partial runs. r0_check.py and wp2a_summary.py always filtered on track_success.)
        ok = [r for r in rs if r.get("status") == "OK" and r.get("track_success") == "1"]
        if not ok:
            out[sq] = (float("nan"), float("nan"), 0, len(rs), float("nan"))
            continue
        ate = [float(r["ate_sim3_rmse"]) for r in ok]
        sc = [float(r["scale_s"]) for r in ok]
        out[sq] = (st.median(ate), iqr(ate), len(ok), len(rs), st.median(sc))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--ref", default="K14_K15")
    ap.add_argument("--cand", default="K13_K14_K15")
    a = ap.parse_args()
    ref, cand = load(a.root, a.ref), load(a.root, a.cand)

    verdict_ok, escalate, other_regression = True, [], []
    print(f"{'sequence':34s} {'ref med (IQR) n':>24s} {'cand med (IQR) n':>24s} {'delta':>9s}  flag")
    for name, seqs in (("TUM", TUM), ("KITTI", KITTI), ("EuRoC (report only)", EUROC),
                       ("mono-VO (report only)", MONO)):
        better, counted = 0, 0
        for sq in seqs:
            if sq not in ref or sq not in cand:
                print(f"{sq:34s} {'-- missing --':>24s}")
                continue
            rm, ri, rn, ra, _ = ref[sq]
            cm, ci, cn, ca, _ = cand[sq]
            counted += 1
            imp = cm < rm
            better += imp
            flag = ""
            if cm > rm + ri:
                flag = "REGRESSED>IQR" + (" (characterised)" if sq in DEFECTIVE else "")
                if sq not in DEFECTIVE and name in ("TUM", "KITTI"):
                    other_regression.append(sq)
            elif abs(cm - rm) < ri:
                flag = "inside IQR"
            if cn / max(1, ca) < rn / max(1, ra):
                flag += " TRACK-WORSE"
                if name in ("TUM", "KITTI"):
                    other_regression.append(sq + " (track)")
            print(f"{sq:34s} {rm:10.3f} ({ri:.3f}) {rn:2d}/{ra:<2d} {cm:10.3f} ({ci:.3f}) {cn:2d}/{ca:<2d} "
                  f"{100*(cm-rm)/rm:+8.1f}%  {flag}")
        if counted:
            rate = better / counted
            verdict = "PASS" if rate >= THRESH else "fail"
            if name in ("TUM", "KITTI"):
                if rate < THRESH:
                    verdict_ok = False
                # Boundary: the dataset sits one sequence either side of the bar. The pre-registration
                # (DECISIONS.md, WP2a-R5) escalates "the sequence THAT DECIDES the threshold", singular --
                # not every sequence that happens to be inside the IQR. Corrected 21 Sep 16:00 before the
                # escalation ran; the earlier code nominated all within-IQR sequences, which is not the text.
                # The deciding sequence is the non-improved one closest to flipping (smallest relative gap),
                # and it only qualifies if it is inside the reference IQR.
                need = int(-(-THRESH * counted // 1))
                if better in (need - 1, need):
                    losers = [sq for sq in seqs if sq in ref and sq in cand and cand[sq][0] >= ref[sq][0]]
                    losers.sort(key=lambda sq: (cand[sq][0] - ref[sq][0]) / max(ref[sq][0], 1e-9))
                    if losers:
                        sq = losers[0]
                        if abs(cand[sq][0] - ref[sq][0]) < ref[sq][1]:
                            escalate.append(sq)
                print(f"  --> {name}: improved {better}/{counted} = {100*rate:.0f}%  {verdict} "
                      f"(rule: >= {100*THRESH:.0f}%)")
            else:
                print(f"  --> {name}: improved {better}/{counted} (reported, not judged)")
        print()
    if other_regression:
        print(f"BLOCKER: regression beyond the reference IQR outside the characterised sequence: "
              f"{sorted(set(other_regression))}")
        verdict_ok = False
    if escalate:
        print(f"Boundary sequences inside the reference IQR (pre-registered escalation to n=10, max 2): "
              f"{sorted(set(escalate))[:2]}"
              + ("  -- MORE THAN TWO QUALIFY => NOT DECIDED at n=5" if len(set(escalate)) > 2 else ""))
    print(f"\nVERDICT (pre-registered): {'ADOPT ' + a.cand if verdict_ok else 'KEEP ' + a.ref}"
          + ("  [pending escalation]" if escalate else ""))


if __name__ == "__main__":
    main()
