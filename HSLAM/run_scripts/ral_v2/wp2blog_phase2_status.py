#!/usr/bin/env python3
"""WP2b-log phase ii: interim status after each completed sequence, and the futility check.

Phase ii runs SEQUENCE-MAJOR (every arm on a sequence, then the next sequence) precisely so that a
complete, comparable slice exists at every checkpoint. This prints that slice, and answers the one
question that is allowed to stop an arm early.

WHAT MAY STOP AN ARM: only condition C2 "breakage" of the adoption rule -- candidate track success
below 50 % where the reference is at or above 80 %. That is already the rule's sole outright veto
(DECISIONS.md, "RULE AMENDMENT -- 2026-09-21 (evening)"), so acting on it applies the pre-registered
rule rather than peeking at it.

WHAT MAY NOT: ATE. A candidate that looks bad on the sequences run so far is NOT grounds to stop.
The adoption rule is a statement about a whole dataset and it is computed once, at the end, over every
sequence. Stopping on an interim ATE would be exactly the selective stopping the pre-registration
exists to prevent, and the printed ledger below is for visibility, not for deciding.

Usage:
  wp2blog_phase2_status.py --root runs/wp2ii_eval-server --ref full --cand A B [--futility-seq 07]
Exit codes: 0 normal; 3 a candidate broke on --futility-seq (the runner drops that arm).
"""
import argparse
import csv
import glob
import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import adoption_rule as AR  # noqa: E402  -- the single implementation; never re-derive the rule

TRACK_BREAK_CAND = 0.50
TRACK_OK_REF = 0.80


def load(root, arm):
    """{sequence: [rows]} for one arm."""
    out = {}
    for f in sorted(glob.glob(os.path.join(root, arm, "summary.csv"))):
        with open(f) as fh:
            for r in csv.DictReader(fh):
                out.setdefault(r["sequence"], []).append(r)
    return out


def full_track(rows):
    return [r for r in rows if r.get("status") == "OK" and r.get("track_success") == "1"]


def cell(rows):
    ok = full_track(rows)
    if not ok:
        return None
    a = [float(r["ate_sim3_rmse"]) for r in ok]
    s = [float(r["scale_s"]) for r in ok]
    return dict(n_ok=len(ok), n=len(rows), ate=st.median(a), scale=st.median(s),
                rate=len(ok) / len(rows))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--ref", default="full")
    ap.add_argument("--cand", nargs="+", required=True)
    ap.add_argument("--futility-seq", default=None,
                    help="if given, exit 3 when a candidate BREAKS on this sequence (C2)")
    a = ap.parse_args()

    ref = load(a.root, a.ref)
    cands = {c: load(a.root, c) for c in a.cand}
    seqs = [s for s in ref if all(s in c for c in cands.values())]
    if not seqs:
        print("[status] no sequence complete across all arms yet")
        return 0

    print(f"\n{'':28s}" + "".join(f"{x:>26s}" for x in [a.ref] + a.cand))
    print(f"{'sequence':28s}" + "".join(f"{'ATE (n/N)  s':>26s}" for _ in [a.ref] + a.cand))
    for s in seqs:
        line = f"{s:28s}"
        for arm in [a.ref] + a.cand:
            src = ref if arm == a.ref else cands[arm]
            c = cell(src[s])
            line += (f"{c['ate']:12.3f} ({c['n_ok']}/{c['n']}) {c['scale']:6.3f}"
                     if c else f"{'-- no full-track --':>26s}")
        print(line)

    # interim adoption-rule ledger, clearly labelled as NOT a decision
    print("\n[interim ledger -- visibility only; the verdict is computed once, over the whole set]")
    for c in a.cand:
        for ds in ("tum", "kitti"):
            try:
                v = AR.judge_dirs(os.path.join(a.root, a.ref), os.path.join(a.root, c), dataset=ds) \
                    if hasattr(AR, "judge_dirs") else None
            except Exception:
                v = None
            if v is None:
                print(f"  {c:14s} {ds:6s} (run adoption_rule.py --ref {a.root}/{a.ref} "
                      f"--cand {a.root}/{c} --dataset {ds} for the ledger)")
                break

    # ---- the only early stop that is allowed ----
    if a.futility_seq:
        fs = a.futility_seq
        rc = 0
        if fs in ref:
            rrate = len(full_track(ref[fs])) / max(len(ref[fs]), 1)
            for c in a.cand:
                if fs not in cands[c]:
                    continue
                crate = len(full_track(cands[c][fs])) / max(len(cands[c][fs]), 1)
                if crate < TRACK_BREAK_CAND and rrate >= TRACK_OK_REF:
                    print(f"\n!!! C2 BREAKAGE on {fs}: {c} track {crate:.0%} vs reference "
                          f"{rrate:.0%} -- this arm is DROPPED (adoption rule, condition C2).")
                    rc = 3
                else:
                    print(f"[futility] {fs}: {c} track {crate:.0%} vs ref {rrate:.0%} -- continue")
        return rc
    return 0


if __name__ == "__main__":
    sys.exit(main())
