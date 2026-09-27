#!/usr/bin/env python3
"""Re-derive the DV7 prior-relative-drift columns from the run logs of finished campaigns.

eval_run.py now writes pa_kf_n, pa_slope, pa_range, pa_snap and pa_collapse_n into every new summary.csv
row. Rows written before that have the [PRIOR_ALIGN] lines in their run.log but not the columns; this
script re-reads each row's log with the SAME parser and the SAME function (imported from eval_run, never
re-derived) and writes one output row per summary.csv row. It adds four descriptive columns computed over
the keyframes with n >= 50 projected points: max |ln s| and the number of keyframes with |ln s| above
0.5 / 0.7 / 1.0. Read-only on its inputs: summary.csv, run.log and runs/ are never written.

Layout: <root>/<arm>/summary.csv and <root>/<arm>/<dataset>_<sequence>_rep<rep>/run.log.

Usage: wp4_prior_align_cols.py --root runs/prewp4_s2_eval-server [--root ...] --out dv7.csv [--sweep]
  --sweep  also print, per (arm, dataset, sequence): reps, the max of pa_max_abs_ln_s over reps and the sum of
           pa_n_gt_0p7 over reps, one line each, sorted (root-prefixed when more than one root is given).
"""
import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eval_run import parse_prior_align, prior_align_dv7      # noqa: E402

N_MIN = 50                                   # the gate's own support floor (FullSystem::priorAlignmentGate)
THRESH = (("pa_n_gt_0p5", 0.5), ("pa_n_gt_0p7", 0.7), ("pa_n_gt_1p0", 1.0))
DV7 = ["pa_kf_n", "pa_slope", "pa_range", "pa_snap", "pa_collapse_n"]
OUT_COLUMNS = (["root", "arm", "dataset", "sequence", "rep", "status", "track_success"] + DV7
               + ["pa_max_abs_ln_s"] + [k for k, _ in THRESH])


def log_columns(log: Path) -> dict:
    """DV7 plus the n >= 50 exceedance columns for one run.log; all nan/0 when the log is missing."""
    nan = float("nan")
    if not log.is_file():
        print(f"[WARN] missing run.log: {log}", file=sys.stderr)
        return {**{k: nan for k in DV7 + ["pa_max_abs_ln_s"]}, **{k: nan for k, _ in THRESH}}
    series = [e for e in parse_prior_align(log.read_text(errors="replace")) if e[1] > 0]
    out = prior_align_dv7(series)
    a = [abs(math.log(s)) for _, s, n, _ in series if n == n and n >= N_MIN]
    out["pa_max_abs_ln_s"] = max(a) if a else nan
    for k, thr in THRESH:
        out[k] = sum(v > thr for v in a)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", action="append", required=True, help="campaign root (repeatable)")
    ap.add_argument("--out", required=True, help="output CSV")
    ap.add_argument("--sweep", action="store_true", help="print the per-(arm, dataset, sequence) sweep")
    a = ap.parse_args()

    rows = []
    for root in a.root:
        summaries = sorted(Path(root).glob("*/summary.csv"))
        if not summaries:
            print(f"[WARN] no <arm>/summary.csv under {root}", file=sys.stderr)
        for sp in summaries:
            with open(sp, newline="") as f:
                srows = list(csv.DictReader(f))
            seen = set()
            for r in srows:
                key = (r["arm"], r["dataset"], r["sequence"], r["rep"])
                if key in seen:     # eval_run appends unconditionally; the log on disk is the last run's
                    print(f"[WARN] duplicate row {key} in {sp}; its run.log belongs to the last one",
                          file=sys.stderr)
                seen.add(key)
                log = sp.parent / f"{r['dataset']}_{r['sequence']}_rep{r['rep']}" / "run.log"
                rows.append({"root": root, "arm": r["arm"], "dataset": r["dataset"],
                             "sequence": r["sequence"], "rep": r["rep"], "status": r["status"],
                             "track_success": r["track_success"], **log_columns(log)})

    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=OUT_COLUMNS)
        w.writeheader()
        w.writerows(rows)
    print(f"[wp4_prior_align_cols] {len(rows)} rows -> {out}", file=sys.stderr)

    if a.sweep:
        multi = len(a.root) > 1
        groups = defaultdict(list)
        for r in rows:
            groups[(r["root"], r["arm"], r["dataset"], r["sequence"])].append(r)

        def seq_key(k):
            return (k[0], k[1], k[2], int(k[3]) if k[3].isdigit() else 10**6, k[3])

        print(f"# {'root ' if multi else ''}arm dataset sequence  reps  max(pa_max_abs_ln_s)  "
              f"sum(pa_n_gt_0p7)   [keyframes with n >= {N_MIN}]")
        for k in sorted(groups, key=seq_key):
            g = groups[k]
            m = [r["pa_max_abs_ln_s"] for r in g if r["pa_max_abs_ln_s"] == r["pa_max_abs_ln_s"]]
            s07 = sum(r["pa_n_gt_0p7"] for r in g if r["pa_n_gt_0p7"] == r["pa_n_gt_0p7"])
            head = (f"{k[0]} " if multi else "") + f"{k[1]:16s} {k[2]:6s} {k[3]:32s}"
            print(f"{head} reps={len(g):2d}  max_abs_ln_s={max(m) if m else float('nan'):.3f}  "
                  f"sum_n_gt_0p7={s07}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
