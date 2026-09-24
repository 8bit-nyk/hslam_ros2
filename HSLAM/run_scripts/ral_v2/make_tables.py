#!/usr/bin/env python3
"""Numbers freeze: summary.csv -> LaTeX tables and a prose-number ledger.

PLAN.md P8: no number is hand-typed anywhere in the v2 paper. Every table cell and every
percentage in the prose is produced here from summary.csv rows, each of which carries the
commit and the full CLI that produced it.

Three refusals are deliberate and are the point of the script:

  * It REFUSES to pool rows from different commits unless --allow-mixed-commits is given.
    "Never pool arms from different binaries" is a standing project rule; a table that
    silently mixes builds is how a defective-geometry number ends up beside a corrected one.
  * It REFUSES rows with dirty=1 unless --allow-dirty. A row whose tree was dirty cannot be
    reproduced from its commit hash, so it cannot appear in a paper.
  * It drops rows with status != OK or track_success != 1 from the statistics and reports
    them as the success rate instead. KITTI 08/09 appear as failures, never as absences
    (PLAN.md P5).

Usage
    make_tables.py --csv runs/wp6 --out paper/tables
    make_tables.py --csv runs/wp6 runs/wp4 --out paper/tables --allow-mixed-commits
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path
from statistics import median

sys.path.insert(0, str(Path(__file__).resolve().parent))
import datasets as ds            # noqa: E402


def _f(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def load_rows(paths: list[Path]) -> list[dict]:
    rows = []
    for p in paths:
        files = sorted(p.rglob("summary.csv")) if p.is_dir() else [p]
        for f in files:
            with open(f, newline="") as fh:
                for r in csv.DictReader(fh):
                    r["_src"] = str(f)
                    rows.append(r)
    return rows


def iqr(vals: list[float]) -> float:
    if len(vals) < 2:
        return float("nan")
    s = sorted(vals)
    n = len(s)

    def q(p):
        i = p * (n - 1)
        lo, hi = int(math.floor(i)), int(math.ceil(i))
        return s[lo] if lo == hi else s[lo] + (s[hi] - s[lo]) * (i - lo)

    return q(0.75) - q(0.25)


# Mirror of eval_run.COVERAGE_SANITY, re-applied at table time for rows written before P5a.
COVERAGE_SANITY_TABLE = 0.5

AGG_FIELDS = ("ate_sim3_rmse", "ate_se3_rmse", "scale_s", "pipeline_fps", "track_fps",
              "scale_drift_pct_per_100m", "rpe_trans", "rpe_rot",
              "peak_gpu_mb", "peak_cpu_mb", "ml_ms")


def aggregate(rows: list[dict]) -> dict:
    """Median + IQR per (arm, dataset, sequence), over usable reps only."""
    groups: dict[tuple, list[dict]] = defaultdict(list)
    for r in rows:
        groups[(r["arm"], r["dataset"], r["sequence"])].append(r)

    out = {}
    for key, reps in groups.items():
        # P5a: track_success in the CSV may predate the absolute floor (eval_run.py's pooled
        # median is relative to the arm's own reps), so it is re-applied here. This makes the
        # table correct for rows written before 2026-09-24 without re-running anything.
        n_img_k = ds.image_count(key[1], key[2])
        def _passes_floor(r, n=n_img_k):
            fr = _f(r.get("frames"))
            return True if (not n or fr is None) else fr >= COVERAGE_SANITY_TABLE * n
        ok = [r for r in reps if r.get("status") == "OK" and r.get("track_success") == "1"
              and _passes_floor(r)]
        cell = {"n": len(reps), "n_ok": len(ok),
                "success_rate": len(ok) / len(reps) if reps else 0.0}
        for field in AGG_FIELDS:
            vals = [v for v in (_f(r.get(field)) for r in ok) if v is not None]
            cell[field] = median(vals) if vals else float("nan")
            cell[field + "_iqr"] = iqr(vals) if vals else float("nan")
        # P5a: coverage of the sequence, over the reps that scored successful. The denominator
        # is images on disk (datasets.SEQUENCE_IMAGE_COUNTS); None means "cannot judge".
        n_img = ds.image_count(key[1], key[2])
        fr = [v for v in (_f(r.get("frames")) for r in ok) if v is not None]
        cell["frame_coverage"] = (median(fr) / n_img) if (fr and n_img) else float("nan")
        cell["lc_gate_fires_total"] = sum(
            int(r["lc_gate_fires"]) for r in ok if str(r.get("lc_gate_fires", "")).isdigit())
        out[key] = cell
    return out



# --- P5a coverage parity (DECISIONS.md "WP6 BLOCKER 2", 2026-09-24) -----------------------
# eval_run.py's track_success is relative to an arm's OWN reps, so an arm that fails identically
# on every rep scores 100 %. eval_run.py now carries an absolute sanity floor; this is the other
# half -- two arms may each be internally consistent and still not have covered the same ground,
# and an ATE over a prefix is not comparable to one over the whole sequence. The bias is not
# symmetric: a run that dies early accumulates less drift, so the shorter arm is flattered.
#
# Deliberately a PARITY test, not a completeness test. TUM fr1_floor stops at ~68 % of its images
# for every arm; that is a property of the sequence and the pairing there is perfectly fair.
COVERAGE_PARITY = 0.90   # min/max of the arms' median coverage


def coverage_parity(agg: dict, arms: list[str]) -> dict:
    """(dataset, sequence) -> dict(valid, ratio, per-arm coverage). Unjudgeable pairs are valid."""
    out = {}
    for (d, sq) in sorted({(d, sq) for (a, d, sq) in agg if a in arms}):
        cov = {}
        for a in arms:
            c = agg.get((a, d, sq))
            if c and math.isfinite(c.get("frame_coverage", float("nan"))):
                cov[a] = c["frame_coverage"]
        if len(cov) < 2:
            out[(d, sq)] = {"valid": True, "ratio": float("nan"), "cov": cov,
                            "note": "not judgeable (fewer than two measurable arms)"}
            continue
        lo, hi = min(cov.values()), max(cov.values())
        ratio = lo / hi if hi else 0.0
        out[(d, sq)] = {"valid": ratio >= COVERAGE_PARITY, "ratio": ratio, "cov": cov,
                        "note": "" if ratio >= COVERAGE_PARITY else
                                "coverage mismatch -- no paired ATE"}
    return out


def fmt(x, prec=3, dash="--"):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return dash
    return f"{x:.{prec}f}"


def _esc(s: str) -> str:
    return s.replace("_", r"\_")


def latex_main_table(agg: dict, arms: list[str], label: str, caption: str,
                     parity: dict | None = None) -> str:
    seqs = sorted({(d, s) for (a, d, s) in agg if a in arms})
    head = " & ".join(rf"\multicolumn{{3}}{{c}}{{{_esc(a)}}}" for a in arms)
    sub = " & ".join([r"ATE$_{Sim3}$ & ATE$_{SE3}$ & $s$"] * len(arms))
    lines = [
        "% GENERATED by run_scripts/ral_v2/make_tables.py -- do not edit by hand",
        r"\begin{table}[t]\centering",
        rf"\caption{{{caption}}}\label{{{label}}}",
        r"\begin{tabular}{ll" + "rrr" * len(arms) + "}",
        r"\toprule",
        rf"Dataset & Seq. & {head} \\",
        rf" &  & {sub} \\",
        r"\midrule",
    ]
    for d, s in seqs:
        pg = (parity or {}).get((d, s))
        cells = []
        for a in arms:
            c = agg.get((a, d, s))
            if not c or c["n_ok"] == 0:
                cells += [r"\textit{fail}", r"\textit{fail}", "--"]
            elif pg and not pg["valid"]:
                # P5a: the arms did not cover the same ground, so no paired ATE is printed.
                # The coverage itself is the measurement and is what the cell reports.
                cells += [rf"\textit{{{c['frame_coverage']:.0%} cov.}}", r"--", "--"]
            else:
                cells += [fmt(c["ate_sim3_rmse"]), fmt(c["ate_se3_rmse"]), fmt(c["scale_s"])]
        lines.append(f"{d} & {_esc(s)} & " + " & ".join(cells) + r" \\")
    lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}", ""]
    return "\n".join(lines)


def latex_ablation_table(agg: dict, baseline: str, label: str, caption: str) -> str:
    arms = sorted({a for (a, _, _) in agg})
    lines = [
        "% GENERATED by run_scripts/ral_v2/make_tables.py -- do not edit by hand",
        r"\begin{table}[t]\centering",
        rf"\caption{{{caption}}}\label{{{label}}}",
        r"\begin{tabular}{lrrrrr}",
        r"\toprule",
        rf"Arm & med. ATE$_{{Sim3}}$ & IQR & med. $s$ & success & $\Delta$ vs {_esc(baseline)} \\",
        r"\midrule",
    ]
    base = {(d, s): c["ate_sim3_rmse"]
            for (a, d, s), c in agg.items() if a == baseline and c["n_ok"]}

    for a in arms:
        cells = [c for (aa, _, _), c in agg.items() if aa == a and c["n_ok"]]
        if not cells:
            continue
        ate = [c["ate_sim3_rmse"] for c in cells if math.isfinite(c["ate_sim3_rmse"])]
        sc = [c["scale_s"] for c in cells if math.isfinite(c["scale_s"])]
        sr = sum(c["success_rate"] for c in cells) / len(cells)
        # Paired by sequence against the baseline -- never a ratio of pooled means.
        deltas = [(c["ate_sim3_rmse"] / base[(d, s)] - 1) * 100
                  for (aa, d, s), c in agg.items()
                  if aa == a and c["n_ok"] and (d, s) in base and base[(d, s)]
                  and math.isfinite(c["ate_sim3_rmse"])]
        dtxt = rf"{median(deltas):+.1f}\%" if deltas and a != baseline else "--"
        lines.append(
            f"{_esc(a)} & {fmt(median(ate) if ate else None)} & "
            f"{fmt(median([c['ate_sim3_rmse_iqr'] for c in cells]))} & "
            f"{fmt(median(sc) if sc else None)} & {sr * 100:.0f}\\% & {dtxt} \\\\")
    lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}", ""]
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", nargs="+", required=True, help="summary.csv files or dirs to scan")
    ap.add_argument("--out", required=True)
    ap.add_argument("--baseline", default="A0")
    ap.add_argument("--main-arms", nargs="+", default=["A0", "full"])
    ap.add_argument("--allow-mixed-commits", action="store_true")
    ap.add_argument("--allow-dirty", action="store_true")
    a = ap.parse_args()

    rows = load_rows([Path(p) for p in a.csv])
    if not rows:
        print("ERROR: no rows found", file=sys.stderr)
        return 2

    dirty = [r for r in rows if r.get("dirty") == "1"]
    if dirty and not a.allow_dirty:
        print(f"ERROR: {len(dirty)} row(s) came from a dirty working tree and so cannot be\n"
              f"       reproduced from their commit hash. Re-run them, or pass --allow-dirty\n"
              f"       for a draft. Example: {dirty[0]['_src']}", file=sys.stderr)
        return 3
    if not a.allow_dirty:
        rows = [r for r in rows if r.get("dirty") != "1"]

    commits = sorted({r.get("commit", "?") for r in rows})
    if len(commits) > 1 and not a.allow_mixed_commits:
        print(f"ERROR: rows span {len(commits)} commits {commits}. Arms from different binaries\n"
              f"       are not poolable. Re-run at one commit, or pass --allow-mixed-commits if\n"
              f"       you have a specific reason and will state it in the caption.",
              file=sys.stderr)
        return 3

    agg = aggregate(rows)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    tag = commits[0] if len(commits) == 1 else "MIXED:" + ",".join(commits)

    parity = coverage_parity(agg, a.main_arms)
    (out / "T1_main.tex").write_text(latex_main_table(
        agg, a.main_arms, "tab:main",
        rf"Trajectory accuracy. Median over reps; ATE in metres. Commit \texttt{{{tag}}}. "
        rf"Cells marked \textit{{cov.}} are P5a coverage mismatches: the arms did not cover the "
        rf"same span, so no paired ATE is reported.", parity))
    (out / "T3_ablation.tex").write_text(latex_ablation_table(
        agg, a.baseline, "tab:ablation",
        rf"Component ablation, medians across sequences. Commit \texttt{{{tag}}}."))

    # Prose ledger: every number the text may quote, with its provenance.
    (out / "numbers.json").write_text(json.dumps({
        "commits": commits,
        "n_rows": len(rows),
        "generated_from": sorted({r["_src"] for r in rows}),
        "cells": {"|".join(k): v for k, v in agg.items()},
        "coverage_parity": {"|".join(k): v for k, v in parity.items()},
    }, indent=2, default=str))

    print(f"commit(s): {', '.join(commits)}")
    print(f"{len(rows)} rows -> {len(agg)} cells")
    for (arm, d, s), c in sorted(agg.items()):
        print(f"  {arm:14s} {d:8s} {s:28s} n={c['n_ok']}/{c['n']} "
              f"ATE_sim3={fmt(c['ate_sim3_rmse'])} ATE_se3={fmt(c['ate_se3_rmse'])} "
              f"s={fmt(c['scale_s'])} fps={fmt(c['pipeline_fps'], 1)}")
    bad = {k: v for k, v in parity.items() if not v["valid"]}
    print(f"\n[P5a] coverage parity over arms {a.main_arms} "
          f"(bar {COVERAGE_PARITY:.2f}): {len(parity) - len(bad)} valid, {len(bad)} MISMATCH")
    for (d, sq), v in sorted(parity.items()):
        mark = "  " if v["valid"] else "XX"
        cov = "  ".join(f"{arm}={c:.0%}" for arm, c in sorted(v["cov"].items()))
        print(f"  {mark} {d:8s} {sq:30s} ratio={v['ratio']:.2f}  {cov}  {v['note']}")
    print(f"\nwrote {out}/T1_main.tex, T3_ablation.tex, numbers.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
