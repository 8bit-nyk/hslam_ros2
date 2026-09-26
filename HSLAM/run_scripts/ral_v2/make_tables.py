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
              "peak_gpu_mb", "peak_cpu_mb", "ml_ms",
              # pre-WP4 (2026-09-25); NaN for rows written before these columns existed
              "postinit_fps", "founding_offset_log", "drift_local_pct_per_100m",
              "drift_postfound_pct_path", "rpe_trans_dist", "rpe_rot_dist", "init_resets")


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


# Pre-WP4 (audit V8, 2026-09-25): a sequence enters an arm's ablation row only as a PAIR with the
# baseline, and only if both cells have at least MIN_REPS usable reps and pass P5a coverage parity.
# The old builder took each arm's medians over whatever sequences it survived and averaged success
# over the cells it did not fail outright, so an arm that failed the hard sequences was flattered.
MIN_REPS = 3


def ablation_pairs(agg: dict, baseline: str, arm: str, min_reps: int = MIN_REPS) -> dict:
    """(dataset, sequence) -> {"state": paired|fail|reps|coverage|unrun, "delta": ATE ratio - 1}.

    Every sequence either arm was run on appears, so exclusions are counted, never silent.
    fail = the arm has no usable rep where the baseline does; reps = either side under min_reps;
    coverage = P5a parity fails between the two; unrun = one side has no rows at all.
    """
    out = {}
    for (d, s) in sorted({(d, s) for (a, d, s) in agg if a in (arm, baseline)}):
        ca, cb = agg.get((arm, d, s)), agg.get((baseline, d, s))
        if ca is None or cb is None:
            out[(d, s)] = {"state": "unrun"}
        elif ca["n_ok"] == 0 and cb["n_ok"] >= min_reps:
            out[(d, s)] = {"state": "fail"}
        elif ca["n_ok"] < min_reps or cb["n_ok"] < min_reps:
            out[(d, s)] = {"state": "reps"}
        elif not coverage_parity(agg, [arm, baseline])[(d, s)]["valid"]:
            out[(d, s)] = {"state": "coverage"}
        elif not (math.isfinite(ca["ate_sim3_rmse"]) and math.isfinite(cb["ate_sim3_rmse"])
                  and cb["ate_sim3_rmse"] > 0):
            out[(d, s)] = {"state": "reps"}
        else:
            out[(d, s)] = {"state": "paired", "delta": ca["ate_sim3_rmse"] / cb["ate_sim3_rmse"] - 1}
    return out


def latex_ablation_table(agg: dict, baseline: str, label: str, caption: str,
                         min_reps: int = MIN_REPS) -> str:
    arms = sorted({a for (a, _, _) in agg if a != baseline})
    lines = [
        "% GENERATED by run_scripts/ral_v2/make_tables.py -- do not edit by hand",
        r"\begin{table}[t]\centering",
        rf"\caption{{{caption}}}\label{{{label}}}",
        r"\begin{tabular}{lrrrrrr}",
        r"\toprule",
        rf"Arm & paired & excl. & success & med. $|\ln s|$ & med. f.o. & med. $\Delta$ATE vs {_esc(baseline)} \\",
        r"\midrule",
    ]
    for a in arms:
        pairs = ablation_pairs(agg, baseline, a, min_reps)
        paired = [k for k, v in pairs.items() if v["state"] == "paired"]
        excl = [k for k, v in pairs.items() if v["state"] in ("fail", "reps", "coverage")]
        cells = [c for (aa, _, _), c in agg.items() if aa == a]
        if not cells:
            continue
        # Success over EVERY sequence the arm was run on, the total failures included.
        sr = sum(c["success_rate"] for c in cells) / len(cells)
        lns = [abs(math.log(agg[(a, d, s)]["scale_s"])) for (d, s) in paired
               if math.isfinite(agg[(a, d, s)]["scale_s"]) and agg[(a, d, s)]["scale_s"] > 0]
        fo = [agg[(a, d, s)]["founding_offset_log"] for (d, s) in paired
              if math.isfinite(agg[(a, d, s)]["founding_offset_log"])]
        deltas = [pairs[k]["delta"] * 100 for k in paired]
        lines.append(
            f"{_esc(a)} & {len(paired)} & {len(excl)} & {sr * 100:.0f}\\% & "
            f"{fmt(median(lns) if lns else None)} & {fmt(median(fo) if fo else None)} & "
            f"{(rf'{median(deltas):+.1f}\%' if deltas else '--')} \\\\")
    lines += [r"\bottomrule", r"\end{tabular}",
              rf"\par\footnotesize{{Paired = sequences where arm and baseline both have $\geq${min_reps} usable reps "
              r"and pass P5a coverage parity; excl.\ = sequences dropped for failure, too few reps or a "
              r"coverage mismatch. Deltas are medians of per-sequence ratios, never ratios of pooled means.}",
              r"\end{table}", ""]
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", nargs="+", required=True, help="summary.csv files or dirs to scan")
    ap.add_argument("--out", required=True)
    ap.add_argument("--baseline", default="A0")
    ap.add_argument("--main-arms", nargs="+", default=["A0", "full"])
    ap.add_argument("--min-reps", type=int, default=MIN_REPS,
                    help="usable reps both cells need before a sequence is paired in T3")
    ap.add_argument("--allow-mixed-commits", action="store_true")
    ap.add_argument("--require-binary", metavar="HASH16",
                    help="refuse any row whose `binary` column differs (WP4: pooling across commits is "
                         "licensed by one binary hash, so the hash is checked, not asserted)")
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

    if a.require_binary:
        other = [r for r in rows if r.get("binary") != a.require_binary]
        if other:
            print(f"ERROR: {len(other)} row(s) were not produced by binary {a.require_binary}, e.g.\n"
                  f"       {other[0]['_src']} (binary {other[0].get('binary') or 'none'}). They are not\n"
                  f"       poolable with the rest.", file=sys.stderr)
            return 3

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
        rf"Component ablation, medians across paired sequences. Commit \texttt{{{tag}}}.",
        a.min_reps))

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
