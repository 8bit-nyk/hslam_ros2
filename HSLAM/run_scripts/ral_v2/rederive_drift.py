#!/usr/bin/env python3
"""Zero-run re-derivation of the drift numbers with the windowed estimator (pre-WP4 stage 0, 0.3).

Every drift number recorded before 2026-09-25 is the PREFIX-fit estimator, which mostly measures the
founding swing (AUDIT_pre_WP4_2026-09-24.md §3.1). This re-reads existing trajectories -- no SLAM run --
through traj_eval._windowed_drift, the estimator eval_run.py now writes as columns, and prints:

  1. the WP6 drift columns: per sequence and arm, medians over usable reps of local drift (%/100 m and
     over the sequence's own path), post-founding drift over the path, and the founding offset;
  2. the F5 data (V2_PAPER_DIRECTION.md §4): per KITTI sequence, the frozen config's founding offset
     against dATE of each b2 weight vs that config, with Spearman rho and an exact permutation p;
     --fig writes the scatter.

Usable rep = status OK and track_success 1 and the P5a sanity floor (re-applied here, as make_tables.py
does, because rows before 2026-09-24 predate it). dATE comes from adoption_rule.classify -- the one
implementation -- so it matches the recorded verdict numbers.

Usage (eval-server, from HSLAM/):
  ~/Dev/evo/evo_env/bin/python run_scripts/ral_v2/rederive_drift.py \
      --arm full=runs/wp2ii_eval-server/full --arm A0=runs/wp6_eval-server/A0 \
      --f5-ref full --f5-cand L_w10000_k3=runs/wp2ii_eval-server/L_w10000_k3 \
      --f5-cand L_w1000_k1=runs/wp2ii_eval-server/L_w1000_k1 --json out.json --fig f5.png
"""
from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import statistics as st
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import adoption_rule as AR       # noqa: E402
import datasets as ds            # noqa: E402
import traj_eval                 # noqa: E402
from evo.core import sync        # noqa: E402

HSLAM_ROOT = Path(__file__).resolve().parents[2]
KEYS = ("drift_local_pct_per_100m", "drift_local_pct_path", "drift_postfound_pct_path",
        "founding_offset_log")


def usable(r: dict) -> bool:
    if r.get("status") != "OK" or r.get("track_success") != "1":
        return False
    n = ds.image_count(r["dataset"], r["sequence"])
    try:
        fr = float(r.get("frames") or "nan")
    except ValueError:
        fr = float("nan")
    return True if (not n or fr != fr) else fr >= 0.5 * n


def per_arm(arm_dir: Path) -> dict:
    """(dataset, sequence) -> list of per-rep windowed-drift dicts."""
    out = defaultdict(list)
    for r in csv.DictReader(open(arm_dir / "summary.csv")):
        if not usable(r):
            continue
        tp = arm_dir / f"{r['dataset']}_{r['sequence']}_rep{r['rep']}" / "result.txt"
        if not tp.exists():
            continue
        spec = ds.resolve(r["dataset"], r["sequence"])
        try:
            gt = traj_eval.load_gt(spec.gt, spec.gt_format, spec.extrinsics)
            est = traj_eval._read_tum_lenient(tp)
            ref_s, est_s = sync.associate_trajectories(gt, est, max_diff=0.02)
        except Exception as exc:                                  # noqa: BLE001 - reported
            print(f"  skip {r['dataset']}/{r['sequence']}/rep{r['rep']}: {exc}", file=sys.stderr)
            continue
        out[(r["dataset"], r["sequence"])].append(traj_eval._windowed_drift(ref_s, est_s))
    return out


def med(v):
    v = [x for x in v if x == x]
    return st.median(v) if v else float("nan")


def spearman_exact(x, y):
    """rho and exact one-sided (rho <= observed) and two-sided permutation p."""
    rx = np.argsort(np.argsort(x)).astype(float)
    ry = np.argsort(np.argsort(y)).astype(float)
    rho = float(np.corrcoef(rx, ry)[0, 1])
    sims = np.array([np.corrcoef(rx, np.array(p, float))[0, 1] for p in itertools.permutations(ry)])
    return rho, float(np.mean(sims <= rho + 1e-12)), float(np.mean(np.abs(sims) >= abs(rho) - 1e-12))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", action="append", default=[], help="name=summary dir (repeatable)")
    ap.add_argument("--f5-ref", default=None, help="arm name (from --arm) that F5 orders by")
    ap.add_argument("--f5-cand", action="append", default=[], help="name=summary dir of a b2 arm")
    ap.add_argument("--json", default=None)
    ap.add_argument("--fig", default=None)
    a = ap.parse_args()

    arms = {}
    for spec in a.arm:
        name, d = spec.split("=", 1)
        p = Path(d) if Path(d).is_absolute() else HSLAM_ROOT / d
        arms[name] = (p, per_arm(p))

    out = {"drift": {}, "f5": {}}
    print("| seq | " + " | ".join(f"{n}: local %/100m · local %/path · post-found %/path · f.o. (n)" for n in arms) + " |")
    print("|---|" + "---|" * len(arms))
    seqs = sorted({k for _, (_, res) in arms.items() for k in res},
                  key=lambda k: (k[0] != "tum", k[1]))
    for key in seqs:
        cells = []
        for name, (_, res) in arms.items():
            L = res.get(key, [])
            m = {k: med([x[k] for x in L]) for k in KEYS}
            out["drift"][f"{name}|{key[0]}|{key[1]}"] = dict(m, n=len(L))
            cells.append("—" if not L else
                         f"{m['drift_local_pct_per_100m']:+.2f} · {m['drift_local_pct_path']:+.1f} · "
                         f"{m['drift_postfound_pct_path']:+.1f} · {m['founding_offset_log']:+.3f} ({len(L)})")
        print(f"| {key[0]} {key[1]} | " + " | ".join(cells) + " |")

    if a.f5_ref and a.f5_cand:
        ref_dir, ref_res = arms[a.f5_ref]
        ref_reps = AR.load_reps([str(ref_dir / "summary.csv")])
        kitti = sorted(k for k in ref_res if k[0] == "kitti")
        fig_pts = {}
        for spec in a.f5_cand:
            name, d = spec.split("=", 1)
            p = Path(d) if Path(d).is_absolute() else HSLAM_ROOT / d
            cand_reps = AR.load_reps([str(p / "summary.csv")])
            xs, ys, labs = [], [], []
            for k in kitti:
                if k not in cand_reps or k not in ref_reps:
                    continue
                fo = med([x["founding_offset_log"] for x in ref_res[k]])
                c = AR.classify(ref_reps[k], cand_reps[k])
                if fo == fo and c["delta"] == c["delta"]:
                    xs.append(abs(fo)); ys.append(c["delta"]); labs.append(k[1])
            rho, p1, p2 = spearman_exact(np.array(xs), np.array(ys)) if len(xs) >= 5 else (float("nan"),) * 3
            out["f5"][name] = {"seq": labs, "abs_fo": xs, "dATE": ys, "rho": rho, "p_one": p1, "p_two": p2}
            fig_pts[name] = (xs, ys, labs, rho, p1)
            print(f"\nF5 {name} vs {a.f5_ref}: n={len(xs)} rho(|f.o.|, dATE) = {rho:+.3f}  "
                  f"exact p one-sided {p1:.4f} two-sided {p2:.4f}")
            for s, x, y in zip(labs, xs, ys):
                print(f"   KITTI {s}: |f.o.| {x:.3f}  dATE {y * 100:+.1f} %")
        if a.fig and fig_pts:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(5.2, 3.6))
            for (name, (xs, ys, labs, rho, p1)), mk in zip(fig_pts.items(), ("o", "s", "^")):
                ax.scatter(xs, [y * 100 for y in ys], marker=mk, label=f"{name} (ρ={rho:+.2f}, p={p1:.3f})")
                for x, y, s in zip(xs, ys, labs):
                    ax.annotate(s, (x, y * 100), fontsize=7, xytext=(3, 2), textcoords="offset points")
            ax.axhline(0, lw=0.6, color="0.5")
            ax.set_xlabel(f"|founding-segment scale offset| of `{a.f5_ref}` (log)")
            ax.set_ylabel("ΔATE Sim(3) vs frozen config (%)")
            ax.set_title("F5 — KITTI, windowed estimator", fontsize=9)
            ax.legend(fontsize=7)
            fig.tight_layout()
            fig.savefig(a.fig, dpi=160)
            print(f"\nwrote {a.fig}")

    if a.json:
        Path(a.json).write_text(json.dumps(out, indent=1, default=float))
        print(f"wrote {a.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
