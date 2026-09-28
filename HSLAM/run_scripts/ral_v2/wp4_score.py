#!/usr/bin/env python3
"""WP4 -- the component ablation at the re-frozen config, scored exactly as pre-registered in DECISIONS.md
"WP4 -- component ablation -- PRE-REGISTERED 2026-09-26" (rules R1/R2/R6 decisional; R3/R4/R5 reported; R7 the
KITTI 02 attribution; K0/K3/K12 readings; Tier-2 H2 rule and escalation), with "WP4 -- PRE-REGISTRATION AMENDMENT 1"
(DV7 prior-relative drift tables; R2 sub-readouts: the KITTI 07 truck wording and init_mode per arm x sequence).
The mechanical choices the text leaves open are fixed in "WP4 -- launch notes and scorer choices", written before any
WP4 row was read; the docstrings below repeat them where they bite.

Attribution rules only: WP4 adopts nothing. adoption_rule is imported for its C2 breakage band (classify), its Fisher
test and P_TRACK, and iqr; make_tables for aggregation (usable = status OK and track_success 1, P5a floor re-applied),
coverage parity and MIN_REPS. No statistic is re-derived here. Computed ONCE per tier, at its end; interim numbers are
never decision inputs. Rows are never dropped: every row counts in n; the one pre-registered exclusion is K0's
mechanism check (a K0 row without init_mode=photometric is VOIDED and listed).

Reference for every arm: stage 2's `full` (--ref-root, default runs/prewp4_s2_eval-server; commit 725c68b9, binary
0155a0d2ab6932b4, TUM n=5 / KITTI n=10). Its DV7 columns come from --dv7-ref (wp4_prior_align_cols.py output), since
the stage-2 summary.csv predates them.

Usage: wp4_score.py --tier 1 [--root runs/wp4_eval-server] [--ref-root runs/prewp4_s2_eval-server]
                    [--dv7-ref docs/ral_v2_resubmission/wp/data/wp4_gapcheck_20260926/dv7_prewp4_s2.csv]
                    [--part all|prov|verdicts|r7|k3|k0|k12|dv7|submode|r3|r4|r5|esc|md]
Output is markdown (pipe tables) so that RESULTS_LOG can carry it verbatim.
"""
import argparse
import csv
import math
import re
import statistics as st
import sys
from collections import Counter, defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import adoption_rule as R                                      # noqa: E402
import arms as A                                               # noqa: E402
import datasets as D                                           # noqa: E402
import make_tables as MT                                       # noqa: E402

EPOCH = "0155a0d2ab6932b4"
REF_ARM = "full"
T1_ARMS = ["A1_strict", "A1_near", "A2", "K3", "K0", "K1", "K1c", "K12", "K14_off", "K15_off", "K8"]
K02_ARMS = ["K14_off", "K15_off", "full_6799be6"]
T2_ARMS = ["K2", "K5", "K6", "K7", "K9_n3", "K9_n5", "K10_q037", "K11", "S1_unc_lo", "S1_unc_hi",
           "S1_alphaw_lo", "S1_alphaw_hi"]
H2_ARMS = ["H2_t03", "H2_t07"]
LADDER = ["A0_tol", "A1_strict", "A1_near", "A2", "K3", "full"]
ABL10 = list(D.SETS["ABL-10"])
LOOPSET = [("kitti", "00"), ("kitti", "05"), ("kitti", "06"), ("tum", "freiburg2_desk")]
K3_CONTROLS = [("kitti", "07"), ("kitti", "10")]
K3_EFFECT_SEQS = [("kitti", "00"), ("kitti", "05"), ("kitti", "06")]
DATASETS = ("tum", "kitti")

R1_DELTA = 0.10                 # ln units
R1_BAND = math.log(1.15)        # the +-15 % band `full` holds
R6_DELTA = math.log(1.25)       # 0.223, the R3b F bar
R2_COV = 0.90                   # P5a coverage ratio
R2_COV_SEQS = 2
R7_BAR = 12.655                 # (d1)'s bar: reference median 10.610 + IQR 2.045 (runs/wp2ii_eval-server/full)
ESC_FPS = (0.90, 1.10)
GPU_MODE_SPLIT = 5700.0         # peak_gpu_mb bimodality: ~5110 vs ~6295 MB (card R4)
TRUCK_PARTICIPATES = 9          # KITTI 07 usable >= 9/10 -> "participates in the truck loss"
TRUCK_AGGRAVATES = 4            # <= 4/10 -> "aggravates"
DV7_COLS = ("pa_slope", "pa_range", "pa_snap")
DV7_NOISE_MULT = 2.0

MIN_REPS = MT.MIN_REPS          # 3: R1/R6 entry bar (both sides), fo definedness, R7 cell readability


# ------------------------------------------------------------------------------------------- loading
def num(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else float("nan")
    except (TypeError, ValueError):
        return float("nan")


def intv(v):
    x = num(v)
    return int(x) if x == x else 0


def f(x, p=3):
    return "--" if x is None or x != x else f"{x:.{p}f}"


def usable(r):
    """make_tables' definition: status OK, track_success 1, P5a floor (a no-op at this epoch, re-applied)."""
    n = D.image_count(r["dataset"], r["sequence"])
    fr = num(r.get("frames"))
    floor = True if (not n or fr != fr) else fr >= MT.COVERAGE_SANITY_TABLE * n
    return r.get("status") == "OK" and r.get("track_success") == "1" and floor


def rows_of(root, arm):
    p = Path(root) / arm / "summary.csv"
    return MT.load_rows([p]) if p.exists() else []


class Data:
    """All rows of the tier's arms plus the reference, grouped; K0's voided rows removed from its cells (listed)."""

    def __init__(self, root, ref_root, arms_, dv7_ref):
        self.root, self.ref_root = Path(root), Path(ref_root)
        self.arms = arms_
        self.rows = {a: rows_of(root, a) for a in arms_}
        self.rows[REF_ARM] = rows_of(ref_root, REF_ARM)
        self.void = defaultdict(list)                       # arm -> rows excluded by a pre-registered check
        if "K0" in self.rows:
            keep = []
            for r in self.rows["K0"]:
                if r.get("init_mode") != "photometric":
                    self.void["K0"].append(r)
                else:
                    keep.append(r)
            self.rows["K0"] = keep
        allrows = [r for v in self.rows.values() for r in v]
        for r in allrows:
            r["_usable"] = usable(r)
        self.agg = MT.aggregate(allrows)
        self.by = defaultdict(list)                          # (arm, ds, sq) -> rows
        for r in allrows:
            self.by[(r["arm"], r["dataset"], r["sequence"])].append(r)
        # DV7 for the reference rows (columns absent from the stage-2 summary.csv): join by (arm, ds, sq, rep)
        self.dv7_ref = {}
        if dv7_ref and Path(dv7_ref).exists():
            for r in csv.DictReader(open(dv7_ref)):
                if r["arm"] == REF_ARM:
                    self.dv7_ref[(r["dataset"], r["sequence"], r["rep"])] = r
        self.stopped = {}
        for p in sorted(self.root.glob("STOPPED_*")):
            m = re.match(r"STOPPED_(.+)_(tum|kitti)$", p.name)
            if m:
                self.stopped[(m.group(1), m.group(2))] = p.read_text().strip()

    def cell(self, arm, ds, sq):
        return self.agg.get((arm, ds, sq))

    def reps(self, arm, ds, sq, only_usable=True):
        rs = self.by.get((arm, ds, sq), [])
        return [r for r in rs if r["_usable"]] if only_usable else rs

    def vals(self, arm, ds, sq, col):
        """finite values of a column over usable reps (reference DV7 columns via the side file)."""
        out = []
        for r in self.reps(arm, ds, sq):
            v = r.get(col)
            if (v is None or v == "") and arm == REF_ARM and col.startswith("pa_"):
                side = self.dv7_ref.get((ds, sq, r["rep"]))
                v = side.get(col) if side else None
            x = num(v)
            if x == x:
                out.append(x)
        return out

    def counts(self, arm, ds, sq):
        rs = self.reps(arm, ds, sq, only_usable=False)
        u = sum(1 for r in rs if r["_usable"])
        return u, len(rs)


# ------------------------------------------------------------------------------------- per-sequence
def seq_stats(d, arm, ds, sq):
    """The DVs one sequence contributes, arm and reference side by side."""
    out = {}
    for side, a in (("arm", arm), ("ref", REF_ARM)):
        c = d.cell(a, ds, sq)
        u, n = d.counts(a, ds, sq)
        s = d.vals(a, ds, sq, "scale_s")
        fo = d.vals(a, ds, sq, "founding_offset_log")
        out[side] = dict(
            present=c is not None, u=u, n=n, rate=(u / n) if n else float("nan"),
            s=st.median(s) if s else float("nan"),
            lns=abs(math.log(st.median(s))) if s else float("nan"),          # |ln median s| (decisive)
            med_lns=st.median(abs(math.log(x)) for x in s) if s else float("nan"),
            fo=st.median(fo) if len(fo) >= MIN_REPS else float("nan"),       # defined iff >= MIN_REPS finite
            fo_n=len(fo),
            cov=c["frame_coverage"] if c else float("nan"),
            ate=c["ate_sim3_rmse"] if c else float("nan"), ate_iqr=c["ate_sim3_rmse_iqr"] if c else float("nan"),
            se3=c["ate_se3_rmse"] if c else float("nan"),
            fps=c["postinit_fps"] if c else float("nan"))
    a, r = out["arm"], out["ref"]
    # R1
    out["r1_in"] = a["u"] >= MIN_REPS and r["u"] >= MIN_REPS
    out["d_lns"] = a["lns"] - r["lns"] if out["r1_in"] else float("nan")
    out["band_exit"] = out["r1_in"] and r["lns"] <= R1_BAND < a["lns"]
    # R6
    out["r6_in"] = out["r1_in"] and a["fo"] == a["fo"] and r["fo"] == r["fo"]
    out["d_fo"] = abs(a["fo"]) - abs(r["fo"]) if out["r6_in"] else float("nan")
    # R2 (a): directional coverage ratio arm / full; an arm with no usable rep where full has coverage covers nothing
    if r["cov"] == r["cov"] and r["cov"] > 0:
        out["cov_ratio"] = (a["cov"] / r["cov"]) if a["cov"] == a["cov"] else (0.0 if a["n"] else float("nan"))
    else:
        out["cov_ratio"] = float("nan")
    out["r2a"] = out["cov_ratio"] == out["cov_ratio"] and out["cov_ratio"] < R2_COV
    # R2 (b): the adoption rule's breakage band, via classify (imported)
    ref_reps = R.load_reps([str(d.ref_root / REF_ARM / "summary.csv")]).get((ds, sq))
    cand_reps = R.load_reps([str(d.root / arm / "summary.csv")]).get((ds, sq))
    if arm == "K0" and cand_reps is not None and d.void.get("K0"):
        # voided rows do not count for K0 (pre-registered); rebuild the counts from the kept rows
        cand_reps = {"ate": [num(x["ate_sim3_rmse"]) for x in d.reps("K0", ds, sq)
                             if num(x["ate_sim3_rmse"]) > 0], "ok": a["u"], "att": a["n"]}
    cls = R.classify(ref_reps, cand_reps) if (ref_reps and cand_reps) else None
    out["r2b"] = bool(cls and cls["breakage"])
    out["cls_p_track"] = cls["p_track"] if cls else float("nan")
    # R2 (c): established track drop, Fisher two-sided (adoption_rule.fisher_p) on the usable counts, computed
    # whenever the arm's usable rate is below full's; not gated by classify's MIN_REPS=5 ATE prerequisite
    out["p_track"] = float("nan")
    if a["n"] and r["n"] and a["rate"] < r["rate"]:
        out["p_track"] = R.fisher_p(a["u"], a["n"] - a["u"], r["u"], r["n"] - r["u"])
    out["r2c"] = out["p_track"] == out["p_track"] and out["p_track"] < R.P_TRACK
    out["parity"] = MT.coverage_parity(d.agg, [arm, REF_ARM]).get((ds, sq), {"valid": True, "ratio": float("nan")})
    return out


def dataset_verdicts(d, arm, ds, seqs):
    """R1, R2, R6 for one arm on one dataset, exactly as pre-registered."""
    per = {sq: seq_stats(d, arm, ds, sq) for (dd, sq) in seqs if dd == ds and d.cell(arm, ds, sq) is not None}
    stopped = (arm, ds) in d.stopped
    n1 = [sq for sq, s in per.items() if s["r1_in"]]
    lb_major = sum(1 for sq in n1 if per[sq]["d_lns"] >= R1_DELTA)
    band = sum(1 for sq in n1 if per[sq]["band_exit"])
    neutral_viol = sum(1 for sq in n1 if abs(per[sq]["d_lns"]) >= R1_DELTA)
    if len(n1) < 3:
        r1 = "NOT DECIDED"
    elif lb_major * 2 > len(n1) or band >= 2:
        r1 = "SCALE-LOAD-BEARING"
    elif neutral_viol <= 1:
        r1 = "SCALE-NEUTRAL"
    else:
        r1 = "MIXED"
    n6 = [sq for sq, s in per.items() if s["r6_in"]]
    lb6 = sum(1 for sq in n6 if per[sq]["d_fo"] >= R6_DELTA)
    neutral6_viol = sum(1 for sq in n6 if abs(per[sq]["d_fo"]) >= R6_DELTA)
    if len(n6) < 3:
        r6 = "NOT DECIDED"
    elif lb6 * 2 > len(n6):
        r6 = "FOUNDING-LOAD-BEARING"
    elif neutral6_viol <= 1:
        r6 = "NEUTRAL"
    else:
        r6 = "MIXED"
    a_seqs = [sq for sq, s in per.items() if s["r2a"]]
    b_seqs = [sq for sq, s in per.items() if s["r2b"]]
    c_seqs = [sq for sq, s in per.items() if s["r2c"]]
    r2 = "RELIABILITY-LOAD-BEARING" if (len(a_seqs) >= R2_COV_SEQS or b_seqs or c_seqs) else "NEUTRAL"
    return dict(per=per, stopped=stopped, n1=n1, lb_major=lb_major, band=band, r1=r1,
                n6=n6, lb6=lb6, r6=r6, a_seqs=a_seqs, b_seqs=b_seqs, c_seqs=c_seqs, r2=r2,
                signs=[(sq, per[sq]["d_lns"]) for sq in n1], signs6=[(sq, per[sq]["d_fo"]) for sq in n6])


def seq_material(s):
    """A sequence-level material effect under R1, R2 or R6 (K3's falsifier, H2's rule)."""
    hits = []
    if s["r1_in"] and (s["d_lns"] >= R1_DELTA or s["band_exit"]):
        hits.append("R1")
    if s["r2a"] or s["r2b"] or s["r2c"]:
        hits.append("R2")
    if s["r6_in"] and s["d_fo"] >= R6_DELTA:
        hits.append("R6")
    return hits


# ---------------------------------------------------------------------------------------- reporting
def seqname(ds, sq):
    return f"{ds} {sq}"


def part_prov(d, tier_arms):
    print("\n## provenance (every row counts; nothing is dropped)\n")
    print("| arm | rows | commit | binary | dirty | host | status | ok∧ts | CLI = arms.build | void |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    bad = []
    for a in [REF_ARM] + tier_arms:
        rs = d.rows.get(a, []) + d.void.get(a, [])
        if not rs:
            print(f"| `{a}` | 0 | — | — | — | — | no rows | — | — | — |")
            continue
        cli_ok = 0
        for r in rs:
            want = [t for t in A.build(a, D.resolve(r["dataset"], r["sequence"])) if "/" not in t]
            have = [t for t in r["cli"].split() if "/" not in t]
            ok = any(have[i:i + len(want)] == want for i in range(len(have) - len(want) + 1))
            cli_ok += ok
            if a != REF_ARM and (r.get("binary") != EPOCH or r.get("dirty") != "0"):
                bad.append((a, r["dataset"], r["sequence"], r["rep"], r.get("binary"), r.get("dirty")))
        stat = dict(sorted(Counter(r["status"] or "(empty)" for r in rs).items()))
        print(f"| `{a}` | {len(rs)} | {sorted({r['commit'] for r in rs})} | {sorted({r.get('binary') for r in rs})} "
              f"| {sorted({r['dirty'] for r in rs})} | {sorted({r['host'] for r in rs})} | {stat} "
              f"| {sum(1 for r in rs if r['_usable'] if '_usable' in r)} | {cli_ok}/{len(rs)} | {len(d.void.get(a, []))} |")
    if bad:
        print(f"\n**WARNING: {len(bad)} WP4 row(s) not at binary {EPOCH} from a clean tree** (make_tables "
              f"--require-binary refuses them; reported, never dropped):")
        for b in bad:
            print(f"- {b}")
    for a, rs in d.void.items():
        print(f"\n**{a}: {len(rs)} row(s) VOIDED by the pre-registered mechanism check (init_mode != photometric)** "
              f"-- stop and report (card §11.4):")
        for r in rs:
            print(f"- {r['dataset']} {r['sequence']} rep {r['rep']}: init_mode={r.get('init_mode')!r} status={r['status']}")
    a1 = [(a, r["dataset"], r["sequence"], r["rep"], r.get("ml_inferences")) for a in ("A1_strict", "A1_near")
          for r in d.rows.get(a, []) if intv(r.get("ml_inferences")) != 1]
    if a1:
        print(f"\n**A1_* rows with ml_inferences != 1 ({len(a1)}) -- the mechanism has not engaged; stop and report:**")
        for x in a1:
            print(f"- {x}")
    if d.stopped:
        print("\n**C2 stops (per dataset, never resumed):**")
        for (a, ds), why in sorted(d.stopped.items()):
            print(f"- `{a}` on {ds}: {why}")


def part_verdicts(d, tier_arms, seqs):
    print("\n## R1 / R2 / R6 verdict ledger (per arm x dataset; Δ = |ln s_arm| − |ln s_full|, Δfo = |fo_arm| − |fo_full|)\n")
    print("| arm | dataset | R1 scale | N₁ · Δ≥0.10 · band exits | R2 reliability | (a) cov<0.90 · (b) breakage · (c) Fisher<0.10 "
          "| R6 founding | N₆ · Δfo≥0.223 | note |")
    print("|---|---|---|---|---|---|---|---|---|")
    ledger = {}
    for a in tier_arms:
        for ds in DATASETS:
            v = dataset_verdicts(d, a, ds, seqs)
            ledger[(a, ds)] = v
            if not v["per"]:
                print(f"| `{a}` | {ds} | no rows | | | | | | |")
                continue
            note = "stopped at C2" if v["stopped"] else ""
            print(f"| `{a}` | {ds} | **{v['r1']}** | {len(v['n1'])} · {v['lb_major']} · {v['band']} | **{v['r2']}** "
                  f"| {v['a_seqs'] or '–'} · {v['b_seqs'] or '–'} · {v['c_seqs'] or '–'} | **{v['r6']}** "
                  f"| {len(v['n6'])} · {v['lb6']} | {note} |")
    print("\n### per sequence: s, |ln s|, fo, usable, coverage, Fisher p (arm vs `full`)\n")
    print("| arm | sequence | s arm / full | Δ\\|ln s\\| | med\\|ln s\\| arm/full | fo arm / full | Δ\\|fo\\| | usable arm / full "
          "| cov ratio | Fisher p (c) | classify p_track | flags |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for a in tier_arms:
        for ds in DATASETS:
            v = ledger[(a, ds)]
            for (dd, sq) in seqs:
                if dd != ds or sq not in v["per"]:
                    continue
                s = v["per"][sq]
                ar, rf = s["arm"], s["ref"]
                flags = []
                if not s["r1_in"]:
                    flags.append("<3 usable: out of R1/R6")
                if s["band_exit"]:
                    flags.append("BAND EXIT")
                if s["r2a"]:
                    flags.append("R2a")
                if s["r2b"]:
                    flags.append("R2b BREAKAGE")
                if s["r2c"]:
                    flags.append("R2c")
                if s["r1_in"] and not s["r6_in"]:
                    flags.append(f"fo undefined (arm {ar['fo_n']}, full {rf['fo_n']} finite): out of R6")
                if ar["u"] and rf["u"] and ((ar["med_lns"] - rf["med_lns"] >= R1_DELTA) != (s["d_lns"] >= R1_DELTA)):
                    flags.append("median-of-|ln s| would read differently")
                print(f"| `{a}` | {seqname(ds, sq)} | {f(ar['s'])} / {f(rf['s'])} | {f(s['d_lns'])} "
                      f"| {f(ar['med_lns'])} / {f(rf['med_lns'])} | {f(ar['fo'])} / {f(rf['fo'])} | {f(s['d_fo'])} "
                      f"| {ar['u']}/{ar['n']} / {rf['u']}/{rf['n']} | {f(s['cov_ratio'], 2)} | {f(s['p_track'])} "
                      f"| {f(s['cls_p_track'])} | {', '.join(flags)} |")
    return ledger


def part_r7(d):
    print("\n## R7 — KITTI 02 attribution of the (d1) cost (POST-HOC), applied once after Tier 1\n")
    ref_csv = Path("runs/wp2ii_eval-server/full/summary.csv")
    if ref_csv.exists():
        rr = [r for r in csv.DictReader(open(ref_csv)) if r["dataset"] == "kitti" and r["sequence"] == "02"]
        u = [num(r["ate_sim3_rmse"]) for r in rr if r["status"] == "OK" and r["track_success"] == "1"]
        u = [x for x in u if x == x and x > 0]
        print(f"reference (R0-licensed `runs/wp2ii_eval-server/full`, KITTI 02): {len(u)}/{len(rr)} usable, median Sim(3) "
              f"ATE {f(st.median(u)) if u else '--'} (IQR {f(R.iqr(u)) if u else '--'}) -> bar {R7_BAR} m as pre-registered")
    print("\n| cell | K14 | K15 | usable | median Sim(3) ATE [m] (IQR) | restores (≤ 12.655)? | s | SE(3) ATE [m] |")
    print("|---|---|---|---|---|---|---|---|")
    res = {}
    spec = {"full_6799be6": ("off", "off"), "K14_off": ("off", "on"), "K15_off": ("on", "off"), REF_ARM: ("on", "on")}
    for a in ["full_6799be6", "K14_off", "K15_off", REF_ARM]:
        c = d.cell(a, "kitti", "02")
        u, n = d.counts(a, "kitti", "02")
        if c is None:
            print(f"| `{a}` | {spec[a][0]} | {spec[a][1]} | no rows | | | | |")
            res[a] = None
            continue
        readable = u >= MIN_REPS
        restores = (c["ate_sim3_rmse"] <= R7_BAR) if readable else None
        res[a] = restores
        tag = "—" if a == REF_ARM else ("NOT DECIDED (< 3 usable)" if not readable else ("restores" if restores else "not"))
        print(f"| `{a}` | {spec[a][0]} | {spec[a][1]} | {u}/{n} | {f(c['ate_sim3_rmse'])} ({f(c['ate_sim3_rmse_iqr'])}) "
              f"| {tag} | {f(c['scale_s'])} | {f(c['ate_se3_rmse'])} |")
    g, k14, k15 = res.get("full_6799be6"), res.get("K14_off"), res.get("K15_off")
    if g is None or (g and (k14 is None or k15 is None)):
        verdict = "NOT DECIDED: a cell is unreadable (< 3 usable reps or no rows) -- report to the user"
    elif not g:
        verdict = ("**the binary (D8) or run-to-run drift** — \"not attributable to the configuration\"; "
                   "the knock-out cells are reported descriptively")
    elif k14 and not k15:
        verdict = "**K14** — \"the K14 fix costs KITTI 02 +33 % Sim(3) ATE while improving its scale and SE(3) ATE\""
    elif k15 and not k14:
        verdict = "**K15** — same wording, naming K15"
    elif k14 and k15:
        verdict = "**K14 × K15: both are needed** — \"the combination of the two fixes\""
    else:
        verdict = "**either fix suffices** — \"either fix alone\""
    print(f"\n**R7 attribution:** {verdict}")


def part_k3(d, seqs):
    print("\n## K3 (≡ A3, `--p2-gate=false`) with the D4 guard on: H2's marginal filter\n")
    print("| sequence | K3 usable / full | H2 evals K3 / full (sum) | H2 fires K3 / full | lc_sim3_guard K3 / full | Δ|ln s| | Δ|fo| "
          "| cov ratio | material (R1/R2/R6) |")
    print("|---|---|---|---|---|---|---|---|---|")
    falsified = []
    for (ds, sq) in seqs:
        if d.cell("K3", ds, sq) is None:
            continue
        s = seq_stats(d, "K3", ds, sq)
        def tot(arm, col):
            return sum(intv(r.get(col)) for r in d.reps(arm, ds, sq, only_usable=False))
        hits = seq_material(s)
        ctrl = (ds, sq) in K3_CONTROLS
        if ctrl and hits:
            falsified.append((ds, sq, hits))
        print(f"| {seqname(ds, sq)}{' (control)' if ctrl else ''} | {s['arm']['u']}/{s['arm']['n']} / {s['ref']['u']}/{s['ref']['n']} "
              f"| {tot('K3', 'lc_gate_evals')} / {tot(REF_ARM, 'lc_gate_evals')} | {tot('K3', 'lc_gate_fires')} / {tot(REF_ARM, 'lc_gate_fires')} "
              f"| {tot('K3', 'lc_sim3_guard')} / {tot(REF_ARM, 'lc_sim3_guard')} | {f(s['d_lns'])} | {f(s['d_fo'])} "
              f"| {f(s['cov_ratio'], 2)} | {', '.join(hits) or '–'} |")
    aborts = [r for r in d.rows.get("K3", []) if r["status"] != "OK" or r["returncode"] not in ("0", "")]
    if aborts:
        print(f"\n**K3 aborts ({len(aborts)}), reported with their site, never attributed to H2:**")
        for r in aborts:
            log = d.root / "K3" / f"{r['dataset']}_{r['sequence']}_rep{r['rep']}" / "run.log"
            site = "(no log)"
            if log.exists():
                text = log.read_text(errors="replace")
                m = re.findall(r"(Assertion .*?failed\.|terminate called.*|Segmentation fault.*|BIG ERROR.*|LOST.*)", text)
                site = m[-1][:160] if m else "(no known signature in the log)"
            print(f"- {r['dataset']} {r['sequence']} rep {r['rep']}: rc={r['returncode']} status={r['status'] or '(empty)'} "
                  f"traj_error={r.get('traj_error', '')[:80]} site: {site}")
    else:
        print("\nK3 aborts: none.")
    if falsified:
        print(f"\n**K3 FALSIFIER FIRED**: a material effect on a negative control ({falsified}) — the attribution of K3's "
              f"effect to H2 is falsified (pre-registered). H2 rejected 0 loops there in 20 `full` reps.")
    else:
        print("\n**K3 falsifier (KITTI 07 / 10 negative controls): not fired** — no sequence-level material effect under "
              "R1, R2 or R6 on either control; any K3 effect is attributable to H2's marginal filter on KITTI 00/05/06.")


def part_k0(d, ledger):
    print("\n## K0 (`--ml-init=false`): P0 off is scale-inconsistent by construction — reading against the expectation\n")
    rs = d.rows.get("K0", []) + d.void.get("K0", [])
    modes = Counter(r.get("init_mode") for r in rs)
    print(f"- (i) mechanism check: init_mode over all {len(rs)} K0 rows = {dict(modes)}; voided = {len(d.void.get('K0', []))}")
    for ds in DATASETS:
        v = ledger.get(("K0", ds))
        if not v or not v["per"]:
            continue
        fos = ", ".join(f"{sq} {f(s['arm']['fo'], 2)} vs {f(s['ref']['fo'], 2)}" for sq, s in v["per"].items())
        print(f"- {ds}: R1 **{v['r1']}**, R2 **{v['r2']}**, R6 **{v['r6']}**; fo arm vs full: {fos}")
    k = ledger.get(("K0", "kitti")); t = ledger.get(("K0", "tum"))
    r6_lb = any(v and v["r6"] == "FOUNDING-LOAD-BEARING" for v in (k, t))
    r1_k = k["r1"] if k else "no rows"
    if r6_lb and r1_k == "SCALE-NEUTRAL":
        print("- **Reading:** (ii) and (iii) hold -> P0 is worded as **\"sets the founding segment's scale\"**; the ongoing prior sets the rest.")
    elif r1_k == "SCALE-LOAD-BEARING":
        print("- **Reading:** R1 LOAD-BEARING on KITTI -> P0 is worded as **\"carries whole-trajectory scale\"**.")
    else:
        print(f"- **Reading:** neither pre-registered branch matches cleanly (R6 load-bearing: {r6_lb}; KITTI R1: {r1_k}) -> "
              f"report as measured; the wording is the user's call.")


def part_k12(d, ledger):
    print("\n## K12 (`--ml-fej-freeze=false`): the paper's named scale mechanism\n")
    allneutral = True
    for ds in DATASETS:
        v = ledger.get(("K12", ds))
        if not v or not v["per"]:
            print(f"- {ds}: no rows"); allneutral = False; continue
        print(f"- {ds}: R1 **{v['r1']}**, R2 **{v['r2']}**, R6 **{v['r6']}**")
        allneutral &= (v["r1"] == "SCALE-NEUTRAL" and v["r2"] == "NEUTRAL" and v["r6"] == "NEUTRAL")
    if allneutral:
        print("- **Reading:** R1, R2 and R6 all NEUTRAL on both datasets -> the paper says **the freeze is not resolvable as the "
              "scale mechanism at 0.10 log units on TUM/KITTI**; DV3 and ATE are reported as measured (R3/R5 below).")
    else:
        print("- **Reading:** not all NEUTRAL -> the effect is reported where it lands (R1/R2/R6 above, DV3/DV7/ATE below).")


def part_dv7(d, tier_arms, seqs):
    print("\n## DV7 — prior-relative drift (Amendment 1; reported, never decisional)\n")
    print("medians over usable reps; `full` from the DV7 side file; ‡ = beyond rep noise (|Δ| > 2 × `full`'s rep IQR of that column "
          "on that sequence); collapse = reps with pa_collapse_n ≥ 1 (sum)\n")
    print("| arm | sequence | pa_kf_n | pa_slope arm / full | pa_range arm / full | pa_snap arm / full | collapse arm / full |")
    print("|---|---|---|---|---|---|---|")
    for a in tier_arms:
        for (ds, sq) in seqs:
            if d.cell(a, ds, sq) is None:
                continue
            cells = []
            kf = d.vals(a, ds, sq, "pa_kf_n")
            for col in DV7_COLS:
                va, vr = d.vals(a, ds, sq, col), d.vals(REF_ARM, ds, sq, col)
                ma = st.median(va) if va else float("nan")
                mr = st.median(vr) if vr else float("nan")
                noise = R.iqr(vr) if len(vr) >= 2 else float("nan")
                mark = "‡" if (ma == ma and mr == mr and noise == noise and abs(ma - mr) > DV7_NOISE_MULT * noise) else ""
                cells.append(f"{f(ma, 4)} / {f(mr, 4)}{mark}")
            ca, cr = d.vals(a, ds, sq, "pa_collapse_n"), d.vals(REF_ARM, ds, sq, "pa_collapse_n")
            coll = f"{sum(1 for x in ca if x >= 1)}/{len(ca)} ({int(sum(ca))}) / {sum(1 for x in cr if x >= 1)}/{len(cr)} ({int(sum(cr))})"
            print(f"| `{a}` | {seqname(ds, sq)} | {f(st.median(kf), 0) if kf else '--'} | " + " | ".join(cells) + f" | {coll} |")


def part_submode(d, tier_arms, seqs):
    print("\n## R2 sub-readouts (Amendment 1; descriptive, never change R2's verdict)\n")
    print("### (a) KITTI 07 — the truck: usable k/n against `full` 7/10, one-sided Fisher p\n")
    try:
        from scipy.stats import fisher_exact
    except ImportError:
        fisher_exact = None
    ru, rn = d.counts(REF_ARM, "kitti", "07")
    print(f"| arm | usable | wording | one-sided Fisher p | two-sided (adoption_rule) |")
    print("|---|---|---|---|---|")
    for a in tier_arms:
        if d.cell(a, "kitti", "07") is None:
            continue
        u, n = d.counts(a, "kitti", "07")
        if u >= TRUCK_PARTICIPATES:
            word, alt = "this channel participates in the truck loss", "greater"
        elif u <= TRUCK_AGGRAVATES:
            word, alt = "aggravates", "less"
        else:
            word, alt = "–", None
        p1 = "--"
        if alt and fisher_exact is not None:
            p1 = f"{fisher_exact([[u, n - u], [ru, rn - ru]], alternative=alt)[1]:.3f}"
        elif alt:
            p1 = "(scipy missing)"
        p2 = R.fisher_p(u, n - u, ru, rn - ru)
        print(f"| `{a}` | {u}/{n} | {word} | {p1} | {p2:.3f} |")
    print("\n### (b) init_mode per arm x sequence (a fallback hand-over where `full` has none is a DV2 finding)\n")
    print("| arm | sequence | init_mode counts (all rows) | fallback reps | `full` |")
    print("|---|---|---|---|---|")
    for a in tier_arms:
        for (ds, sq) in seqs:
            rs = d.reps(a, ds, sq, only_usable=False) + [r for r in d.void.get(a, []) if r["dataset"] == ds and r["sequence"] == sq]
            if not rs:
                continue
            cnt = Counter(r.get("init_mode") or "(empty)" for r in rs)
            fb = [r["rep"] for r in rs if "fallback" in (r.get("init_mode") or "")]
            rf = Counter(r.get("init_mode") or "(empty)" for r in d.reps(REF_ARM, ds, sq, only_usable=False))
            mark = " **DV2 finding**" if fb and not any("fallback" in k for k in rf) else ""
            print(f"| `{a}` | {seqname(ds, sq)} | {dict(cnt)} | {fb or '–'}{mark} | {dict(rf)} |")


def part_r3(d, tier_arms, seqs):
    print("\n## R3 — scale consistency (DV3; descriptive: 'consistent with', never a verdict)\n")
    print("| arm | sequence | drift_local_pct_path arm / full | drift_postfound_pct_path arm / full | %/100 m arm / full (KITTI) |")
    print("|---|---|---|---|---|")
    for a in tier_arms:
        for (ds, sq) in seqs:
            if d.cell(a, ds, sq) is None:
                continue
            def m(arm, col):
                v = d.vals(arm, ds, sq, col)
                return st.median(v) if v else float("nan")
            k = f"{f(m(a, 'drift_local_pct_per_100m'), 2)} / {f(m(REF_ARM, 'drift_local_pct_per_100m'), 2)}" if ds == "kitti" else "–"
            print(f"| `{a}` | {seqname(ds, sq)} | {f(m(a, 'drift_local_pct_path'), 2)} / {f(m(REF_ARM, 'drift_local_pct_path'), 2)} "
                  f"| {f(m(a, 'drift_postfound_pct_path'), 2)} / {f(m(REF_ARM, 'drift_postfound_pct_path'), 2)} | {k} |")


def part_r4(d, tier_arms, seqs):
    print("\n## R4 — cost (DV4): postinit_fps and peak_gpu_mb, `ws` rows only; the fps ratio to `full` is the cost column\n")
    print("| arm | sequence | postinit_fps median (range) arm | full | ratio | peak_gpu_mb median (range) | modes ≈5110 / ≈6295 (n) |")
    print("|---|---|---|---|---|---|---|")
    ratios = {}
    for a in tier_arms:
        rr = []
        for (ds, sq) in seqs:
            if d.cell(a, ds, sq) is None:
                continue
            fa = [num(r["postinit_fps"]) for r in d.reps(a, ds, sq) if r["host"] == "eval-server"]
            fa = [x for x in fa if x == x]
            fr = d.vals(REF_ARM, ds, sq, "postinit_fps")
            ga = [num(r["peak_gpu_mb"]) for r in d.reps(a, ds, sq, only_usable=False) if r["host"] == "eval-server"]
            ga = [x for x in ga if x == x]
            ratio = (st.median(fa) / st.median(fr)) if (fa and fr) else float("nan")
            if ratio == ratio:
                rr.append(ratio)
            lo = sum(1 for x in ga if x < GPU_MODE_SPLIT); hi = len(ga) - lo
            print(f"| `{a}` | {seqname(ds, sq)} | {f(st.median(fa), 1) if fa else '--'} ({f(min(fa), 1) if fa else '--'}–{f(max(fa), 1) if fa else '--'}) "
                  f"| {f(st.median(fr), 1) if fr else '--'} | {f(ratio, 3)} | {f(st.median(ga), 0) if ga else '--'} "
                  f"({f(min(ga), 0) if ga else '--'}–{f(max(ga), 0) if ga else '--'}) | {lo} / {hi} ({len(ga)}) |")
        ratios[a] = st.median(rr) if rr else float("nan")
    print("\n**Cost column** (median over sequences of postinit_fps arm / `full`; escalation trigger outside [0.90, 1.10] applies to Tier 2):\n")
    print("| arm | fps ratio | outside [0.90, 1.10]? |")
    print("|---|---|---|")
    for a, r in ratios.items():
        out = "yes" if (r == r and not (ESC_FPS[0] <= r <= ESC_FPS[1])) else ("--" if r != r else "no")
        print(f"| `{a}` | {f(r, 3)} | {out} |")
    return ratios


def part_r5(d, tier_arms, seqs):
    print("\n## R5 — ATE (DV5; reported, never ranked): pairs that pass P5a parity; MISMATCH shows both coverages\n")
    print("| arm | sequence | Sim(3) ATE arm (IQR) | full (IQR) | SE(3) arm / full | rpe_trans_dist arm / full | usable | parity |")
    print("|---|---|---|---|---|---|---|---|")
    for a in tier_arms:
        for (ds, sq) in seqs:
            c = d.cell(a, ds, sq); r = d.cell(REF_ARM, ds, sq)
            if c is None or r is None:
                continue
            pv = MT.coverage_parity(d.agg, [a, REF_ARM]).get((ds, sq))
            if pv and not pv["valid"]:
                par = "MISMATCH " + " ".join(f"{k}={v:.0%}" for k, v in sorted(pv["cov"].items()))
            else:
                par = f"ok ({f(pv['ratio'], 2) if pv else '--'})"
            print(f"| `{a}` | {seqname(ds, sq)} | {f(c['ate_sim3_rmse'])} ({f(c['ate_sim3_rmse_iqr'])}) | {f(r['ate_sim3_rmse'])} ({f(r['ate_sim3_rmse_iqr'])}) "
                  f"| {f(c['ate_se3_rmse'])} / {f(r['ate_se3_rmse'])} | {f(c['rpe_trans_dist'])} / {f(r['rpe_trans_dist'])} "
                  f"| {c['n_ok']}/{c['n']} / {r['n_ok']}/{r['n']} | {par} |")


def part_esc(d, tier_arms, ledger, ratios):
    print("\n## Tier-2 escalation triggers and the H2 threshold rule\n")
    for a in tier_arms:
        why = []
        for ds in DATASETS:
            v = ledger.get((a, ds))
            if v and v["per"] and (v["r1"] == "SCALE-LOAD-BEARING" or v["r2"] == "RELIABILITY-LOAD-BEARING"
                                   or v["r6"] == "FOUNDING-LOAD-BEARING"):
                why.append(f"{ds}: {v['r1']}/{v['r2']}/{v['r6']}")
        r = ratios.get(a, float("nan"))
        if r == r and not (ESC_FPS[0] <= r <= ESC_FPS[1]):
            why.append(f"fps ratio {r:.3f}")
        if a in H2_ARMS:
            hits = [sq for (ds, sq) in LOOPSET if (c := d.cell(a, ds, sq)) is not None
                    and (s := seq_stats(d, a, ds, sq))["r1_in"] and abs(s["d_lns"]) >= R1_DELTA]
            r2 = [sq for (ds, sq) in LOOPSET if d.cell(a, ds, sq) is not None
                  and any(seq_stats(d, a, ds, sq)[k] for k in ("r2a", "r2b", "r2c"))]
            sens = len(hits) >= 2 or bool(r2)
            fires = {sq: (sum(intv(x.get("lc_gate_fires")) for x in d.reps(a, ds, sq, only_usable=False)),
                          sum(intv(x.get("lc_gate_fires")) for x in d.reps(REF_ARM, ds, sq, only_usable=False)))
                     for (ds, sq) in LOOPSET}
            print(f"- `{a}`: {'**SENSITIVE**' if sens else 'INSENSITIVE in [0.3, 0.7] at 0.10 log units, n=5'}; "
                  f"|Δ ln s| ≥ 0.10 on {hits}; R2 fires on {r2}; H2 fires arm/full {fires}")
            if sens:
                why.append("H2 SENSITIVE")
        print(f"- `{a}`: {'**ESCALATE** (' + '; '.join(why) + ')' if why else 'no escalation trigger'}")


def part_md(d, tier_arms, seqs):
    """RESULTS_LOG per-arm x sequence tables (descriptive)."""
    arms_ = [REF_ARM] + tier_arms
    hdr = "| sequence | " + " | ".join(f"`{a}`" for a in arms_) + " |"
    sep = "|---|" + "---|" * len(arms_)
    def c1(a, ds, sq):
        c = d.cell(a, ds, sq); u, n = d.counts(a, ds, sq)
        return "not run" if c is None else f"{f(c['ate_sim3_rmse'])} ({f(c['ate_sim3_rmse_iqr'])}) · {u}/{n}"
    def c2(a, ds, sq):
        c = d.cell(a, ds, sq)
        return "not run" if c is None else f"{f(c['ate_se3_rmse'])} · {f(c['scale_s'])}"
    def c3(a, ds, sq):
        c = d.cell(a, ds, sq)
        if c is None:
            return "not run"
        fo = d.vals(a, ds, sq, "founding_offset_log")
        return f"{f(st.median(fo), 2) if len(fo) >= MIN_REPS else '--'} · {f(c['postinit_fps'], 1)}"
    def c4(a, ds, sq):
        c = d.cell(a, ds, sq)
        if c is None:
            return "not run"
        rs = d.reps(a, ds, sq, only_usable=False)
        im = Counter(r.get("init_mode") or "(empty)" for r in rs)
        guard = sum(intv(r.get("lc_sim3_guard")) for r in rs)
        return f"{f(c['frame_coverage'], 2)} · {'/'.join(f'{k}:{v}' for k, v in sorted(im.items()))} · {guard}"
    for title, fn in (("Sim(3) ATE [m], median (IQR) · usable k/n", c1), ("SE(3) ATE [m], median · scale s, median", c2),
                      ("founding offset (log), median (≥ 3 finite) · post-init fps, median", c3),
                      ("P5a frame coverage · init_mode counts · lc_sim3_guard (sum)", c4)):
        print(f"\n#### {title}\n\n{hdr}\n{sep}")
        for (ds, sq) in seqs:
            print(f"| {seqname(ds, sq)} | " + " | ".join(fn(a, ds, sq) for a in arms_) + " |")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tier", required=True, choices=["1", "2", "esc"])
    ap.add_argument("--root", default="runs/wp4_eval-server")
    ap.add_argument("--ref-root", default="runs/prewp4_s2_eval-server")
    ap.add_argument("--dv7-ref", default="docs/ral_v2_resubmission/wp/data/wp4_gapcheck_20260926/dv7_prewp4_s2.csv")
    ap.add_argument("--arms", nargs="*", help="override the tier's arm list (esc: the escalated arms)")
    ap.add_argument("--part", default="all",
                    choices=["all", "prov", "verdicts", "r7", "k3", "k0", "k12", "dv7", "submode", "r3", "r4", "r5", "esc", "md"])
    a = ap.parse_args()
    tier_arms = a.arms or {"1": T1_ARMS + ["full_6799be6"], "2": T2_ARMS + H2_ARMS, "esc": []}[a.tier]
    if a.tier == "esc" and not tier_arms:
        ap.error("--tier esc needs --arms")
    seqs = ABL10 + ([("kitti", "02")] if a.tier == "1" else [])
    d = Data(a.root, a.ref_root, tier_arms, a.dv7_ref)
    print(f"# WP4 Tier {a.tier} — scored {Path(a.root)} against {Path(a.ref_root)}/{REF_ARM} (binary {EPOCH}); "
          f"rules as pre-registered, computed once")
    p = a.part
    ledger, ratios = {}, {}
    if p in ("all", "prov"):
        part_prov(d, tier_arms)
    if p in ("all", "verdicts", "k0", "k12", "esc"):
        ledger = part_verdicts(d, [x for x in tier_arms if x != "full_6799be6"], seqs)
    if a.tier == "1" and p in ("all", "r7"):
        part_r7(d)
    if "K3" in tier_arms and p in ("all", "k3"):
        part_k3(d, seqs)
    if "K0" in tier_arms and p in ("all", "k0"):
        part_k0(d, ledger)
    if "K12" in tier_arms and p in ("all", "k12"):
        part_k12(d, ledger)
    if p in ("all", "dv7"):
        part_dv7(d, tier_arms, seqs)
    if p in ("all", "submode"):
        part_submode(d, tier_arms, seqs)
    if p in ("all", "r3"):
        part_r3(d, tier_arms, seqs)
    if p in ("all", "r4", "esc"):
        ratios = part_r4(d, tier_arms, seqs)
    if p in ("all", "r5"):
        part_r5(d, tier_arms, seqs)
    if a.tier != "1" and p in ("all", "esc"):
        part_esc(d, tier_arms, ledger, ratios)
    if p == "md":
        part_md(d, tier_arms, seqs)
    return 0


if __name__ == "__main__":
    sys.exit(main())
