#!/usr/bin/env python3
"""Pre-WP4 stage 2, the re-freeze campaign, scored as pre-registered in DECISIONS.md "PRE-WP4 STAGE 0 --
PRE-REGISTRATIONS" (d), amended by "PRE-WP4 STAGE 2 -- two USER DECISIONS", with the mechanical choices the
text leaves open fixed in "PRE-WP4 STAGE 2 -- launch notes and scorer choices" before any stage-2 row was read.

Parts (run one, or `all`):
  d1     re-freeze no-regression: `full` vs the R0-licensed reference (runs/wp2ii_eval-server/full), on every
         sequence where the reference has >= 5 usable reps. Regression = (i) median Sim(3) ATE > ref median +
         ref IQR, (ii) |ln s_new| > |ln s_ref| + ln 1.10, (iii) breakage (new < 50 % usable, ref >= 80 %).
         fr2_desk's scale move to ~1.22 is the pre-registered known cost under (ii). Any regression -> exit 1.
  adopt  (d2) K13 and (d4) anchor: adoption_rule.evaluate -- imported, never re-derived -- per dataset, with
         P5a coverage parity (make_tables.coverage_parity) printed beside each ledger, flagged, never re-scored.
  d3     A0 vs A0_tol reliability contrast: C(M) = {sequences `full` tracks reliably (>= 80 % usable) and M
         fails (< 50 % usable)}; outcomes O1/O2/O3 with O3 taking precedence where they overlap.
  k0809  (d4') KITTI 08/09 reported as measured, every arm.
  tables RESULTS_LOG markdown for every arm x sequence (descriptive; not part of `all`).

Usable = status OK and track_success 1 (the CSV stamp, P5a floor included at this epoch; the floor is
re-applied here as make_tables does, idempotently). Runs continuing on a rejected initialisation count as
usable (user, 25 Sep); init_rejected_accepted is descriptive only. Rates are usable / completed rows.

Usage: prewp4_stage2_score.py --root runs/prewp4_s2_eval-server --ref runs/wp2ii_eval-server/full [--part d1]
       [--shipped full_K13]      # d3: also score C(M) against an adopted feature's arm
"""
import argparse
import csv
import functools
import math
import statistics as st
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import adoption_rule as R                                      # noqa: E402
import datasets as D                                           # noqa: E402
import make_tables as MT                                       # noqa: E402
from eval_run import apply_track_success                       # noqa: E402

TOL = math.log(1.10)                        # P4c's scale tolerance, (d1)(ii)
KNOWN_COST = ("tum", "freiburg2_desk")      # (d1): K14 moves its scale 1.10 -> ~1.22 (pre-registered)
KNOWN_COST_S = 1.22                         # the cost covers |ln s| <= ln 1.22 + ln 1.10 (scorer choice, fixed pre-data)
NEVER_K14 = {("tum", "freiburg1_360"), ("tum", "freiburg1_desk2"), ("tum", "freiburg1_floor"),
             ("tum", "freiburg2_360_hemisphere"), ("tum", "freiburg2_large_with_loop"),
             ("kitti", "01"), ("kitti", "02"), ("kitti", "03"), ("kitti", "04")}
TUM10 = ["freiburg1_360", "freiburg1_desk", "freiburg1_desk2", "freiburg1_floor", "freiburg1_room",
         "freiburg2_360_hemisphere", "freiburg2_desk", "freiburg2_large_no_loop", "freiburg2_large_with_loop",
         "freiburg3_long_office_household"]
KITTI11 = ["00", "01", "02", "03", "04", "05", "06", "07", "08", "09", "10"]
SEQS = [("tum", s) for s in TUM10] + [("kitti", s) for s in KITTI11]
CANDIDATES = {"full_K13": ("d2", {"freiburg2_large_no_loop"}), "full_R3b_anchor": ("d4", set())}


def num(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else float("nan")
    except (TypeError, ValueError):
        return float("nan")


def intv(v):
    x = num(v)
    return int(x) if x == x else 0


@functools.lru_cache(maxsize=None)
def load(path, flag=True):
    """rows grouped by (dataset, sequence), each row tagged _usable. Flags (never applies) any row whose
    track_success would change if re-pooled over all of its (arm, sequence)'s reps -- only a resume top-up
    (a second eval_run invocation) can cause that. The reference keeps its own per-invocation stamps
    unflagged (flag=False), as R0 read it."""
    p = Path(path)
    p = p if p.suffix == ".csv" else p / "summary.csv"
    if not p.exists():
        return {}
    rows = list(csv.DictReader(open(p)))
    typed = [dict(r, poses=intv(r["poses"]), frames=intv(r["frames"]),
                  ts_plausible=r.get("ts_plausible", "")) for r in rows]
    apply_track_success(typed)
    g = defaultdict(list)
    for r, t in zip(rows, typed):
        n_img = D.image_count(r["dataset"], r["sequence"])
        floor = (not n_img) or intv(r["frames"]) >= MT.COVERAGE_SANITY_TABLE * n_img
        r["_usable"] = r["status"] == "OK" and r["track_success"] == "1" and floor
        if flag and (t["track_success"] == 1) != (r["track_success"] == "1" and floor):
            print(f"  FLAG {p.parent.name} {r['dataset']} {r['sequence']} rep{r['rep']}: track_success "
                  f"{r['track_success']} as stamped, {t['track_success']} if re-pooled (a top-up?) -- stamp used")
        g[(r["dataset"], r["sequence"])].append(r)
    return g


def cell(rows):
    u = [r for r in rows if r["_usable"]]
    ate = [x for x in (num(r["ate_sim3_rmse"]) for r in u) if x == x and x > 0]
    se3 = [x for x in (num(r["ate_se3_rmse"]) for r in u) if x == x and x > 0]
    s = [x for x in (num(r["scale_s"]) for r in u) if x == x and x > 0]
    n_img = D.image_count(rows[0]["dataset"], rows[0]["sequence"]) if rows else None
    fr = [intv(r["frames"]) for r in u]
    return dict(
        n=len(rows), u=len(u), rate=len(u) / len(rows) if rows else float("nan"),
        ate=st.median(ate) if ate else float("nan"), ate_iqr=R.iqr(ate) if ate else float("nan"),
        se3=st.median(se3) if se3 else float("nan"),
        s=st.median(s) if s else float("nan"),
        lns=abs(math.log(st.median(s))) if s else float("nan"),               # |ln median s|: decisive
        med_lns=st.median(abs(math.log(x)) for x in s) if s else float("nan"),  # printed beside
        cov=(st.median(fr) / n_img) if (fr and n_img) else float("nan"),
        guard=sum(intv(r.get("lc_sim3_guard")) for r in rows),
        resets=[intv(r.get("init_resets")) for r in rows],
        verdicts=[intv(r.get("init_fail_verdicts")) for r in rows],
        rejacc=sum(intv(r.get("init_rejected_accepted")) for r in rows),
        fo=st.median(v) if (v := [x for x in (num(r.get("founding_offset_log")) for r in u) if x == x])
        else float("nan"),
        fps=st.median(v) if (v := [x for x in (num(r.get("postinit_fps")) for r in u) if x == x])
        else float("nan"))


def f(x, p=3):
    return "--" if x != x else f"{x:.{p}f}"


# ------------------------------------------------------------------------------------------------ (d1)
def part_d1(root, ref_path):
    print("\n==== (d1) re-freeze no-regression: full (epoch 0155a0d2ab6932b4) vs R0-licensed reference ====")
    ref, new = load(str(ref_path), flag=False), load(str(Path(root) / "full"))
    bref = sorted({r.get("binary") or "?" for v in ref.values() for r in v})
    cref = sorted({r["commit"] for v in ref.values() for r in v})
    bnew = sorted({r.get("binary") or "?" for v in new.values() for r in v})
    print(f"  reference: {sum(len(v) for v in ref.values())} rows, commits {cref}, binary column {bref} "
          f"(pre-column rows: 692adec6141bda91 per R0)")
    print(f"  new      : {sum(len(v) for v in new.values())} rows, binary {bnew}")
    print(f"  {'':2s}{'sequence':34s} {'use new':>7s} {'use ref':>7s} {'ATE new':>8s} {'ref+IQR':>8s} (i) "
          f"{'s new':>6s} {'s ref':>6s} (ii) (iii)  guard  verdict")
    regs, unscored = [], []
    for k in SEQS:
        rr, nn = ref.get(k, []), new.get(k, [])
        c_ref, c_new = cell(rr), cell(nn)
        tag = "* " if k in NEVER_K14 else "  "
        name = f"{k[0]} {k[1]}"
        if c_ref["u"] < 5:
            unscored.append(k)
            print(f"  {tag}{name:34s} reference has {c_ref['u']} usable reps (< 5): not scored under (d1)"
                  f"{'  [KITTI 08/09 -> (d4prime)]' if k[1] in ('08', '09') else ''}")
            continue
        if not nn:
            regs.append((k, "no stage-2 rows"))
            print(f"  {tag}{name:34s} NO STAGE-2 ROWS -- cannot pass no-regression")
            continue
        bar = c_ref["ate"] + c_ref["ate_iqr"]
        i_reg = c_new["u"] > 0 and c_new["ate"] > bar
        s_lim = c_ref["lns"] + TOL
        ii_reg = c_new["u"] > 0 and c_new["lns"] > s_lim
        known = False
        if k == KNOWN_COST and ii_reg:
            known = c_new["lns"] <= math.log(KNOWN_COST_S) + TOL
            ii_reg = not known
        iii_reg = c_new["rate"] < R.BREAKAGE_CAND and c_ref["rate"] >= R.BREAKAGE_REF
        why = [w for w, b in (("(i) ATE", i_reg), ("(ii) scale", ii_reg), ("(iii) breakage", iii_reg)) if b]
        if why:
            regs.append((k, ", ".join(why)))
        yn = lambda b: "REG" if b else "ok "
        verdict = ("REGRESSION: " + ", ".join(why)) if why else ("known cost (ii)" if known else "no regression")
        print(f"  {tag}{name:34s} {c_new['u']:>3d}/{c_new['n']:<3d} {c_ref['u']:>3d}/{c_ref['n']:<3d} "
              f"{f(c_new['ate']):>8s} {f(bar):>8s} {yn(i_reg)} {f(c_new['s']):>6s} {f(c_ref['s']):>6s} "
              f"{'KC ' if known else yn(ii_reg)}  {yn(iii_reg)}  {c_new['guard']:>5d}  {verdict}")
        if c_new["u"] and c_ref["u"] and (c_new["med_lns"] > c_ref["med_lns"] + TOL) != (c_new["lns"] > s_lim):
            print(f"      note: (ii) would read differently on the median of |ln s| "
                  f"({f(c_new['med_lns'])} vs {f(c_ref['med_lns'])} + ln 1.10)")
        if k == KNOWN_COST:
            print(f"      fr2_desk known cost: s {f(c_ref['s'])} -> {f(c_new['s'])}; covered while |ln s| <= "
                  f"ln {KNOWN_COST_S} + ln 1.10 = {math.log(KNOWN_COST_S) + TOL:.3f} (s <= {KNOWN_COST_S * 1.1:.3f})")
        if c_new["rate"] < R.BREAKAGE_REF and not iii_reg:
            print(f"      note: {c_new['u']}/{c_new['n']} usable -- below the 80 % reliability line, not a breakage")
    print("  * = never measured with K14/K15 before this campaign (read with particular attention)")
    print("  guard = lc_sim3_guard firings summed over the arm's reps (D4 is active in full; R0 OUTCOME)")
    if regs:
        print(f"\n  (d1) OUTCOME: {len(regs)} REGRESSION(S) -> STOP AND REPORT (kickoff section 8; no automatic action)")
        for k, w in regs:
            print(f"     {k[0]} {k[1]}: {w}")
        return False
    print(f"\n  (d1) OUTCOME: no regression on {len(SEQS) - len(unscored)} scored sequences "
          f"(unscored: {', '.join(' '.join(k) for k in unscored) or 'none'})")
    return True


# --------------------------------------------------------------------------------------- (d2) / (d4)
def part_adopt(root):
    verdicts = {}
    for arm, (label, exempt) in CANDIDATES.items():
        print(f"\n==== ({label}) {arm} vs full -- adoption_rule.py, per dataset"
              f"{', exempt: ' + ', '.join(sorted(exempt)) if exempt else ', no exemption'} ====")
        stop = Path(root) / f"STOPPED_{arm}"
        if stop.exists():
            print(f"  arm STOPPED early: {stop.read_text().strip()} -- scored over the sequences it ran; "
                  f"its breakage fails the dataset by C2")
        ref_csv, cand_csv = str(Path(root) / "full" / "summary.csv"), str(Path(root) / arm / "summary.csv")
        res = R.evaluate([ref_csv], [cand_csv], exempt=exempt, datasets=["tum", "kitti"])
        rows = MT.load_rows([Path(c) for c in (ref_csv, cand_csv) if Path(c).exists()])
        parity = MT.coverage_parity(MT.aggregate(rows), ["full", arm])
        for ds_, (per, v) in sorted(res.items()):
            R.print_ledger(f"{label} {arm} -- {ds_}", per, v)
            print(f"  P5a coverage parity full vs {arm} (bar {MT.COVERAGE_PARITY:.2f}; flagged, never re-scored):")
            for (d, sq), pv in sorted(parity.items()):
                if d != ds_:
                    continue
                cov = "  ".join(f"{a}={c:.0%}" for a, c in sorted(pv["cov"].items()))
                print(f"    {'  ' if pv['valid'] else 'XX'} {sq:34s} ratio={f(pv['ratio'], 2)}  {cov}  {pv['note']}")
        missing = [d for d in ("tum", "kitti") if d not in res]
        passed = [d for d, (_, v) in res.items() if v["verdict"] == "PASS" and not v.get("thin")]
        thin = [d for d, (_, v) in res.items() if v["verdict"] == "PASS" and v.get("thin")]
        undecided = [d for d, (_, v) in res.items() if v["verdict"] == "NOT DECIDED"]
        adopted = len(passed) == 2
        print(f"\n  ({label}) {arm}: PASS on {passed or 'none'}"
              f"{'; THIN on ' + str(thin) + ' (cannot ship by itself)' if thin else ''}"
              f"{'; NOT DECIDED on ' + str(undecided) + ' -> STOP AND REPORT' if undecided else ''}"
              f"{'; no rows for ' + str(missing) if missing else ''}"
              f"  =>  {'ADOPTED' if adopted else 'NOT ADOPTED'} (needs PASS on both TUM and KITTI)")
        verdicts[arm] = dict(adopted=adopted, undecided=bool(undecided), thin=bool(thin))
    adopted = [a for a, v in verdicts.items() if v["adopted"]]
    print()
    if len(adopted) == 2:
        print("  OUTCOME: BOTH adopted -- their combination is unmeasured: STOP AND REPORT; neither enters PAPER_CONFIG")
    elif len(adopted) == 1:
        print(f"  OUTCOME: exactly one adopted ({adopted[0]}) -> its flag enters PAPER_CONFIG (arms.py); its "
              f"stage-2 rows are the paper's full rows")
    else:
        print("  OUTCOME: neither adopted -> PAPER_CONFIG stays `full`")
    if any(v["undecided"] or v["thin"] for v in verdicts.values()):
        print("  NOTE: a NOT DECIDED or THIN verdict above -> report it to the user with the outcome")
    return verdicts


# ------------------------------------------------------------------------------------------------ (d3)
def contrast(ref, mono):
    cset, conv, unrun = [], [], []
    for k in SEQS:
        if not ref.get(k) or not mono.get(k):
            unrun.append(k)
            continue
        cr, cm = cell(ref[k]), cell(mono[k])
        if cr["rate"] >= R.BREAKAGE_REF and cm["rate"] < R.BREAKAGE_CAND:
            cset.append(k)
        if cm["rate"] >= R.BREAKAGE_REF and cr["rate"] < R.BREAKAGE_CAND:
            conv.append(k)
    return cset, conv, unrun


def part_d3(root, shipped):
    print("\n==== (d3) reliability contrast: does it survive the matched initialisation bar? ====")
    arms = {a: load(str(Path(root) / a)) for a in ["full", "A0", "A0_tol"] + ([shipped] if shipped != "full" else [])}
    print(f"  reliable = usable >= {R.BREAKAGE_REF:.0%} of reps; fails = usable < {R.BREAKAGE_CAND:.0%} "
          f"(adoption rule C2 constants)")
    print(f"  {'sequence':34s} " + "  ".join(f"{a:>22s}" for a in arms))
    print(f"  {'':34s} " + "  ".join(f"{'use  cov  rst/vrd rej':>22s}" for _ in arms))
    for k in SEQS:
        cells = []
        for a, g in arms.items():
            if not g.get(k):
                cells.append(f"{'unrun':>22s}")
                continue
            c = cell(g[k])
            mark = "R" if c["rate"] >= R.BREAKAGE_REF else ("F" if c["rate"] < R.BREAKAGE_CAND else "-")
            cells.append(f"{mark} {c['u']:>2d}/{c['n']:<2d} {f(c['cov'], 2):>4s} "
                         f"{sum(c['resets']):>3d}/{sum(c['verdicts']):<3d} {c['rejacc']:>2d}")
        print(f"  {k[0] + ' ' + k[1]:34s} " + "  ".join(cells))
    print("  R reliable, F fails; cov = P5a frame coverage (median over usable reps); rst/vrd = init rebuilds / "
          "failure verdicts summed over reps; rej = runs that continued on a rejected init (usable, descriptive)")
    outcome = None
    for ref_arm in (["full"] + ([shipped] if shipped != "full" else [])):
        sets = {}
        for m in ("A0", "A0_tol"):
            sets[m] = contrast(arms[ref_arm], arms[m])
        ca, ct = len(sets["A0"][0]), len(sets["A0_tol"][0])
        print(f"\n  against {ref_arm}:")
        for m in ("A0", "A0_tol"):
            cs, conv, unrun = sets[m]
            print(f"    C({m}) = {len(cs)}: {', '.join(' '.join(k) for k in cs) or '-'}")
            print(f"      converse ({m} reliable, {ref_arm} fails) = {len(conv)}: "
                  f"{', '.join(' '.join(k) for k in conv) or '-'}"
                  f"{'   unrun: ' + ', '.join(' '.join(k) for k in unrun) if unrun else ''}")
        # O3 first: the three outcome conditions overlap when |C(A0_tol)| = 0 (scorer choice, fixed pre-data)
        if ct == 0:
            o = "O3 WITHDRAWN: the reliability claim is withdrawn and the threshold finding reported"
        elif ct >= ca - 1:
            o = (f"O1 SURVIVES: \"Held to the same initialisation bar as the full system, the monocular backbone "
                 f"still fails on {ct} of {len(SEQS)} sequences that the full system tracks.\"")
        else:
            o = (f"O2 PARTLY THE BAR: the paper states |C(A0_tol)| = {ct} only; the difference {ca - ct} is "
                 f"attributed to the stricter bar the backbone ships with (constant described with its anchor)")
        print(f"    |C(A0)| = {ca}, |C(A0_tol)| = {ct}  =>  {o}")
        if ref_arm == shipped:
            outcome = o
    return outcome


# ----------------------------------------------------------------------------------------------- (d4')
def part_k0809(root):
    print("\n==== (d4') KITTI 08 / 09, reported as measured (never run at this config before) ====")
    for sq in ("08", "09"):
        for a in ["full", "full_K13", "full_R3b_anchor", "A0", "A0_tol"]:
            g = load(str(Path(root) / a)).get(("kitti", sq), [])
            if not g:
                print(f"  kitti {sq} {a:16s} no rows")
                continue
            c = cell(g)
            print(f"  kitti {sq} {a:16s} usable {c['u']}/{c['n']}  ATE Sim3 {f(c['ate'])} (IQR {f(c['ate_iqr'])}) "
                  f"SE3 {f(c['se3'])}  s {f(c['s'])}  cov {f(c['cov'], 2)}  guard {c['guard']}  "
                  f"status {sorted({r['status'] for r in g})}")


# --------------------------------------------------------------------------------------- tables (descriptive)
ARMS = ["full", "full_K13", "full_R3b_anchor", "A0", "A0_tol"]


def part_tables(root):
    """RESULTS_LOG markdown: every arm x sequence. Descriptive only -- no verdict is computed here."""
    g = {a: load(str(Path(root) / a)) for a in ARMS}
    print("\n#### provenance")
    for a in ARMS:
        rows = [r for v in g[a].values() for r in v]
        if not rows:
            print(f"- `{a}`: no rows")
            continue
        clis = sorted({" ".join(t for t in r["cli"].split() if "/" not in t and not t.startswith("--files")
                                and not t.startswith("--calib") and not t.startswith("--gamma")
                                and not t.startswith("--vignette"))
                       for r in rows})
        print(f"- `{a}`: {len(rows)} rows; commit {sorted({r['commit'] for r in rows})}; binary "
              f"{sorted({r.get('binary') for r in rows})}; dirty {sorted({r['dirty'] for r in rows})}; "
              f"status {dict(sorted(((st_, sum(r['status'] == st_ for r in rows)) for st_ in {r['status'] for r in rows})))}")
        for c in clis[:4]:
            print(f"  - CLI (paths stripped; {len(clis)} distinct): `{c}`")
    hdr = "| sequence | " + " | ".join(ARMS) + " |"
    sep = "|---|" + "---|" * len(ARMS)
    for title, fn in (
            ("Sim(3) ATE [m], median (IQR) · usable k/n", lambda c: f"{f(c['ate'])} ({f(c['ate_iqr'])}) · {c['u']}/{c['n']}"),
            ("SE(3) ATE [m], median · scale s, median", lambda c: f"{f(c['se3'])} · {f(c['s'])}"),
            ("founding offset (log), median · post-init fps, median", lambda c: f"{f(c['fo'])} · {f(c['fps'], 1)}"),
            ("P5a frame coverage · init rebuilds/verdicts · rejected-init runs · lc_sim3_guard",
             lambda c: f"{f(c['cov'], 2)} · {sum(c['resets'])}/{sum(c['verdicts'])} · {c['rejacc']} · {c['guard']}")):
        print(f"\n#### {title}\n\n{hdr}\n{sep}")
        for k in SEQS:
            cells = [fn(cell(g[a][k])) if g[a].get(k) else "not run" for a in ARMS]
            print(f"| {k[0]} {k[1]} | " + " | ".join(cells) + " |")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", required=True)
    ap.add_argument("--ref", default="runs/wp2ii_eval-server/full")
    ap.add_argument("--part", default="all", choices=["all", "d1", "adopt", "d3", "k0809", "tables"])
    ap.add_argument("--shipped", default="full", help="d3: the arm the paper ships, if a feature was adopted")
    a = ap.parse_args()
    rc = 0
    if a.part in ("all", "d1"):
        rc |= 0 if part_d1(a.root, a.ref) else 1
    if a.part in ("all", "adopt"):
        part_adopt(a.root)
    if a.part in ("all", "d3"):
        part_d3(a.root, a.shipped)
    if a.part in ("all", "k0809"):
        part_k0809(a.root)
    if a.part == "tables":
        part_tables(a.root)
    return rc


if __name__ == "__main__":
    sys.exit(main())
