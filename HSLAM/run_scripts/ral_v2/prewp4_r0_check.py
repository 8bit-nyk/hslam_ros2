#!/usr/bin/env python3
"""Pre-WP4 stage 1, R0 at the re-freeze epoch, with the (b') environment control.

Pre-registered in DECISIONS.md "PRE-WP4 STAGE 0 -- PRE-REGISTRATIONS" (b) and (b'), applied as written:
  P4c (a) DECISIVE  median scale_s within +/-10 % of the reference median on EACH of fr1_room, KITTI 07, 00.
  P4c (b) REPORTED  Mann-Whitney p and range overlap on Sim(3) ATE; vetoes only if p < 0.05 AND disjoint.
  (b') reading, P4c (a) as the bar for each pair:
      old binary now vs reference fails            -> ENVIRONMENT DRIFT  (exit 2; nothing licensed)
      old passes, new binary (with (b) veto) fails  -> BINARY CHANGE      (exit 1)
      both pass                                     -> LICENSED           (exit 0)

Usable = status OK and track_success, where track_success is re-pooled over ALL of a (binary, sequence)'s
reps with eval_run.apply_track_success. stage 1 interleaves the two binaries one rep at a time, so each
eval_run invocation holds one rep and its own stamp degenerates to the absolute P5a floor; the
protocol's definition pools reps run together, which these were. Any row where the two disagree is
printed.

Usage: prewp4_r0_check.py --ref runs/wp2ii_eval-server/full/summary.csv \
         --new <root>/r0/full_6799be6/summary.csv --old <root>/r0_oldbin/full_6799be6/summary.csv
"""
import argparse
import csv
import statistics as st
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from adoption_rule import mannwhitney_p          # noqa: E402
from eval_run import apply_track_success         # noqa: E402

SEQS = [("kitti", "07"), ("kitti", "00"), ("tum", "freiburg1_room")]
SCALE_TOL = 0.10          # P4c (a)
VETO_P = 0.05             # P4c (b)


def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def _int(v):
    x = _num(v)
    return int(x) if x == x else 0


def load(path, repool):
    rows = list(csv.DictReader(open(path)))
    if not repool:                     # the reference rows keep their own (n=5/10, pooled) stamps
        for r in rows:
            r["_usable"] = r["status"] == "OK" and r["track_success"] == "1"
        return rows
    typed = [dict(r, poses=_int(r["poses"]), frames=_int(r["frames"])) for r in rows]
    apply_track_success(typed)
    for r, t in zip(rows, typed):
        if str(t["track_success"]) != r["track_success"]:
            print(f"  note: {Path(path).parent.name} {r['sequence']} rep{r['rep']}: track_success "
                  f"{r['track_success']} per invocation -> {t['track_success']} pooled")
        r["_usable"] = r["status"] == "OK" and t["track_success"] == 1
    return rows


def med(rows, key):
    v = [_num(r.get(key)) for r in rows]
    v = [x for x in v if x == x]
    return (st.median(v), v) if v else (float("nan"), [])


def compare(label, rows, ref, sq, veto):
    g = [r for r in rows if r["sequence"] == sq]
    u = [r for r in g if r["_usable"]]
    rr = [r for r in ref if r["sequence"] == sq and r["_usable"]]
    s, _ = med(u, "scale_s")
    s_ref, _ = med(rr, "scale_s")
    _, ate = med(u, "ate_sim3_rmse")
    _, ate_ref = med(rr, "ate_sim3_rmse")
    dev = s / s_ref - 1 if u else float("nan")
    a_ok = bool(u) and abs(dev) <= SCALE_TOL
    p = mannwhitney_p(ate, ate_ref) if ate and ate_ref else float("nan")
    disjoint = bool(ate and ate_ref) and (max(ate) < min(ate_ref) or min(ate) > max(ate_ref))
    vetoed = p < VETO_P and disjoint
    ok = a_ok and not (veto and vetoed)
    gpu = sorted(_int(r.get("peak_gpu_mb")) for r in g)
    print(f"  {label:4s} {sq:15s} usable {len(u)}/{len(g)}  s {s:.4f} vs ref {s_ref:.4f} ({100*dev:+.1f} %) "
          f"{'within' if a_ok else 'OUTSIDE'} +/-10 %")
    if ate:
        print(f"       ATE Sim(3) med {st.median(ate):.4f} [{min(ate):.4f}, {max(ate):.4f}] vs ref "
              f"med {st.median(ate_ref):.4f} [{min(ate_ref):.4f}, {max(ate_ref):.4f}]  MW p {p:.3f}"
              f"{'  DISJOINT' if disjoint else ''}  -> {'VETO' if vetoed else 'no veto'}"
              f"{'' if veto else ' (reported only)'}")
    print(f"       GPU peak MB {gpu}  |  init_mode {sorted({r.get('init_mode', '') for r in g})}  "
          f"lc_sim3_guard {[r.get('lc_sim3_guard', '') for r in g]}  "
          f"init_rejected_accepted {sum(_int(r.get('init_rejected_accepted')) for r in g)}  "
          f"postinit_fps med {med(u, 'postinit_fps')[0]:.1f}")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", required=True)
    ap.add_argument("--new", required=True)
    ap.add_argument("--old", required=True)
    a = ap.parse_args()
    ref = load(a.ref, repool=False)
    new = load(a.new, repool=True)
    old = load(a.old, repool=True)
    for name, rows in (("new", new), ("old", old)):
        print(f"  {name}: binary {sorted({r.get('binary', '?') for r in rows})}, commit "
              f"{sorted({r['commit'] for r in rows})}, {len(rows)} rows")
    new_ok = old_ok = True
    for _, sq in SEQS:
        old_ok &= compare("old", old, ref, sq, veto=False)
        new_ok &= compare("new", new, ref, sq, veto=True)
    if not old_ok:
        print("  R0 (b') ENVIRONMENT DRIFT: the old binary no longer reproduces its own rows -- nothing licensed")
        return 2
    if not new_ok:
        print("  R0 FAILED: BINARY CHANGE (the old binary reproduces, the new one does not)")
        return 1
    print("  R0 PASSED: both binaries reproduce the reference; wp2ii_eval-server/full licensed for stage 2")
    return 0


if __name__ == "__main__":
    sys.exit(main())
