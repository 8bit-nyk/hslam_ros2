#!/usr/bin/env bash
# WP3 -- EuRoC audit and re-test.
#
# Pre-registered in docs/ral_v2_resubmission/DECISIONS.md ("WP3 ... PRE-REGISTERED 2026-09-18").
# Read that before changing anything here. In short:
#
#   R0        reproduction check FIRST: arm=full, TUM fr1_room, n=5, against WP1's 0.3072 /
#             1.1282 at +/-5% (P4b). WP3 does not run if R0 fails -- the point of R0 is that
#             this epoch (3ff5389) claims to be diagnostic-only versus the G1-frozen 6799be6.
#   arms      A0 (mono), legacy_geom (v1 shipped geometry), full (frozen paper config)
#   seqs      MH_01_easy, V1_01_easy, V2_02_medium, n=5, camera-frame GT via T_BC
#   verdict   per sequence, full vs A0: scale in [0.8,1.25] AND ATE <= 1.5x mono -> geometry
#             was the cause; scale ok but ATE > 3x mono -> prior quality; else both.
#             Decided on >= 2 of 3 sequences. Track success counts as part of the verdict.
#
# The offline half is already closed (wp/WP3a_geometry_audit_offline.md): the anisotropy is
# manufactured by our own rectification, F2 is validated to 0.06%, and F1 without F2 on EuRoC
# is +29.5% wrong. What is NOT settled is whether corrected geometry actually rescues EuRoC --
# its failure is 12-19x mono, far larger than a 29.5% factor explains. That is this campaign.
#
# Usage:
#   tmux new -s wp3
#   REPS=5 bash run_scripts/ral_v2/wp3_campaign.sh 2>&1 | tee ~/wp3.log
#   SKIP_R0=1 REPS=5 bash run_scripts/ral_v2/wp3_campaign.sh     # only if R0 already passed
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/../.."          # -> HSLAM/
REPS="${REPS:-5}"
OUT="${OUT:-runs/wp3_$(hostname -s)}"
R0_OUT="${R0_OUT:-runs/wp3_r0_$(hostname -s)}"

PY=python3
[ -x "$HOME/Dev/evo/evo_env/bin/python" ] && PY="$HOME/Dev/evo/evo_env/bin/python"
"$PY" -c 'import evo' 2>/dev/null || { echo "ERROR: no python with evo"; exit 2; }

if [ -n "$(git status --porcelain)" ]; then
    echo "ERROR: working tree is dirty. WP3 rows must be reproducible from their commit."
    git status --short | head; exit 3
fi

# EuRoC ships no times.txt; without it the reader reads nanosecond filenames as seconds and
# every pose associates with the wrong GT pose. Refuse to start rather than discover it in the
# rows -- eval_run.py would catch it per row, but burning 45 runs to learn this is not a plan.
echo "=== EuRoC timestamp preparation ==="
"$PY" run_scripts/ral_v2/prepare_euroc.py --check || {
    echo "ERROR: EuRoC times.txt missing or wrong. Run: $PY run_scripts/ral_v2/prepare_euroc.py"
    exit 4
}
echo

echo "host   : $(hostname)"
echo "commit : $(git rev-parse --short HEAD)"
echo "gpu    : $(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null || echo NONE)"
echo "reps   : $REPS"
echo "out    : $OUT"
echo

# --------------------------------------------------------------- R0: reproduction check
if [ -z "${SKIP_R0:-}" ]; then
    echo "=== R0  reproduction check: arm=full TUM freiburg1_room (n=$REPS) ==="
    echo "    reference WP1 @6799be6: ATE Sim(3) 0.3072, scale 1.1282; tolerance +/-5% (P4b)"
    "$PY" run_scripts/ral_v2/eval_run.py \
        --dataset tum --sequence freiburg1_room --arm full --reps "$REPS" --out "$R0_OUT"

    "$PY" - "$R0_OUT/summary.csv" <<'PYEOF' || { echo; echo "R0 FAILED -- WP3 does not run. See DECISIONS.md."; exit 5; }
import csv, statistics as st, sys
rows = [r for r in csv.DictReader(open(sys.argv[1]))
        if r["status"] == "OK" and r["track_success"] == "1"]
if not rows:
    print("R0: no usable rows"); sys.exit(1)
REF = {"ate_sim3_rmse": 0.3072, "scale_s": 1.1282}
TOL = 0.05
ok = True
print(f"\n--- R0 vs WP1 @6799be6 (arm=full, n={len(rows)}) ---")
for field, ref in REF.items():
    got = st.median(float(r[field]) for r in rows)
    dev = got / ref - 1
    hit = abs(dev) <= TOL
    ok &= hit
    print(f"  {field:16s} {got:8.4f}  ref {ref:8.4f}  {100*dev:+6.2f}%  "
          f"{'within' if hit else 'OUTSIDE'} +/-{100*TOL:.0f}%")
print(f"  epoch 3ff5389 {'IS' if ok else 'is NOT'} reproduction-equivalent to 6799be6")
sys.exit(0 if ok else 1)
PYEOF
    echo
    echo "R0 PASSED -- epoch holds; WP1 and WP3 rows are poolable."
    echo
else
    echo "=== R0 SKIPPED (SKIP_R0 set) ==="; echo
fi

# --------------------------------------------------------------- WP3 proper
for arm in A0 legacy_geom full; do
    for seq in MH_01_easy V1_01_easy V2_02_medium; do
        echo "=== arm=$arm  euroc $seq  (n=$REPS) ==="
        "$PY" run_scripts/ral_v2/eval_run.py \
            --dataset euroc --sequence "$seq" --arm "$arm" --reps "$REPS" --out "$OUT"
        echo
    done
done

echo "=== WP3 summary ($(hostname -s)) ==="
"$PY" - "$OUT/summary.csv" <<'PYEOF'
import csv, statistics as st, sys

all_rows = list(csv.DictReader(open(sys.argv[1])))
rows = [r for r in all_rows if r["status"] == "OK" and r["track_success"] == "1"]
key = lambda r: (r["arm"], r["sequence"])
groups, attempts = {}, {}
for r in all_rows:
    attempts.setdefault(key(r), 0)
    attempts[key(r)] += 1
for r in rows:
    groups.setdefault(key(r), []).append(r)

SEQS = ["MH_01_easy", "V1_01_easy", "V2_02_medium"]
ARMS = ["A0", "legacy_geom", "full"]

print(f"\n{'arm':12s} {'seq':14s} {'ok/n':>6s} {'ATE_sim3':>9s} {'IQR':>7s} {'ATE_se3':>9s} "
      f"{'scale':>7s} {'drift%':>7s} {'fps':>6s}")
for arm in ARMS:
    for sq in SEQS:
        g = groups.get((arm, sq))
        n = attempts.get((arm, sq), 0)
        if not g:
            print(f"{arm:12s} {sq:14s} {'0/'+str(n):>6s}  -- no usable run --")
            continue
        med = lambda f: st.median(float(r[f]) for r in g)
        s = sorted(float(r["ate_sim3_rmse"]) for r in g)
        iqr = s[3*len(s)//4] - s[len(s)//4] if len(s) > 3 else float("nan")
        print(f"{arm:12s} {sq:14s} {str(len(g))+'/'+str(n):>6s} {med('ate_sim3_rmse'):9.4f} "
              f"{iqr:7.4f} {med('ate_se3_rmse'):9.4f} {med('scale_s'):7.4f} "
              f"{med('scale_drift_pct_per_100m'):7.2f} {med('pipeline_fps'):6.2f}")

# Pre-registered discriminator. Reported here; the verdict is written into DECISIONS.md.
print("\n--- WP3 discriminator (full vs A0, per sequence) ---")
print("    scale in [0.8,1.25] AND ATE <= 1.5x mono -> GEOMETRY")
print("    scale in [0.8,1.25] AND ATE  > 3.0x mono -> PRIOR QUALITY")
print("    anything else                            -> BOTH\n")
verdicts = []
for sq in SEQS:
    a, b = groups.get(("A0", sq)), groups.get(("full", sq))
    na, nb = attempts.get(("A0", sq), 0), attempts.get(("full", sq), 0)
    if not a or not b:
        print(f"  {sq:14s} NOT EVALUABLE (mono {len(a or [])}/{na}, full {len(b or [])}/{nb})")
        verdicts.append("NOT EVALUABLE")
        continue
    mono = st.median(float(r["ate_sim3_rmse"]) for r in a)
    ml   = st.median(float(r["ate_sim3_rmse"]) for r in b)
    sc   = st.median(float(r["scale_s"]) for r in b)
    sb   = sorted(float(r["ate_sim3_rmse"]) for r in b)
    iqr  = (sb[3*len(sb)//4] - sb[len(sb)//4]) if len(sb) > 3 else float("nan")
    ratio = ml / mono
    in_band = 0.8 <= sc <= 1.25
    # Track success is part of the verdict, not a footnote (DECISIONS.md).
    if len(b) < 3 <= len(a):
        v = "ML ARM FAILS TO TRACK"
    elif in_band and ratio <= 1.5:
        v = "GEOMETRY"
    elif in_band and ratio > 3.0:
        v = "PRIOR QUALITY"
    else:
        v = "BOTH"
    verdicts.append(v)
    print(f"  {sq:14s} mono {mono:8.4f}  full {ml:8.4f}  ratio {ratio:6.2f}x  "
          f"(full IQR {iqr:.4f})  scale {sc:.4f} {'in' if in_band else 'OUT OF'} band  "
          f"track {len(b)}/{nb} vs {len(a)}/{na}  -> {v}")

# Aggregation fixed before the run: >= 2 of 3, a 1-1-1 split reads as BOTH.
from collections import Counter
c = Counter(verdicts)
top, n = c.most_common(1)[0]
overall = top if n >= 2 else "BOTH"
print(f"\n  per-sequence: {verdicts}")
print(f"  OVERALL (>=2 of 3): {overall}")
print("  -> EuRoC-6 joins WP6" if overall == "GEOMETRY"
      else "  -> EuRoC stays a characterised limitation (EuRoC-3)")

# Does the fix move anything at all? legacy_geom is what makes this a measurement.
print("\n--- did corrected geometry move EuRoC? (full vs legacy_geom) ---")
for sq in SEQS:
    l, f = groups.get(("legacy_geom", sq)), groups.get(("full", sq))
    if not l or not f:
        print(f"  {sq:14s} n/a (legacy {len(l or [])}, full {len(f or [])})")
        continue
    la = st.median(float(r["ate_sim3_rmse"]) for r in l)
    fa = st.median(float(r["ate_sim3_rmse"]) for r in f)
    ls = st.median(float(r["scale_s"]) for r in l)
    fs = st.median(float(r["scale_s"]) for r in f)
    print(f"  {sq:14s} ATE {la:8.4f} -> {fa:8.4f} ({100*(fa/la-1):+6.1f}%)   "
          f"scale {ls:6.4f} -> {fs:6.4f}")
PYEOF
