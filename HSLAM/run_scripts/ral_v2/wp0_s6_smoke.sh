#!/usr/bin/env bash
# WP0-S6 — the smoke that gates G0.
#
# Pre-registered rule (docs/ral_v2_resubmission/wp/WP0_hygiene_and_freeze.md S6): at the
# provisional paper configuration, n=5, the corrected geometry must reproduce the Sprint 11
# post-fix scale within IQR:
#
#     TUM fr1_room   median Sim(3) scale s in [1.09, 1.14]
#     KITTI 07       median Sim(3) scale s in [0.91, 0.95]
#
# This is a REPRODUCTION check on scale, not an fps gate and not an accuracy claim. If it
# fails, the corrected-geometry pipeline does not reproduce its own recorded result and
# nothing downstream (WP1, G1, the frozen config) may proceed.
#
# Run it on the eval server, in tmux, so it survives a disconnect:
#     tmux new -s s6
#     bash run_scripts/ral_v2/wp0_s6_smoke.sh 2>&1 | tee ~/wp0_s6.log
#     # Ctrl-b d to detach
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/../.."          # -> HSLAM/
OUT="${1:-runs/wp0_s6}"
REPS="${REPS:-5}"

# evo lives in its own venv on the eval server; the system python3 there has neither evo nor
# numpy (WORKFLOW.md gotcha 4). Prefer the venv, fall back to whatever python3 has evo.
PY=python3
if [ -x "$HOME/Dev/evo/evo_env/bin/python" ]; then
    PY="$HOME/Dev/evo/evo_env/bin/python"
fi
"$PY" -c 'import evo' 2>/dev/null || { echo "ERROR: no python with evo on PATH"; exit 2; }

echo "host      : $(hostname)"
echo "commit    : $(git rev-parse --short HEAD)$( [ -n "$(git status --porcelain)" ] && echo ' (DIRTY)')"
echo "python    : $PY ($("$PY" -c 'import evo;print(evo.__version__)'))"
echo "gpu       : $(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null || echo 'NO GPU -- driver mismatch?')"
echo "reps      : $REPS"
echo "out       : $OUT"
echo

for seq in "tum freiburg1_room" "kitti 07"; do
    set -- $seq
    echo "=== $1 $2 (arm=full, n=$REPS) ==="
    "$PY" run_scripts/ral_v2/eval_run.py \
        --dataset "$1" --sequence "$2" --arm full --reps "$REPS" --out "$OUT"
    echo
done

echo "=== S6 verdict ==="
"$PY" - "$OUT/summary.csv" <<'PYEOF'
import csv, statistics, sys
BANDS = {("tum", "freiburg1_room"): (1.09, 1.14), ("kitti", "07"): (0.91, 0.95)}
rows = list(csv.DictReader(open(sys.argv[1])))
allpass = True
for key, (lo, hi) in BANDS.items():
    ok = [r for r in rows if (r["dataset"], r["sequence"]) == key
          and r["status"] == "OK" and r["track_success"] == "1"]
    if not ok:
        print(f"  {key[0]} {key[1]:16s} NO USABLE RUNS -> FAIL")
        allpass = False
        continue
    s = sorted(float(r["scale_s"]) for r in ok)
    med = statistics.median(s)
    q1, q3 = s[len(s) // 4], s[max(0, (3 * len(s)) // 4 - (len(s) % 2 == 0))]
    hit = lo <= med <= hi
    allpass &= hit
    print(f"  {key[0]} {key[1]:16s} n={len(ok)}/{sum(1 for r in rows if (r['dataset'],r['sequence'])==key)} "
          f"median s={med:.4f} (IQR {q1:.4f}-{q3:.4f}) target [{lo}, {hi}] -> {'PASS' if hit else 'FAIL'}")
    ate = [float(r["ate_sim3_rmse"]) for r in ok]
    se3 = [float(r["ate_se3_rmse"]) for r in ok]
    fps = [float(r["pipeline_fps"]) for r in ok]
    print(f"     ATE_Sim3 {statistics.median(ate):.4f} m   ATE_SE3 {statistics.median(se3):.4f} m   "
          f"pipeline_fps {statistics.median(fps):.2f}")
print(f"\n  G0 scale reproduction: {'PASS' if allpass else 'FAIL'}")
sys.exit(0 if allpass else 1)
PYEOF
