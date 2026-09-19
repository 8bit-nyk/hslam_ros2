#!/usr/bin/env bash
# WP3e/WP3f -- init-scale fix, its combinations, the explicit prior in place of the hidden pull, the
# blend fix, and the TUM mono-VO ML runs (H7). Pre-registered in DECISIONS.md (WP3e-1, WP3e-2, WP3f).
# Runs at the commit adding --ml-init-scale / --p1-blend-grad-fix; the binary must be rebuilt first.
#   tmux new -s wp3f;  REPS=3 bash run_scripts/ral_v2/wp3f_campaign.sh 2>&1 | tee ~/wp3f.log
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
REPS="${REPS:-3}"
ROOT="${OUT:-runs/wp3f_$(hostname -s)}"
PY=python3
[ -x "$HOME/Dev/evo/evo_env/bin/python" ] && PY="$HOME/Dev/evo/evo_env/bin/python"
"$PY" -c 'import evo' 2>/dev/null || { echo "ERROR: no python with evo"; exit 2; }
if [ -n "$(git status --porcelain)" ]; then echo "ERROR: dirty tree"; git status --short | head; exit 3; fi
./build/bin/HSLAM --help 2>&1 | grep -q "ml-init-scale" || { echo "ERROR: binary has no --ml-init-scale; rebuild first"; exit 5; }
./build/bin/HSLAM --help 2>&1 | grep -q "p1-blend-grad-fix" || { echo "ERROR: binary has no --p1-blend-grad-fix; rebuild first"; exit 5; }
echo "host $(hostname)  commit $(git rev-parse --short HEAD)  binary $(sha256sum build/bin/HSLAM | cut -c1-16)  reps $REPS  out $ROOT/<arm>/"
run() { echo "=== arm=$1  $2 $3  (n=$4) ==="; "$PY" run_scripts/ral_v2/eval_run.py --dataset "$2" --sequence "$3" --arm "$1" --reps "$4" --out "$ROOT/$1" || echo "!!! run failed: $1 $2 $3 (continuing)"; echo; }

# R0 at this binary (source change; default path unchanged)
run full tum freiburg1_room 5
"$PY" - "$ROOT/full/summary.csv" <<'PYEOF' || { echo "R0 FAILED -- stop (DECISIONS.md)"; exit 6; }
import csv, statistics as st, sys
rows = [r for r in csv.DictReader(open(sys.argv[1])) if r["status"] == "OK" and r["track_success"] == "1" and r["sequence"] == "freiburg1_room"]
REF = {"ate_sim3_rmse": 0.3072, "scale_s": 1.1282}; ok = True
for f, ref in REF.items():
    got = st.median(float(r[f]) for r in rows); dev = got / ref - 1; hit = abs(dev) <= 0.05; ok &= hit
    print(f"  R0 {f:14s} {got:.4f} ref {ref:.4f} {100*dev:+.2f}% {'within' if hit else 'OUTSIDE'} +/-5%")
sys.exit(0 if ok else 1)
PYEOF

SEQ5() { for s in MH_01_easy V1_01_easy V2_02_medium; do run "$1" euroc "$s" "$REPS"; done; run "$1" tum freiburg1_room "$REPS"; run "$1" kitti 07 "$REPS"; }
# most informative first
SEQ5 K14_init_median
SEQ5 K12_K14
SEQ5 K1_K12_K14
# H7: the camera-class test, both arms at this binary (the domain campaign lost its ML arm to an ICL crash)
run A0   tummonovo sequence_31 "$REPS"
run full tummonovo sequence_31 "$REPS"
SEQ5 K12_P2
SEQ5 K12_K14_P2
SEQ5 K15_blendfix
SEQ5 K12_K13_K14
echo "=== WP3f done: $(date) ==="
