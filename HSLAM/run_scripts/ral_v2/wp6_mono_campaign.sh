#!/usr/bin/env bash
# WP6 -- the monocular half of the full table: arm A0 on TUM-10 and KITTI-11, n=5 (PLAN.md WP6; the
# Full-arm half waits for G2). No GPU: the backbone runs monocular. KITTI 08/09 do not track at HEAD
# and are reported as failures, not dropped (P5).
#   tmux new -s wp6mono;  REPS=5 bash run_scripts/ral_v2/wp6_mono_campaign.sh 2>&1 | tee ~/wp6_mono.log
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
REPS="${REPS:-5}"
ROOT="${OUT:-runs/wp6_$(hostname -s)}"
PY=python3
[ -x "$HOME/Dev/evo/evo_env/bin/python" ] && PY="$HOME/Dev/evo/evo_env/bin/python"
"$PY" -c 'import evo' 2>/dev/null || { echo "ERROR: no python with evo"; exit 2; }
if [ -n "$(git status --porcelain)" ]; then echo "ERROR: dirty tree"; git status --short | head; exit 3; fi
echo "host $(hostname)  commit $(git rev-parse --short HEAD)  binary $(sha256sum build/bin/HSLAM | cut -c1-16)  reps $REPS  out $ROOT/A0/"
run() { echo "=== arm=A0  $1 $2  (n=$REPS) ==="; "$PY" run_scripts/ral_v2/eval_run.py --dataset "$1" --sequence "$2" --arm A0 --reps "$REPS" --out "$ROOT/A0" || echo "!!! failed: $1 $2 (continuing)"; echo; }
while read -r ds seq; do run "$ds" "$seq"; done < <("$PY" -c "
import sys; sys.path.insert(0,'run_scripts/ral_v2'); import datasets as ds
for d,s in ds.SETS['TUM-10'] + ds.SETS['KITTI-11']: print(d, s)")
echo "=== WP6 mono done: $(date) ==="
