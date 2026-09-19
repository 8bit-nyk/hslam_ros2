#!/usr/bin/env bash
# WP3c -- the ML linearisation freeze (--ml-fej-freeze). Pre-registered in DECISIONS.md
# ("WP3c ... PRE-REGISTERED 2026-09-19"). Runs at the commit that adds the flag; the binary MUST be
# rebuilt from it first (this is a source change, hence R0 before anything else).
#
#   tmux new -s wp3c
#   REPS=3 bash run_scripts/ral_v2/wp3c_campaign.sh 2>&1 | tee ~/wp3c.log
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."          # -> HSLAM/
REPS="${REPS:-3}"
ROOT="${OUT:-runs/wp3c_$(hostname -s)}"
PY=python3
[ -x "$HOME/Dev/evo/evo_env/bin/python" ] && PY="$HOME/Dev/evo/evo_env/bin/python"
"$PY" -c 'import evo' 2>/dev/null || { echo "ERROR: no python with evo"; exit 2; }
if [ -n "$(git status --porcelain)" ]; then echo "ERROR: dirty tree"; git status --short | head; exit 3; fi
# the binary must carry the flag
./build/bin/HSLAM --help 2>&1 | grep -q "ml-fej-freeze" || { echo "ERROR: binary has no --ml-fej-freeze; rebuild first"; exit 5; }

echo "host $(hostname)  commit $(git rev-parse --short HEAD)  binary $(sha256sum build/bin/HSLAM | cut -c1-16)  reps $REPS  out $ROOT/<arm>/"
run() { echo "=== arm=$1  $2 $3  (n=$4) ==="; "$PY" run_scripts/ral_v2/eval_run.py --dataset "$2" --sequence "$3" --arm "$1" --reps "$4" --out "$ROOT/$1"; echo; }

# R0: the default path is unchanged code -> must reproduce WP1 within +/-5 % (P4b)
run full tum freiburg1_room 5
"$PY" - "$ROOT/full/summary.csv" <<'PYEOF' || { echo "R0 FAILED -- WP3c stops here (DECISIONS.md rule 1)"; exit 6; }
import csv, statistics as st, sys
rows = [r for r in csv.DictReader(open(sys.argv[1])) if r["status"] == "OK" and r["track_success"] == "1" and r["sequence"] == "freiburg1_room"]
REF = {"ate_sim3_rmse": 0.3072, "scale_s": 1.1282}; ok = True
for f, ref in REF.items():
    got = st.median(float(r[f]) for r in rows); dev = got / ref - 1; hit = abs(dev) <= 0.05; ok &= hit
    print(f"  R0 {f:14s} {got:.4f} ref {ref:.4f} {100*dev:+.2f}% {'within' if hit else 'OUTSIDE'} +/-5%")
sys.exit(0 if ok else 1)
PYEOF

for seq in MH_01_easy V1_01_easy V2_02_medium; do run K12 euroc "$seq" "$REPS"; done
run K12 tum freiburg1_room "$REPS"
run K12 kitti 07 "$REPS"
for seq in MH_01_easy V1_01_easy V2_02_medium; do run K1_K12 euroc "$seq" "$REPS"; done

# ---------------------------------------------------------------- WP3d: the prior's source (same binary)
./build/bin/HSLAM --help 2>&1 | grep -q "ml-prior-source" || { echo "ERROR: binary has no --ml-prior-source"; exit 5; }
for arm in K13_fresh K13_fresh_n1 K12_K13 K12_K13_n1; do
    for seq in MH_01_easy V1_01_easy V2_02_medium; do run "$arm" euroc "$seq" "$REPS"; done
done
run K13_fresh tum freiburg1_room "$REPS"
run K13_fresh kitti 07 "$REPS"
run K12_K13 tum freiburg1_room "$REPS"
run K12_K13 kitti 07 "$REPS"
echo "=== WP3c+WP3d done: $(date) ==="
