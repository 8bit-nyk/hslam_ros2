#!/usr/bin/env bash
# WP3b part 3 -- the camera-class test and the in-distribution control (H7 in DECISIONS.md).
#
#   tummonovo sequence_31   wide-FOV global-shutter grayscale handheld camera with photometric
#                           calibration; shares EuRoC's camera CLASS but not its MAV motion or
#                           Vicon-room content. GT covers only the mocap start/end segments, so
#                           Sim(3) ATE here is the classic mono-VO alignment (drift) error.
#   iclnuim living_room_traj0   synthetic indoor, in-distribution static control.
#
# Runs after wp3b_campaign.sh (separate session so the EuRoC rows keep one commit).
#   tmux new -s wp3b_domain
#   REPS=3 bash run_scripts/ral_v2/wp3b_domain_campaign.sh 2>&1 | tee ~/wp3b_domain.log
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."          # -> HSLAM/
REPS="${REPS:-3}"
ROOT="${OUT:-runs/wp3b_domain_$(hostname -s)}"
PY=python3
[ -x "$HOME/Dev/evo/evo_env/bin/python" ] && PY="$HOME/Dev/evo/evo_env/bin/python"
"$PY" -c 'import evo' 2>/dev/null || { echo "ERROR: no python with evo"; exit 2; }
if [ -n "$(git status --porcelain)" ]; then echo "ERROR: dirty tree"; git status --short | head; exit 3; fi

echo "host $(hostname)  commit $(git rev-parse --short HEAD)  binary $(sha256sum build/bin/HSLAM | cut -c1-16)  reps $REPS  out $ROOT/<arm>/"
run() { echo "=== arm=$1  $2 $3  (n=$4) ==="; "$PY" run_scripts/ral_v2/eval_run.py --dataset "$2" --sequence "$3" --arm "$1" --reps "$4" --out "$ROOT/$1"; echo; }

for arm in A0 full; do
    run "$arm" tummonovo sequence_31 "$REPS"
    run "$arm" iclnuim living_room_traj0 "$REPS"
done
echo "=== WP3b domain done: $(date) ==="
