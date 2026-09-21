#!/usr/bin/env bash
# WP2a-R5, condition-4 escalation -- KITTI 07 track success judged at n=10 for BOTH arms, symmetrically.
# Pre-registered in DECISIONS.md ("WP2a-R5 -- a latent KITTI 07 hazard found mid-campaign, and the
# condition-4 amendment", 2026-09-21 14:40), written before the campaign finished and before any verdict
# was computed. Reason: five of 25 KITTI 07 reps run on 21 Sep lost tracking at the SAME event (a delivery
# truck crossing close in front of the camera, frames 000634-000644), across arms and across both binaries.
# A hazard every arm shares cannot be read as an arm effect at n=5, so both arms go to n=10 and the
# comparison ("candidate not worse than the reference") is unchanged.
#
# Rows land in <arm>_esc/ and are pooled with <arm>/ by wp2a_r5_rule.py, which already reads that suffix.
# Run ONLY after the main campaign's tmux session has ended (one campaign at a time: fps and timing must
# not be contaminated).
#   tmux new -s wp2a_r5_esc;  bash run_scripts/ral_v2/wp2a_r5_esc.sh 2>&1 | tee ~/wp2a_r5_esc.log
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
REPS="${REPS:-5}"
ROOT="${OUT:-runs/wp2a_r5b_$(hostname -s)}"
PY=python3
[ -x "$HOME/Dev/evo/evo_env/bin/python" ] && PY="$HOME/Dev/evo/evo_env/bin/python"
"$PY" -c 'import evo' 2>/dev/null || { echo "ERROR: no python with evo"; exit 2; }
if [ -n "$(git status --porcelain)" ]; then echo "ERROR: dirty tree"; git status --short | head; exit 3; fi
while tmux has-session -t "=wp2a_r5" 2>/dev/null; do echo "$(date) waiting for the main campaign to end"; sleep 120; done
echo "host $(hostname)  commit $(git rev-parse --short HEAD)  binary $(sha256sum build/bin/HSLAM | cut -c1-16)  reps $REPS  out $ROOT/<arm>_esc/  start $(date)"
# SEQS: the sequences the pre-registered escalation nominates. KITTI 07 comes from the condition-4
# amendment (shared truck hazard, 14:40). TUM freiburg1_desk comes from the boundary clause: TUM landed at
# exactly 3/5 and fr1_desk is the sequence that decides that threshold, inside the reference IQR
# (DECISIONS.md, "WP2a-R5 -- the n=5 reading", 16:00). Two sequences = the pre-registered cap.
SEQS="${SEQS:-kitti:07 tum:freiburg1_desk}"
for arm in K14_K15 K13_K14_K15; do
  for spec in $SEQS; do
    ds="${spec%%:*}"; sq="${spec##*:}"
    echo "=== esc arm=$arm $ds $sq (n=$REPS)  $(date +%H:%M) ==="
    "$PY" run_scripts/ral_v2/eval_run.py --dataset "$ds" --sequence "$sq" --arm "$arm" --reps "$REPS" --out "$ROOT/${arm}_esc" \
      || echo "!!! run failed: $arm $ds $sq (continuing)"
  done
done
echo "=== WP2a-R5 escalation done: $(date) ==="
"$PY" run_scripts/ral_v2/wp2a_r5_rule.py --root "$ROOT" --ref K14_K15 --cand K13_K14_K15 || true
