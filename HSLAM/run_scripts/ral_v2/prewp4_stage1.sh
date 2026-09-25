#!/usr/bin/env bash
# Pre-WP4 stage 1 (wp/PRE_WP4_KICKOFF.md §4; DECISIONS.md "PRE-WP4 STAGE 0 -- PRE-REGISTRATIONS" (b), (b'), (c)).
#   Part A, R0: arm full_6799be6, n=5 on KITTI 07, KITTI 00, fr1_room, on the new binary (the epoch) AND the
#     preserved old binary 692adec6141bda91 (b'), interleaved one rep at a time in ABBA order so a slow drift
#     in the machine lands on both. Then prewp4_r0_check.py; any R0 failure stops the campaign here.
#   Part B, R3b screen: arms full, full_R3b_relin, full_R3b_anchor, n=3, sequence-major, hazard-first.
# The whole stage holds /tmp/wp7_gpu.lock (the WP7 B6 agents take it before any GPU job), and every run
# first waits for an empty GPU. Launch from a detached, non-interactive tmux session so the environment
# is pinned (R0 (b')):
#   tmux new-session -d -s prewp4_s1 \
#     "env -u LD_LIBRARY_PATH bash run_scripts/ral_v2/prewp4_stage1.sh > ~/prewp4_s1.log 2>&1"
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
source run_scripts/ral_v2/campaign_lib.sh

EPOCH=0155a0d2ab6932b4
OLD_EPOCH=692adec6141bda91
OLD_BIN="$HOME/Dev/baselines/_ws/hslam2_b6/bin/HSLAM_$OLD_EPOCH"
ROOT="${OUT:-runs/prewp4_s1_$(hostname -s)}"
REF=runs/wp2ii_eval-server/full/summary.csv

echo "=== launch env ==="; env | sort; echo "=== end env ==="
campaign_preflight "$EPOCH" init-founding-fix init-fail-thresholds
[ "$(sha256sum "$OLD_BIN" | cut -c1-16)" = "$OLD_EPOCH" ] || { echo "ERROR: old binary is not $OLD_EPOCH"; exit 7; }
[ -f "$REF" ] || { echo "ERROR: R0 reference rows missing: $REF"; exit 2; }

exec 9>/tmp/wp7_gpu.lock
echo "waiting for /tmp/wp7_gpu.lock $(date +%H:%M:%S)"
flock 9
echo "GPU lock held $(date +%H:%M:%S)"

# gpu_quiet: wait until no compute process is on the GPU (another job would contaminate R0 (b')).
gpu_quiet() {
  local apps
  while apps="$(nvidia-smi --query-compute-apps=pid,process_name --format=csv,noheader)"; [ -n "$apps" ]; do
    echo "$(date +%H:%M:%S) GPU busy, waiting: $apps"; sleep 30
  done
}
r0_new() { gpu_quiet; campaign_run "$ROOT/r0" full_6799be6 "$1" "$2" "$3"; }
r0_old() { gpu_quiet; ( export HSLAM_BINARY="$OLD_BIN"; campaign_run "$ROOT/r0_oldbin" full_6799be6 "$1" "$2" "$3" ); }

echo "##### Part A: R0 $(date)"
for dsq in "kitti 07" "kitti 00" "tum freiburg1_room"; do
  set -- $dsq
  for rep in 1 2 3 4 5; do
    if [ $((rep % 2)) -eq 1 ]; then r0_new "$1" "$2" "$rep"; r0_old "$1" "$2" "$rep"
    else                            r0_old "$1" "$2" "$rep"; r0_new "$1" "$2" "$rep"; fi
  done
done
echo "##### R0 check $(date)"
if ! "$PY" run_scripts/ral_v2/prewp4_r0_check.py --ref "$REF" \
       --new "$ROOT/r0/full_6799be6/summary.csv" --old "$ROOT/r0_oldbin/full_6799be6/summary.csv"; then
  echo "##### R0 NOT PASSED -- stage 1 stops here (card §8: stop and report) $(date)"
  exit 10
fi

echo "##### Part B: R3b screen $(date)"
for dsq in "kitti 04" "kitti 03" "kitti 06" "tum freiburg3_long_office_household" "tum freiburg2_desk" \
           "kitti 05" "kitti 07" "tum freiburg1_room" "kitti 02"; do
  set -- $dsq
  for arm in full full_R3b_relin full_R3b_anchor; do
    gpu_quiet; campaign_run "$ROOT/r3b" "$arm" "$1" "$2" 3
  done
done
echo "##### stage 1 done $(date)"
