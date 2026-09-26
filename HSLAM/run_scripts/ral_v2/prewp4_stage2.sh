#!/usr/bin/env bash
# Pre-WP4 stage 2, the re-freeze campaign (wp/PRE_WP4_KICKOFF.md §5; DECISIONS.md "PRE-WP4 STAGE 0" (d) as
# amended by "PRE-WP4 STAGE 2 -- two USER DECISIONS"). 690 runs at epoch 0155a0d2ab6932b4:
#   full, full_K13, full_R3b_anchor : TUM n=5, KITTI n=10 (KITTI-11 incl. 08 and 09)
#   A0, A0_tol                      : n=5 everywhere
# Sequence-major, hazard-first, in (d)'s order; per sequence the arm order above. The only early stop is the
# adoption rule's C2 breakage band, for the two candidate arms: after each sequence adoption_rule.classify
# compares the candidate's rows with full's, and a breakage writes $ROOT/STOPPED_<arm> -- that arm is skipped
# for the rest of the campaign and never resumed. Resume-safe (campaign_run tops up only missing reps).
# Holds /tmp/wp7_gpu.lock throughout; waits for an empty GPU before every run. Launch as stage 1:
#   tmux new-session -d -s prewp4_s2 \
#     "env -u LD_LIBRARY_PATH bash run_scripts/ral_v2/prewp4_stage2.sh > ~/prewp4_s2.log 2>&1"
# DRY=1 prints the plan (arm, sequence, n) and runs nothing.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
source run_scripts/ral_v2/campaign_lib.sh

EPOCH=0155a0d2ab6932b4
ROOT="${OUT:-runs/prewp4_s2_$(hostname -s)}"
DRY="${DRY:-0}"
SEQS=("tum freiburg2_large_with_loop" "tum freiburg2_large_no_loop"
      "kitti 08" "kitti 09" "kitti 07" "kitti 01" "kitti 03" "kitti 04"
      "tum freiburg1_360" "tum freiburg1_desk" "tum freiburg1_desk2" "tum freiburg1_floor"
      "tum freiburg2_360_hemisphere" "tum freiburg2_desk" "tum freiburg3_long_office_household"
      "tum freiburg1_room" "kitti 06" "kitti 10" "kitti 05" "kitti 00" "kitti 02")
CANDIDATES=(full_K13 full_R3b_anchor)

gpu_quiet() {
  local apps
  while apps="$(nvidia-smi --query-compute-apps=pid,process_name --format=csv,noheader)"; [ -n "$apps" ]; do
    echo "$(date +%H:%M:%S) GPU busy, waiting: $apps"; sleep 30
  done
}

# breakage ARM DATASET SEQUENCE -> exit 0 iff adoption_rule.classify flags a C2 breakage vs full
breakage() {
  "$PY" - "$ROOT/full/summary.csv" "$ROOT/$1/summary.csv" "$2" "$3" <<'EOF'
import sys
sys.path.insert(0, "run_scripts/ral_v2")
import adoption_rule as R
ref, cand = R.load_reps([sys.argv[1]]), R.load_reps([sys.argv[2]])
k = (sys.argv[3], sys.argv[4])
if k not in ref or k not in cand:
    sys.exit(1)
c = R.classify(ref[k], cand[k])
print(f"    C2 check {k[0]} {k[1]}: cand {c['track_cand'][0]}/{c['track_cand'][1]} usable, "
      f"full {c['track_ref'][0]}/{c['track_ref'][1]} -> {'BREAKAGE' if c['breakage'] else 'ok'}")
sys.exit(0 if c["breakage"] else 1)
EOF
}

if [ "$DRY" = 1 ]; then
  total=0
  for dsq in "${SEQS[@]}"; do
    set -- $dsq
    for arm in full full_K13 full_R3b_anchor A0 A0_tol; do
      n=5; [ "$1" = kitti ] && [[ "$arm" == full* ]] && n=10
      echo "$arm $1 $2 n=$n"; total=$((total + n))
    done
  done
  echo "TOTAL $total runs"; exit 0
fi

echo "=== launch env ==="; env | sort; echo "=== end env ==="
campaign_preflight "$EPOCH" init-founding-fix init-fail-thresholds ml-prior-source
exec 9>/tmp/wp7_gpu.lock
echo "waiting for /tmp/wp7_gpu.lock $(date +%H:%M:%S)"
flock 9
echo "GPU lock held $(date +%H:%M:%S)"

for dsq in "${SEQS[@]}"; do
  set -- $dsq
  echo "##### $1 $2 $(date)"
  for arm in full full_K13 full_R3b_anchor A0 A0_tol; do
    if [ -e "$ROOT/STOPPED_$arm" ]; then echo "=== skip arm=$arm (stopped: $(cat "$ROOT/STOPPED_$arm")) ==="; continue; fi
    n=5; [ "$1" = kitti ] && [[ "$arm" == full* ]] && n=10
    gpu_quiet; campaign_run "$ROOT" "$arm" "$1" "$2" "$n"
  done
  for arm in "${CANDIDATES[@]}"; do
    [ -e "$ROOT/STOPPED_$arm" ] && continue
    if breakage "$arm" "$1" "$2"; then
      echo "C2 breakage on $1 $2 at $(date)" > "$ROOT/STOPPED_$arm"
      echo "!!!!! arm $arm STOPPED: C2 breakage on $1 $2 (adoption rule; never resumed)"
    fi
  done
done
echo "##### stage 2 done $(date)"
