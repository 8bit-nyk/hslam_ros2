#!/usr/bin/env bash
# WP2a-R2: K14 + K15 as one arm (DECISIONS.md "WP2a-R2", pre-registered 2026-09-20 11:30). Runs on the
# WP2a binary at commit 4fce1b6 without touching the repo: --arm K14_init_median --extra --p1-blend-grad-fix=true.
set -uo pipefail
cd ~/Dev/hslam_ros2_ws/src/HSLAM
PY=~/Dev/evo/evo_env/bin/python; OUT=runs/wp2a_eval-server/K14_K15
[ -z "$(git status --porcelain)" ] || { echo "dirty tree"; exit 3; }
echo "host $(hostname) commit $(git rev-parse --short HEAD) binary $(sha256sum build/bin/HSLAM | cut -c1-16) start $(date)"
run() { echo "=== arm=K14_K15  $1 $2  (n=$3)  $(date +%H:%M) ==="; "$PY" run_scripts/ral_v2/eval_run.py --dataset "$1" --sequence "$2" --arm K14_init_median --reps "$3" --out "$OUT" --extra --p1-blend-grad-fix=true || echo "!!! failed $1 $2"; echo; }
run tum freiburg1_room 5; run kitti 07 5
for s in freiburg1_desk freiburg2_desk freiburg2_large_no_loop freiburg3_long_office_household; do run tum $s 5; done
for s in 00 05 06 10; do run kitti $s 5; done
for s in MH_01_easy V1_01_easy V2_02_medium; do run euroc $s 3; done
run tummonovo sequence_31 3
echo "=== WP2a-R2 done: $(date) ==="
