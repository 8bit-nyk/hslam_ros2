#!/usr/bin/env bash
# WP2a -- integration hygiene on ABL-10 at n=5 (+ EuRoC-3 and TUM mono-VO at n=3, information only).
# Pre-registered in DECISIONS.md ("WP2a -- integration hygiene -- PRE-REGISTERED 2026-09-19") before
# the first run. Runs at the commit adding [INIT_FOUNDING_DIAG] and the K13_K14_K15 arm; the binary
# must be rebuilt first (new epoch => R0 first, embedded below).
#   tmux new -s wp2a;  WAIT_FOR=wp6mono REPS=5 bash run_scripts/ral_v2/wp2a_campaign.sh 2>&1 | tee ~/wp2a.log
# WAIT_FOR=<tmux session> makes it wait for that session to end before starting (the WP6 monocular
# half must not be disturbed and its fps must not be contaminated). One --out per arm.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
REPS="${REPS:-5}"; EREPS="${EREPS:-3}"
ROOT="${OUT:-runs/wp2a_$(hostname -s)}"
PY=python3
[ -x "$HOME/Dev/evo/evo_env/bin/python" ] && PY="$HOME/Dev/evo/evo_env/bin/python"
"$PY" -c 'import evo' 2>/dev/null || { echo "ERROR: no python with evo"; exit 2; }
if [ -n "$(git status --porcelain)" ]; then echo "ERROR: dirty tree"; git status --short | head; exit 3; fi
grep -q "INIT_FOUNDING_DIAG" build/bin/HSLAM || { echo "ERROR: binary has no [INIT_FOUNDING_DIAG]; rebuild first"; exit 5; }
for f in ml-prior-source ml-init-scale p1-blend-grad-fix ml-fej-freeze; do
  ./build/bin/HSLAM --help 2>&1 | grep -q -- "--$f" || { echo "ERROR: binary lacks --$f; rebuild first"; exit 5; }
done
if [ -n "${WAIT_FOR:-}" ]; then
  while tmux has-session -t "=$WAIT_FOR" 2>/dev/null; do echo "$(date) waiting for tmux session '$WAIT_FOR' to end"; sleep 300; done
fi
echo "host $(hostname)  commit $(git rev-parse --short HEAD)  binary $(sha256sum build/bin/HSLAM | cut -c1-16)  reps $REPS/$EREPS  out $ROOT/<arm>/  start $(date)"
run() { echo "=== arm=$1  $2 $3  (n=$4)  $(date +%H:%M) ==="; "$PY" run_scripts/ral_v2/eval_run.py --dataset "$2" --sequence "$3" --arm "$1" --reps "$4" --out "$ROOT/$1" || echo "!!! run failed: $1 $2 $3 (continuing)"; echo; }

# --- R0 at this binary (source change: one diagnostic printf; default path otherwise unchanged) ---
run full tum freiburg1_room "$REPS"
run full kitti 07 "$REPS"
if ! "$PY" run_scripts/ral_v2/r0_check.py --csv "$ROOT/full/summary.csv" --require freiburg1_room 07; then
  # Pre-registered alternative (user decision 5, 19 Sep): KITTI 07 may be judged at n >= 10 instead.
  echo "R0 not met at n=$REPS -- extending fr1_room and KITTI 07 by 5 reps each (separate --out), judging on the pooled rows"
  "$PY" run_scripts/ral_v2/eval_run.py --dataset tum --sequence freiburg1_room --arm full --reps 5 --out "$ROOT/full_r0ext" || true
  "$PY" run_scripts/ral_v2/eval_run.py --dataset kitti --sequence 07 --arm full --reps 5 --out "$ROOT/full_r0ext" || true
  "$PY" run_scripts/ral_v2/r0_check.py --csv "$ROOT/full/summary.csv" "$ROOT/full_r0ext/summary.csv" --require freiburg1_room 07 \
    || { echo "R0 FAILED at n=10 -- stop (DECISIONS.md WP2a)"; exit 6; }
fi

ABL8()   { for s in freiburg1_desk freiburg2_desk freiburg2_large_no_loop freiburg3_long_office_household; do run "$1" tum "$s" "$REPS"; done
           for s in 00 05 06 10; do run "$1" kitti "$s" "$REPS"; done; }
ABL10()  { run "$1" tum freiburg1_room "$REPS"; run "$1" kitti 07 "$REPS"; ABL8 "$1"; }
EUROC3() { for s in MH_01_easy V1_01_easy V2_02_medium; do run "$1" euroc "$s" "$EREPS"; done; }

ABL8  full                 # completes the G2-b reference (fr1_room + 07 are the R0 rows above)
ABL10 K13_K14_K15          # the candidate hygiene config: most informative first
ABL10 K13_fresh
ABL10 K15_blendfix
ABL10 K14_init_median
# information only (no rule): EuRoC-3 (comparator = WP3 A0) and the camera-class sequence
EUROC3 K13_K14_K15; EUROC3 K13_fresh; EUROC3 K15_blendfix; EUROC3 K14_init_median
run K13_K14_K15 tummonovo sequence_31 "$EREPS"
run full        tummonovo sequence_31 "$EREPS"
echo "=== WP2a done: $(date) ==="
"$PY" run_scripts/ral_v2/wp2a_summary.py --root "$ROOT" --ref full || true
