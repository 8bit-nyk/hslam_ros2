#!/usr/bin/env bash
# WP2a-R5 -- confirmation of the three-fix hygiene combination under the dataset-level adoption rule
# (user rule, 2026-09-21). Pre-registered in DECISIONS.md ("WP2a-R5 -- PRE-REGISTERED 2026-09-21")
# before the first run. Runs BOTH arms fresh at ONE binary (never pool arms across binaries):
#   K14_K15        -- the adopted hygiene pair, the reference of the rule
#   K13_K14_K15    -- the candidate (adds the own-view prior --ml-prior-source=fresh)
# ABL-10 n=5 (decides) + EuRoC-3 and TUM mono-VO seq 31 n=3 (information only, no rule).
#   tmux new -s wp2a_r5;  bash run_scripts/ral_v2/wp2a_r5_campaign.sh 2>&1 | tee ~/wp2a_r5.log
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
REPS="${REPS:-5}"; EREPS="${EREPS:-3}"
ROOT="${OUT:-runs/wp2a_r5_$(hostname -s)}"
PY=python3
[ -x "$HOME/Dev/evo/evo_env/bin/python" ] && PY="$HOME/Dev/evo/evo_env/bin/python"
"$PY" -c 'import evo' 2>/dev/null || { echo "ERROR: no python with evo"; exit 2; }
if [ -n "$(git status --porcelain)" ]; then echo "ERROR: dirty tree"; git status --short | head; exit 3; fi
# The binary must be the WP2c epoch (no C++ change since): it carries the 2c flags at shipped defaults.
for f in ml-prior-source ml-init-scale p1-blend-grad-fix ml-prior-weight ml-align-gate; do
  ./build/bin/HSLAM --help 2>&1 | grep -q -- "--$f" || { echo "ERROR: binary lacks --$f; rebuild first"; exit 5; }
done
if [ -n "${WAIT_FOR:-}" ]; then
  while tmux has-session -t "=$WAIT_FOR" 2>/dev/null; do echo "$(date) waiting for tmux session '$WAIT_FOR'"; sleep 300; done
fi
echo "host $(hostname)  commit $(git rev-parse --short HEAD)  binary $(sha256sum build/bin/HSLAM | cut -c1-16)  reps $REPS/$EREPS  out $ROOT/<arm>/  start $(date)"
run() { echo "=== arm=$1  $2 $3  (n=$4)  $(date +%H:%M) ==="; "$PY" run_scripts/ral_v2/eval_run.py --dataset "$2" --sequence "$3" --arm "$1" --reps "$4" --out "$ROOT/$1" || echo "!!! run failed: $1 $2 $3 (continuing)"; echo; }

# --- R0 guard at this binary (no C++ change since the 20 Sep 13:06 pass; this re-asserts the epoch) ---
run full tum freiburg1_room "$REPS"
run full kitti 07 "$REPS"
"$PY" run_scripts/ral_v2/r0_check.py --csv "$ROOT/full/summary.csv" --require freiburg1_room 07 \
  || { echo "R0 FAILED -- stop (DECISIONS.md WP2a-R5)"; exit 6; }

ABL10() { run "$1" tum freiburg1_room "$REPS"; run "$1" kitti 07 "$REPS"
          for s in freiburg1_desk freiburg2_desk freiburg2_large_no_loop freiburg3_long_office_household; do run "$1" tum "$s" "$REPS"; done
          for s in 00 05 06 10; do run "$1" kitti "$s" "$REPS"; done; }
EUROC3(){ for s in MH_01_easy V1_01_easy V2_02_medium; do run "$1" euroc "$s" "$EREPS"; done; }

ABL10 K14_K15                 # the reference of the rule, first so a crash leaves it complete
ABL10 K13_K14_K15             # the candidate
EUROC3 K14_K15; EUROC3 K13_K14_K15
run K14_K15 tummonovo sequence_31 "$EREPS"; run K13_K14_K15 tummonovo sequence_31 "$EREPS"
echo "=== WP2a-R5 done: $(date) ==="
"$PY" run_scripts/ral_v2/wp2a_summary.py --root "$ROOT" --ref K14_K15 || true
