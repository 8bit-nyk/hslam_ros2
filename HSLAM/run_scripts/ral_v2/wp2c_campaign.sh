#!/usr/bin/env bash
# WP2c -- the explicit weighted, gated prior in place of the linearisation freeze.
# Pre-registered in DECISIONS.md ("WP2c -- ... PRE-REGISTERED 2026-09-20"). Phases:
#   PHASE=i    weight x self-gate sweep on fr1_room + KITTI 07, n=3 (15 arms, 90 runs)   [default]
#   PHASE=seed --ml-seed variants of the phase-i pick (ARM=<pick>), same two sequences, n=3
#   PHASE=ii   the pick (ARM=<pick>) on ABL-10, n=3, no keyframe gate -> thr from [PRIOR_ALIGN]
#   PHASE=iii  the gated candidate (ARM=C_final) on EuRoC-3 + mono-VO, n=3 (screen)
#   PHASE=g2   C_final at n=5 on EuRoC-3 and ABL-10 (the G2 decision runs)
# New binary => R0 first (r0_check.py), embedded in PHASE=i. One --out per arm.
#   tmux new -s wp2c;  PHASE=i bash run_scripts/ral_v2/wp2c_campaign.sh 2>&1 | tee -a ~/wp2c.log
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
PHASE="${PHASE:-i}"; ARM="${ARM:-}"; REPS="${REPS:-3}"
ROOT="${OUT:-runs/wp2c_$(hostname -s)}"
PY=python3
[ -x "$HOME/Dev/evo/evo_env/bin/python" ] && PY="$HOME/Dev/evo/evo_env/bin/python"
"$PY" -c 'import evo' 2>/dev/null || { echo "ERROR: no python with evo"; exit 2; }
if [ -n "$(git status --porcelain)" ]; then echo "ERROR: dirty tree"; git status --short | head; exit 3; fi
for f in ml-prior-weight ml-prior-gate-k ml-align-gate ml-seed ml-fej-freeze ml-prior-source ml-init-scale p1-blend-grad-fix; do
  ./build/bin/HSLAM --help 2>&1 | grep -q -- "--$f" || { echo "ERROR: binary lacks --$f; rebuild first"; exit 5; }
done
if [ -n "${WAIT_FOR:-}" ]; then
  while tmux has-session -t "=$WAIT_FOR" 2>/dev/null; do echo "$(date) waiting for tmux session '$WAIT_FOR' to end"; sleep 300; done
fi
echo "host $(hostname)  commit $(git rev-parse --short HEAD)  binary $(sha256sum build/bin/HSLAM | cut -c1-16)  phase $PHASE  arm '$ARM'  reps $REPS  out $ROOT/<arm>/  start $(date)"
run() { echo "=== arm=$1  $2 $3  (n=$4)  $(date +%H:%M) ==="; "$PY" run_scripts/ral_v2/eval_run.py --dataset "$2" --sequence "$3" --arm "$1" --reps "$4" --out "$ROOT/$1" || echo "!!! run failed: $1 $2 $3 (continuing)"; echo; }
TWO()    { run "$1" tum freiburg1_room "$2"; run "$1" kitti 07 "$2"; }
ABL10()  { run "$1" tum freiburg1_room "$2"; run "$1" kitti 07 "$2"
           for s in freiburg1_desk freiburg2_desk freiburg2_large_no_loop freiburg3_long_office_household; do run "$1" tum "$s" "$2"; done
           for s in 00 05 06 10; do run "$1" kitti "$s" "$2"; done; }
EUROC3() { for s in MH_01_easy V1_01_easy V2_02_medium; do run "$1" euroc "$s" "$2"; done; }

case "$PHASE" in
  i)
    # R0 at this binary (default path: the WP2c flags at default are byte-identical in arithmetic)
    run full tum freiburg1_room 5
    run full kitti 07 5
    "$PY" run_scripts/ral_v2/r0_check.py --csv "$ROOT/full/summary.csv" --require freiburg1_room 07 \
      || { echo "R0 FAILED -- stop (DECISIONS.md WP2c; the n=10 alternative is run by hand if KITTI 07 alone is outside)"; exit 6; }
    TWO C_hyg_only "$REPS"; TWO C_base "$REPS"
    for w in 10 100 1 1000; do for k in k1 k3 knone; do TWO "C_w${w}_${k}" "$REPS"; done; done
    TWO C_w100_k0 "$REPS"
    ;;
  seed)
    [ -n "$ARM" ] || { echo "PHASE=seed needs ARM=<phase-i pick>"; exit 4; }
    TWO "${ARM}_seedmid" "$REPS"; TWO "${ARM}_seedbr" "$REPS"
    ;;
  ii)
    [ -n "$ARM" ] || { echo "PHASE=ii needs ARM=<pick>"; exit 4; }
    ABL10 "$ARM" "$REPS"
    ;;
  iii)
    EUROC3 "${ARM:-C_final}" "$REPS"; run "${ARM:-C_final}" tummonovo sequence_31 "$REPS"
    ;;
  g2)
    EUROC3 "${ARM:-C_final}" 5; ABL10 "${ARM:-C_final}" 5
    ;;
  *) echo "unknown PHASE=$PHASE"; exit 4 ;;
esac
echo "=== WP2c phase $PHASE done: $(date) ==="
"$PY" run_scripts/ral_v2/wp2a_summary.py --root "$ROOT" --ref full || true
