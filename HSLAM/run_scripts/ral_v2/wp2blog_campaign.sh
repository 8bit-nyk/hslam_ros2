#!/usr/bin/env bash
# WP2b-log (card b2) -- the prior as a RELATIVE (log-depth) residual.
# Pre-registered in DECISIONS.md ("WP2b-log (card b2) -- PRE-REGISTERED 2026-09-21"). Phases:
#   PHASE=i    15 L arms on fr1_room + KITTI 07, n=3 (90 runs)                      [default]
#   PHASE=ii   the pick (ARM=<pick>) on ABL-10, n=3 -> dataset-level rule + gate threshold
#   PHASE=iii  the pick on EuRoC-3 + mono-VO, n=3 (screen)
# New binary => R0 first (r0_check.py), embedded in PHASE=i, with the pre-registered n>=10 clause
# for KITTI 07 (CV ~ 30 %: a +-20 % band at n=5 is a coin flip -- see the WP2a-R5 R0 amendment).
#   tmux new -s wp2blog;  PHASE=i bash run_scripts/ral_v2/wp2blog_campaign.sh 2>&1 | tee -a ~/wp2blog.log
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
PHASE="${PHASE:-i}"; ARM="${ARM:-}"; REPS="${REPS:-3}"
ROOT="${OUT:-runs/wp2blog_$(hostname -s)}"
PY=python3
[ -x "$HOME/Dev/evo/evo_env/bin/python" ] && PY="$HOME/Dev/evo/evo_env/bin/python"
"$PY" -c 'import evo' 2>/dev/null || { echo "ERROR: no python with evo"; exit 2; }
if [ -n "$(git status --porcelain)" ]; then echo "ERROR: dirty tree"; git status --short | head; exit 3; fi
for f in ml-prior-param ml-prior-sigma-log ml-prior-weight ml-prior-gate-k ml-fej-freeze ml-init-scale p1-blend-grad-fix; do
  ./build/bin/HSLAM --help 2>&1 | grep -q -- "--$f" || { echo "ERROR: binary lacks --$f; rebuild first"; exit 5; }
done
if [ -n "${WAIT_FOR:-}" ]; then
  while tmux has-session -t "=$WAIT_FOR" 2>/dev/null; do echo "$(date) waiting for tmux session '$WAIT_FOR'"; sleep 300; done
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
    # SKIP_R0=1 when R0 has already been judged for this binary epoch -- which is the case for the
    # epoch carrying --ml-prior-param: R0 passed 21 Sep 12:31 (DECISIONS.md, "Next action -- b2").
    # Re-running it spends 10 runs to re-answer a settled question. Harmless if run anyway.
    if [ "${SKIP_R0:-0}" = "1" ]; then
      echo "SKIP_R0=1: R0 already judged for this epoch (DECISIONS.md, b2 next-action entry)"
    else
    run full tum freiburg1_room 5
    run full kitti 07 5
    if ! "$PY" run_scripts/ral_v2/r0_check.py --csv "$ROOT/full/summary.csv" --require freiburg1_room 07; then
      echo "R0 not met at n=5 -- pre-registered extension: 5 more reps of each, judged on the pooled rows"
      "$PY" run_scripts/ral_v2/eval_run.py --dataset tum --sequence freiburg1_room --arm full --reps 5 --out "$ROOT/full_r0ext" || true
      "$PY" run_scripts/ral_v2/eval_run.py --dataset kitti --sequence 07 --arm full --reps 5 --out "$ROOT/full_r0ext" || true
      "$PY" run_scripts/ral_v2/r0_check.py --csv "$ROOT/full/summary.csv" "$ROOT/full_r0ext/summary.csv" --require freiburg1_room 07 \
        || { echo "R0 FAILED at n=10 -- stop (DECISIONS.md WP2b-log)"; exit 6; }
    fi
    fi
    TWO L_base "$REPS"
    for w in 1 10 100 1000; do for k in k1 k3 knone; do TWO "L_w${w}_${k}" "$REPS"; done; done
    TWO L_w10_k3_s015 "$REPS"; TWO L_w10_k3_s06 "$REPS"
    "$PY" run_scripts/ral_v2/wp2blog_rule.py --root "$ROOT" || true
    ;;
  ii)
    # Judged by the dataset-level adoption rule v2 (DECISIONS.md, "RULE AMENDMENT -- 2026-09-21
    # (evening)"). That rule REQUIRES n=10 on KITTI: at n=5 it adopts a true -15 % effect only 40.8 %
    # of the time, at n=10 84.2 % (wp/RULE_CALIBRATION.md section 5). TUM is adequately powered at n=5.
    # Verdict comes from adoption_rule.py -- do NOT re-derive the rule in a new script.
    [ -n "$ARM" ] || { echo "ERROR: PHASE=ii needs ARM=<pick>"; exit 4; }
    KREPS="${KREPS:-10}"
    run "$ARM" tum freiburg1_room "$REPS"; run "$ARM" kitti 07 "$KREPS"
    for s in freiburg1_desk freiburg2_desk freiburg2_large_no_loop freiburg3_long_office_household; do
      run "$ARM" tum "$s" "$REPS"; done
    for s in 00 05 06 10; do run "$ARM" kitti "$s" "$KREPS"; done
    "$PY" run_scripts/ral_v2/wp2c_gate_thr.py --root "$ROOT" --arm "$ARM" || true
    ;;
  iii)
    [ -n "$ARM" ] || { echo "ERROR: PHASE=iii needs ARM=<pick>"; exit 4; }
    EUROC3 "$ARM" "$REPS"; run "$ARM" tummonovo sequence_31 "$REPS"
    ;;
  *) echo "unknown PHASE=$PHASE"; exit 4;;
esac
echo "=== WP2b-log phase $PHASE done: $(date) ==="
