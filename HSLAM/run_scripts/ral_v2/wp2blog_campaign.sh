#!/usr/bin/env bash
# WP2b-log (card b2) -- the prior as a RELATIVE (log-depth) residual.
# Pre-registered in DECISIONS.md ("WP2b-log (card b2) -- PRE-REGISTERED 2026-09-21"). Phases:
#   PHASE=i    15 L arms on fr1_room + KITTI 07, n=3 (90 runs)                      [default]
#   PHASE=iboundary  w=3000/10000 x {k1,k3} on fr1_room + KITTI 07, n=3 (24 runs) -- DIAGNOSTIC
#                    only: phase i cleared bar 1 solely at the top of its grid. The pick rule
#                    takes the smallest passing w, so these cannot displace L_w1000_k1.
#   PHASE=ii   the pick (ARM=<pick>) on ABL-10, n=3 -> dataset-level rule + gate threshold
#   PHASE=ii2  THREE arms (full + two candidates) on ABL-10, SEQUENCE-MAJOR and hazard-first,
#              TUM n=5 / KITTI n=10 as the adoption rule requires. 225 runs ~6.3 h.
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
  ii2)
    # DECISIONS.md "WP2b-log phase ii -- TWO CANDIDATES -- PRE-REGISTERED 2026-09-22".
    #
    # SEQUENCE-MAJOR on purpose. Arm-major (all sequences for arm A, then arm B) leaves you with a
    # complete arm and nothing to compare it against if the campaign is stopped or dies; sequence-major
    # means a complete, comparable three-arm slice exists after every sequence.
    #
    # HAZARD-FIRST on purpose. fr2_large_no_loop (the prior sits at 0.23x Kinect on frame 0; the
    # sequence that failed K13) and KITTI 07 (the named truck hazard at frames 000634-000644) are the
    # two places a too-strong prior should break. They run first, so that failure mode is visible in
    # ~45 min rather than at hour 6. fr3_long_office_household is third: it is the long TUM sequence
    # that tests whether the inherited prior drift costs anything over 40-90 m.
    #
    # TUM n=5, KITTI n=10 -- NOT the runner's n=3 default. The adoption rule classes a sequence with
    # fewer than 5 usable reps on either arm as "unresolved" and drops it from B's denominator, and
    # KITTI needs n=10 for 84 % power (wp/RULE_CALIBRATION.md section 5).
    #
    # The reference `full` is re-run at THIS binary epoch rather than reused from wp2a_eval-server
    # (commit 4fce1b66): the three recorded epochs agree on fr1_room (0.301/0.295/0.306) but the
    # standing rule is that arms from different binaries are not poolable, and the adoption rule
    # compares rep SETS. These rows also seed the re-freeze chain.
    ARMS="${ARMS:-full L_w1000_k1 L_w10000_k3}"
    CANDS="${CANDS:-L_w1000_k1 L_w10000_k3}"
    TREPS="${TREPS:-5}"; KREPS="${KREPS:-10}"
    STATUS="$PY run_scripts/ral_v2/wp2blog_phase2_status.py --root $ROOT --ref full"
    # sequence, dataset, reps -- in the order described above
    SEQS="tum:freiburg2_large_no_loop:$TREPS kitti:07:$KREPS tum:freiburg3_long_office_household:$TREPS
          tum:freiburg1_room:$TREPS tum:freiburg1_desk:$TREPS tum:freiburg2_desk:$TREPS
          kitti:10:$KREPS kitti:06:$KREPS kitti:05:$KREPS kitti:00:$KREPS"
    for entry in $SEQS; do
      ds="${entry%%:*}"; rest="${entry#*:}"; sq="${rest%%:*}"; nr="${rest##*:}"
      for arm in $ARMS; do run "$arm" "$ds" "$sq" "$nr"; done
      # futility applies ONLY to the two hazard sequences, and ONLY on C2 breakage (track), never ATE
      FUT=""
      case "$sq" in freiburg2_large_no_loop|07) FUT="--futility-seq $sq";; esac
      set +e
      $STATUS --cand $CANDS $FUT
      rc=$?
      set -e
      if [ "$rc" = "3" ]; then
        KEEP=""
        for c in $CANDS; do
          if $STATUS --cand "$c" --futility-seq "$sq" >/dev/null 2>&1; then KEEP="$KEEP $c"; fi
        done
        echo "=== futility: candidate set was '$CANDS', continuing with '${KEEP:-NONE}' ==="
        CANDS="$(echo $KEEP)"
        ARMS="full $CANDS"
        [ -n "$CANDS" ] || { echo "both candidates broke -- stopping"; break; }
      fi
    done
    echo "=== phase ii complete: verdict below comes from adoption_rule.py, the single implementation ==="
    for c in $CANDS; do
      for ds in tum kitti; do
        echo "--- adoption rule: $c vs full, $ds ---"
        "$PY" run_scripts/ral_v2/adoption_rule.py --ref "$ROOT/full" --cand "$ROOT/$c" --dataset "$ds" || true
      done
    done
    "$PY" run_scripts/ral_v2/wp2c_gate_thr.py --root "$ROOT" --arm "${CANDS%% *}" || true
    ;;
  iboundary)
    # DECISIONS.md "WP2b-log -- boundary probe -- PRE-REGISTERED 2026-09-22". Same binary epoch as
    # phase i (692adec6...), arms.py change only, so no rebuild and no new R0 epoch.
    # wp2blog_rule.py needs a 'full' arm under $ROOT for its parity bar; phase i ran with SKIP_R0=1,
    # so the epoch's R0 rows live under runs/wp2a_r5b_eval-server/full (commit a50855f8, the R0 that
    # passed 21 Sep 12:31). Link them in rather than re-running 10 reps of a settled question.
    if [ ! -e "$ROOT/full" ]; then
      ln -s "$PWD/runs/wp2a_r5b_eval-server/full" "$ROOT/full"
      echo "linked parity reference: $ROOT/full -> runs/wp2a_r5b_eval-server/full"
    fi
    for w in 3000 10000; do for k in k1 k3; do TWO "L_w${w}_${k}" "$REPS"; done; done
    "$PY" run_scripts/ral_v2/wp2blog_rule.py --root "$ROOT" || true
    ;;
  *) echo "unknown PHASE=$PHASE"; exit 4;;
esac
echo "=== WP2b-log phase $PHASE done: $(date) ==="
