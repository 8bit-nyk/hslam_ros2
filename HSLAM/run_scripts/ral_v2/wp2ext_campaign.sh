#!/usr/bin/env bash
# WP2b-log phase ii-EXT -- enlarge N_eff so the dataset verdict is not stamped THIN EVIDENCE.
#
# Pre-registered in DECISIONS.md ("WP2b-log phase ii-EXT -- PRE-REGISTERED 2026-09-22 (evening)").
# Read that entry before changing anything here.
#
# WHY THIS EXISTS. adoption_rule.py stamps a dataset THIN EVIDENCE below N_EFF_THIN = 7 materially
# changed sequences, and its text says a THIN verdict "may not by itself decide anything that ships".
# ABL-10 supplies FIVE sequences per dataset, so N_eff <= 5 and every per-dataset verdict phase ii can
# produce is THIN by construction. G2 is a shipping decision. This run adds the sequences the paper's
# own full-table sets already contain, so both datasets can clear the stamp.
#
# THE SET IS NOT CHOSEN ON THE DATA. It is exactly TUM-10 minus ABL-10, and KITTI-11 minus ABL-10
# minus {08,09}, taken from datasets.py SETS, which come from PAPER_CONFIG_AND_GATES.md section 7 and
# predate b2 entirely. 08/09 do not track at HEAD and are excluded by standing protocol, not by this
# card. No sequence here was picked for being favourable, and no threshold or rule is changed.
#
# ARMS: `full` + `L_w1000_k1` only. L_w10000_k3 is NOT extended: its TUM verdict is already FAIL on
# C1 (fr2_large_no_loop +64.5 %), and C1 is an absolute per-sequence veto -- no sequence added here
# can undo it. It still completes phase ii on ABL-10 and is reported in full.
#
# REPS: TUM n=8, KITTI n=10. The rule needs >= 5 USABLE reps on BOTH arms or the sequence is
# "unresolved" and contributes nothing. On the A0 monocular rows three of the five new TUM sequences
# fail that at n=5 (fr1_360 2/5, fr2_360_hemisphere 3/5, fr2_large_with_loop 1/5), so TUM gets
# insurance reps; KITTI 01/02/03 are 5/5 on A0 and stay at the rule's n=10.
# ALL usable reps enter the verdict. That is conservative, not permissive: crossing N_eff >= 7 swaps
# breadth from strict majority (reg+1) to ceil(0.7*N_eff), which is a STRICTER bar.
#
# OUT is the phase-ii root ON PURPOSE: same binary epoch, same protocol, so the rows pool with ABL-10
# and adoption_rule.py sees TUM-10 and KITTI-9 in one pass. Rows are told apart by timestamp/commit.
#
#   tmux new -s wp2ext; bash run_scripts/ral_v2/wp2ext_campaign.sh 2>&1 | tee -a ~/wp2ext.log
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
ROOT="${OUT:-runs/wp2ii_$(hostname -s)}"
TREPS="${TREPS:-8}"; KREPS="${KREPS:-10}"
ARMS="${ARMS:-full L_w1000_k1}"
CAND="${CAND:-L_w1000_k1}"
EPOCH="${EPOCH:-692adec6141bda91}"     # the phase-ii binary. Extension rows MUST share it.
PY=python3
[ -x "$HOME/Dev/evo/evo_env/bin/python" ] && PY="$HOME/Dev/evo/evo_env/bin/python"
"$PY" -c 'import evo' 2>/dev/null || { echo "ERROR: no python with evo"; exit 2; }
if [ -n "$(git status --porcelain)" ]; then echo "ERROR: dirty tree"; git status --short | head; exit 3; fi
SHA="$(sha256sum build/bin/HSLAM | cut -c1-16)"
[ "$SHA" = "$EPOCH" ] || { echo "ERROR: binary $SHA != phase-ii epoch $EPOCH -- rows would not pool"; exit 7; }
for f in ml-prior-param ml-prior-sigma-log ml-prior-weight ml-prior-gate-k ml-fej-freeze ml-init-scale p1-blend-grad-fix; do
  ./build/bin/HSLAM --help 2>&1 | grep -q -- "--$f" || { echo "ERROR: binary lacks --$f"; exit 5; }
done
if [ -n "${WAIT_FOR:-}" ]; then
  while tmux has-session -t "=$WAIT_FOR" 2>/dev/null; do echo "$(date) waiting for tmux session '$WAIT_FOR'"; sleep 120; done
  echo "$(date) '$WAIT_FOR' finished -- starting"
fi
echo "host $(hostname)  commit $(git rev-parse --short HEAD)  binary $SHA  arms '$ARMS'  TUM n=$TREPS KITTI n=$KREPS  out $ROOT/<arm>/  start $(date)"
run() { echo "=== arm=$1  $2 $3  (n=$4)  $(date +%H:%M) ==="; "$PY" run_scripts/ral_v2/eval_run.py --dataset "$2" --sequence "$3" --arm "$1" --reps "$4" --out "$ROOT/$1" || echo "!!! run failed: $1 $2 $3 (continuing)"; echo; }
STATUS="$PY run_scripts/ral_v2/wp2blog_phase2_status.py --root $ROOT --ref full"

# HAZARD-FIRST, then cost-last. The four sequences least likely to resolve run first, so that is known
# in ~1 h instead of at hour 4. KITTI 02 is last because it is 4661 frames (~1.7 h of the ~3.8 h total)
# and because TUM and KITTI each already reach N_eff = 7 without it if the safe ones resolve.
SEQS="tum:freiburg2_large_with_loop:$TREPS
      kitti:01:$KREPS
      tum:freiburg1_360:$TREPS
      kitti:04:$KREPS
      tum:freiburg1_desk2:$TREPS
      kitti:03:$KREPS
      tum:freiburg1_floor:$TREPS
      tum:freiburg2_360_hemisphere:$TREPS
      kitti:02:$KREPS"

for entry in $SEQS; do
  ds="${entry%%:*}"; rest="${entry#*:}"; sq="${rest%%:*}"; nr="${rest##*:}"
  for arm in $ARMS; do run "$arm" "$ds" "$sq" "$nr"; done
  # Same discipline as phase ii: the slice is printed for visibility, and the ONLY thing that may stop
  # the arm is the adoption rule's own C2 breakage band. Interim ATE is not a decision input.
  set +e
  $STATUS --cand "$CAND" --futility-seq "$sq"
  rc=$?
  set -e
  if [ "$rc" = "3" ]; then
    echo "=== C2 BREAKAGE on $sq -- $CAND is vetoed on this dataset; stopping the extension ==="
    break
  fi
done

echo "=== phase ii-EXT complete: verdict below is over ABL-10 + the extension, pooled ==="
for ds in tum kitti; do
  echo "--- adoption rule: $CAND vs full, $ds (TUM-10 / KITTI-9) ---"
  "$PY" run_scripts/ral_v2/adoption_rule.py --ref "$ROOT/full" --cand "$ROOT/$CAND" --dataset "$ds" || true
done
echo "--- for the record, the arm that already failed TUM on C1, on ABL-10 only ---"
for ds in tum kitti; do
  "$PY" run_scripts/ral_v2/adoption_rule.py --ref "$ROOT/full" --cand "$ROOT/L_w10000_k3" --dataset "$ds" || true
done
echo "=== WP2b-log phase ii-EXT done: $(date) ==="
