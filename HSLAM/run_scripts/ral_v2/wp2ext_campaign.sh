#!/usr/bin/env bash
# WP2b-log phase ii-EXT -- TUM-10 / KITTI-9 breadth beyond ABL-10.
#
# Pre-registered in DECISIONS.md ("WP2b-log phase ii-EXT -- PRE-REGISTERED 2026-09-22 (evening)"),
# RE-SCOPED the same evening once phase ii returned (see "phase ii OUTCOME"). Read both before
# changing anything here.
#
# ORIGINAL PURPOSE (now void): raise N_eff past N_EFF_THIN = 7 so the dataset verdict would not be
# stamped THIN EVIDENCE. Phase ii then failed BOTH candidates on C1 -- L_w1000_k1 on KITTI (05 +36.4 %,
# 06 +158.2 %), L_w10000_k3 on TUM (fr2_large_no_loop +64.5 %) and KITTI (05 +63.1 %). C1 is an
# absolute per-sequence veto, so no sequence added here can lift either failure and there is no
# adoption verdict left to strengthen.
#
# CURRENT PURPOSE, user decision 22 Sep evening: run all THREE arms for (a) `full` rows on the paper's
# full-table sequences at n=8 / n=10, which WP6 needs anyway, and (b) the live diagnostic question --
# phase ii's KITTI results order almost perfectly by the FROZEN config's own scale drift. b2 wins where
# there is drift to fix (06 -12.34 %/100m, 07 -3.96, 10 -2.07) and costs where there is none
# (05 -0.44, monotone in weight). If that holds on KITTI 01-04 it is a mechanism statement worth the
# paper; if it does not, it was five sequences and a story. Either way it is measured, not asserted.
#
# THE SET IS NOT CHOSEN ON THE DATA. It is exactly TUM-10 minus ABL-10, and KITTI-11 minus ABL-10
# minus {08,09}, taken from datasets.py SETS, which come from PAPER_CONFIG_AND_GATES.md section 7 and
# predate b2 entirely. 08/09 do not track at HEAD and are excluded by standing protocol.
#
# REPS: TUM n=8, KITTI n=10. The rule needs >= 5 USABLE reps on BOTH arms or a sequence is unresolved.
# On the A0 monocular rows three of the five new TUM sequences miss that at n=5 (fr1_360 2/5,
# fr2_360_hemisphere 3/5, fr2_large_with_loop 1/5), so TUM gets insurance reps. KITTI 01/02/03 are 5/5
# on A0 and stay at the rule's n=10. ~5.7 h for three arms.
#
# OUT is the phase-ii root ON PURPOSE: same binary epoch, same protocol, so rows pool with ABL-10 and
# adoption_rule.py reads TUM-10 / KITTI-9 in one pass. Rows are told apart by timestamp and commit.
#
# RESUMABLE. eval_run.py appends unconditionally, so re-running this script naively would DUPLICATE
# rows and corrupt every median. `have()` below skips any (arm, dataset, sequence) that already has
# enough rows in summary.csv -- the authoritative record, since eval_run.py writes it only after all
# reps of an invocation complete. Rep directories are NOT a valid progress signal: an interrupted run
# leaves one behind with no row.
#
#   tmux new -s wp2ext; bash run_scripts/ral_v2/wp2ext_campaign.sh 2>&1 | tee -a ~/wp2ext.log
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
ROOT="${OUT:-runs/wp2ii_$(hostname -s)}"
TREPS="${TREPS:-8}"; KREPS="${KREPS:-10}"
ARMS="${ARMS:-full L_w1000_k1 L_w10000_k3}"
CANDS="${CANDS:-L_w1000_k1 L_w10000_k3}"
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

# exit 0 when $ROOT/$1/summary.csv already holds >= $4 rows for dataset $2 sequence $3
have() {
  "$PY" -c 'import csv, sys
p, ds, sq, n = sys.argv[1], sys.argv[2], sys.argv[3], int(sys.argv[4])
try:
    rows = list(csv.DictReader(open(p)))
except FileNotFoundError:
    sys.exit(1)
c = sum(1 for r in rows if r["dataset"] == ds and r["sequence"] == sq)
sys.exit(0 if c >= n else 1)' "$ROOT/$1/summary.csv" "$2" "$3" "$4"
}
run() {
  if have "$1" "$2" "$3" "$4"; then echo "=== skip arm=$1  $2 $3: already has >= $4 rows ==="; echo; return 0; fi
  echo "=== arm=$1  $2 $3  (n=$4)  $(date +%H:%M) ==="
  "$PY" run_scripts/ral_v2/eval_run.py --dataset "$2" --sequence "$3" --arm "$1" --reps "$4" --out "$ROOT/$1" \
    || echo "!!! run failed: $1 $2 $3 (continuing)"
  echo
}
STATUS="$PY run_scripts/ral_v2/wp2blog_phase2_status.py --root $ROOT --ref full"

# HAZARD-FIRST, then cost-last. The four sequences least likely to resolve run first, so that is known
# in ~1 h instead of at hour 5. KITTI 02 is last: 4661 frames, ~2.6 h of the ~5.7 h total for 3 arms.
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
  # an arm is the adoption rule's own C2 breakage band. Interim ATE is not a decision input.
  set +e
  $STATUS --cand $CANDS --futility-seq "$sq"
  rc=$?
  set -e
  if [ "$rc" = "3" ]; then
    KEEP=""
    for c in $CANDS; do
      if $STATUS --cand "$c" --futility-seq "$sq" >/dev/null 2>&1; then KEEP="$KEEP $c"; fi
    done
    echo "=== C2 breakage on $sq: candidate set was '$CANDS', continuing with '${KEEP:-NONE}' ==="
    CANDS="$(echo $KEEP)"
    ARMS="full $CANDS"
    [ -n "$CANDS" ] || { echo "both candidates broke -- stopping"; break; }
  fi
done

echo "=== phase ii-EXT complete: ledger below is over ABL-10 + the extension, pooled ==="
echo "=== NOTE: phase ii already vetoed both candidates on C1. These are breadth numbers, not a"
echo "===       re-run of the adoption decision, which is settled and recorded in DECISIONS.md. ==="
for c in $CANDS; do
  for ds in tum kitti; do
    echo "--- adoption rule: $c vs full, $ds ---"
    "$PY" run_scripts/ral_v2/adoption_rule.py --ref "$ROOT/full" --cand "$ROOT/$c" --dataset "$ds" || true
  done
done
echo "=== WP2b-log phase ii-EXT done: $(date) ==="
