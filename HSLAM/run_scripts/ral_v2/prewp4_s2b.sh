#!/usr/bin/env bash
# Pre-WP4 stage 2b (DECISIONS.md "PRE-WP4 STAGE 2b"): the softer-pin-weight screen for
# --init-founding-fix=anchor, modelled on prewp4_stage1.sh Part B. Binary epoch 0155a0d2ab6932b4
# (the re-freeze epoch, same as stage 1 and stage 2).
#
# Per sequence, sequence-major, hazard-first:
#   1. an n=1 environment bracket (arm `full`, root $ROOT/bracket) -- descriptive only, read by
#      prewp4_s2b_screen.py --bracket-root against the stage-2 reference's median scale.
#   2. the four screen arms at n=3 (n=5 on the two named risks, KITTI 04 and fr2_desk):
#        full_R3b_anchor_aw2500  full_R3b_anchor_aw1000   (the softer-pin candidates)
#        S1_alphaw_lo            S1_alphaw_1000           (weight-only controls, no anchor fix)
# The reference is NOT re-run here: prewp4_s2b_screen.py reads it live from the stage-2 root
# (--ref-root, default runs/prewp4_s2_eval-server), so this script only ever writes new arms.
#
# Holds /tmp/wp7_gpu.lock throughout (the WP7 B6 agents take it before any GPU job; never delete
# the file), and every run first waits for an empty GPU. Launch from a detached, non-interactive
# tmux session so the environment is pinned, as stage 1 and stage 2 do:
#   tmux new-session -d -s prewp4_s2b \
#     "env -u LD_LIBRARY_PATH bash run_scripts/ral_v2/prewp4_s2b.sh > ~/prewp4_s2b.log 2>&1"
# DRY=1 prints the plan (target, arm, dataset, sequence, n) and its cost from the stage-2
# reference's measured wall_s medians, and runs nothing -- no GPU, no lock, no preflight.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
source run_scripts/ral_v2/campaign_lib.sh

EPOCH=0155a0d2ab6932b4
DRY="${DRY:-0}"
ROOT="${OUT:-runs/prewp4_s2b_$(hostname -s)}"
REF="${REF_ROOT:-runs/prewp4_s2_eval-server}"
REUSED_ROOT="runs/prewp4_s2_eval-server"

# The reused root is read-only for stage 2b: it is the frozen stage-2 reference and other work
# (WP4) already reads it as-is. Same check as wp4_campaign.sh, kept to exit 2 as pre-registered.
case "$(realpath -m "$ROOT")/" in
  "$(realpath -m "$REUSED_ROOT")"/*)
    echo "ERROR: output root $ROOT is inside the reused root $REUSED_ROOT -- stage 2b never writes there"
    exit 2 ;;
esac

# Sequence-major, hazard-first: the two named risks first (KITTI 04's short founding segment,
# then KITTI 03), then the rest cheap to costly.
ORDER=("kitti 04" "kitti 03" "tum freiburg2_desk" "kitti 06" "tum freiburg3_long_office_household"
       "kitti 05" "kitti 07" "tum freiburg1_room" "kitti 02")
ARMS=(full_R3b_anchor_aw2500 full_R3b_anchor_aw1000 S1_alphaw_lo S1_alphaw_1000)

# plan -> "target arm dataset sequence n" lines, sequence-major. target is "bracket" (arm always
# `full`, n=1, written under $ROOT/bracket) or "main" (one of ARMS, under $ROOT itself).
plan() {
  local dsq ds sq n arm
  for dsq in "${ORDER[@]}"; do
    set -- $dsq; ds="$1"; sq="$2"
    n=3
    if [ "$ds $sq" = "kitti 04" ] || [ "$ds $sq" = "tum freiburg2_desk" ]; then n=5; fi
    echo "bracket full $ds $sq 1"
    for arm in "${ARMS[@]}"; do echo "main $arm $ds $sq $n"; done
  done
}

# cost PLANFILE -> runs and hours still to run, from the median wall_s of REF's `full` rows per
# sequence (same idiom as wp4_campaign.sh's cost()). Rows already in ROOT are subtracted.
cost() {
  "$PY" - "$REF/full/summary.csv" "$1" "$ROOT" <<'EOF'
import csv, os, statistics as st, sys
from collections import defaultdict
wall = defaultdict(list)
for r in csv.DictReader(open(sys.argv[1])):
    wall[(r["dataset"], r["sequence"])].append(float(r["wall_s"]))
root = sys.argv[3]
def have(target, arm, ds, sq):
    sub = "bracket" if target == "bracket" else ""
    p = os.path.join(root, sub, arm, "summary.csv") if sub else os.path.join(root, arm, "summary.csv")
    if not os.path.exists(p):
        return 0
    return sum(1 for r in csv.DictReader(open(p)) if r["dataset"] == ds and r["sequence"] == sq)
runs, secs = 0, 0.0
miss = set()
per_arm = defaultdict(lambda: [0, 0.0])
for line in open(sys.argv[2]):
    target, arm, ds, sq, n = line.split()
    n = max(int(n) - have(target, arm, ds, sq), 0)
    w = st.median(wall[(ds, sq)]) if wall[(ds, sq)] else float("nan")
    if w != w:
        miss.add((ds, sq))
    runs += n; secs += n * (w if w == w else 0.0)
    key = f"{target}:{arm}"
    per_arm[key][0] += n; per_arm[key][1] += n * (w if w == w else 0.0)
for arm, (n, s) in per_arm.items():
    print(f"  {arm:28s} {n:4d} runs  {s / 3600:5.2f} h")
print(f"TOTAL {runs} runs, {secs / 3600:.2f} h of HSLAM wall time (stage-2 `full` medians)")
if miss:
    print(f"  (no reference wall time for {sorted(miss)} -- not costed)")
EOF
}

PLANFILE="$(mktemp)"; trap 'rm -f "$PLANFILE"' EXIT
plan > "$PLANFILE"

if [ "$DRY" = 1 ]; then
  cat "$PLANFILE"; cost "$PLANFILE"; exit 0
fi

echo "=== launch env ==="; env | sort; echo "=== end env ==="
campaign_preflight "$EPOCH" init-founding-fix ml-alpha-w

exec 9>/tmp/wp7_gpu.lock
echo "waiting for /tmp/wp7_gpu.lock $(date +%H:%M:%S)"
flock 9
echo "GPU lock held $(date +%H:%M:%S)"

# gpu_quiet: wait until no compute process is on the GPU (another job would contaminate the row).
gpu_quiet() {
  local apps
  while apps="$(nvidia-smi --query-compute-apps=pid,process_name --format=csv,noheader)"; [ -n "$apps" ]; do
    echo "$(date +%H:%M:%S) GPU busy, waiting: $apps"; sleep 30
  done
}

prev=""
while read -r target arm ds sq n <&3; do
  if [ "$ds $sq" != "$prev" ]; then
    prev="$ds $sq"
    echo "##### $ds $sq $(date)"
  fi
  root="$ROOT"; [ "$target" = bracket ] && root="$ROOT/bracket"
  # campaign_run's own campaign_count is the resume guard: a cell whose rows (any status) already
  # meet n is skipped, and a top-up only runs the missing reps -- no extra bookkeeping needed here.
  gpu_quiet
  campaign_run "$root" "$arm" "$ds" "$sq" "$n"
done 3< "$PLANFILE"
echo "##### stage 2b done $(date)"
