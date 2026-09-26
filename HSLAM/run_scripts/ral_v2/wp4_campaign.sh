#!/usr/bin/env bash
# WP4 -- the component ablation at the re-frozen config (wp/WP4_component_ablation.md; DECISIONS.md "WP4 --
# component ablation -- PRE-REGISTERED 2026-09-26"). Binary epoch 0155a0d2ab6932b4, the stage-2 epoch.
#
# Reuses stage 2's rows (REF_ROOT: full, full_K13, A0, A0_tol -- same binary, checked row by row at preflight) and
# runs only the new arms:
#   TIER=1    11 arms on ABL-10 (TUM n=5, KITTI n=10), then the KITTI 02 attribution cells
#             (K14_off, K15_off, full_6799be6 at n=10)                                         855 runs
#   TIER=2    12 arms on ABL-10 at n=5, and H2_t03 / H2_t07 on the loop set at n=5              640 runs
#   TIER=esc  ESC_ARMS="<arm> ..." -- tops the named Tier-2 arms up to Tier-1 reps (KITTI n=10)
# Sequence-major, hazard-first (card §6.2); per sequence the arm order of the tier's list. The only early stop is the
# adoption rule's C2 breakage band against stage 2's `full`, scoped to the DATASET: a breakage writes
# $ROOT/STOPPED_<arm>_<dataset> and that arm skips the dataset's remaining sequences (R1/R2 are per-dataset verdicts,
# and the breakage has already decided R2 there; the other dataset is an independent measurement). Never resumed.
# Resume-safe (campaign_run tops up only missing reps). Holds /tmp/wp7_gpu.lock throughout and waits for an empty
# GPU before every run.
#
# Launch ONLY after the user's go-ahead and the WP7 handover (a non-wp7_ tmux session makes WP7's runner yield
# before its next run; this runner then blocks on the lock until WP7's in-flight run ends):
#   tmux new-session -d -s wp4_t1 \
#     "env -u LD_LIBRARY_PATH TIER=1 bash run_scripts/ral_v2/wp4_campaign.sh > ~/wp4_t1.log 2>&1"
# DRY=1 prints the plan and costs it from REF_ROOT's measured `full` wall times; runs nothing.
# SMOKE=1 (laptop): every WP4 arm, n=1, --endindex 250, on KITTI 07 + fr1_room, binary from HSLAM_BINARY, thermal
#   gate between runs, rows to runs/wp4_smoke_<host>. No epoch check: the laptop build carries its own hash.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
source run_scripts/ral_v2/campaign_lib.sh

EPOCH=0155a0d2ab6932b4
HOST="$(hostname -s)"
DRY="${DRY:-0}"
SMOKE="${SMOKE:-0}"
TIER="${TIER:-}"
[ "$SMOKE" = 1 ] && TIER=smoke
ROOT="${OUT:-runs/wp4_$HOST}"
[ "$SMOKE" = 1 ] && ROOT="${OUT:-runs/wp4_smoke_$HOST}"
REF="${REF_ROOT:-runs/prewp4_s2_$HOST}"

# Hazard-first (card §6.2): the prior's worst frame-0 sequence, the truck hazard / no-loop control, then cheap to costly.
ABL10=("tum freiburg2_large_no_loop" "kitti 07" "tum freiburg1_room" "kitti 06" "tum freiburg1_desk"
       "kitti 10" "tum freiburg3_long_office_household" "tum freiburg2_desk" "kitti 05" "kitti 00")
LOOPSET="|kitti 06|tum freiburg2_desk|kitti 05|kitti 00|"    # where H2 evaluates loops at `full` (stage 2)
T1_ARMS=(A1_strict A1_near A2 K3 K0 K1 K1c K12 K14_off K15_off K8)
T1_K02_ARMS=(K14_off K15_off full_6799be6)                    # KITTI 02: attribution of the (d1) cost
T2_ARMS=(K2 K5 K6 K7 K9_n3 K9_n5 K10_q037 K11 S1_unc_lo S1_unc_hi S1_alphaw_lo S1_alphaw_hi)
H2_ARMS=(H2_t03 H2_t07)
FLAGS=(ml-inference-mode ml-prior-source ml-seed p1-clamps ml-fej-freeze indirect-mp-ml-storage ml-init-scale
       p1-blend-grad-fix p2-gate-thresh ml-idepth-rel-q ml-normal-channel init-fail-thresholds)

case "$TIER" in
  1|2|smoke) ;;
  esc)
    [ -n "${ESC_ARMS:-}" ] || { echo "ERROR: TIER=esc needs ESC_ARMS=\"<Tier-2 arm> ...\""; exit 2; }
    for a in $ESC_ARMS; do
      [[ " ${T2_ARMS[*]} ${H2_ARMS[*]} " == *" $a "* ]] || { echo "ERROR: $a is not a Tier-2 arm"; exit 2; }
    done ;;
  *) echo "ERROR: set TIER=1, 2 or esc (or SMOKE=1)"; exit 2 ;;
esac

# plan -> "arm dataset sequence n" lines, sequence-major
plan() {
  local dsq arm
  case "$TIER" in
  1)
    for dsq in "${ABL10[@]}"; do
      set -- $dsq
      for arm in "${T1_ARMS[@]}"; do echo "$arm $1 $2 $([ "$1" = kitti ] && echo 10 || echo 5)"; done
    done
    for arm in "${T1_K02_ARMS[@]}"; do echo "$arm kitti 02 10"; done ;;
  2)
    for dsq in "${ABL10[@]}"; do
      set -- $dsq
      for arm in "${T2_ARMS[@]}"; do echo "$arm $1 $2 5"; done
      if [[ "$LOOPSET" == *"|$dsq|"* ]]; then for arm in "${H2_ARMS[@]}"; do echo "$arm $1 $2 5"; done; fi
    done ;;
  esc)
    for dsq in "${ABL10[@]}"; do
      set -- $dsq
      [ "$1" = kitti ] || continue                          # TUM is already at Tier-1 reps (n=5)
      for arm in $ESC_ARMS; do
        if [[ " ${H2_ARMS[*]} " == *" $arm "* ]] && [[ "$LOOPSET" != *"|$dsq|"* ]]; then continue; fi
        echo "$arm $1 $2 10"
      done
    done ;;
  smoke)
    for dsq in "kitti 07" "tum freiburg1_room"; do
      set -- $dsq
      for arm in full "${T1_ARMS[@]}" full_6799be6 "${T2_ARMS[@]}" "${H2_ARMS[@]}"; do echo "$arm $1 $2 1"; done
    done ;;
  esac
}

# cost PLANFILE -> runs and hours still to run, from the median wall_s of REF's `full` rows per sequence (HSLAM wall
# only; stage 2's evo/IO overhead measured 0.3 %). Rows already in ROOT are subtracted, as campaign_run would; for
# TIER=esc the Tier-2 reps (5) are assumed present. An upper bound for arms with fewer inferences (A1_*, K9_*).
cost() {
  "$PY" - "$REF/full/summary.csv" "$1" "$ROOT" "$([ "$TIER" = esc ] && echo 5 || echo 0)" <<'EOF'
import csv, os, statistics as st, sys
from collections import defaultdict
wall = defaultdict(list)
for r in csv.DictReader(open(sys.argv[1])):
    wall[(r["dataset"], r["sequence"])].append(float(r["wall_s"]))
root, base = sys.argv[3], int(sys.argv[4])
def have(arm, ds, sq):
    p = os.path.join(root, arm, "summary.csv")
    if not os.path.exists(p):
        return 0
    return sum(1 for r in csv.DictReader(open(p)) if r["dataset"] == ds and r["sequence"] == sq)
runs, secs, miss = 0, 0.0, set()
per_arm = defaultdict(lambda: [0, 0.0])
for line in open(sys.argv[2]):
    arm, ds, sq, n = line.split(); n = max(int(n) - max(have(arm, ds, sq), base), 0)
    w = st.median(wall[(ds, sq)]) if wall[(ds, sq)] else float("nan")
    if w != w: miss.add((ds, sq))
    runs += n; secs += n * (w if w == w else 0.0)
    per_arm[arm][0] += n; per_arm[arm][1] += n * (w if w == w else 0.0)
for arm, (n, s) in per_arm.items():
    print(f"  {arm:14s} {n:4d} runs  {s/3600:5.2f} h")
print(f"TOTAL {runs} runs, {secs/3600:.1f} h of HSLAM wall time (stage-2 `full` medians)")
if miss: print(f"  (no reference wall time for {sorted(miss)} -- not costed)")
EOF
}

# ref_check -> the reused rows exist, are from the epoch binary and a clean tree, and cover WP4's reference cells
ref_check() {
  "$PY" - "$REF" "$EPOCH" <<'EOF'
import csv, os, sys
sys.path.insert(0, "run_scripts/ral_v2")
import datasets
ref, epoch = sys.argv[1], sys.argv[2]
bad = False
for arm in ("full", "full_K13", "A0", "A0_tol"):
    p = os.path.join(ref, arm, "summary.csv")
    if not os.path.exists(p):
        print(f"ERROR: reused rows missing: {p}"); sys.exit(8)
    rows = list(csv.DictReader(open(p)))
    off = [r for r in rows if r.get("binary") != epoch or r.get("dirty") != "0"]
    print(f"  reuse {arm:9s} {len(rows):4d} rows  binary {sorted({r.get('binary') for r in rows})}  "
          f"commit {sorted({r.get('commit') for r in rows})}")
    if off:
        print(f"ERROR: {len(off)} row(s) in {p} are not binary {epoch} from a clean tree -- not poolable"); bad = True
full = list(csv.DictReader(open(os.path.join(ref, "full", "summary.csv"))))
for ds, sq in datasets.SETS["ABL-10"] + [("kitti", "02")]:
    need = 10 if ds == "kitti" else 5
    have = sum(1 for r in full if r["dataset"] == ds and r["sequence"] == sq)
    if have < need:
        print(f"ERROR: reference `full` has {have} < {need} rows on {ds} {sq}"); bad = True
sys.exit(8 if bad else 0)
EOF
}

# breakage ARM DATASET SEQUENCE -> exit 0 iff adoption_rule.classify flags a C2 breakage against stage 2's `full`
breakage() {
  "$PY" - "$REF/full/summary.csv" "$ROOT/$1/summary.csv" "$2" "$3" "$1" <<'EOF'
import sys
sys.path.insert(0, "run_scripts/ral_v2")
import adoption_rule as R
ref, cand = R.load_reps([sys.argv[1]]), R.load_reps([sys.argv[2]])
k = (sys.argv[3], sys.argv[4])
if k not in ref or k not in cand:
    sys.exit(1)
c = R.classify(ref[k], cand[k])
print(f"    C2 check {sys.argv[5]:13s} {k[0]} {k[1]}: {c['track_cand'][0]}/{c['track_cand'][1]} usable, "
      f"full {c['track_ref'][0]}/{c['track_ref'][1]} -> {'BREAKAGE' if c['breakage'] else 'ok'}")
sys.exit(0 if c["breakage"] else 1)
EOF
}

gpu_quiet() {
  local apps
  while apps="$(nvidia-smi --query-compute-apps=pid,process_name --format=csv,noheader)"; [ -n "$apps" ]; do
    echo "$(date +%H:%M:%S) GPU busy, waiting: $apps"; sleep 30
  done
}

smoke_preflight() {
  "$PY" -c 'import evo' 2>/dev/null || { echo "ERROR: no python with evo"; exit 2; }
  if [ -n "$(git status --porcelain)" ]; then echo "ERROR: dirty tree"; git status --short | head; exit 3; fi
  : "${HSLAM_BINARY:?SMOKE=1 needs HSLAM_BINARY (the laptop builds in build_wp3c/)}"
  export HSLAM_BINARY
  local f help
  help="$(LD_LIBRARY_PATH="$PWD/Thirdparty/onnxruntime/lib:$PWD/Thirdparty/CompiledLibs/lib_compat:${LD_LIBRARY_PATH:-}" \
          "$HSLAM_BINARY" --help 2>&1)"
  for f in "${FLAGS[@]}"; do grep -q -- "--$f" <<<"$help" || { echo "ERROR: binary lacks --$f"; exit 5; }; done
  echo "SMOKE host $(hostname)  commit $(git rev-parse --short HEAD)  binary $(sha256sum "$HSLAM_BINARY" | cut -c1-16)" \
       "(laptop build, not the epoch)  start $(date)"
}

PLANFILE="$(mktemp)"; trap 'rm -f "$PLANFILE"' EXIT
plan > "$PLANFILE"

if [ "$DRY" = 1 ]; then
  cat "$PLANFILE"; cost "$PLANFILE"; exit 0
fi

if [ "$SMOKE" = 1 ]; then
  smoke_preflight
else
  echo "=== launch env ==="; env | sort; echo "=== end env ==="
  campaign_preflight "$EPOCH" "${FLAGS[@]}"
fi
echo "=== reused rows ($REF) ==="; ref_check
exec 9>/tmp/wp7_gpu.lock
echo "waiting for /tmp/wp7_gpu.lock $(date +%H:%M:%S)"
flock 9
echo "GPU lock held $(date +%H:%M:%S)  tier $TIER  root $ROOT  $(wc -l < "$PLANFILE") cells"

EXTRA=()
[ "$SMOKE" = 1 ] && EXTRA=(--endindex 250)

# c2_after "DATASET SEQUENCE" ARM... -> per-dataset stop marker for every arm that breaks on the sequence
c2_after() {
  local ds sq arm
  read -r ds sq <<<"$1"; shift
  for arm in "$@"; do
    [ -e "$ROOT/STOPPED_${arm}_$ds" ] && continue
    if breakage "$arm" "$ds" "$sq"; then
      echo "C2 breakage on $ds $sq at $(date)" > "$ROOT/STOPPED_${arm}_$ds"
      echo "!!!!! arm $arm STOPPED on $ds: C2 breakage on $ds $sq (never resumed)"
    fi
  done
}

prev=""
ran=()
while read -r arm ds sq n <&3; do
  if [ "$ds $sq" != "$prev" ]; then
    [ -n "$prev" ] && c2_after "$prev" "${ran[@]}"
    prev="$ds $sq"; ran=()
    echo "##### $ds $sq $(date)"
  fi
  if [ -e "$ROOT/STOPPED_${arm}_$ds" ]; then
    echo "=== skip arm=$arm $ds $sq (stopped: $(cat "$ROOT/STOPPED_${arm}_$ds")) ==="; continue
  fi
  ran+=("$arm")
  [ "$SMOKE" = 1 ] && gate_thermal
  gpu_quiet
  campaign_run "$ROOT" "$arm" "$ds" "$sq" "$n" "${EXTRA[@]}"
done 3< "$PLANFILE"
[ -n "$prev" ] && c2_after "$prev" "${ran[@]}"
echo "##### wp4 tier $TIER done $(date)"
