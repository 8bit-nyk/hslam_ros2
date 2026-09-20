#!/usr/bin/env bash
# WP2a R1-esc (pre-registered, DECISIONS.md WP2a): every (fix arm, ABL-10 sequence) that fails R1 at n=5
# is re-run at n=10 for BOTH arms (5 more reps each, separate --out <arm>_esc / full_esc) and judged at
# n=10 by the same inequality. Self-contained on purpose: it lives outside the repo and changes neither
# the commit nor the binary the wp2a rows ran on. Waits for tmux session 'wp2a' to end first.
#   tmux new -s wp2a_esc;  bash ~/wp2a_escalate.sh 2>&1 | tee ~/wp2a_esc.log
set -uo pipefail
cd ~/Dev/hslam_ros2_ws/src/HSLAM
PY=~/Dev/evo/evo_env/bin/python
ROOT=runs/wp2a_eval-server
while tmux has-session -t "=wp2a" 2>/dev/null; do echo "$(date) waiting for wp2a"; sleep 300; done
echo "wp2a ended $(date); commit $(git rev-parse --short HEAD) binary $(sha256sum build/bin/HSLAM | cut -c1-16)"
FAILS=$("$PY" - "$ROOT" <<'PYEOF'
import csv, glob, os, statistics as st, sys
root = sys.argv[1]
ABL = {("tum", s) for s in ("freiburg1_desk", "freiburg1_room", "freiburg2_desk", "freiburg2_large_no_loop",
       "freiburg3_long_office_household")} | {("kitti", s) for s in ("00", "05", "06", "07", "10")}
rows = {}
for f in glob.glob(os.path.join(root, "*", "summary.csv")):
    arm = os.path.basename(os.path.dirname(f))
    if arm.endswith("_esc") or arm.endswith("_r0ext"):
        continue
    for r in csv.DictReader(open(f)):
        k = (arm, r["dataset"], r["sequence"])
        if (r["dataset"], r["sequence"]) in ABL and r["status"] == "OK" and r["track_success"] == "1":
            rows.setdefault(k, []).append(float(r["ate_sim3_rmse"]))
def iqr(v):
    s = sorted(v); return (s[3 * len(s) // 4] - s[len(s) // 4]) if len(s) > 3 else (s[-1] - s[0])
for arm in ("K13_K14_K15", "K13_fresh", "K15_blendfix", "K14_init_median"):
    for ds, sq in sorted(ABL):
        ref = rows.get(("full", ds, sq)); g = rows.get((arm, ds, sq))
        if not ref or g is None:
            continue
        fail = (not g) or st.median(g) > st.median(ref) + iqr(ref) or len(g) < len(ref)
        if fail:
            print(arm, ds, sq)
PYEOF
)
echo "R1 failures at n=5 (arm dataset sequence):"; echo "$FAILS"
[ -z "$FAILS" ] && { echo "nothing to escalate"; exit 0; }
declare -A DONE_FULL
while read -r arm ds sq; do
  [ -z "$arm" ] && continue
  echo "=== escalate $arm $ds $sq -> n=10  $(date +%H:%M) ==="
  "$PY" run_scripts/ral_v2/eval_run.py --dataset "$ds" --sequence "$sq" --arm "$arm" --reps 5 --out "$ROOT/${arm}_esc" || echo "!!! failed $arm $ds $sq"
  if [ -z "${DONE_FULL[$ds/$sq]:-}" ]; then
    "$PY" run_scripts/ral_v2/eval_run.py --dataset "$ds" --sequence "$sq" --arm full --reps 5 --out "$ROOT/full_esc" || echo "!!! failed full $ds $sq"
    DONE_FULL[$ds/$sq]=1
  fi
done <<< "$FAILS"
echo "=== WP2a escalation done: $(date) ==="
