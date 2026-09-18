#!/usr/bin/env bash
# WP1 — corrected geometry as the paper config, and the cadence lever.
#
# Pre-registered in docs/ral_v2_resubmission/DECISIONS.md ("WP1 ... PRE-REGISTERED 2026-09-18").
# Read that before changing anything here. In short:
#
#   arms      full (paper config)  and  K9_n3 (+ --ml-inference-every-n 3)
#   sequences TUM fr1_room, KITTI 07, KITTI 00
#   G1        geometry is adopted unconditionally; cadence 3 ships only if scale stays in
#             [0.85, 1.15] on fr1_room and KITTI 07 AND ATE moves less than noise everywhere;
#             real-time is worded per platform and NO further lever may be invented.
#
# fp16 is parked and no-pad was killed on its own offline check, so cadence 3 is the only
# surviving throughput lever -- which is why this campaign is two arms and not five.
#
# Usage:
#   tmux new -s wp1
#   REPS=5 bash run_scripts/ral_v2/wp1_campaign.sh 2>&1 | tee ~/wp1.log     # eval server
#   REPS=3 bash run_scripts/ral_v2/wp1_campaign.sh 2>&1 | tee ~/wp1_laptop.log
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/../.."          # -> HSLAM/
REPS="${REPS:-5}"
OUT="${OUT:-runs/wp1_$(hostname -s)}"

PY=python3
[ -x "$HOME/Dev/evo/evo_env/bin/python" ] && PY="$HOME/Dev/evo/evo_env/bin/python"
"$PY" -c 'import evo' 2>/dev/null || { echo "ERROR: no python with evo"; exit 2; }

if [ -n "$(git status --porcelain)" ]; then
    echo "ERROR: working tree is dirty. WP1 rows must be reproducible from their commit."
    git status --short | head; exit 3
fi

echo "host   : $(hostname)"
echo "commit : $(git rev-parse --short HEAD)"
echo "gpu    : $(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null || echo NONE)"
echo "reps   : $REPS"
echo "out    : $OUT"
echo

for arm in full K9_n3; do
    for seq in "tum freiburg1_room" "kitti 07" "kitti 00"; do
        set -- $seq
        echo "=== arm=$arm  $1 $2  (n=$REPS) ==="
        "$PY" run_scripts/ral_v2/eval_run.py \
            --dataset "$1" --sequence "$2" --arm "$arm" --reps "$REPS" --out "$OUT"
        echo
    done
done

echo "=== WP1 summary ($(hostname -s)) ==="
"$PY" - "$OUT/summary.csv" <<'PYEOF'
import csv, statistics as st, sys
rows = [r for r in csv.DictReader(open(sys.argv[1]))
        if r["status"] == "OK" and r["track_success"] == "1"]
key = lambda r: (r["arm"], r["dataset"], r["sequence"])
CAP = {"tum": 30.0, "kitti": 10.0}
groups = {}
for r in rows:
    groups.setdefault(key(r), []).append(r)

print(f"{'arm':7s} {'seq':20s} {'n':>3s} {'ATE_sim3':>9s} {'IQR':>7s} {'ATE_se3':>9s} "
      f"{'scale':>7s} {'fps':>6s} {'rt':>4s}")
for k in sorted(groups):
    g = groups[k]
    med = lambda f: st.median(float(r[f]) for r in g)
    s = sorted(float(r["ate_sim3_rmse"]) for r in g)
    iqr = s[3*len(s)//4] - s[len(s)//4] if len(s) > 3 else float("nan")
    fps = med("pipeline_fps")
    rt = "yes" if fps >= CAP[k[1]] else "NO"
    print(f"{k[0]:7s} {k[1]+' '+k[2]:20s} {len(g):3d} {med('ate_sim3_rmse'):9.4f} {iqr:7.4f} "
          f"{med('ate_se3_rmse'):9.4f} {med('scale_s'):7.4f} {fps:6.2f} {rt:>4s}")

# G1 rule 2: cadence 3 ships only if scale stays in [0.85,1.15] on fr1_room and k07 and ATE
# moves less than noise. Reported, not decided here -- the verdict goes in DECISIONS.md.
print("\n--- cadence 3 vs paper config (G1 rule 2) ---")
for ds, sq in (("tum", "freiburg1_room"), ("kitti", "07"), ("kitti", "00")):
    a = groups.get(("full", ds, sq)); b = groups.get(("K9_n3", ds, sq))
    if not a or not b:
        continue
    fa = st.median(float(r["ate_sim3_rmse"]) for r in a)
    fb = st.median(float(r["ate_sim3_rmse"]) for r in b)
    sa = sorted(float(r["ate_sim3_rmse"]) for r in a)
    noise = (sa[3*len(sa)//4] - sa[len(sa)//4]) if len(sa) > 3 else float("nan")
    sb = st.median(float(r["scale_s"]) for r in b)
    in_band = 0.85 <= sb <= 1.15
    dfps = st.median(float(r["pipeline_fps"]) for r in b) - st.median(float(r["pipeline_fps"]) for r in a)
    verdict = "within noise" if abs(fb - fa) < noise else "EXCEEDS noise"
    print(f"  {ds} {sq:16s} dATE {100*(fb/fa-1):+6.1f}% ({verdict}, IQR {noise:.4f})  "
          f"scale {sb:.4f} {'in' if in_band else 'OUT OF'} [0.85,1.15]  dfps {dfps:+.2f}")
PYEOF
