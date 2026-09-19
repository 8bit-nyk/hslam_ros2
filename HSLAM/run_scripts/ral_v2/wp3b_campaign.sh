#!/usr/bin/env bash
# WP3b -- EuRoC mechanism investigation (user-requested, 2026-09-19). DIAGNOSTIC, not a paper WP.
#
# Pre-registered in docs/ral_v2_resubmission/DECISIONS.md ("WP3b ... PRE-REGISTERED 2026-09-19")
# and docs/ral_v2_resubmission/wp/WP3b_euroc_mechanism_investigation.md. Read those first.
#
# WP3 (19 Sep) characterised EuRoC -- scale self-consistent but ~45 % off, Sim(3) ATE 2-26x
# monocular -- without diagnosing it. The WP3 artefacts show the ML arm's per-keyframe RPE is
# 2.5-20x monocular and its scale drift 5-10x monocular while the in-run prior/photometric ratio
# sits at 1.00: the map FOLLOWS the prior. This campaign attributes the damage to a consumer of
# the prior (knock-outs), separates "the prior keeps feeding the map" from "the prior sets the
# scale" (init-only), and measures whether the failure is sequence-, scene- or camera-level
# (the eight EuRoC sequences WP3 did not run).
#
# Every arm writes to its OWN directory. eval_run.py names run dirs <dataset>_<seq>_rep<k>, so
# two arms sharing --out overwrite each other's run.log and result.txt -- WP3 lost its A0 and
# legacy_geom trajectories that way; only summary.csv rows survived.
#
# Usage:
#   tmux new -s wp3b
#   REPS=3 bash run_scripts/ral_v2/wp3b_campaign.sh 2>&1 | tee ~/wp3b.log
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/../.."          # -> HSLAM/
REPS="${REPS:-3}"
ROOT="${OUT:-runs/wp3b_$(hostname -s)}"

PY=python3
[ -x "$HOME/Dev/evo/evo_env/bin/python" ] && PY="$HOME/Dev/evo/evo_env/bin/python"
"$PY" -c 'import evo' 2>/dev/null || { echo "ERROR: no python with evo"; exit 2; }

if [ -n "$(git status --porcelain)" ]; then
    echo "ERROR: working tree is dirty."; git status --short | head; exit 3
fi

echo "=== EuRoC timestamp preparation (all 11 sequences) ==="
"$PY" run_scripts/ral_v2/prepare_euroc.py --check || { echo "ERROR: run prepare_euroc.py"; exit 4; }
echo
echo "host   : $(hostname)"
echo "commit : $(git rev-parse --short HEAD)"
echo "binary : $(sha256sum build/bin/HSLAM | cut -c1-16)  (must equal the WP3 binary -- see DECISIONS.md)"
echo "gpu    : $(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null || echo NONE)"
echo "reps   : $REPS"
echo "out    : $ROOT/<arm>/"
echo

run() {   # run <arm> <dataset> <seq> <reps>
    local arm=$1 dataset=$2 seq=$3 reps=$4
    echo "=== arm=$arm  $dataset $seq  (n=$reps) ==="
    "$PY" run_scripts/ral_v2/eval_run.py \
        --dataset "$dataset" --sequence "$seq" --arm "$arm" --reps "$reps" --out "$ROOT/$arm"
    echo
}

# ---------------------------------------------------------------- Part 1: knock-outs, EuRoC-3
# Order chosen so the most diagnostic arms land first if the campaign is interrupted.
SEQ3="MH_01_easy V1_01_easy V2_02_medium"
for arm in A0 K1 M1_init_only A1 K3 K2 K11 S1_unc_hi K10; do
    for seq in $SEQ3; do run "$arm" euroc "$seq" "$REPS"; done
done
for seq in $SEQ3; do run full_diag euroc "$seq" 1; done

# ---------------------------------------------------------------- Part 2: the other 8 EuRoC sequences
SEQ8="MH_02_easy MH_03_medium MH_04_difficult MH_05_difficult V1_02_medium V1_03_difficult V2_01_easy V2_03_difficult"
for seq in $SEQ8; do
    run A0   euroc "$seq" "$REPS"
    run full euroc "$seq" "$REPS"
done

echo "=== WP3b done: $(date) ==="
