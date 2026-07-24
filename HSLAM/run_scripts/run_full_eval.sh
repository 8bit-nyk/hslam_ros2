#!/bin/bash
# run_full_eval.sh — Master evaluation script
#
# Runs HSLAM on all three datasets sequentially, then invokes evo evaluation
# and prints a compact iteration summary.
#
# Invoke after a successful build to get a full pass/fail verdict on the change.
#
# Usage:
#   ./run_scripts/run_full_eval.sh [--skip-tum] [--skip-kitti] [--skip-euroc]
#
# Output: iteration_summary_<timestamp>.txt in hslam_evaluation/results/

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Portable across machines with different $HOME (laptop=/home/aub, eval-server=/home/nyk).
# Override EVO_ROOT if evo lives elsewhere.
EVO_ROOT="${EVO_ROOT:-$HOME/Dev/evo}"
EVO_EVAL_SCRIPT="$EVO_ROOT/hslam_evaluation/scripts/run_hslam_eval_all.sh"
EVO_ACTIVATE="$EVO_ROOT/activate_evo.sh"

SKIP_TUM=false; SKIP_KITTI=false; SKIP_EUROC=false
EVO_ARGS=(-a "$EVO_ACTIVATE")

usage() {
    cat <<EOF
Usage: $0 [OPTIONS]

Runs HSLAM on all three benchmark datasets, then evaluates with EVO and prints
a compact iteration summary (ATE / Scale / RPE per dataset).

OPTIONS:
    --skip-tum      Skip TUM freiburg1_room
    --skip-kitti    Skip KITTI 07
    --skip-euroc    Skip EuRoC MH_01_easy
    -h, --help      Show this help
EOF
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-tum)   SKIP_TUM=true;   EVO_ARGS+=(--skip-tum);   shift ;;
        --skip-kitti) SKIP_KITTI=true; EVO_ARGS+=(--skip-kitti); shift ;;
        --skip-euroc) SKIP_EUROC=true; EVO_ARGS+=(--skip-euroc); shift ;;
        -h|--help)    usage ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

echo "========================================================================"
echo "  HSLAM Full Evaluation Pipeline"
echo "  TUM: $(  [[ "$SKIP_TUM"   == false ]] && echo "YES" || echo "SKIP")  |  KITTI: $([[ "$SKIP_KITTI" == false ]] && echo "YES" || echo "SKIP")  |  EuRoC: $([[ "$SKIP_EUROC" == false ]] && echo "YES" || echo "SKIP")"
echo "========================================================================"
echo ""

# ---------------------------------------------------------------------------
# Step 1: Run HSLAM on each dataset
# ---------------------------------------------------------------------------
[[ "$SKIP_TUM"   == false ]] && "$SCRIPT_DIR/hslam_run_tumrgbd_ml_depth.sh" freiburg1_room
[[ "$SKIP_KITTI" == false ]] && "$SCRIPT_DIR/hslam_run_kitti_ml_depth.sh"   07
[[ "$SKIP_EUROC" == false ]] && "$SCRIPT_DIR/hslam_run_euroc_ml_depth.sh"   MH_01_easy

# ---------------------------------------------------------------------------
# Step 2: Evaluate with EVO — reads from latest-* symlinks, prints summary
# ---------------------------------------------------------------------------
echo ""
echo "========================================================================"
echo "  EVO Evaluation"
echo "========================================================================"
"$EVO_EVAL_SCRIPT" "${EVO_ARGS[@]}"
