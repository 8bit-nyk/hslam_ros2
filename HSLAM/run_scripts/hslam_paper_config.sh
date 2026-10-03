#!/bin/bash
# Paper (metric-scale) configuration for the plain run scripts. SOURCE this, do not execute it:
#     source "$HSLAM_SCRIPT_DIR/hslam_paper_config.sh"
#
# WHY THIS EXISTS. The C++ defaults for the flags that make the depth prior metric
# (--ml-canonical-scale, --ml-init-scale=median, --p1-blend-grad-fix, --ml-isotropic-input, ...) are OFF,
# on purpose: flipping them would change every frozen eval row and every older track. So a bare
# `HSLAM --ml-depth --ml-model ...` runs, tracks, and returns a trajectory at the WRONG scale, silently.
# The configuration that gives metric scale lives in ral_v2/arms.py (PAPER_CONFIG); this file hands it to
# the run scripts so nobody retypes the flags and nothing drifts from the eval pipeline.
#
#   hslam_paper_flags <dataset> [model]
#                                 -> sets HSLAM_ARM_FLAGS (the arm's flags, shell-quoted; use with eval) and
#                                    HSLAM_CONFIG_NAME (for the banner).
#                                    dataset = tum | kitti | euroc | iclnuim (datasets.py keys).
#                                    --ml-isotropic-input is added automatically where fx != fy.
#   hslam_scale_check <dataset> <sequence> <trajectory> <log>
#                                 -> post-run: was it the paper config, and is the scale ~1?
#
# Opt-out for ablation work only:  HSLAM_LEGACY_FLAGS=1  restores the old bare flag set. The run is then
# NOT metric-scale, and the banner and the scale check say so.

_HSLAM_PC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_HSLAM_PC_PY="${HSLAM_PYTHON:-python3}"

hslam_paper_flags() {
    local dataset="$1" model="${2:-}"
    if [ "${HSLAM_LEGACY_FLAGS:-0}" = "1" ]; then
        echo -e "\033[1;33m[WARN]\033[0m HSLAM_LEGACY_FLAGS=1: NOT the metric-scale configuration (ablation use only)"
        HSLAM_ARM_FLAGS="--ml-depth --ml-gpu --ml-model ${model} --ml-strategy keyframe_only --ml-init=true"
        HSLAM_CONFIG_NAME="LEGACY (not metric scale)"
        return 0
    fi
    # arms.py is stdlib-only, so plain python3 is enough here (numpy/evo are needed only for the scale check).
    if ! HSLAM_ARM_FLAGS="$("$_HSLAM_PC_PY" "$_HSLAM_PC_DIR/ral_v2/arms.py" --print-cli "$dataset")"; then
        echo -e "\033[1;31m[ERROR]\033[0m could not resolve the paper config for dataset '$dataset' (ral_v2/arms.py)" >&2
        return 1
    fi
    HSLAM_CONFIG_NAME="PAPER CONFIG (metric scale), ral_v2/arms.py"
    return 0
}

hslam_scale_check() {
    # Never fails the run: it reports. Needs numpy+evo for the trajectory half; the log half is stdlib.
    local py="$_HSLAM_PC_PY"
    if ! "$py" -c "import numpy, evo" 2>/dev/null && [ -x "$HOME/Dev/evo/evo_env/bin/python" ]; then
        py="$HOME/Dev/evo/evo_env/bin/python"
    fi
    "$py" "$_HSLAM_PC_DIR/ral_v2/scale_check.py" "$1" "$2" "$3" --log "$4"
    local rc=$?
    case $rc in
        0) echo -e "\033[1;32m[SUCCESS]\033[0m scale check passed" ;;
        2) echo -e "\033[1;33m[WARN]\033[0m config OK but scale outside the expected band -- see above" ;;
        *) echo -e "\033[1;31m[ERROR]\033[0m scale check FAILED -- this run should not be used as a metric-scale result" ;;
    esac
    return 0
}
