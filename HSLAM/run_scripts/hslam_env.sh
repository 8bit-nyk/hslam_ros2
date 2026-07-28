#!/bin/bash
# Shared runtime environment for HSLAM run scripts. SOURCE this, do not execute it:
#     source "$(dirname "${BASH_SOURCE[0]}")/hslam_env.sh"
#
# Idempotent and safe to source repeatedly.
#
# It does two things:
#
# 1. ONNX Runtime (GPU) on LD_LIBRARY_PATH — required for ML depth inference.
#
# 2. SONAME compat shims. The PREBUILT `Thirdparty/CompiledLibs/lib/libpangolin.so` was linked
#    against `librealsense2.so.2.55`; Ubuntu 22.04 now ships 2.58, so a fresh checkout fails at
#    load time with "librealsense2.so.2.55 => not found". Note the HSLAM binary *itself* correctly
#    links 2.58 — only the stale prebuilt Pangolin is affected.
#
#    THE PROPER FIX IS TO REBUILD PANGOLIN (`bash Thirdparty/build.sh`, or `setup.sh thirdparty`
#    on the workstation), which relinks it against the installed realsense. This shim exists so an
#    existing checkout runs without a ~20 min Thirdparty rebuild; it is a stopgap, not the answer.
#
#    The map below is an EXPLICIT allow-list on purpose. A generic "symlink any missing SONAME to
#    whatever version is installed" scan would silently substitute an arbitrary ABI and can crash at
#    runtime. Only add an entry here when you have reason to believe the versions are compatible.

_HSLAM_ENV_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# --- 1. ONNX Runtime -----------------------------------------------------------------------------
if [[ ":${LD_LIBRARY_PATH:-}:" != *":$_HSLAM_ENV_ROOT/Thirdparty/onnxruntime/lib:"* ]]; then
    export LD_LIBRARY_PATH="$_HSLAM_ENV_ROOT/Thirdparty/onnxruntime/lib:${LD_LIBRARY_PATH:-}"
fi

# --- 2. SONAME compat shims ----------------------------------------------------------------------
# "<missing soname>|<glob of acceptable installed replacements>"
_HSLAM_COMPAT_MAP=(
    "librealsense2.so.2.55|/usr/lib/x86_64-linux-gnu/librealsense2.so.2.5[6-9]*"
)

_HSLAM_COMPAT_DIR="$_HSLAM_ENV_ROOT/Thirdparty/CompiledLibs/lib_compat"

hslam_setup_lib_compat() {
    local quiet="${1:-quiet}"
    local entry soname glob target made=0
    for entry in "${_HSLAM_COMPAT_MAP[@]}"; do
        soname="${entry%%|*}"; glob="${entry#*|}"
        # Already resolvable? Then the shim is unnecessary (e.g. Pangolin was rebuilt).
        if ldconfig -p 2>/dev/null | grep -q "[[:space:]]$soname[[:space:]]"; then continue; fi
        [ -e "$_HSLAM_COMPAT_DIR/$soname" ] && continue
        # shellcheck disable=SC2086
        target="$(ls -1 $glob 2>/dev/null | sort -V | tail -1)"
        if [ -z "$target" ]; then
            [ "$quiet" = "verbose" ] && echo "  [lib_compat] no installed replacement for $soname — skipping"
            continue
        fi
        mkdir -p "$_HSLAM_COMPAT_DIR"
        ln -sfn "$target" "$_HSLAM_COMPAT_DIR/$soname"
        made=1
        [ "$quiet" = "verbose" ] && echo "  [lib_compat] $soname -> $target (stopgap; rebuild Pangolin for the real fix)"
    done
    if [ -d "$_HSLAM_COMPAT_DIR" ] && [[ ":${LD_LIBRARY_PATH:-}:" != *":$_HSLAM_COMPAT_DIR:"* ]]; then
        export LD_LIBRARY_PATH="$_HSLAM_COMPAT_DIR:${LD_LIBRARY_PATH:-}"
    fi
    return 0
}

hslam_setup_lib_compat "${HSLAM_ENV_VERBOSE:-quiet}"

# --- 3. Optional sanity check --------------------------------------------------------------------
# hslam_check_libs <path-to-binary> — prints unresolved SONAMEs, returns 1 if any. Non-fatal by
# design: callers decide whether a missing lib is fatal for their mode.
hslam_check_libs() {
    local bin="$1" missing
    [ -x "$bin" ] || return 0
    missing="$(ldd "$bin" 2>/dev/null | grep 'not found' || true)"
    if [ -n "$missing" ]; then
        echo "  [lib_compat] WARNING — unresolved shared libraries for $bin:"
        echo "$missing" | sed 's/^/    /'
        echo "    Add an entry to _HSLAM_COMPAT_MAP in run_scripts/hslam_env.sh, or rebuild Thirdparty."
        return 1
    fi
    return 0
}
