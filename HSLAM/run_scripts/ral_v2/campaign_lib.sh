#!/usr/bin/env bash
# Shared campaign runner template (pre-WP4, 2026-09-25). SOURCE it from a campaign script:
#
#   set -euo pipefail
#   cd "$(dirname "${BASH_SOURCE[0]}")/../.."
#   source run_scripts/ral_v2/campaign_lib.sh
#   campaign_preflight "<expected 16-hex epoch>" init-fail-thresholds init-founding-fix
#   campaign_run <root> <arm> <dataset> <sequence> <n>
#
# What it enforces, and why (AUDIT_pre_WP4_2026-09-24.md §6.2 / code audit D10-D11):
#   * EPOCH. The binary's sha256 (16 hex) must equal the pre-registered epoch, or rows from two
#     builds would pool under one commit string -- make_tables.py pools by commit and cannot see a
#     stale build. eval_run.py also stamps the hash on every row (`binary` column).
#   * CLEAN TREE. A dirty tree stamps every row dirty=1, which make_tables.py refuses.
#   * RESUME WITHOUT DUPLICATES. eval_run.py appends unconditionally and writes a (arm, sequence)'s
#     rows only after ALL reps of that invocation finish, so summary.csv is the progress record and
#     every row in it is a completed run. campaign_run tops up only the MISSING reps, numbering them
#     after the existing ones (--rep-start), instead of re-running the whole batch as the old have()
#     guard did when n grew.
#
#     It counts rows of ANY status, deliberately. A completed run that crashed, lost tracking or
#     stamped NO_ML is a measurement (house rule: failures are reported, never dropped). Counting
#     only status=OK rows would re-run exactly those, until a success replaced them -- survivorship
#     bias built into the runner. Interrupted invocations write no row at all, so nothing partial is
#     ever counted. (This departs from the kickoff card's "OK-only" wording on purpose; the reason is
#     recorded in DECISIONS.md "Re-freeze epoch -- code changes and their classes".)
#
#     Caveat: a top-up is a separate invocation, so its reps get their own track-success pool
#     (eval_run.py pools per invocation). Uninterrupted campaigns never top up.

PY=python3
[ -x "$HOME/Dev/evo/evo_env/bin/python" ] && PY="$HOME/Dev/evo/evo_env/bin/python"

# campaign_preflight EPOCH [required-flag ...]
campaign_preflight() {
  local epoch="$1"; shift
  "$PY" -c 'import evo' 2>/dev/null || { echo "ERROR: no python with evo"; exit 2; }
  if [ -n "$(git status --porcelain)" ]; then echo "ERROR: dirty tree"; git status --short | head; exit 3; fi
  local sha
  sha="$(sha256sum build/bin/HSLAM | cut -c1-16)"
  [ "$sha" = "$epoch" ] || { echo "ERROR: binary $sha != pre-registered epoch $epoch -- rows would not pool"; exit 7; }
  local f
  for f in "$@"; do
    ./build/bin/HSLAM --help 2>&1 | grep -q -- "--$f" || { echo "ERROR: binary lacks --$f"; exit 5; }
  done
  echo "host $(hostname)  commit $(git rev-parse --short HEAD)  binary $sha  start $(date)"
}

# campaign_count ROOT ARM DATASET SEQUENCE -> number of completed rows (any status)
campaign_count() {
  "$PY" -c 'import csv, sys
p, ds, sq = sys.argv[1], sys.argv[2], sys.argv[3]
try:
    rows = list(csv.DictReader(open(p)))
except FileNotFoundError:
    print(0); sys.exit(0)
print(sum(1 for r in rows if r["dataset"] == ds and r["sequence"] == sq))' "$1/$2/summary.csv" "$3" "$4"
}

# campaign_run ROOT ARM DATASET SEQUENCE N [extra eval_run.py args ...]
campaign_run() {
  local root="$1" arm="$2" ds="$3" sq="$4" n="$5"; shift 5
  local have
  have="$(campaign_count "$root" "$arm" "$ds" "$sq")"
  if [ "$have" -ge "$n" ]; then echo "=== skip arm=$arm  $ds $sq: $have/$n rows ==="; return 0; fi
  local todo=$((n - have)) start=$((have + 1))
  echo "=== arm=$arm  $ds $sq  reps $start..$n  $(date +%H:%M) ==="
  "$PY" run_scripts/ral_v2/eval_run.py --dataset "$ds" --sequence "$sq" --arm "$arm" \
    --reps "$todo" --rep-start "$start" --out "$root/$arm" "$@" \
    || echo "!!! eval_run exited non-zero: $arm $ds $sq (continuing; the rows say why)"
  echo
}

# gate_thermal: laptop only -- wait until GPU <= 70 C and CPU package <= 78 C between runs
# (memory: feedback_thermal_protection). No-op where the sensors are missing.
gate_thermal() {
  local g c
  while :; do
    g="$(nvidia-smi --query-gpu=temperature.gpu --format=csv,noheader,nounits 2>/dev/null | head -1)"
    c="$(sensors 2>/dev/null | awk '/Package id 0/{gsub(/[+°C]/,"",$4); print int($4)}')"
    [ -z "$g" ] && g=0; [ -z "$c" ] && c=0
    [ "$g" -le 70 ] && [ "$c" -le 78 ] && return 0
    echo "$(date +%H:%M:%S) thermal gate: GPU ${g}C CPU ${c}C -- waiting"; sleep 20
  done
}
