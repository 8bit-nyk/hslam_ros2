#!/usr/bin/env python3
"""RA-L v2 eval driver: run HSLAM once, evaluate it, append one summary.csv row.

Every number in the v2 paper comes from a summary.csv produced here, at one frozen commit
and one CLI line (PLAN.md P3, P8). Nothing is hand-typed and no v1 number is reused.

Two rules this script exists to enforce, both learned the hard way:

 1. A run whose [RUN_SUMMARY] says status != OK is a FAILED row whatever its exit code,
    and its fps is never recorded. Observed on 2026-09-18: an fp16 warmup failure disabled
    ML, the run finished monocular, exited 0, and reported pipeline_fps=837.44. A fast,
    plausible fps from a run that did no ML is exactly the kind of number that produced
    the v1 record.

 2. pipeline_fps comes from [PERF_SUMMARY], never from the console "N Frames (X fps)"
    line, which is the dataset capture rate.

Usage
    eval_run.py --dataset kitti --sequence 07 --arm full --reps 5 --out runs/wp0_smoke
    eval_run.py --dataset tum --sequence freiburg1_room --arm full --extra --ml-alpha-w 2500
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import re
import shlex
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import arms as arms_mod          # noqa: E402
import datasets as ds            # noqa: E402
import traj_eval                 # noqa: E402

HSLAM_ROOT = Path(__file__).resolve().parents[2]
# HSLAM_BINARY overrides the default for laptop smokes (the laptop builds in build_wp3c/). Campaigns
# run the default, and every row records the binary's hash, so an override can never pass unseen.
BINARY = Path(os.environ.get("HSLAM_BINARY", HSLAM_ROOT / "build" / "bin" / "HSLAM"))

# Per-distance RPE pair spacing (pre-WP4, pre-registered in DECISIONS.md "Re-freeze epoch"): 1 m on the
# room- and building-scale datasets, 100 m on KITTI (the shortest KITTI benchmark segment).
RPE_DELTA_M = {"tum": 1.0, "iclnuim": 1.0, "euroc": 1.0, "tummonovo": 1.0, "kitti": 100.0}

COLUMNS = [
    "timestamp", "commit", "dirty", "host", "evo_version", "arm", "dataset", "sequence", "rep",
    "returncode", "status", "track_success", "wall_s",
    "frames", "poses", "matched_poses", "keyframes", "ml_inferences",
    "ate_sim3_rmse", "ate_se3_rmse", "scale_s", "scale_drift_pct_per_100m",
    "rpe_trans", "rpe_rot", "gt_distance_m",
    "pipeline_fps", "track_fps", "ml_ms", "capture_hz", "realtime",
    "peak_gpu_mb", "peak_cpu_mb",
    "lc_gate_enabled", "lc_gate_evals", "lc_gate_fires", "lc_gate_bypass",
    "agreement_gate_fire_rate",
    "prior_align_n", "prior_align_s_med", "prior_align_s_iqr",
    "geom", "canon", "iso", "idepth_prior", "ml_gpu", "fp16", "model",
    "ts_source", "ts_hz", "ts_plausible",
    "traj_error", "cli",
    # pre-WP4 (2026-09-25), appended so every earlier column keeps its position
    "binary",                                       # sha256 of the binary, 16 hex (the epoch string)
    "drift_local_pct_per_100m", "drift_local_pct_path", "drift_postfound_pct_path",
    "founding_offset_log",                          # windowed drift; scale_drift_pct_per_100m is provenance
    "rpe_dist_m", "rpe_trans_dist", "rpe_rot_dist",  # RPE per fixed distance; rpe_trans/rot are per-KF
    "postinit_frames", "postinit_fps",              # cost DV: frames and wall time over the same span
    "init_thresh", "init_thresh_mode", "init_mode", "init_resets",   # D6, V9
    "founding_fix", "lc_sim3_guard",                # D2, D4
]


# ---------------------------------------------------------------- provenance

def toolchain() -> tuple[str, str]:
    """Host and evo version, recorded per row.

    The laptop and the eval server carry different evo releases (1.31 vs 1.37). The metrics
    are stable across them, but a paper number should still name the toolchain that produced
    it, and a row that cannot say where it ran cannot be reproduced.
    """
    import platform
    try:
        import evo
        ver = str(evo.__version__)
    except Exception:
        ver = "unknown"
    return platform.node(), ver


def git_commit() -> tuple[str, bool]:
    def g(*a):
        return subprocess.run(["git", *a], cwd=HSLAM_ROOT, capture_output=True,
                              text=True).stdout.strip()
    return g("rev-parse", "--short", "HEAD"), bool(g("status", "--porcelain"))


def binary_hash(path: Path) -> str:
    """First 16 hex of the binary's sha256 -- the epoch string campaign scripts check (audit D10).

    The commit alone cannot detect a stale build: make_tables.py pools by commit, and a binary built
    before the last pull carries the new commit's name.
    """
    import hashlib
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:16]


# ---------------------------------------------------------------- GPU sampling

class ResourceSampler(threading.Thread):
    """Sample peak GPU and host memory for one HSLAM process.

    Host RSS comes from /proc/<pid>/status VmHWM, the kernel's own high-water mark (the same
    number the `time -v` utility reports) -- so we do NOT wrap the binary in a timing wrapper.
    Wrapping was the first attempt and it silently broke GPU attribution: nvidia-smi reports
    the *HSLAM* pid, while Popen held the wrapper's pid, so no row ever matched and
    peak_gpu_mb came back nan on every run.

    Both are best-effort: a missing nvidia-smi or an already-exited process yields nan rather
    than failing the run. nan is honest; 0 would read as "used no memory".
    """

    def __init__(self, pid: int, interval: float = 0.5):
        super().__init__(daemon=True)
        self.pid, self.interval = pid, interval
        self.peak_gpu_mb = float("nan")
        self.peak_cpu_mb = float("nan")
        self._done = threading.Event()

    def _sample_gpu(self) -> int:
        try:
            out = subprocess.run(
                ["nvidia-smi", "--query-compute-apps=pid,used_memory",
                 "--format=csv,noheader,nounits"],
                capture_output=True, text=True, timeout=5).stdout
            for line in out.strip().splitlines():
                parts = [p.strip() for p in line.split(",")]
                if len(parts) == 2 and parts[0].isdigit() and int(parts[0]) == self.pid:
                    return int(parts[1])
        except Exception:
            pass
        return 0

    def _sample_cpu(self) -> int:
        try:
            for line in Path(f"/proc/{self.pid}/status").read_text().splitlines():
                if line.startswith("VmHWM:"):
                    return int(line.split()[1])          # kB
        except Exception:
            pass
        return 0

    def run(self):
        gpu = cpu = 0
        while not self._done.is_set():
            gpu = max(gpu, self._sample_gpu())
            cpu = max(cpu, self._sample_cpu())
            self._done.wait(self.interval)
        cpu = max(cpu, self._sample_cpu())   # short runs can end before the first tick
        if gpu:
            self.peak_gpu_mb = float(gpu)
        if cpu:
            self.peak_cpu_mb = cpu / 1024.0

    def stop(self):
        self._done.set()
        self.join(timeout=3)


# ---------------------------------------------------------------- log parsing

def _f(m, i=1, default=float("nan")):
    try:
        return float(m.group(i))
    except Exception:
        return default


def parse_log(text: str) -> dict:
    d = {k: float("nan") for k in ("pipeline_fps", "track_fps", "ml_ms", "postinit_fps")}
    d.update(status="", frames=0, keyframes=0, ml_inferences=0,
             geom="", canon="", iso="", idepth_prior="", ml_gpu="", fp16="", model="",
             ts_source="", ts_hz=float("nan"), ts_plausible="",
             lc_gate_enabled="", lc_gate_evals=0, lc_gate_fires=0, lc_gate_bypass=0,
             agreement_gate_fire_rate=float("nan"),
             prior_align_n=0, prior_align_s_med=float("nan"), prior_align_s_iqr=float("nan"),
             postinit_frames=0, init_thresh="", init_thresh_mode="", init_mode="",
             init_resets=0, founding_fix="", lc_sim3_guard=0)

    if (m := re.search(r"\[PERF_SUMMARY\][^\n]*", text)):
        line = m.group(0)
        for key, pat in (("track_fps", r"track_fps=([\d.]+)"),
                         ("pipeline_fps", r"pipeline_fps=([\d.]+)"),
                         ("ml_ms", r"ml_ms=([\d.]+)"),
                         ("postinit_fps", r"postinit_fps=([\d.]+)")):
            if (mm := re.search(pat, line)):
                d[key] = float(mm.group(1))
        if (mm := re.search(r"postinit_frames=(\d+)", line)):
            d["postinit_frames"] = int(mm.group(1))

    if (m := re.search(r"\[RUN_SUMMARY\][^\n]*", text)):
        line = m.group(0)
        for key, pat in (("status", r"status=(\w+)"), ("geom", r"geom=(\w+)"),
                         ("canon", r"canon=(\w+)"), ("iso", r"iso=(\w+)"),
                         ("idepth_prior", r"idepth_prior=(\w+)"), ("ml_gpu", r"ml_gpu=(\w+)"),
                         ("fp16", r"fp16=(\w+)"), ("model", r"model=(\S+)"),
                         ("init_thresh", r"\binit_thresh=(\w+)"),
                         ("init_thresh_mode", r"init_thresh_mode=(\w+)"),
                         ("init_mode", r"\binit_mode=(\w+)"), ("founding_fix", r"founding_fix=(\w+)")):
            if (mm := re.search(pat, line)):
                d[key] = mm.group(1)
        for key, pat in (("frames", r"\bframes=(\d+)"), ("keyframes", r"kfs=(\d+)"),
                         ("ml_inferences", r"ml_inferences=(\d+)"),
                         ("init_resets", r"init_resets=(\d+)"), ("lc_sim3_guard", r"lc_sim3_guard=(\d+)")):
            if (mm := re.search(pat, line)):
                d[key] = int(mm.group(1))

    # Timestamp provenance (DatasetReader.h). This is a row-level column, not a diagnostic to
    # grep by hand, because the two worst defects this project has found were both a silently
    # wrong timestamp source: KITTI's scientific-notation misparse (6799be6) and EuRoC's missing
    # times.txt, where the filename fallback reads nanoseconds as seconds. Neither crashes.
    # The LAST line wins (pre-WP4 D7): on the associations path the folder reader prints one for a
    # directory it never reads, and main.cpp then prints the timestamps actually fed.
    if (ms := re.findall(r"\[TIMESTAMPS\] source=(\S+)[^\n]*?\(([\d.]+) Hz\)[^\n]*?plausible=(\w+)",
                         text)):
        d["ts_source"], d["ts_hz"], d["ts_plausible"] = ms[-1][0], float(ms[-1][1]), ms[-1][2]

    if (m := re.search(r"\[LC_SCALE_GATE\][^\n]*", text)):
        line = m.group(0)
        if (mm := re.search(r"enabled=(\w+)", line)):
            d["lc_gate_enabled"] = mm.group(1)
        for key, pat in (("lc_gate_evals", r"evals=(\d+)"), ("lc_gate_fires", r"fires=(\d+)"),
                         ("lc_gate_bypass", r"bypass_coverage_low=(\d+)")):
            if (mm := re.search(pat, line)):
                d[key] = int(mm.group(1))

    # WP2's gate does not exist yet; record its rate only once it starts printing.
    fires = len(re.findall(r"\[AGREEMENT_GATE\]\s+fire", text))
    total = len(re.findall(r"\[AGREEMENT_GATE\]", text))
    if total:
        d["agreement_gate_fire_rate"] = fires / total

    # WP2c: one [PRIOR_ALIGN] line per ML keyframe (s_k = median map/prior depth ratio). G2-d judges the
    # spread of s_k over a run on healthy data: IQR < 0.3 (PLAN.md G2). Kept as columns so the gate
    # threshold can be set from summary.csv without re-reading logs.
    pa = [float(v) for v in re.findall(r"\[PRIOR_ALIGN\][^\n]*?\bs=(-?[\d.]+)", text)]
    pa = [v for v in pa if v == v]
    if pa:
        from statistics import median
        srt = sorted(pa)
        d["prior_align_n"] = len(pa)
        d["prior_align_s_med"] = median(pa)
        d["prior_align_s_iqr"] = srt[(3 * len(srt)) // 4] - srt[len(srt) // 4]
    return d


# ---------------------------------------------------------------- one run

def run_once(spec: ds.SeqSpec, arm_args: list[str], rep: int, outdir: Path,
             endindex: int | None, timeout_s: int) -> dict:
    rundir = outdir / f"{spec.dataset}_{spec.sequence}_rep{rep}"
    rundir.mkdir(parents=True, exist_ok=True)

    # Pick the reader's input mode. --associations changes how image paths resolve, so
    # --files must change with it (see SeqSpec). Associations are used only where they are
    # actually needed: ICL-NUIM for frame ordering, and any GT-depth arm for the depth
    # column. Attaching them to a TUM ML-depth run makes every path <root>/rgb/rgb/... and
    # the run dies with "Could not load RGB image" on every frame.
    gt_depth_arm = any(a.startswith("--depth-source=gt") for a in arm_args)
    use_assoc = (spec.needs_associations or gt_depth_arm) and \
                spec.associations is not None and spec.associations.exists()

    cli = [str(BINARY),
           "--files", str(spec.root if use_assoc else spec.images),
           "--calib", str(spec.calib),
           "--vocab", str(HSLAM_ROOT / "misc" / "orbvoc.dbow3"),
           "--colour", "--nogui=true", "--nolog", "--loopclosure",
           *spec.extra_cli,            # dataset-level flags (photometric calibration), before the arm
           *arm_args]
    if use_assoc:
        cli += ["--associations", str(spec.associations)]
    elif gt_depth_arm:
        raise SystemExit(f"ERROR: {spec.dataset} {spec.sequence}: --depth-source=gt needs "
                         f"associations.txt, not found at {spec.associations}")
    if endindex:
        cli += ["--endindex", str(endindex)]

    env = os.environ.copy()
    ort = HSLAM_ROOT / "Thirdparty" / "onnxruntime" / "lib"
    compat = HSLAM_ROOT / "Thirdparty" / "CompiledLibs" / "lib_compat"
    env["LD_LIBRARY_PATH"] = f"{ort}:{compat}:" + env.get("LD_LIBRARY_PATH", "")

    log_path = rundir / "run.log"

    t0 = time.time()
    with open(log_path, "w") as log:
        proc = subprocess.Popen(cli, cwd=rundir, stdout=log,
                                stderr=subprocess.STDOUT, env=env)
        sampler = ResourceSampler(proc.pid)
        sampler.start()
        try:
            rc = proc.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
            rc = -9
        finally:
            sampler.stop()
    wall = time.time() - t0

    text = log_path.read_text(errors="replace")
    row = parse_log(text)
    row["returncode"] = rc
    row["wall_s"] = round(wall, 1)
    row["peak_gpu_mb"] = sampler.peak_gpu_mb
    row["peak_cpu_mb"] = sampler.peak_cpu_mb

    est = rundir / "result.txt"
    if est.exists() and est.stat().st_size > 0:
        row.update(traj_eval.evaluate(est, spec.gt, spec.gt_format, spec.extrinsics,
                                      rpe_delta_m=RPE_DELTA_M.get(spec.dataset)))
    else:
        row.update({k: float("nan") for k in (
            "ate_sim3_rmse", "ate_se3_rmse", "scale_s", "scale_drift_pct_per_100m",
            "rpe_trans", "rpe_rot", "gt_distance_m",
            "rpe_dist_m", "rpe_trans_dist", "rpe_rot_dist",
            "drift_local_pct_per_100m", "drift_local_pct_path",
            "drift_postfound_pct_path", "founding_offset_log")})
        row.update(poses=0, matched_poses=0, traj_error="no result.txt written")

    # Rule 1: a non-OK status invalidates the whole row, fps included.
    if row["status"] != "OK" or rc != 0:
        row["pipeline_fps"] = float("nan")
        row["track_fps"] = float("nan")
        row["postinit_fps"] = float("nan")
        if not row["traj_error"]:
            row["traj_error"] = f"status={row['status'] or 'MISSING'} rc={rc}"

    # Rule 1b (WP3): an implausible timestamp source invalidates the row too. The run may have
    # succeeded and the throughput may be real, but every pose is associated with the wrong
    # ground-truth pose, so nothing here can enter a table. Fail loudly rather than publish a
    # number whose error bar is meaningless -- this is the check that KITTI 00 needed and did
    # not have. `ts_source` / `ts_hz` stay in the row so the cause is visible without the log.
    if row["ts_plausible"] == "NO":
        row["pipeline_fps"] = float("nan")
        row["track_fps"] = float("nan")
        row["postinit_fps"] = float("nan")
        row["traj_error"] = (f"implausible timestamps: source={row['ts_source'] or 'MISSING'} "
                             f"{row['ts_hz']:.2f} Hz").strip()

    row["capture_hz"] = spec.capture_hz
    fps = row["pipeline_fps"]
    row["realtime"] = "" if fps != fps else ("yes" if fps >= spec.capture_hz else "no")
    row["cli"] = shlex.join(cli)
    return row


# ---------------------------------------------------------------- track success

def apply_track_success(rows: list[dict]) -> None:
    """Success = pose count >= 0.7 x the pooled median for that sequence (PLAN.md P5).

    Pooled per (dataset, sequence) across the rows in this invocation, so it is only
    meaningful when reps were run together -- which is how the protocol calls for it.
    """
    from statistics import median

    # Rule 1b: a row whose timestamps are implausible is not a success even if it tracked the
    # whole sequence -- its poses are associated with the wrong ground truth. It is also kept
    # out of the pool, so it cannot move the median that the other reps are judged against.
    def usable(r: dict) -> bool:
        return r["status"] == "OK" and r["ts_plausible"] != "NO"

    groups: dict[tuple, list[int]] = {}
    for r in rows:
        if usable(r) and r["poses"] > 0:
            groups.setdefault((r["dataset"], r["sequence"]), []).append(r["poses"])
    for r in rows:
        pool = groups.get((r["dataset"], r["sequence"]))
        if not pool:
            r["track_success"] = 0
        else:
            r["track_success"] = int(usable(r) and r["poses"] >= 0.7 * median(pool)
                                     and _covers_enough(r))


# P5a (2026-09-24, DECISIONS.md "WP6 BLOCKER 2"). The pooled-median test above is RELATIVE to the
# arm's own reps, so it has no absolute floor: when every rep of an arm dies at the same point the
# median moves with them and all of them score successful. That is how WP6's monocular arm came to
# report a 0.196 m ATE on KITTI 04 from runs of 12-19 frames out of 271. This is the absolute
# sanity floor that the relative test cannot supply.
#
# It is deliberately NOT a completeness requirement. TUM fr1_floor stops at ~68 % of its images for
# EVERY arm, so a bar near 1.0 would mark a sequence that both arms handle identically as
# universally failed. Comparability BETWEEN arms is a separate test and lives in make_tables.py.
COVERAGE_SANITY = 0.5   # a rep that processed under half the sequence is not a trajectory for it


def _covers_enough(r: dict) -> bool:
    """False only when we can measure coverage AND it is below the floor.

    An unmeasured sequence (image count not in ds.SEQUENCE_IMAGE_COUNTS) passes, because
    the honest default for "cannot judge" is not to veto -- but it is reported, never silent.
    """
    n = ds.image_count(r["dataset"], r["sequence"])
    if not n or not r.get("frames"):
        if not n:
            print(f"[COVERAGE] {r['dataset']}/{r['sequence']}: no image count on record, "
                  f"coverage floor not applied", file=sys.stderr)
        return True
    return r["frames"] >= COVERAGE_SANITY * n


def write_csv(rows: list[dict], path: Path) -> None:
    new = not path.exists()
    path.parent.mkdir(parents=True, exist_ok=True)
    # Appending to a summary.csv written by an earlier COLUMNS list would put every value under the
    # wrong header, silently. Refuse instead: a new column set means a new output directory.
    if not new:
        with open(path, newline="") as f:
            header = next(csv.reader(f), [])
        if header != COLUMNS:
            raise SystemExit(f"ERROR: {path} was written with a different column set "
                             f"({len(header)} vs {len(COLUMNS)} columns); write to a new --out")
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS, extrasaction="ignore")
        if new:
            w.writeheader()
        for r in rows:
            w.writerow(r)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--sequence", required=True)
    ap.add_argument("--arm", default="full", help=f"one of: {', '.join(arms_mod.ARMS)}")
    ap.add_argument("--reps", type=int, default=1)
    ap.add_argument("--rep-start", type=int, default=1,
                    help="number of the first rep (a resume top-up continues the numbering instead of "
                         "overwriting earlier rep directories)")
    ap.add_argument("--out", default="runs/adhoc", help="output dir, relative to HSLAM/")
    ap.add_argument("--endindex", type=int, default=None)
    ap.add_argument("--timeout", type=int, default=3600)
    ap.add_argument("--extra", nargs=argparse.REMAINDER, default=[],
                    help="extra HSLAM flags appended verbatim (must come last)")
    a = ap.parse_args()

    if not BINARY.exists():
        print(f"ERROR: {BINARY} not found; build first", file=sys.stderr)
        return 2

    spec = ds.resolve(a.dataset, a.sequence)
    for label, p in (("images", spec.images), ("root", spec.root),
                    ("calib", spec.calib), ("gt", spec.gt)):
        if not Path(p).exists():
            print(f"ERROR: {label} not found: {p}", file=sys.stderr)
            return 2

    arm_args = arms_mod.build(a.arm, spec) + list(a.extra)
    commit, dirty = git_commit()
    host, evo_ver = toolchain()
    bin_hash = binary_hash(BINARY)
    if BINARY != HSLAM_ROOT / "build" / "bin" / "HSLAM":
        print(f"NOTE: HSLAM_BINARY override in use: {BINARY} (binary {bin_hash})", file=sys.stderr)
    outdir = (HSLAM_ROOT / a.out) if not Path(a.out).is_absolute() else Path(a.out)
    outdir.mkdir(parents=True, exist_ok=True)

    if dirty:
        print("WARNING: working tree is dirty; this row is not reproducible from `commit` alone.",
              file=sys.stderr)

    rows = []
    for rep in range(a.rep_start, a.rep_start + a.reps):
        print(f"[{rep - a.rep_start + 1}/{a.reps}] {a.arm} {a.dataset} {a.sequence} rep{rep} ...", flush=True)
        r = run_once(spec, arm_args, rep, outdir, a.endindex, a.timeout)
        r.update(timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"), commit=commit,
                 dirty=int(dirty), host=host, evo_version=evo_ver, arm=a.arm,
                 dataset=a.dataset, sequence=a.sequence, rep=rep, binary=bin_hash)
        rows.append(r)
        print(f"     status={r['status']} rc={r['returncode']} "
              f"ate_sim3={r['ate_sim3_rmse']:.4f} s={r['scale_s']:.4f} "
              f"fps={r['pipeline_fps']:.1f} {r['traj_error']}", flush=True)

    apply_track_success(rows)
    csv_path = outdir / "summary.csv"
    write_csv(rows, csv_path)

    ok = [r for r in rows if r["status"] == "OK" and r["track_success"]]
    print(f"\n{len(ok)}/{len(rows)} usable -> {csv_path}")
    if ok:
        from statistics import median
        for k in ("ate_sim3_rmse", "ate_se3_rmse", "scale_s", "founding_offset_log", "postinit_fps"):
            vals = [r[k] for r in ok if r[k] == r[k]]
            if vals:
                print(f"  median {k:26s} {median(vals):.4f}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
