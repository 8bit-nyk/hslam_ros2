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
BINARY = HSLAM_ROOT / "build" / "bin" / "HSLAM"

COLUMNS = [
    "timestamp", "commit", "dirty", "arm", "dataset", "sequence", "rep",
    "returncode", "status", "track_success", "wall_s",
    "frames", "poses", "matched_poses", "keyframes", "ml_inferences",
    "ate_sim3_rmse", "ate_se3_rmse", "scale_s", "scale_drift_pct_per_100m",
    "rpe_trans", "rpe_rot", "gt_distance_m",
    "pipeline_fps", "track_fps", "ml_ms", "capture_hz", "realtime",
    "peak_gpu_mb", "peak_cpu_mb",
    "lc_gate_enabled", "lc_gate_evals", "lc_gate_fires", "lc_gate_bypass",
    "agreement_gate_fire_rate",
    "geom", "canon", "iso", "idepth_prior", "ml_gpu", "fp16", "model",
    "traj_error", "cli",
]


# ---------------------------------------------------------------- provenance

def git_commit() -> tuple[str, bool]:
    def g(*a):
        return subprocess.run(["git", *a], cwd=HSLAM_ROOT, capture_output=True,
                              text=True).stdout.strip()
    return g("rev-parse", "--short", "HEAD"), bool(g("status", "--porcelain"))


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
    d = {k: float("nan") for k in ("pipeline_fps", "track_fps", "ml_ms")}
    d.update(status="", frames=0, keyframes=0, ml_inferences=0,
             geom="", canon="", iso="", idepth_prior="", ml_gpu="", fp16="", model="",
             lc_gate_enabled="", lc_gate_evals=0, lc_gate_fires=0, lc_gate_bypass=0,
             agreement_gate_fire_rate=float("nan"))

    if (m := re.search(r"\[PERF_SUMMARY\][^\n]*", text)):
        line = m.group(0)
        for key, pat in (("track_fps", r"track_fps=([\d.]+)"),
                         ("pipeline_fps", r"pipeline_fps=([\d.]+)"),
                         ("ml_ms", r"ml_ms=([\d.]+)")):
            if (mm := re.search(pat, line)):
                d[key] = float(mm.group(1))

    if (m := re.search(r"\[RUN_SUMMARY\][^\n]*", text)):
        line = m.group(0)
        for key, pat in (("status", r"status=(\w+)"), ("geom", r"geom=(\w+)"),
                         ("canon", r"canon=(\w+)"), ("iso", r"iso=(\w+)"),
                         ("idepth_prior", r"idepth_prior=(\w+)"), ("ml_gpu", r"ml_gpu=(\w+)"),
                         ("fp16", r"fp16=(\w+)"), ("model", r"model=(\S+)")):
            if (mm := re.search(pat, line)):
                d[key] = mm.group(1)
        for key, pat in (("frames", r"frames=(\d+)"), ("keyframes", r"kfs=(\d+)"),
                         ("ml_inferences", r"ml_inferences=(\d+)")):
            if (mm := re.search(pat, line)):
                d[key] = int(mm.group(1))

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
    return d


# ---------------------------------------------------------------- one run

def run_once(spec: ds.SeqSpec, arm_args: list[str], rep: int, outdir: Path,
             endindex: int | None, timeout_s: int) -> dict:
    rundir = outdir / f"{spec.dataset}_{spec.sequence}_rep{rep}"
    rundir.mkdir(parents=True, exist_ok=True)

    cli = [str(BINARY),
           "--files", str(spec.images),
           "--calib", str(spec.calib),
           "--vocab", str(HSLAM_ROOT / "misc" / "orbvoc.dbow3"),
           "--colour", "--nogui=true", "--nolog", "--loopclosure",
           *arm_args]
    if spec.associations and spec.associations.exists():
        cli += ["--associations", str(spec.associations)]
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
        row.update(traj_eval.evaluate(est, spec.gt, spec.gt_format, spec.extrinsics))
    else:
        row.update({k: float("nan") for k in (
            "ate_sim3_rmse", "ate_se3_rmse", "scale_s", "scale_drift_pct_per_100m",
            "rpe_trans", "rpe_rot", "gt_distance_m")})
        row.update(poses=0, matched_poses=0, traj_error="no result.txt written")

    # Rule 1: a non-OK status invalidates the whole row, fps included.
    if row["status"] != "OK" or rc != 0:
        row["pipeline_fps"] = float("nan")
        row["track_fps"] = float("nan")
        if not row["traj_error"]:
            row["traj_error"] = f"status={row['status'] or 'MISSING'} rc={rc}"

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
    groups: dict[tuple, list[int]] = {}
    for r in rows:
        if r["status"] == "OK" and r["poses"] > 0:
            groups.setdefault((r["dataset"], r["sequence"]), []).append(r["poses"])
    for r in rows:
        pool = groups.get((r["dataset"], r["sequence"]))
        if not pool:
            r["track_success"] = 0
        else:
            r["track_success"] = int(r["status"] == "OK" and r["poses"] >= 0.7 * median(pool))


def write_csv(rows: list[dict], path: Path) -> None:
    new = not path.exists()
    path.parent.mkdir(parents=True, exist_ok=True)
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
    for label, p in (("images", spec.images), ("calib", spec.calib), ("gt", spec.gt)):
        if not Path(p).exists():
            print(f"ERROR: {label} not found: {p}", file=sys.stderr)
            return 2

    arm_args = arms_mod.build(a.arm, spec) + list(a.extra)
    commit, dirty = git_commit()
    outdir = (HSLAM_ROOT / a.out) if not Path(a.out).is_absolute() else Path(a.out)
    outdir.mkdir(parents=True, exist_ok=True)

    if dirty:
        print("WARNING: working tree is dirty; this row is not reproducible from `commit` alone.",
              file=sys.stderr)

    rows = []
    for rep in range(1, a.reps + 1):
        print(f"[{rep}/{a.reps}] {a.arm} {a.dataset} {a.sequence} ...", flush=True)
        r = run_once(spec, arm_args, rep, outdir, a.endindex, a.timeout)
        r.update(timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"), commit=commit,
                 dirty=int(dirty), arm=a.arm, dataset=a.dataset,
                 sequence=a.sequence, rep=rep)
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
        for k in ("ate_sim3_rmse", "ate_se3_rmse", "scale_s", "pipeline_fps"):
            vals = [r[k] for r in ok if r[k] == r[k]]
            if vals:
                print(f"  median {k:26s} {median(vals):.4f}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
