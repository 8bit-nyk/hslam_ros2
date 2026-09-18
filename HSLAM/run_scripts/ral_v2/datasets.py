"""Dataset registry for the RA-L v2 eval pipeline.

One place that knows, per dataset: where the images are, which calib file, where the
ground truth is and in what frame, and what the capture rate is (for the real-time
wording rule at G1). Adding a sequence must never mean editing eval_run.py.

Frame conventions that have bitten this project before:
  * EuRoC ground truth is in the BODY frame. Comparing it to a camera trajectory
    without applying T_BC inflates ATE. See NIGHT_RUN_2026-07-30.md:72-74.
  * KITTI timestamps must come from times.txt, never a synthetic 10 Hz ramp.
    groundtruth_tum.txt already carries the real ones; verified 18 Sep 2026.
"""
from __future__ import annotations

import re
from pathlib import Path
from typing import NamedTuple, Optional

DATASET_ROOT = Path("/home/aub/datasets")
HSLAM_ROOT = Path(__file__).resolve().parents[2]


class SeqSpec(NamedTuple):
    dataset: str
    sequence: str
    images: Path
    calib: Path
    gt: Path
    gt_format: str          # "tum" | "euroc"
    capture_hz: float       # for the G1 "real-time" wording rule
    gt_frame: str           # "camera" | "body"
    extrinsics: Optional[Path] = None   # cam0/sensor.yaml when gt_frame == "body"
    associations: Optional[Path] = None


def _tum(seq: str) -> SeqSpec:
    d = DATASET_ROOT / "TUM_RGBD" / f"rgbd_dataset_{seq}"
    m = re.match(r"(freiburg\d)", seq)
    if not m:
        raise ValueError(f"TUM sequence {seq!r} does not start with freiburgN")
    calib = HSLAM_ROOT / "run_scripts" / f"camera_{m.group(1)}.txt"
    return SeqSpec("tum", seq, d / "rgb", calib, d / "groundtruth.txt",
                   "tum", 30.0, "camera",
                   associations=d / "associations.txt")


def _kitti(seq: str) -> SeqSpec:
    d = DATASET_ROOT / "KITTI" / seq
    n = int(seq)
    calib = "Kitti00-02.txt" if n <= 2 else "Kitti03.txt" if n == 3 else "Kitti04-12.txt"
    return SeqSpec("kitti", seq, d / "image_2", HSLAM_ROOT / "misc" / "Kitti" / calib,
                   d / "groundtruth_tum.txt", "tum", 10.0, "camera")


def _euroc(seq: str) -> SeqSpec:
    d = DATASET_ROOT / "EuRoC" / seq / "mav0"
    return SeqSpec("euroc", seq, d / "cam0" / "data",
                   HSLAM_ROOT / "misc" / "EuroC" / "camera.txt",
                   d / "state_groundtruth_estimate0" / "data.csv",
                   "euroc", 20.0, "body",
                   extrinsics=d / "cam0" / "sensor.yaml")


def _icl(seq: str) -> SeqSpec:
    d = DATASET_ROOT / "ICL_NUIM" / seq
    tag = seq.replace("living_room_traj", "livingRoom").replace("office_room_traj", "officeRoom")
    return SeqSpec("iclnuim", seq, d / "rgb",
                   HSLAM_ROOT / "run_scripts" / "camera_icl_nuim.txt",
                   d / f"{tag}.gt.freiburg", "tum", 30.0, "camera",
                   associations=d / "associations.txt")


_BUILDERS = {"tum": _tum, "kitti": _kitti, "euroc": _euroc, "iclnuim": _icl}


def resolve(dataset: str, sequence: str) -> SeqSpec:
    dataset = dataset.lower()
    if dataset not in _BUILDERS:
        raise ValueError(f"unknown dataset {dataset!r}; known: {sorted(_BUILDERS)}")
    return _BUILDERS[dataset](sequence)


# --- sequence sets from PAPER_CONFIG_AND_GATES.md section 7 -------------------------------
SETS = {
    "TUM-10": [("tum", s) for s in (
        "freiburg1_360", "freiburg1_desk", "freiburg1_desk2", "freiburg1_floor",
        "freiburg1_room", "freiburg2_360_hemisphere", "freiburg2_desk",
        "freiburg2_large_no_loop", "freiburg2_large_with_loop", "freiburg3_long_office_household")],
    # KITTI 08/09 do not track at HEAD. They stay in the list on purpose: the protocol
    # reports them as failures rather than silently dropping them (PLAN.md P5).
    "KITTI-11": [("kitti", f"{i:02d}") for i in range(11)],
    "EuRoC-6": [("euroc", s) for s in (
        "MH_01_easy", "MH_02_easy", "V1_01_easy", "V1_02_medium", "V2_01_easy", "V2_02_medium")],
    "ABL-10": [("tum", s) for s in (
        "freiburg1_desk", "freiburg1_room", "freiburg2_desk",
        "freiburg2_large_no_loop", "freiburg3_long_office_household")]
        + [("kitti", s) for s in ("00", "05", "06", "07", "10")],
    "GT-5": [("tum", s) for s in ("freiburg1_desk", "freiburg1_room", "freiburg2_desk")]
        + [("kitti", "07"), ("iclnuim", "living_room_traj0")],
    "TUNE-2": [("tum", "freiburg2_360_hemisphere"), ("kitti", "03")],
}
