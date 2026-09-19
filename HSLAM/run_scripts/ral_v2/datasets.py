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

import os
import re
from pathlib import Path
from typing import NamedTuple, Optional

# ~/datasets on BOTH machines. Never hardcode a home directory: the laptop is /home/aub and
# eval-server is /home/nyk, and the two-machine workflow requires run scripts to be portable
# (docs/workstation_setup/WORKFLOW.md, "Code + data sync"). HSLAM_DATASETS overrides.
DATASET_ROOT = Path(os.environ.get("HSLAM_DATASETS", Path.home() / "datasets"))
HSLAM_ROOT = Path(__file__).resolve().parents[2]


class SeqSpec(NamedTuple):
    """One sequence, in both of the reader's two input modes.

    HSLAM's DatasetReader resolves image paths differently depending on whether
    --associations is given, and getting this wrong fails loudly but confusingly:

      * folder mode (no --associations): --files <root>/rgb, image names read from the dir.
      * associations mode:               --files <root>, because associations.txt already
        holds paths like "rgb/<stamp>.png". Passing <root>/rgb here yields <root>/rgb/rgb/...
        and every imread fails.

    So `images` is the folder-mode argument and `root` the associations-mode one. Only
    ICL-NUIM needs associations unconditionally (for frame ordering -- see
    hslam_run_iclnuim_ml_depth.sh); TUM needs them only for a GT-depth arm. The reference
    TUM ML-depth script passes no associations at all.
    """
    dataset: str
    sequence: str
    images: Path            # folder mode:       --files <this>
    root: Path              # associations mode: --files <this>
    calib: Path
    gt: Path
    gt_format: str          # "tum" | "euroc"
    capture_hz: float       # for the G1 "real-time" wording rule
    gt_frame: str           # "camera" | "body"
    extrinsics: Optional[Path] = None   # cam0/sensor.yaml when gt_frame == "body"
    associations: Optional[Path] = None
    needs_associations: bool = False    # True only where ordering demands it (ICL-NUIM)
    extra_cli: tuple = ()               # dataset-level HSLAM flags (photometric calibration files)


def _tum(seq: str) -> SeqSpec:
    d = DATASET_ROOT / "TUM_RGBD" / f"rgbd_dataset_{seq}"
    m = re.match(r"(freiburg\d)", seq)
    if not m:
        raise ValueError(f"TUM sequence {seq!r} does not start with freiburgN")
    calib = HSLAM_ROOT / "run_scripts" / f"camera_{m.group(1)}.txt"
    return SeqSpec("tum", seq, d / "rgb", d, calib, d / "groundtruth.txt",
                   "tum", 30.0, "camera",
                   associations=d / "associations.txt", needs_associations=False)


def _kitti(seq: str) -> SeqSpec:
    d = DATASET_ROOT / "KITTI" / seq
    n = int(seq)
    calib = "Kitti00-02.txt" if n <= 2 else "Kitti03.txt" if n == 3 else "Kitti04-12.txt"
    return SeqSpec("kitti", seq, d / "image_2", d, HSLAM_ROOT / "misc" / "Kitti" / calib,
                   d / "groundtruth_tum.txt", "tum", 10.0, "camera")


def _euroc(seq: str) -> SeqSpec:
    d = DATASET_ROOT / "EuRoC" / seq / "mav0"
    # The dataset-local cam0/camera.txt is what hslam_run_euroc_*.sh actually passes, and it
    # differs materially from misc/EuroC/camera.txt: pixel intrinsics + "crop" (DSO computes
    # the rectified intrinsics at runtime) versus normalized intrinsics with a hardcoded
    # output of 0.6 0.9 0.5 0.5. They rectify differently, and EuRoC geometry is precisely
    # what WP3 is auditing -- using the wrong file would silently change the thing measured.
    return SeqSpec("euroc", seq, d / "cam0" / "data", d,
                   d / "cam0" / "camera.txt",
                   d / "state_groundtruth_estimate0" / "data.csv",
                   "euroc", 20.0, "body",
                   extrinsics=d / "cam0" / "sensor.yaml")


def _icl(seq: str) -> SeqSpec:
    d = DATASET_ROOT / "ICL_NUIM" / seq
    tag = seq.replace("living_room_traj", "livingRoom").replace("office_room_traj", "officeRoom")
    return SeqSpec("iclnuim", seq, d / "rgb", d,
                   HSLAM_ROOT / "run_scripts" / "camera_icl_nuim.txt",
                   d / f"{tag}.gt.freiburg", "tum", 30.0, "camera",
                   associations=d / "associations.txt", needs_associations=True)


def _tummonovo(seq: str) -> SeqSpec:
    """TUM mono-VO (Engel et al. 2016): wide-FOV global-shutter grayscale handheld camera with
    full photometric calibration. WP3b's camera-class test -- it shares EuRoC's camera class
    (wide FOV, grayscale, global shutter) but not its MAV motion or Vicon-room content.

    Ground truth covers only the mocap-tracked start and end segments (NaN elsewhere);
    groundtruth_clean.txt is groundtruthSync.txt with the NaN rows dropped, so Sim(3) ATE here
    is the classic mono-VO "alignment error" -- drift between the two segments -- not a full-
    trajectory ATE. Both arms are judged on the same thing, so the ratio is still meaningful.
    times.txt is the 3-column DSO form (id stamp exposure), which the reader parses natively.

    Photometric calibration is NOT used, deliberately: HSLAM's PhotometricUndistorter checks the
    vignette against the RECTIFIED size (640x480) instead of the sensor size (1280x1024) and
    aborts ("Invalid vignette image size", observed 2026-09-19), so --mode 0 cannot run without a
    source change. Both arms run --mode 1 (no photometric calibration), the same mode every other
    dataset in this pipeline uses, so the mono-vs-ML ratio is unaffected; absolute numbers are
    not comparable with DSO's published mono-VO results.
    """
    d = DATASET_ROOT / "TUM_monoVO" / seq
    return SeqSpec("tummonovo", seq, d / "images", d, d / "camera.txt",
                   d / "groundtruth_clean.txt", "tum", 21.0, "camera")


_BUILDERS = {"tum": _tum, "kitti": _kitti, "euroc": _euroc, "iclnuim": _icl,
             "tummonovo": _tummonovo}


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
