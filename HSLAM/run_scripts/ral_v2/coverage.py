"""Per-run coverage and time to first pose for the WP7 as-run table.

Implements section 1 of DECISIONS.md "WP7 as-run table -- coverage, run success and reliability
metrics" (PRE-REGISTERED 2026-09-25). There is one implementation for every system, HSLAM included;
wp7_reliability.py is its caller. Import it, never re-derive it.

Coverage C of one run, on the sequence's image timeline [T0, T1]:

    C_span = (min(t_last + g, T1) - max(t_first, T0)) / (T1 - T0)
             g = the run's own largest interval between consecutive own poses. This is the tail credit:
             a keyframe-only trajectory cannot show a pose for its final keyframe interval (MASt3R-SLAM
             ends 2.8 s early on fr1_desk, equal to its own largest interval).
    F_own  = unique own poses / frames the system was given     (every-frame output only)
    C      = min(C_span, F_own) for every-frame output;  C = C_span for keyframe-only output

Own poses are unique timestamps in DATASET time. A repeated timestamp counts once: ORB-SLAM3 and DropD
write a RECENTLY_LOST frame with the previous frame's timestamp and pose (Tracking.cc:2317-2324).

Why not a gap-based measure (OpenLORIS's min(dt, delta)): keyframe-only systems leave 2.8-5.8 s between
poses while tracking normally (MASt3R-SLAM on fr1_desk; HSLAM and ORB-SLAM3 at a stop on KITTI 07),
which any gap threshold would read as tracking loss.

Two checks guard the inputs:
  * load_timeline() asserts the image count and the sampling rate. The KITTI times.txt on both
    machines is two-column ("000000 0.000000"); a stock reader gets frame indices as seconds.
    Monotonicity catches neither that nor the EuRoC defect; asserting the rate catches both.
  * coverage() refuses a run whose own poses do not sit on image timestamps. That is a
    wrong time mapping, never a coverage result.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import NamedTuple, Optional

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import datasets as ds            # noqa: E402

MATCH_TOL_S = 1e-3        # an own pose must sit on an image timestamp to within this
MIN_MATCHED_FRAC = 0.99   # fewer matched than this => the time mapping is wrong
RATE_TOL = 0.2            # timeline median interval within +-20 % of 1 / capture_hz
STAMP_DECIMALS = 6        # repeated timestamps are compared at microsecond resolution

TIME_MAPS = ("native", "index", "index/<K>")
OUTPUTS = ("every_frame", "keyframe")


class TimelineError(RuntimeError):
    """The sequence's image timeline failed its count or rate assertion."""


class TimeMappingError(RuntimeError):
    """A trajectory's timestamps do not map onto the image timeline."""


def load_timeline(dataset: str, sequence: str) -> np.ndarray:
    """Image timestamps of a sequence in dataset time, count- and rate-asserted.

    TUM: rgb.txt, the images every system is given. KITTI: times.txt, one or two columns.
    """
    spec = ds.resolve(dataset, sequence)
    if spec.dataset == "tum":
        path = spec.root / "rgb.txt"
    elif spec.dataset == "kitti":
        path = spec.root / "times.txt"
    else:
        raise TimelineError(f"no image timeline is defined for dataset {dataset!r}")

    stamps = []
    for row, line in enumerate(l for l in path.read_text().splitlines()
                               if l.strip() and not l.lstrip().startswith("#")):
        parts = line.split()
        if spec.dataset == "tum":
            stamps.append(float(parts[0]))
        elif len(parts) == 1:
            stamps.append(float(parts[0]))
        elif len(parts) == 2 and int(parts[0]) == row:
            stamps.append(float(parts[1]))      # "000000 0.000000": column 1 is the frame index
        else:
            raise TimelineError(f"{path}: row {row} is neither 'time' nor '<index> time': {line!r}")
    t = np.asarray(stamps, dtype=float)

    n = ds.image_count(dataset, sequence)
    if n is None:
        raise TimelineError(f"{dataset}/{sequence}: image count not on record "
                            f"(datasets.SEQUENCE_IMAGE_COUNTS); cannot judge coverage")
    if len(t) != n:
        raise TimelineError(f"{path}: {len(t)} timestamps, {n} images on record")
    d = np.diff(t)
    if not np.all(d > 0):
        raise TimelineError(f"{path}: timestamps are not strictly increasing")
    expect, med = 1.0 / spec.capture_hz, float(np.median(d))
    if abs(med - expect) > RATE_TOL * expect:
        raise TimelineError(f"{path}: median interval {med:.4f} s, expected {expect:.4f} s "
                            f"({spec.capture_hz:g} Hz)")
    return t


def map_times(raw: np.ndarray, time_map: str, timeline: np.ndarray) -> np.ndarray:
    """Map a trajectory's own timestamps to dataset time.

    native     the file already holds dataset time (seconds)
    index      the file holds the original frame index (DPVO on KITTI)
    index/<K>  the file holds frame index / K (MASt3R-SLAM's image-folder reader: index / 30)
    """
    raw = np.asarray(raw, dtype=float)
    if time_map == "native":
        return raw
    if time_map == "index":
        k = 1.0
    elif time_map.startswith("index/"):
        k = float(time_map.split("/", 1)[1])
    else:
        raise ValueError(f"unknown time_map {time_map!r}; known: {', '.join(TIME_MAPS)}")
    x = raw * k
    idx = np.rint(x).astype(int)
    if len(x) and float(np.abs(x - idx).max()) > 1e-2:
        raise TimeMappingError(f"stamps are not frame indices / {k:g} "
                               f"(largest residual {float(np.abs(x - idx).max()):.3g})")
    if len(idx) and (idx.min() < 0 or idx.max() >= len(timeline)):
        raise TimeMappingError(f"frame index {idx.min()}..{idx.max()} outside 0..{len(timeline) - 1}")
    return timeline[idx]


def _nearest_dist(t: np.ndarray, timeline: np.ndarray) -> np.ndarray:
    j = np.clip(np.searchsorted(timeline, t), 1, len(timeline) - 1)
    return np.minimum(np.abs(t - timeline[j - 1]), np.abs(t - timeline[j]))


def own_pose_index(t_dataset: np.ndarray, timeline: np.ndarray) -> np.ndarray:
    """Indices of the own poses: first occurrence of each timestamp, on the timeline, in time order.

    Raises TimeMappingError when fewer than MIN_MATCHED_FRAC of the unique timestamps sit on an
    image timestamp.
    """
    t = np.round(np.asarray(t_dataset, dtype=float), STAMP_DECIMALS)
    if len(t) == 0:
        return np.zeros(0, dtype=int)
    _, first = np.unique(t, return_index=True)          # sorted by time, first occurrence kept
    on = _nearest_dist(t[first], timeline) <= MATCH_TOL_S
    if on.mean() < MIN_MATCHED_FRAC:
        raise TimeMappingError(f"only {on.mean():.1%} of {len(first)} own poses sit on an image "
                               f"timestamp (tolerance {MATCH_TOL_S * 1e3:g} ms): wrong time_map?")
    return first[on]


class Coverage(NamedTuple):
    n_poses: int           # rows in the file
    n_own: int             # unique timestamps on the timeline
    n_dup: int             # repeated-timestamp rows collapsed
    t_first: float         # dataset time of the first / last own pose (NaN if none)
    t_last: float
    t_first_s: float       # R3: t_first - T0
    gap_max_s: float       # g, the tail credit
    c_span_raw: float      # without the tail credit (reported, not used)
    c_span: float
    f_own: float           # NaN for keyframe-only output
    c: float


def coverage(t_dataset: np.ndarray, timeline: np.ndarray, output: str,
             frames_given: Optional[int] = None) -> Coverage:
    """Coverage of one run (pre-registration section 1). Fewer than 2 own poses gives C = 0."""
    if output not in OUTPUTS:
        raise ValueError(f"unknown output {output!r}; known: {', '.join(OUTPUTS)}")
    if output == "every_frame" and not frames_given:
        raise ValueError("every_frame output needs frames_given (frames after the system's stride)")
    nan = float("nan")
    keep = own_pose_index(t_dataset, timeline)
    u = np.round(np.asarray(t_dataset, dtype=float), STAMP_DECIMALS)[keep]
    n_poses = len(t_dataset)
    n_dup = n_poses - len(np.unique(np.round(np.asarray(t_dataset, dtype=float), STAMP_DECIMALS)))
    if len(u) < 2:
        return Coverage(n_poses, len(u), n_dup, nan, nan, nan, nan, 0.0, 0.0,
                        0.0 if output == "every_frame" else nan, 0.0)

    T0, T1 = float(timeline[0]), float(timeline[-1])
    g = float(np.diff(u).max())
    span_raw = (min(u[-1], T1) - max(u[0], T0)) / (T1 - T0)
    span = (min(u[-1] + g, T1) - max(u[0], T0)) / (T1 - T0)
    f_own = nan
    c = span
    if output == "every_frame":
        f_own = len(u) / frames_given
        if f_own > 1.0 + 1e-9:
            raise TimeMappingError(f"{len(u)} own poses but only {frames_given} frames given: "
                                   f"frames_given is wrong")
        c = min(span, f_own)
    return Coverage(n_poses, len(u), n_dup, float(u[0]), float(u[-1]), float(u[0]) - T0, g,
                    max(0.0, span_raw), max(0.0, span), f_own, max(0.0, min(1.0, c)))


def selftest() -> None:
    """Synthetic cases for every rule above. Raises AssertionError on failure."""
    tl = np.arange(0.0, 100.0, 0.1)                                   # 1000 frames, T1 = 99.9
    close = lambda a, b: math.isclose(a, b, abs_tol=1e-9)             # noqa: E731

    # Complete every-frame run: C = 1.
    c = coverage(tl, tl, "every_frame", 1000)
    assert close(c.c, 1.0) and close(c.f_own, 1.0) and close(c.t_first_s, 0.0), c

    # Keyframe-only, last keyframe 2 s early but keyframes 2 s apart: the tail credit restores it.
    kf = tl[::20]                                                     # 0, 2, ..., 98
    c = coverage(kf, tl, "keyframe")
    assert close(c.c_span_raw, 98.0 / 99.9) and close(c.c, 1.0) and math.isnan(c.f_own), c

    # Late start (HSLAM mono): 8 s missing at the head, no credit there.
    c = coverage(tl[80:], tl, "keyframe")
    assert close(c.t_first_s, 8.0) and close(c.c, (99.9 - 8.0) / 99.9), c

    # Truncated tail (MASt3R-SLAM on KITTI 07): credit is one interval only.
    c = coverage(tl[:420], tl, "keyframe")
    assert close(c.c, (41.9 + 0.1) / 99.9), c

    # RECENTLY_LOST duplicates: 100 frames written with the previous timestamp. Span stays ~1,
    # the own-pose fraction drops, and C takes the lower of the two.
    dup = np.concatenate([tl[:450], np.repeat(tl[449], 100), tl[550:]])
    c = coverage(dup, tl, "every_frame", 1000)
    assert c.n_dup == 100 and close(c.f_own, 900 / 1000) and close(c.c, 0.9), c

    # Strided every-frame run (DROID stride 2): F_own against the frames actually given.
    c = coverage(tl[::2], tl, "every_frame", 500)
    assert close(c.f_own, 1.0) and close(c.c, 1.0), c

    # Time maps: frame index (DPVO KITTI) and index / 30 (MASt3R-SLAM).
    assert np.allclose(map_times(np.array([0.0, 2.0, 4.0]), "index", tl), [0.0, 0.2, 0.4])
    assert np.allclose(map_times(np.array([0, 1, 2]) / 30.0, "index/30", tl), [0.0, 0.1, 0.2])
    for bad in (lambda: map_times(np.array([0.05]), "index", tl),
                lambda: map_times(np.array([5000.0]), "index", tl),
                lambda: coverage(tl + 0.05, tl, "keyframe"),               # off the timeline
                lambda: coverage(np.arange(0, 1100, 2.0), tl, "keyframe"),  # indices read as seconds
                lambda: coverage(tl, tl, "every_frame", 500)):              # frames_given wrong
        try:
            bad()
        except TimeMappingError:
            continue
        raise AssertionError("a bad time mapping was accepted")

    # Fewer than two own poses: C = 0.
    assert coverage(tl[:1], tl, "keyframe").c == 0.0
    assert coverage(np.zeros(0), tl, "every_frame", 1000).c == 0.0


if __name__ == "__main__":
    selftest()
    print("coverage.py selftest: OK")
