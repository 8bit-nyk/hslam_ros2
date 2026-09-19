#!/usr/bin/env python3
"""Generate EuRoC `mav0/cam0/times.txt` from the dataset's own `cam0/data.csv`.

Why this exists
---------------
HSLAM's `ImageFolderReader::loadTimestamps()` looks for `<images>/../times.txt`, i.e.
`mav0/cam0/times.txt`. **EuRoC does not ship that file.** When it is absent the reader falls
back to parsing the timestamp out of the image *filename* -- and EuRoC filenames are
**nanoseconds**: `stod("1403715273262142976")` = 1.403715e18, which is then used as a value in
SECONDS. The estimated trajectory then carries stamps ~1.4e18 while the ground truth in
`state_groundtruth_estimate0/data.csv` is read in seconds, so association matches nothing.

Only `MH_01_easy` had ever been prepared by hand (April 2026). `V1_01_easy` and `V2_02_medium`
had not -- which is why WP3 could not have produced a trustworthy number on them, and why
`FINDINGS.md`'s "EuRoC ships the two-column form" was true of MH_01 and of nothing else.

This is the same failure mode as the KITTI `times.txt` misparse fixed in `6799be6`: a wrong
timestamp source never crashes, it quietly associates every pose with the wrong GT pose.

Output format
-------------
Two columns, `<nanosecond id> <seconds>`, matching the hand-made MH_01 file **byte for byte**
so the whole EuRoC set is uniform and MH_01 is not silently rewritten. The reader's 2-column
branch (`ntok == 2`) consumes it, and the `[TIMESTAMPS]` diagnostic then reports
`source=times.txt(2col) ... (20.00 Hz)` instead of `source=filenames ... (0.00 Hz)`.

Usage
-----
    python3 run_scripts/ral_v2/prepare_euroc.py --check    # verify only, write nothing
    python3 run_scripts/ral_v2/prepare_euroc.py            # every sequence under ~/datasets/EuRoC
    python3 run_scripts/ral_v2/prepare_euroc.py --seq V1_01_easy

Run it on **both** machines: it prepares data, and `~/datasets` is not synced by git.
`--check` re-derives MH_01's shipped file and diffs against it, which is this converter's own
validation -- an `OK already correct` on MH_01 means the rendering matches the file that
produced every historically valid EuRoC run.
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

# ~/datasets on BOTH machines (laptop /home/aub, eval-server /home/nyk). Never hardcode a home
# directory -- docs/workstation_setup/WORKFLOW.md, "Code + data sync".
DEFAULT_ROOT = Path(os.environ.get("HSLAM_DATASETS", Path.home() / "datasets")) / "EuRoC"


def read_data_csv(csv_path: Path) -> list[int]:
    """Return the nanosecond stamps in `cam0/data.csv`, in file order."""
    stamps: list[int] = []
    for line in csv_path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        stamps.append(int(line.split(",")[0]))
    return stamps


def render(stamps: list[int]) -> str:
    """`<ns> <seconds>` per line.

    %.7f is 100 ns resolution -- five orders of magnitude finer than the 50 ms frame interval,
    and it reproduces the hand-made MH_01 file exactly (see `--check`).
    """
    return "".join(f"{ns} {ns / 1e9:.7f}\n" for ns in stamps)


def sequences(root: Path) -> list[Path]:
    return sorted(p for p in root.iterdir()
                  if p.is_dir() and (p / "mav0" / "cam0" / "data.csv").is_file())


def convert(seq_dir: Path, *, write: bool) -> tuple[str, str]:
    cam0 = seq_dir / "mav0" / "cam0"
    stamps = read_data_csv(cam0 / "data.csv")
    if not stamps:
        return "ERROR", "data.csv holds no entries"
    if stamps != sorted(stamps):
        return "ERROR", "data.csv stamps are not monotonic"

    # The reader lists the image directory itself, so a stamp with no image (or an image with no
    # stamp) shifts every later association by one frame. MH_01 ships a stray `exiftool.m` in
    # cam0/data; getdir() filters by extension, so count *.png, exactly as the reader does.
    n_png = len(list((cam0 / "data").glob("*.png")))
    if n_png != len(stamps):
        return "ERROR", f"{n_png} png files but {len(stamps)} stamps in data.csv"

    dt = (stamps[-1] - stamps[0]) / 1e9 / (len(stamps) - 1)
    text = render(stamps)
    out = cam0 / "times.txt"

    if out.is_file():
        old = out.read_text()
        if old == text:
            return "OK", f"already correct ({len(stamps)} stamps, {1 / dt:.2f} Hz)"
        if not write:
            return "DIFFERS", f"existing times.txt differs ({len(old.splitlines())} lines)"
        (cam0 / "times.txt.bak").write_text(old)
        out.write_text(text)
        return "REWROTE", f"{len(stamps)} stamps, {1 / dt:.2f} Hz (old kept as times.txt.bak)"

    if not write:
        return "MISSING", f"would write {len(stamps)} stamps, {1 / dt:.2f} Hz"
    out.write_text(text)
    return "WROTE", f"{len(stamps)} stamps, {1 / dt:.2f} Hz"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    ap.add_argument("--seq", action="append", help="sequence name; repeatable; default all")
    ap.add_argument("--check", action="store_true", help="report only, write nothing")
    a = ap.parse_args()

    if not a.root.is_dir():
        print(f"ERROR: {a.root} is not a directory", file=sys.stderr)
        return 2

    dirs = [a.root / s for s in a.seq] if a.seq else sequences(a.root)
    if not dirs:
        print(f"ERROR: no sequence with mav0/cam0/data.csv under {a.root}", file=sys.stderr)
        return 2

    print(f"root: {a.root}   mode: {'check' if a.check else 'write'}")
    bad = 0
    for d in dirs:
        if not (d / "mav0" / "cam0" / "data.csv").is_file():
            print(f"  {d.name:16s} ERROR    no mav0/cam0/data.csv")
            bad += 1
            continue
        verdict, detail = convert(d, write=not a.check)
        print(f"  {d.name:16s} {verdict:8s} {detail}")
        if verdict in ("ERROR", "DIFFERS") or (a.check and verdict == "MISSING"):
            bad += 1
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
