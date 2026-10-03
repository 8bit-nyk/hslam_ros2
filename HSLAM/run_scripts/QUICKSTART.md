# Running HSLAM with metric-scale ML depth

```bash
cd HSLAM/build && cmake .. && make -j$(nproc)          # rebuild after every pull

cd ..
./run_scripts/hslam_run_tumrgbd_ml_depth.sh freiburg1_room
./run_scripts/hslam_run_kitti_ml_depth.sh   07
./run_scripts/hslam_run_euroc_ml_depth.sh   MH_01_easy
```

Each script runs the **paper configuration** (the metric-scale one) and ends with a scale check. Add
`--endindex N` for a short test run, `--quiet` for less console output.

## Why the flags matter

The C++ defaults for the flags that make Metric3D's depth metric are **off**
(`--ml-canonical-scale`, `--ml-input-geometry=metric3d`, `--ml-init-scale=median`, `--p1-blend-grad-fix`,
and `--ml-isotropic-input` where fx != fy). A bare `HSLAM --ml-depth --ml-model ...` tracks fine and returns a
trajectory at the **wrong scale** (TUM fr1_desk: s = 0.56, SE(3) ATE 0.69 m, versus s = 1.1 and 0.09 m with the
paper config). The run scripts take the flags from `ral_v2/arms.py` (`PAPER_CONFIG`), the same source the eval
pipeline uses; `--ml-isotropic-input` is added automatically for KITTI and EuRoC. To see them:

```bash
python3 run_scripts/ral_v2/arms.py --print-cli kitti
```

If you run the binary by hand, pass that output. Do not edit the flag list by hand; change `arms.py`.

## Reading the scale check

```
LOG   : OK -- paper configuration, status=OK, timestamps plausible
TRAJ  : scale s = 0.934   ATE Sim(3) = 5.4 m   ATE SE(3) = 8.3 m
```

* **LOG** reads `[RUN_SUMMARY]` / `[TIMESTAMPS]` / `[ML_GEOM]` and fails if the run was not the paper
  configuration. This part needs only python3.
* **TRAJ** needs `numpy` and `evo` (`pip install evo`; set `HSLAM_PYTHON` to that interpreter if it is not the
  default `python3`). `s` is the Umeyama scale to ground truth: **s near 1 and SE(3) ATE close to Sim(3) ATE
  means the trajectory is metric.** A truncated run (`--endindex`) gives a short-baseline `s` that is not
  comparable to the full-sequence numbers; the check says so.
* To check a run you did by hand:
  `python3 run_scripts/ral_v2/scale_check.py <tum|kitti|euroc> <sequence> <trajectory.txt> --log <run.log>`

Full-sequence scale measured with this configuration (n = 5, eval server). Expect roughly:

| Sequence | s |
|---|---|
| TUM freiburg1_room | 1.13 |
| KITTI 07 | 0.92 |
| KITTI 00 | 0.96 |

EuRoC is the weakest dataset: the scale is self-consistent but a large residual error remains.

## Environment variables

| Variable | Effect |
|---|---|
| `HSLAM_DATASETS` | dataset root (default `~/datasets`) |
| `HSLAM_BUILD_DIR` | use another build tree than `HSLAM/build` |
| `HSLAM_ML_MODEL` | Metric3D ONNX path (default `models/metric3d-vit-small/onnx/model.onnx`) |
| `HSLAM_PYTHON` | python used for the flag lookup and the scale check |
| `HSLAM_LEGACY_FLAGS=1` | old bare flag set — **not metric scale**, for ablations only |

## If the scale is wrong

1. `LOG` says "NOT the paper configuration": you are on an old build (`git pull`, rebuild) or set
   `HSLAM_LEGACY_FLAGS`.
2. `unrecognised option` at start-up: the binary predates the flags — rebuild.
3. `[TIMESTAMPS] plausible=no`, or a sampling rate that does not match the dataset (KITTI 10 Hz, EuRoC 20 Hz,
   TUM 30 Hz): a dataset-reading problem, not a scale problem. EuRoC needs
   `python3 run_scripts/ral_v2/prepare_euroc.py` after extraction.
4. `ML_GEOM ... WARNING` about isotropic input: only possible with hand-typed flags.
