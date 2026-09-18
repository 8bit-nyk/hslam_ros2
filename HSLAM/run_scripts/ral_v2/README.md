# `run_scripts/ral_v2` — the RA-L v2 eval pipeline

Every number in the v2 paper is produced here, from one frozen commit and one CLI line
(`docs/ral_v2_resubmission/PLAN.md` P3, P8). No v1 number is reused and nothing is hand-typed.

| File | What it is |
|---|---|
| `datasets.py` | Where each sequence's images, calib and ground truth live, in which frame, at what capture rate. Plus the sequence sets (TUM-10, KITTI-11, ABL-10, GT-5 …). |
| `arms.py` | The paper configuration and every ablation arm, as deltas. An arm is never hand-typed into a shell line. |
| `traj_eval.py` | Sim(3) **and** SE(3) ATE, Umeyama scale `s`, scale drift, RPE — via evo as a library, because the CLI does not return the alignment scale. |
| `eval_run.py` | Runs HSLAM, samples GPU/host memory, parses the diagnostic tags, appends one `summary.csv` row per rep. |
| `make_tables.py` | `summary.csv` → LaTeX tables + `numbers.json` (the prose ledger). |

## Use

```bash
# one arm, one sequence, n=5
python3 run_scripts/ral_v2/eval_run.py --dataset kitti --sequence 07 --arm full --reps 5 --out runs/wp1

# the monocular baseline
python3 run_scripts/ral_v2/eval_run.py --dataset kitti --sequence 07 --arm A0 --reps 5 --out runs/wp1

# tables (refuses mixed commits and dirty rows)
python3 run_scripts/ral_v2/make_tables.py --csv runs/wp1 --out docs/ral_v2_resubmission/tables
```

`--extra` passes flags through verbatim and must come last:
`... --arm full --extra --ml-alpha-w 2500`

## Rules the code enforces, and why

1. **`status != OK` ⇒ failed row, whatever the exit code, and its fps is discarded.**
   Observed 2026-09-18: an fp16 warmup failure disabled ML, the run finished monocular,
   exited 0, and reported `pipeline_fps=837.44`. A fast, plausible fps from a run that did no
   ML is exactly the kind of number that produced the v1 record.
2. **`pipeline_fps` comes from `[PERF_SUMMARY]`**, never the console `N Frames (X fps)` line,
   which is the dataset capture rate.
3. **No pooling across commits** (`make_tables.py` refuses) — arms from different binaries are
   not comparable.
4. **No dirty rows in a paper table** — a row whose tree was dirty is not reproducible from its
   commit hash.
5. **Failures are reported, not dropped.** KITTI 08/09 do not track at HEAD; they appear in the
   sequence set and surface as failures with a success rate.
6. **EuRoC ground truth is transformed to the camera frame** with `T_BC` from
   `cam0/sensor.yaml`. Verified: `|T_WC − T_WB|` is constant at 0.0689 m over all 36,382 MH_01
   poses, matching `|t_BC|` computed independently from the YAML.
7. **`--depth-source=none` is required for the monocular arm.** Merely omitting `--ml-depth`
   leaves `depth_src=ml` with zero inferences, which `[RUN_SUMMARY]` correctly stamps `NO_ML`.

## Scale honesty

Every trajectory is reported as Sim(3) ATE, SE(3) ATE, scale `s`, drift and RPE. Reviewer
R11-c's objection to v1 was that metric scale was claimed but Sim(3)-aligned away — so a
number that only exists after Sim(3) alignment is never presented as evidence the system is
metric. The KITTI 07 smoke shows why in one line: monocular reaches ATE$_{Sim3}$ 1.43 m but
ATE$_{SE3}$ 35.6 m at `s`=11.5, while the corrected-geometry arm is 3.39 m / 3.50 m at `s`=1.02.
