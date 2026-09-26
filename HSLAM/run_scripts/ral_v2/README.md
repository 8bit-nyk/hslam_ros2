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
| `coverage.py` | Per-run coverage and time to first pose, for every system (WP7 pre-registration §1): image timeline count- and rate-asserted, time maps, span with tail credit, own-pose fraction. |
| `wp7_reliability.py` | WP7 as-run table from a manifest of runs: run success, the ATE cell, reliability metrics R1–R4, sensitivity analyses S1–S5. `--validate` must pass before first use (`wp7_b6_manifest.csv`). |
| `adoption_rule.py` | The dataset-level adoption rule v2 — the **only** implementation (conditions A/B/C, C2 breakage band). Import it, never re-derive it. |
| `campaign_lib.sh` | Runner template: epoch (binary-hash) check, clean-tree and required-flag checks, and a resume guard that tops up only the missing reps. Source it from every campaign script. |
| `prewp4_stage1.sh`, `prewp4_r0_check.py`, `prewp4_r3b_screen.py` | Pre-WP4 stage 1 (done 25 Sep): R0 with the old-binary (b′) control, then the R3b founding-segment screen, each with its pre-registered scorer. |
| `prewp4_stage2.sh` | Pre-WP4 stage 2, the re-freeze campaign: 690 runs, 5 arms, hazard-first, with the C2 breakage early stop for the candidate arms. `DRY=1` prints the plan. |
| `prewp4_stage2_score.py` | Stage 2's pre-registered scorers: (d1) re-freeze no-regression against the R0-licensed rows, (d2)/(d4) `adoption_rule.py` verdicts with P5a parity beside them, (d3) the A0 vs A0_tol reliability contrast, and KITTI 08/09 as measured. `--part` selects one. |
| `prewp4_matched_span.py` | (d5) matched-span ATE for the P5a coverage mismatches (reviewer R10.6): common associated interval over every usable rep of both arms, re-scored with `traj_eval.evaluate(t_range=…)`. Supplementary table only. |

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
