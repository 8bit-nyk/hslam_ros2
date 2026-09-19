"""Ablation arms for RA-L v2, as code.

PAPER_CONFIG_AND_GATES.md section 3 as executable definitions, so that an arm is never
hand-typed into a shell line. Every arm is a delta from PAPER_CONFIG; eval_run.py records
the resolved CLI in the `cli` column of every row, so a row is always reproducible even if
this file changes later.

Two notes that are easy to get wrong and expensive to get wrong:

  * --ml-isotropic-input is applied where fx != fy (KITTI, EuRoC). That is derived from
    calibration, not tuned, so it does not violate the one-configuration rule (PLAN.md P3).
    Leaving F1 on without F2 on a non-square-pixel camera is ill-posed -- on EuRoC the
    letterbox binding axis flips and the canonical factor is ~30 % wrong.

  * --p2-gate is the validated Indirect.H2 loop-closure scale gate (cad6539). It is NOT
    setting_disableIndirectP2LoopCloser, which is a different, never-validated gate that
    stays off. Do not "simplify" these into one flag.

The paper configuration is provisional until gate G1 (WP1), after which PAPER_CONFIG below
is frozen and its commit + CLI are written into DECISIONS.md.
"""
from __future__ import annotations

from pathlib import Path

HSLAM_ROOT = Path(__file__).resolve().parents[2]

# Absolute, because eval_run.py runs each rep in its own working directory (the binary
# writes result.txt into CWD). A relative --ml-model would resolve against that dir and fail.
MODEL = HSLAM_ROOT / "models" / "metric3d-vit-small" / "onnx" / "model.onnx"

# Cameras whose rectified fx != fy need the isotropic pre-resize (Sprint 11 F2).
# TUM mono-VO's FOV-model rectification also lands at fx != fy (277.34 / 291.40, ratio 0.952 --
# observed 2026-09-19 in [ML_GEOM]); the rule is calibration-derived, so it joins the set.
_NON_SQUARE_PIXEL_DATASETS = {"kitti", "euroc", "tummonovo"}

# --- the paper configuration (provisional until G1) ---------------------------------------
PAPER_CONFIG = [
    "--ml-depth", "--ml-gpu",
    "--ml-model", str(MODEL),
    "--ml-model-type", "metric3d",
    "--ml-input-geometry=metric3d",          # F0: Metric3D's 616x1064, not 518x518
    "--ml-canonical-scale=true",             # F1: D * fx_eff * letterbox_s / 1000
    "--ml-init=true", "--ml-alpha-w", "10000",
    "--ml-idepth-prior=box", "--ml-idepth-uncertainty", "0.30",
    "--ml-inference-mode", "0", "--ml-inference-every-n", "2", "--ml-mean-strategy", "0",
    "--p2=false", "--p3=false", "--vs=false",
    "--ml-indirect-filter=true",
    "--indirect-ml-semantic-fix=true", "--p2-gate=true", "--p2-gate-thresh", "0.5",
    "--indirect-mp-ml-storage=true",
    "--ml-normal-channel=off",               # normals are out of v2 unless WP1 says otherwise
]

# Monocular backbone: same build, same commit, no --ml-depth. Arm A0 everywhere.
#
# --depth-source=none is REQUIRED, not decoration. setting_depthSource defaults to ML, so a run
# that merely omits --ml-depth still declares depth_src=ml with zero inferences, and
# [RUN_SUMMARY] correctly stamps it status=NO_ML -- which eval_run.py then treats as a failed
# row. Caught on 2026-09-18 when the first A0 run came back unusable. Declaring the arm
# honestly makes it arm=mono, status=OK.
MONO = ["--depth-source=none", "--ml-init=false"]

# Cumulative build-up (A0..A5) and knock-outs / knock-ins (K*, S*), as deltas.
_DELTAS: dict[str, list[str]] = {
    "full": [],
    "A0": None,                                          # sentinel: monocular, see build()
    "A1": ["--ml-inference-mode", "1", "--ml-idepth-prior=none",
           "--ml-indirect-filter=false", "--p2-gate=false"],
    "A2": ["--ml-indirect-filter=false", "--p2-gate=false"],
    "A3": ["--p2-gate=false"],
    "A4": [],                                            # == full until WP2 ships A5
    "K1": ["--ml-idepth-prior=none"],                    # - P1 bound (the TRUE P1 ablation)
    "K2": ["--ml-indirect-filter=false"],                # - Step-2 matcher filter
    "K3": ["--p2-gate=false"],                           # - loop-closure scale gate (H2)
    "K5": ["--p2=true"],                                 # + Direct.P2 BA term
    "K6": ["--p3=true"],                                 # + Direct.P3 tracker fusion
    "K7": ["--vs=true"],                                 # + virtual stereo
    "K8": ["--ml-normal-channel=on"],                    # + normal channel (foreshortening+AngMF)
    "K9_n3": ["--ml-inference-every-n", "3"],
    "K9_n5": ["--ml-inference-every-n", "5"],
    "K10": ["--ml-idepth-prior=relative", "--ml-idepth-rel-q", "0.30"],
    "K11": ["--indirect-mp-ml-storage=false"],           # - Indirect.P0 MapPoint ML storage
    "S1_alphaw_lo": ["--ml-alpha-w", "2500"],
    "S1_alphaw_hi": ["--ml-alpha-w", "40000"],
    "S1_unc_lo": ["--ml-idepth-uncertainty", "0.10"],
    "S1_unc_hi": ["--ml-idepth-uncertainty", "0.70"],
    # WP3b (EuRoC mechanism investigation, 2026-09-19) -- diagnostic arms, never paper arms.
    # M1: ML inference at the first keyframe only (metric init), then the backbone runs
    #     monocular. Isolates "the prior keeps feeding the map" from "the prior sets the scale".
    "M1_init_only": ["--ml-inference-mode", "1"],
    # full_diag: the paper config plus the trace/activation statistics printf. The flag is
    #     documented DIAGNOSTIC ONLY (ImmaturePoint.h:99) -- no behavioural effect.
    "full_diag": ["--diag-trace-stats=true"],
    # WP3c (2026-09-19): the ML linearisation freeze. Shipped behaviour keeps an ML-seeded point's
    # BA linearisation depth (idepth_zero) at the prior for its whole life; false = stock DSO.
    "K12": ["--ml-fej-freeze=false"],
    "K1_K12": ["--ml-idepth-prior=none", "--ml-fej-freeze=false"],
    # Throughput levers (WP1). fp16 is PARKED by decision 2026-09-18: the fp16 graph needs a
    # float16 input tensor that preprocessing does not produce. No fp16 arm is defined here
    # on purpose -- an arm that silently falls back to fp32 would be worse than none.
    # L_nopad -- **KILLED 2026-09-18 by its own pre-registered offline check.** Kept defined so the
    # negative result is reproducible, but it must not enter any table as a candidate: against
    # KITTI-07 LiDAR it moves the depth scale -23.3% (1.0060 -> 0.7717) versus a 3% kill threshold.
    "L_nopad": ["--ml-input-geometry=metric3d-nopad"],
    # Legacy (defective) geometry, for the WP5 three-arm prior-quality experiment only.
    "legacy_geom": ["--ml-input-geometry=legacy", "--ml-canonical-scale=false",
                    "--ml-isotropic-input=false"],
    "gt_depth": ["--depth-source=gt"],
}

ARMS = ["A0"] + [k for k in _DELTAS if k != "A0"]


def _apply(base: list[str], delta: list[str]) -> list[str]:
    """Append delta, dropping any earlier setting of the same option.

    cxxopts takes the last occurrence, so appending alone would work, but a CLI that
    contradicts itself is unreadable in the `cli` column and invites mis-transcription.
    """
    def key(tok: str) -> str | None:
        return tok.split("=", 1)[0] if tok.startswith("--") else None

    overridden = {key(t) for t in delta if key(t)}
    out, skip_value = [], False
    for tok in base:
        if skip_value:
            skip_value = False
            continue
        k = key(tok)
        if k and k in overridden:
            skip_value = "=" not in tok      # space-separated form: drop its value too
            continue
        out.append(tok)
    return out + delta


def build(arm: str, spec) -> list[str]:
    """Resolve an arm name to a full HSLAM argument list for this sequence."""
    if arm not in _DELTAS:
        raise ValueError(f"unknown arm {arm!r}; known: {', '.join(ARMS)}")

    if arm == "A0":
        return list(MONO)

    args = _apply(list(PAPER_CONFIG), _DELTAS[arm])

    # Calibration-derived, not tuned: square-pixel pre-resize where fx != fy.
    if spec.dataset in _NON_SQUARE_PIXEL_DATASETS and "--ml-input-geometry=legacy" not in args:
        args = _apply(args, ["--ml-isotropic-input=true"])
    return args
