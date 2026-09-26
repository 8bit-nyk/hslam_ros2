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

# --- the paper configuration as FROZEN BY G1 at 6799be6 (18 Sep 2026) -----------------------
# Kept verbatim: it is the R0 reference CLI at the re-freeze epoch (arm `full_6799be6`) and the CLI
# every row before the re-freeze was produced with. Do not edit it.
PAPER_CONFIG_6799BE6 = [
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

# --- the RE-FROZEN paper configuration (pre-WP4, DECISIONS.md "Re-freeze epoch", 2026-09-25) ---------
# = 6799be6 + the two WP2a hygiene bug fixes adopted 20 Sep (D1: K14 Phase-0 factor without the second
# division, K15 defined blend gradient) - the unconsumed --indirect-ml-semantic-fix (D8). The binary-side
# bug fixes (D4 loop-Sim(3) guard, D7 associations path, D8 use-after-free) are default-on and need no
# flag. Stage 1 appends --init-founding-fix=<mode> here if and only if the R3b screen passes as
# pre-registered.
PAPER_CONFIG = [t for t in PAPER_CONFIG_6799BE6 if t != "--indirect-ml-semantic-fix=true"] + [
    "--ml-init-scale=median",                # K14 (WP3e-1 / WP2a)
    "--p1-blend-grad-fix=true",              # K15 (WP3e-2 / WP2a)
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
# ⚠ Since the re-freeze (2026-09-25) every delta below applies to the RE-FROZEN PAPER_CONFIG. Rows
# written before it carry their full CLI in the `cli` column; the arm NAME alone is not provenance.
_DELTAS: dict[str, list[str]] = {
    "full": [],
    "A0": None,                                          # sentinel: monocular, see build()
    # --- pre-WP4 re-freeze epoch (DECISIONS.md "Re-freeze epoch", 2026-09-25) ---
    "full_6799be6": None,                                # sentinel: the G1 CLI verbatim (R0), see build()
    "A0_tol": None,                                      # sentinel: monocular + the ML arms' init bar (D6)
    "full_K13": ["--ml-prior-source=fresh"],             # D3: own-view prior on the re-frozen config
    "full_R3b_relin": ["--init-founding-fix=relin"],     # D2 / WP2a-R3b screen arms
    "full_R3b_anchor": ["--init-founding-fix=anchor"],
    # --- WP4 (DECISIONS.md "WP4 -- PRE-REGISTERED 2026-09-26"; wp/WP4_component_ablation.md §4) ---------------
    # New names on purpose: "A1" and "K10" below already carry WP3b EuRoC rows under their old definitions.
    # A1_near: the nearest runnable init-only arm (audit V1). Inference once (mode 1); fresh_only gives every
    #   keyframe after KfId 1 no prior; no Step-2 filter, no H2. Residue: KfId 1's own-view prior, the founding
    #   points' frozen linearisation, the ML arms' init bar.
    "A1_near": ["--ml-inference-mode", "1", "--ml-prior-source=fresh_only",
                "--ml-indirect-filter=false", "--p2-gate=false"],
    # A1_strict: A1_near with every prior channel on later points off -- "no prior on later points". Residue
    #   against A0_tol: Phase 0 itself and one extra inference.
    "A1_strict": ["--ml-inference-mode", "1", "--ml-prior-source=fresh_only",
                  "--ml-indirect-filter=false", "--p2-gate=false",
                  "--ml-idepth-prior=none", "--ml-seed=midpoint", "--p1-clamps=false",
                  "--ml-fej-freeze=false", "--indirect-mp-ml-storage=false"],
    "K0": ["--ml-init=false"],                           # - P0: photometric founding pair, metric priors after it
    "K1c": ["--p1-clamps=false"],                        # - trace-time bracket intersection + activation clamp
    "K14_off": ["--ml-init-scale=legacy"],               # - K14 (re-freeze bug fix): the double division back
    "K15_off": ["--p1-blend-grad-fix=false"],            # - K15 (re-freeze bug fix): the UB blend back
    "K10_q037": ["--ml-idepth-prior=relative", "--ml-idepth-rel-q", "0.37"],   # calibration-derived q (audit §5)
    "H2_t03": ["--p2-gate-thresh", "0.3"],               # H2 threshold sensitivity (stricter)
    "H2_t07": ["--p2-gate-thresh", "0.7"],               # H2 threshold sensitivity (looser)
    # ---------------------------------------------------------------------------------------------------------
    # A1 is NOT init-only (audit V1: the first keyframe's map is re-injected into every later point). Kept only so
    # the WP3b rows reproduce; never a WP4 arm, never cited as "init only".
    "A1": ["--ml-inference-mode", "1", "--ml-idepth-prior=none",
           "--ml-indirect-filter=false", "--p2-gate=false"],
    "A2": ["--ml-indirect-filter=false", "--p2-gate=false"],
    "A3": ["--p2-gate=false"],
    "A4": [],                                            # == full until WP2 ships A5
    "K1": ["--ml-idepth-prior=none"],                    # - the INITIAL BOX only (audit V2; seed, freeze, clamps stay)
    "K2": ["--ml-indirect-filter=false"],                # - Step-2 matcher filter
    "K3": ["--p2-gate=false"],                           # - loop-closure scale gate (H2)
    "K5": ["--p2=true"],                                 # + Direct.P2 BA term
    "K6": ["--p3=true"],                                 # + Direct.P3 tracker fusion
    "K7": ["--vs=true"],                                 # + virtual stereo
    "K8": ["--ml-normal-channel=on"],                    # + normal channel (foreshortening+AngMF)
    "K9_n3": ["--ml-inference-every-n", "3"],
    "K9_n5": ["--ml-inference-every-n", "5"],
    "K10": ["--ml-idepth-prior=relative", "--ml-idepth-rel-q", "0.30"],   # WP3b rows only; WP4 uses K10_q037
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
    # WP3d (2026-09-19): which inference seeds a keyframe's points. Shipped = the previous ML
    # keyframe's map sampled at the current pixels (no warp); fresh = the keyframe's own map.
    "K13_fresh": ["--ml-prior-source=fresh"],
    "K13_fresh_n1": ["--ml-prior-source=fresh", "--ml-inference-every-n", "1"],   # every KF gets its own map
    "K13_fresh_only": ["--ml-prior-source=fresh_only"],
    "K12_K13": ["--ml-fej-freeze=false", "--ml-prior-source=fresh"],
    "K12_K13_n1": ["--ml-fej-freeze=false", "--ml-prior-source=fresh", "--ml-inference-every-n", "1"],
    # WP3e-1 (2026-09-19, reviewer finding F1): Phase-0 metric factor without the second division
    # by photometricScale, so the founding pair sits at the prior's scale like every later point.
    "K14_init_median": ["--ml-init-scale=median"],
    "K12_K13_K14": ["--ml-fej-freeze=false", "--ml-prior-source=fresh", "--ml-init-scale=median"],
    "K12_K14": ["--ml-fej-freeze=false", "--ml-init-scale=median"],
    "K1_K12_K14": ["--ml-idepth-prior=none", "--ml-fej-freeze=false", "--ml-init-scale=median"],
    # WP3f: replace the hidden pull (freeze) by the EXPLICIT weighted prior term (Direct.P2, --p2=true)
    "K12_P2": ["--ml-fej-freeze=false", "--p2=true"],
    "K12_K14_P2": ["--ml-fej-freeze=false", "--ml-init-scale=median", "--p2=true"],
    # WP3e-2 (reviewer B F1): the disjoint-bracket blend on a defined gradient instead of garbage
    "K15_blendfix": ["--p1-blend-grad-fix=true"],
    # WP2a (2026-09-19): the three integration-hygiene fixes together -- the candidate hygiene
    # configuration (own-view prior, Phase-0 factor without the double division, defined blend).
    # Each single is a K13/K14/K15 arm above; this is their interaction check. Not a paper arm
    # until G2 passes.
    "K13_K14_K15": ["--ml-prior-source=fresh", "--ml-init-scale=median", "--p1-blend-grad-fix=true"],
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

# --- WP2c (2026-09-19): the explicit weighted, gated prior in place of the linearisation freeze ---
# Base of every 2c arm: freeze OFF, Direct.P2 ON, plus the integration-hygiene fixes that passed WP2a
# (DECISIONS.md "WP2a OUTCOME" / "WP2a-R2 OUTCOME", 2026-09-20): K14 init scale + K15 blend fix.
# K13 (--ml-prior-source=fresh) is NOT in it: it regresses fr2_large_no_loop 2x at n=10.
_HYGIENE = ["--ml-init-scale=median", "--p1-blend-grad-fix=true"]
_DELTAS["K14_K15"] = list(_HYGIENE)          # the candidate hygiene configuration (freeze ON, P2 off)
_C_BASE = ["--ml-fej-freeze=false", "--p2=true"]


def _c_arm(w, k, seed=None, gate=None):
    """--ml-prior-weight w, --ml-prior-gate-k k (0 = legacy tau, >=100 = no self-gate)."""
    d = list(_C_BASE) + list(_HYGIENE) + ["--ml-prior-weight", str(w), "--ml-prior-gate-k", str(k)]
    if seed:
        d += ["--ml-seed", seed]
    if gate is not None:
        d += ["--ml-align-gate", str(gate)]
    return d


for _w in (1, 10, 100, 1000):
    for _k, _kl in ((1, "k1"), (3, "k3"), (1000, "knone")):
        _DELTAS[f"C_w{_w}_{_kl}"] = _c_arm(_w, _k)
_DELTAS["C_w100_k0"] = _c_arm(100, 0)                       # legacy absolute tau, strong weight
_DELTAS["C_base"] = list(_C_BASE) + list(_HYGIENE)         # freeze off + P2 legacy + hygiene (= K12_K13_K14 + blend + P2)
_DELTAS["C_hyg_only"] = ["--ml-fej-freeze=false"] + list(_HYGIENE)   # freeze off, no explicit prior
# --ml-seed variants (phase "seed") of every sweep arm, so the pick needs no new arm definition;
# the gated candidate C_final is appended once phase ii has set thr -- see DECISIONS.md.
for _name in [k for k in _DELTAS if k.startswith("C_w")]:
    _DELTAS[f"{_name}_seedmid"] = _DELTAS[_name] + ["--ml-seed", "midpoint"]
    _DELTAS[f"{_name}_seedbr"] = _DELTAS[_name] + ["--ml-seed", "prior_if_in_bracket"]

# WP2b-log (card b2, DECISIONS.md "WP2b-log -- PRE-REGISTERED"): the same explicit prior with a
# RELATIVE (log-depth) residual, r = log(d/d_ML), weighted by one dimensionless sigma instead of the
# point's absolute P1 half-width. Same base as the C arms (freeze OFF + P2 on + the hygiene base), so
# an L arm differs from its C twin in the parameterisation alone.
def _l_arm(w, k, slog=0.30, seed=None, gate=None):
    d = list(_C_BASE) + list(_HYGIENE) + ["--ml-prior-param=log", "--ml-prior-weight", str(w),
                                          "--ml-prior-gate-k", str(k), "--ml-prior-sigma-log", str(slog)]
    if seed:
        d += ["--ml-seed", seed]
    if gate is not None:
        d += ["--ml-align-gate", str(gate)]
    return d


for _w in (1, 10, 100, 1000):
    for _k, _kl in ((1, "k1"), (3, "k3"), (0, "knone")):     # log mode: k = 0 means NO self-gate
        _DELTAS[f"L_w{_w}_{_kl}"] = _l_arm(_w, _k)
# sigma_log sensitivity at the reference weight (one cross-dataset constant; this checks it is not a
# knife edge, it is not a per-dataset choice)
for _sl in (0.15, 0.60):
    _DELTAS[f"L_w10_k3_s{str(_sl).replace('.', '')}"] = _l_arm(10, 3, _sl)
_DELTAS["L_base"] = _l_arm(1, 0)                             # log residual at the shipped multiplier, no gate

# WP2b-log BOUNDARY PROBE (DECISIONS.md "WP2b-log -- boundary probe -- PRE-REGISTERED 2026-09-22").
# Phase i cleared bar 1 (KITTI 07 scale) ONLY at w=1000, the top of its grid: w=1/10/100 sat at
# 0.67-0.82 and w=1000 jumped to 0.94-0.97. These arms ask whether w=1000 is the low edge of a
# plateau or a peak. They are DIAGNOSTIC: the pre-registered pick rule takes the SMALLEST w clearing
# all four bars, so no probe arm can displace L_w1000_k1. Kept out of the PHASE=i grid on purpose --
# re-running phase i must reproduce phase i.
for _w in (3000, 10000):
    for _k, _kl in ((1, "k1"), (3, "k3")):
        _DELTAS[f"L_w{_w}_{_kl}"] = _l_arm(_w, _k)

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
    if arm == "A0_tol":
        # D6 fairness arm: monocular held to the same initialisation-failure bar as every ML arm.
        return list(MONO) + ["--init-fail-thresholds=tolerant"]

    base = PAPER_CONFIG_6799BE6 if arm == "full_6799be6" else PAPER_CONFIG
    args = _apply(list(base), _DELTAS[arm] or [])

    # Calibration-derived, not tuned: square-pixel pre-resize where fx != fy.
    if spec.dataset in _NON_SQUARE_PIXEL_DATASETS and "--ml-input-geometry=legacy" not in args:
        args = _apply(args, ["--ml-isotropic-input=true"])
    return args
