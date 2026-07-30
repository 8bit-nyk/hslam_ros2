#include <thread>
#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fstream>
#include <iomanip>

#include "IOWrapper/Output3DWrapper.h"

#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "util/DatasetReader.h"
#include <pangolin/pangolin.h>
#include "util/globalCalib.h"

#include "util/NumType.h"
#include "util/DepthLogger.h"
#include "FullSystem/FullSystem.h"

#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"

#include <cxxopts.hpp>

// RGB-D pipeline includes
#include "tum_benchmark/tum_benchmark.hpp"
#include <opencv2/opencv.hpp>

// ML depth integration includes
#include "ML/MLInference.h"

// FPS logging includes
#include "util/FPSLogger.h"

// Custom structure for parsing associations.txt
struct RGBDAssociation {
    double rgb_timestamp;
    std::string rgb_file;
    double depth_timestamp;
    std::string depth_file;
};

// Stream operator for parsing associations.txt
inline std::istream& operator>>(std::istream& is, RGBDAssociation& assoc) {
    return is >> assoc.rgb_timestamp >> assoc.rgb_file >> assoc.depth_timestamp >> assoc.depth_file;
}

using namespace HSLAM;

/**
 * Converts HSLAM's undistorted float channels to RGB Mat for ML processing
 */
cv::Mat convertUndistortedToRGB(ImageAndExposure* img) {
    cv::Mat rgb_color = cv::Mat(img->h, img->w, CV_8UC3);
    
    // Optimized conversion using direct pointer access
    unsigned char* rgb_ptr = rgb_color.ptr<unsigned char>(0);
    int total_pixels = img->h * img->w;
    
    for (int i = 0; i < total_pixels; i++) {
        // BGR order for OpenCV Mat
        rgb_ptr[i * 3 + 0] = (unsigned char)std::min(255.0f, std::max(0.0f, img->b_image[i]));
        rgb_ptr[i * 3 + 1] = (unsigned char)std::min(255.0f, std::max(0.0f, img->g_image[i]));
        rgb_ptr[i * 3 + 2] = (unsigned char)std::min(255.0f, std::max(0.0f, img->r_image[i]));
    }
    
    return rgb_color;
}

void my_exit_handler(int s)
{
	printf("Caught signal %d\n",s);
	exit(1);
}

void exitThread()
{
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_exit_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	while(true) pause();
}


int main(int argc, char **argv)
{
	boost::thread exThread = boost::thread(exitThread); // hook crtl+C.

	cxxopts::Options options("HSLAM", "Direct Indirect Feature Fusion SLAM");

	std::string source = "";
	std::string calib = "";
	std::string vocabPath = "";
	std::string vignette = "";
	std::string gammaCalib = "";
	std::string associations = "";

	options.add_options()
        ("f,files", "Input images path - mandatory input", cxxopts::value(source))
		("c,calib", "Camera intrinsic callibration - mandatory input", cxxopts::value(calib))
		("v,vocab", "Path to Vocabulary file - required for loop closure", cxxopts::value(vocabPath))
		("n,vignette", "Path to photmetric calibration Vignette model", cxxopts::value(vignette))
		("g,gamma", "Path to photmetric calibration gamma response Model", cxxopts::value(gammaCalib))
		("a,associations", "Path to associations.txt file for RGB-D data", cxxopts::value(associations))
		("l,loopclosure", "Enable-Disable loop closure", cxxopts::value<bool>()->default_value("false"))
		("r,reverse", "Play a sequence in reverse", cxxopts::value<bool>()->default_value("false"))
		("preload", "Preload all images into memory", cxxopts::value<bool>()->default_value("false"))
		("usesampleoutput", "Replace pangolinViewer with another output wrapper", cxxopts::value<bool>()->default_value("false"))
		("nolog", "Disable logging optimization data", cxxopts::value<bool>()->default_value("false"))
		("nogui", "Disable GUI", cxxopts::value<bool>()->default_value("false"))
		("save", "Save debug images", cxxopts::value<bool>()->default_value("false"))
		("quiet", "Disable message printing", cxxopts::value<bool>()->default_value("true"))
		("nomt", "Turns off multiThreading", cxxopts::value<bool>()->default_value("false"))
		("s,startindex", "Image to start from", cxxopts::value<int>()->default_value("0"))
        ("e,endindex", "Last image to be processed", cxxopts::value<int>()->default_value("100000"))
		("m,mode", "System mode: 0: use precalibrated gamma and vignette -1: photometric mode without calibration - 2: photometric mode with perfect images", cxxopts::value<int>()->default_value("1"))
        ("preset", "Preset configuration}", cxxopts::value<int>()->default_value("0"))
		("speed", "Enforce playback Speed to real-time", cxxopts::value<float>()->default_value("0.0"))
        ("C,colour", "Read colour images", cxxopts::value<bool>()->default_value("false"))
		("S,use16bit", "Read 16 bit images", cxxopts::value<bool>()->default_value("false"))
		("P,outPC", "Output point cloud", cxxopts::value<bool>()->default_value("false"))
		("E,pauseEnd", "Pause at end", cxxopts::value<bool>()->default_value("false"))
		("ml-depth", "Enable ML depth estimation", cxxopts::value<bool>()->default_value("false"))
		("ml-model", "Path to ONNX ML depth model", cxxopts::value<std::string>()->default_value("models/metric3d-vit-small/onnx/model.onnx"))
		// NOTE: "every_frame" was advertised here but never implemented — the validator below
		// silently rewrites it to keyframe_only, and no per-frame inference path exists (ML is
		// gated inside makeKeyFrame). Use --ml-inference-every-n to control cadence instead.
		("ml-strategy", "ML inference strategy (keyframe_only|snapshot_mode)", cxxopts::value<std::string>()->default_value("keyframe_only"))
		("ml-snapshot-interval", "Frames between ML inference in snapshot mode", cxxopts::value<int>()->default_value("5"))
		("ml-benchmark", "Enable ML performance benchmarking", cxxopts::value<bool>()->default_value("false"))
		("ml-gpu", "Enable GPU acceleration for ML inference", cxxopts::value<bool>()->default_value("false"))
		("ml-fp16", "Enable FP16 optimization for GPU inference", cxxopts::value<bool>()->default_value("false"))
		("ml-gpu-device", "GPU device ID for ML inference", cxxopts::value<int>()->default_value("0"))
		("ml-gpu-memory", "GPU memory limit in MB", cxxopts::value<size_t>()->default_value("2048"))
		("ml-model-type", "ML model type: metric3d|depth_anything_v2|midas (default metric3d)", cxxopts::value<std::string>()->default_value("metric3d"))
		("ml-input-width", "Model input width (default 518 for metric3d/dav2)", cxxopts::value<int>()->default_value("518"))
		("ml-input-height", "Model input height (default 518 for metric3d/dav2)", cxxopts::value<int>()->default_value("518"))
		("ml-output-scale", "Multiplier on relative-depth model output (DA V2 only). Default 1.0 = pure relative.", cxxopts::value<float>()->default_value("1.0"))
		("ml-alpha-w", "Phase 0 alpha-prior strength when ML-seeded (default 10000=cross-regime sweet spot; 22500 = mono). Tighter prior reduces iR scatter on jerky motion.", cxxopts::value<float>()->default_value("10000"))
		("ml-idepth-uncertainty", "Phase 1 base idepth uncertainty for ML bounds (default 0.30 = re-calibrated for AngMF per Sprint 3a; was 0.20 pre-Sprint3a). Smaller = tighter.", cxxopts::value<float>()->default_value("0.30"))
		("ml-mean-strategy", "mlMeanDepth computation: 0=arith mean of valid (legacy), 1=median, 2=trimmed mean 5-95%. Default 0.", cxxopts::value<int>()->default_value("0"))
		("ml-inference-mode", "ML inference cadence: 0=every Nth KF (legacy), 1=init_only (lean), 2=disabled. Default 0.", cxxopts::value<int>()->default_value("0"))
		("ml-inference-every-n", "ML inference cadence N (mode 0): run ML every Nth keyframe. 1=every keyframe (densest normals, ~2x ML cost), 2=default (Paper Table V).", cxxopts::value<int>()->default_value("2"))
		("ml-indirect-filter", "Indirect.Step2 ML depth-ratio filter in feature matchers. true=enabled (legacy), false=disabled. Default true.", cxxopts::value<bool>()->default_value("true"))
		("ml-init", "Enable ML depth for metric scale initialization", cxxopts::value<bool>()->default_value("true"))
		("depth-source", "Depth source: ml|gt|none (default ml). GT requires --associations and uses the same files as ML depth would be computed from.", cxxopts::value<std::string>()->default_value("ml"))
		// Phase toggles for the Phase C config matrix (Phase B/C research). Defaults match current production.
		("p0", "Direct.P0: ML-based metric scale initialization (default: on)", cxxopts::value<bool>()->default_value("true"))
		("p1", "Direct.P1: ML depth bounds in ImmaturePoint tracing (default: on)", cxxopts::value<bool>()->default_value("true"))
		("p2", "Direct.P2: ML energy term in photometric BA (default: off — memory says structurally dead)", cxxopts::value<bool>()->default_value("false"))
		("p3", "Direct.P3: ML depth fusion in CoarseTracker (default: off — marginal benefit)", cxxopts::value<bool>()->default_value("false"))
		("vs", "Direct.VS: virtual stereo in DSO sliding-window BA (default: OFF — futility-tested, no measurable contribution on TUM)", cxxopts::value<bool>()->default_value("false"))
		("vs-weight", "Direct.VS weight multiplier (default matches settings.cpp; typical sweep 1e-5..1e-2)", cxxopts::value<float>()->default_value("-1"))
		("depth-scale", "Divisor applied to 16-bit depth PNG values to get meters (TUM=5000; KITTI projected depth we write at 500 for max ~130m range).", cxxopts::value<float>()->default_value("5000.0"))
		("export-map-ply", "Export marginalized PointHessians to ASCII PLY at end of run (default false)", cxxopts::value<bool>()->default_value("false"))
		("map-ply-out", "Output path for PLY export (default: same directory as result.txt with .ply extension)", cxxopts::value<std::string>()->default_value(""))
		("use-normal-integration", "Sprint 1 master gate: enable downstream consumers of predicted_normal (default false — normals plumbed but not yet read)", cxxopts::value<bool>()->default_value("false"))
		("ml-prior-centred-trace", "Sprint 12: when the epipolar search segment exceeds maxPixSearch, centre the retained window on the ML prediction instead of anchoring it at uMin. Only affects points whose search was already being truncated. (default false)", cxxopts::value<bool>()->default_value("false"))
		("diag-trace-stats", "Emit [TRACE_STATS] (first-epipolar-trace status histogram) and [ACT_STATS] (activated points that never completed a trace, and those with idepth_min<0). Pure instrumentation, no behavioural effect. Measures whether the ML idepth bound is narrowing DSO's search or translating it off the prediction. (default false)", cxxopts::value<bool>()->default_value("false"))
		("ml-normal-channel", "Phase 0 control arm: 'on' (default) or 'off'. OFF removes the ENTIRE normal head from the pipeline — confidence forced to 1.0 (kappa is a normal-head output, so --ml-foreshortening=false alone never achieved this) and the normal map not extracted. Required to separate 'the normal channel adds value' from 'the depth fix adds value'.", cxxopts::value<std::string>()->default_value("on"))
		("ml-input-geometry", "Sprint 11 F0: ML input geometry for METRIC3D_V2. 'legacy'=518x518 (Depth-Anything's geometry, what production shipped); 'metric3d'=616x1064 (Metric3D-v2's own ViT recipe). Must be combined with --ml-canonical-scale to be correct. (default legacy)", cxxopts::value<std::string>()->default_value("legacy"))
		("ml-canonical-scale", "Sprint 11 F1: apply Metric3D's canonical->real depth rescale D*fx_eff*letterbox_s/1000. The ONNX graph has no intrinsics input, so this MUST be applied externally and never was — see INTEGRATION_DAMAGE_AUDIT.md D1 (default false)", cxxopts::value<bool>()->default_value("false"))
		("ml-isotropic-input", "Sprint 11 F2: pre-resize the ML input to square pixels when rectified fx != fy (KITTI ships 368.88/703.52 = 1.9x anisotropic; degrades predicted normals 3.1deg -> 12.5deg) (default false)", cxxopts::value<bool>()->default_value("false"))
		("ml-foreshortening", "Sprint 2 CD-H5: apply |n·z_hat| foreshortening factor to idepth uncertainty (default true after WIN verdict)", cxxopts::value<bool>()->default_value("true"))
		("ml-angmf-confidence", "Sprint 3 A.2: use AngMF expected-angle formula 1-E_angle(kappa)/pi instead of mis-oriented sigmoid 1/(1+kappa/5) (default true after Sprint 3a WIN: unc recalibrated to 0.30)", cxxopts::value<bool>()->default_value("true"))
		("ml-normal-gapfill-mask", "Sprint 4 CD-H3: gate CoarseTracker lvl=0 gap-fill on normal-cosine agreement (default false — KILL: cos_min=0.873 at threshold 0.866, 0% skip on TUM)", cxxopts::value<bool>()->default_value("false"))
		("ml-normal-gapfill-cos", "Sprint 4 CD-H3: cosine threshold for normal gap-fill mask; cos(30deg)=0.866, cos(40deg)=0.766 (default 0.866)", cxxopts::value<float>()->default_value("0.866"))
		("ml-normal-optreg", "Sprint 5 CD-H7/B-NEW-1: normal-cosine-weighted iR smoothing in CoarseInitializer::optReg (default false until WIN verdict)", cxxopts::value<bool>()->default_value("false"))
		("ml-normal-pixel-gate", "Sprint 6 CD-H2: gate lvl-0 pixel selection on |n·z_hat| (skip edge-on surfaces) (default false until WIN verdict)", cxxopts::value<bool>()->default_value("false"))
		("ml-normal-pixel-gate-cos", "Sprint 6 CD-H2: cosine threshold for the pixel gate; 0.5=60deg (default), 0.3=73deg (permissive)", cxxopts::value<float>()->default_value("0.5"))
		("ml-normal-indirect-info", "Sprint 7 C-NEW-2: weight indirect PoseOptimization edge information by |n·z_hat| (default false until WIN verdict)", cxxopts::value<bool>()->default_value("false"))
		("ml-normal-indirect-info-floor", "Sprint 7 C-NEW-2: floor for the |n·z_hat| edge weight; 0.2=78deg (default)", cxxopts::value<float>()->default_value("0.2"))
		("ml-gravity-test-rot", "Sprint 8 CD-H6 gauge probe: fixed world-frame rotation (deg, about camera x) applied at init; 0=off", cxxopts::value<float>()->default_value("0.0"))
		("ml-dn-ba", "Sprint 10 E3/B.4: depth-normal surface-consistency residual in photometric BA (default false until WIN verdict)", cxxopts::value<bool>()->default_value("false"))
		("ml-dn-weight", "Sprint 10 E3/B.4: lambda — DN prior stiffness as a fraction of each point's photometric Hessian (default 0.1)", cxxopts::value<float>()->default_value("0.1"))
		("ml-dn-radius", "Sprint 10 E3/B.4: neighbourhood radius in lvl-0 px for the local-plane prediction (default 30)", cxxopts::value<float>()->default_value("30.0"))
		("ml-dn-min-neighbors", "Sprint 10 E3/B.4: minimum valid neighbours before a point gets a DN prior (default 4)", cxxopts::value<int>()->default_value("4"))
		("h,help", "Print usage")
    ;

	auto result = options.parse(argc, argv);

	if (result.count("help"))
    {
		std::cout << options.help() << std::endl;
		exit(0);
    }

	LoopClosure = result["loopclosure"].as<bool>();
	bool reverse = result["reverse"].as<bool>();
	bool preload = result["preload"].as<bool>();
	bool useSampleOutput = result["usesampleoutput"].as<bool>();
	setting_logStuff = !result["nolog"].as<bool>();
	disableAllDisplay = result["nogui"].as<bool>();
	outputPC = result["outPC"].as<bool>();
	debugSaveImages = result["save"].as<bool>();
	setting_debugout_runquiet = result["quiet"].as<bool>();
	multiThreading = !result["nomt"].as<bool>();

	int startIndex = result["startindex"].as<int>();
	int endIndex = result["endindex"].as<int>();
	int preset = result["preset"].as<int>();
	int mode = result["mode"].as<int>();
	float playbackSpeed = result["speed"].as<float>(); // 0 for linearize (play as fast as possible, while sequentializing tracking & mapping). otherwise, factor on timestamps.

	bool use_colour = result["colour"].as<bool>();
	bool use_16bit = result["use16bit"].as<bool>(); 

	bool pauseEnd = result["pauseEnd"].as<bool>(); 

	// ML depth options (Phase 2 + GPU Integration)
	bool ml_depth_enabled = result["ml-depth"].as<bool>();
	std::string ml_model_path = result["ml-model"].as<std::string>();
	std::string ml_strategy = result["ml-strategy"].as<std::string>();
	int ml_snapshot_interval = result["ml-snapshot-interval"].as<int>();
	bool ml_benchmark_enabled = result["ml-benchmark"].as<bool>();
	std::string ml_model_type_str = result["ml-model-type"].as<std::string>();
	int ml_input_width = result["ml-input-width"].as<int>();
	int ml_input_height = result["ml-input-height"].as<int>();
	float ml_output_scale = result["ml-output-scale"].as<float>();
	setting_alphaWForMLInit = result["ml-alpha-w"].as<float>();
	setting_idepthUncertaintyForMLInit = result["ml-idepth-uncertainty"].as<float>();
	setting_mlMeanDepthStrategy = result["ml-mean-strategy"].as<int>();
	setting_mlInferenceMode = result["ml-inference-mode"].as<int>();
	setting_mlInferenceEveryN = std::max(1, result["ml-inference-every-n"].as<int>());
	setting_indirectMatcherUseML = result["ml-indirect-filter"].as<bool>();
	
	// Validate and normalize ML strategy parameters for ablation study
	if (ml_strategy != "keyframe_only" && ml_strategy != "snapshot_mode") {
		printf("WARNING: Invalid ml-strategy '%s', using 'keyframe_only'\n", ml_strategy.c_str());
		ml_strategy = "keyframe_only";
	}
	
	// Validate snapshot interval (repurpose as snapshot_rate for keyframes)
	if (ml_snapshot_interval < 1) {
		printf("WARNING: Invalid snapshot rate %d, using 1\n", ml_snapshot_interval);
		ml_snapshot_interval = 1;
	}
	
	// GPU acceleration options (Phase 3)
	bool ml_gpu_enabled = result["ml-gpu"].as<bool>();
	bool ml_fp16_enabled = result["ml-fp16"].as<bool>();
	int ml_gpu_device = result["ml-gpu-device"].as<int>();
	size_t ml_gpu_memory_mb = result["ml-gpu-memory"].as<size_t>();
	
	// ML initialization control (for A/B testing)
	bool ml_init_enabled = result["ml-init"].as<bool>();

	// Depth PNG scaling (for RGB-D associations loop; 5000 for TUM, ~500 for KITTI)
	float depth_scale = result["depth-scale"].as<float>();
	if (depth_scale <= 0.0f) { printf("ERROR: --depth-scale must be positive.\n"); return 0; }
	printf("[DEPTH_SCALE] divisor=%.1f (maps uint16 PNG -> meters)\n", depth_scale);

	// Map PLY export (Sprint 0d)
	setting_exportMapPly = result["export-map-ply"].as<bool>();
	setting_mapPlyOut = result["map-ply-out"].as<std::string>();
	// When PLY export is requested, enable point collection (allMargPointsHistory is gated on outputPC)
	if (setting_exportMapPly) outputPC = true;

	// Surface Normal Integration (Sprint 1) — master gate
	setting_useNormalIntegration = result["use-normal-integration"].as<bool>();
	printf("[PHASE_CONFIG] use-normal-integration=%s\n", setting_useNormalIntegration ? "on" : "off");

	// Phase 0 — master normal-channel switch (the A0/A4 control arm)
	setting_mlPriorCentredTrace = result["ml-prior-centred-trace"].as<bool>();
	if (setting_mlPriorCentredTrace)
		printf("[PHASE_CONFIG] ml-prior-centred-trace=on\n");
	setting_diagTraceStats = result["diag-trace-stats"].as<bool>();
	if (setting_diagTraceStats)
		printf("[PHASE_CONFIG] diag-trace-stats=on (instrumentation only; [TRACE_STATS] + [ACT_STATS])\n");

	{
		const std::string nc = result["ml-normal-channel"].as<std::string>();
		if (nc != "on" && nc != "off") {
			printf("ERROR: --ml-normal-channel must be 'on' or 'off' (got '%s').\n", nc.c_str());
			return 0;
		}
		setting_mlNormalChannelOff = (nc == "off");
		printf("[PHASE_CONFIG] ml-normal-channel=%s\n", nc.c_str());
		if (setting_mlNormalChannelOff && (setting_useNormalForeshortening || setting_useAngmfConfidence))
			printf("[PHASE_CONFIG] note: normal channel OFF overrides --ml-foreshortening / "
			       "--ml-angmf-confidence, which are default-true.\n");
	}

	// Sprint 11 — D0/D1/D2 Metric3D input-geometry and depth-scale correctness
	{
		const std::string geom = result["ml-input-geometry"].as<std::string>();
		if (geom != "legacy" && geom != "metric3d") {
			printf("ERROR: --ml-input-geometry must be 'legacy' or 'metric3d' (got '%s').\n", geom.c_str());
			return 0;
		}
		setting_mlMetric3dRefGeometry = (geom == "metric3d");
	}
	setting_mlCanonicalScale = result["ml-canonical-scale"].as<bool>();
	setting_mlIsotropicInput = result["ml-isotropic-input"].as<bool>();
	printf("[PHASE_CONFIG] ml-input-geometry=%s ml-canonical-scale=%s ml-isotropic-input=%s\n",
	       setting_mlMetric3dRefGeometry ? "metric3d(616x1064)" : "legacy(518x518)",
	       setting_mlCanonicalScale ? "on" : "off",
	       setting_mlIsotropicInput ? "on" : "off");
	if (setting_mlMetric3dRefGeometry != setting_mlCanonicalScale)
		printf("[PHASE_CONFIG] WARNING: F0 and F1 are only correct TOGETHER — at 518x518 the canonical "
		       "factor over-corrects (ICL 0.845 vs 1.016 at 616x1064). See INTEGRATION_DAMAGE_AUDIT.md.\n");
	// Sprint 11: ~14 sweep harnesses under scripts/ hardcode "--ml-input-width 518", so a caller can
	// silently measure the pre-fix pipeline while believing otherwise. The flags default OFF (project
	// guardrail: every new feature gated off), which makes silence the dangerous state — announce it.
	if (ml_model_type_str == "metric3d" && !setting_mlMetric3dRefGeometry && !setting_mlCanonicalScale)
		printf("[PHASE_CONFIG] WARNING: metric3d running in FULLY LEGACY geometry (D0+D1 present): "
		       "518x518 Depth-Anything letterbox and no canonical->real rescale. Depth is ~1/c too "
		       "large (TUM c=0.663, KITTI c=0.614). Intentional only for pre-fix reproduction; pass "
		       "--ml-input-geometry=metric3d --ml-canonical-scale=true for corrected runs.\n");

	// Sprint 2 — CD-H5: foreshortening factor in idepth uncertainty
	setting_useNormalForeshortening = result["ml-foreshortening"].as<bool>();
	printf("[PHASE_CONFIG] ml-foreshortening=%s\n", setting_useNormalForeshortening ? "on" : "off");

	// Sprint 3 — A.2: κ → AngMF confidence formula (replaces mis-oriented sigmoid)
	setting_useAngmfConfidence = result["ml-angmf-confidence"].as<bool>();
	printf("[PHASE_CONFIG] ml-angmf-confidence=%s\n", setting_useAngmfConfidence ? "on" : "off");

	// Sprint 4 — CD-H3: CoarseTracker lvl=0 gap-fill masking by normal discontinuity
	setting_useNormalGapfillMask = result["ml-normal-gapfill-mask"].as<bool>();
	setting_normalGapfillCosThreshold = result["ml-normal-gapfill-cos"].as<float>();
	printf("[PHASE_CONFIG] ml-normal-gapfill-mask=%s cos_thresh=%.3f\n",
	       setting_useNormalGapfillMask ? "on" : "off", setting_normalGapfillCosThreshold);

	// Sprint 5 — CD-H7/B-NEW-1: normal-cosine-weighted iR smoothing in CoarseInitializer::optReg
	setting_useNormalOptReg = result["ml-normal-optreg"].as<bool>();
	printf("[PHASE_CONFIG] ml-normal-optreg=%s\n", setting_useNormalOptReg ? "on" : "off");

	// Sprint 6 — CD-H2: normal-direction gate on lvl-0 pixel selection
	setting_useNormalPixelGate = result["ml-normal-pixel-gate"].as<bool>();
	setting_normalPixelGateCos = result["ml-normal-pixel-gate-cos"].as<float>();
	printf("[PHASE_CONFIG] ml-normal-pixel-gate=%s cos_thresh=%.3f\n",
	       setting_useNormalPixelGate ? "on" : "off", setting_normalPixelGateCos);

	// Sprint 7 — C-NEW-2: normal-viewing-angle information weighting in indirect PoseOptimization
	setting_useNormalIndirectInfo = result["ml-normal-indirect-info"].as<bool>();
	setting_normalIndirectInfoFloor = result["ml-normal-indirect-info-floor"].as<float>();
	printf("[PHASE_CONFIG] ml-normal-indirect-info=%s floor=%.3f\n",
	       setting_useNormalIndirectInfo ? "on" : "off", setting_normalIndirectInfoFloor);

	// Sprint 8 — CD-H6 gauge probe
	setting_gravityAlignTestRotDeg = result["ml-gravity-test-rot"].as<float>();
	if(setting_gravityAlignTestRotDeg != 0.0f)
		printf("[PHASE_CONFIG] ml-gravity-test-rot=%.1f deg\n", setting_gravityAlignTestRotDeg);

	// Sprint 10 — E3 / Phase B.4: depth-normal surface-consistency residual in BA
	setting_useDepthNormalBA = result["ml-dn-ba"].as<bool>();
	setting_dnWeight = result["ml-dn-weight"].as<float>();
	setting_dnNeighborRadius = result["ml-dn-radius"].as<float>();
	setting_dnMinNeighbors = result["ml-dn-min-neighbors"].as<int>();
	printf("[PHASE_CONFIG] ml-dn-ba=%s weight=%.4f radius=%.1f min_nb=%d\n",
	       setting_useDepthNormalBA ? "on" : "off", setting_dnWeight,
	       setting_dnNeighborRadius, setting_dnMinNeighbors);

	// Depth source selection (Phase B — research-only validation switch)
	{
		std::string dsrc = result["depth-source"].as<std::string>();
		if (dsrc == "ml")        setting_depthSource = DEPTH_SOURCE_ML;
		else if (dsrc == "gt")   setting_depthSource = DEPTH_SOURCE_GT;
		else if (dsrc == "none") setting_depthSource = DEPTH_SOURCE_NONE;
		else {
			printf("ERROR: invalid --depth-source='%s' (expected ml|gt|none). Using 'ml'.\n", dsrc.c_str());
			setting_depthSource = DEPTH_SOURCE_ML;
		}
		const char* label = (setting_depthSource == DEPTH_SOURCE_ML) ? "ML (Metric3D, production)"
		                   : (setting_depthSource == DEPTH_SOURCE_GT) ? "GT (Kinect/LiDAR from associations.txt)"
		                                                               : "NONE (pure monocular)";
		printf("[DEPTH_SOURCE] %s\n", label);
		if (setting_depthSource == DEPTH_SOURCE_GT && associations.empty()) {
			printf("ERROR: --depth-source=gt requires --associations=<path/to/associations.txt>\n");
			return 0;
		}
	}

	// Phase toggles (Phase C matrix). Each flag overrides the settings.cpp default.
	// Convention: p0/p1/p2/p3/vs = true means enable; settings polarity is flipped per flag.
	setting_useMLForInitialization = result["p0"].as<bool>();
	setting_enableDirectP1Bounds   = result["p1"].as<bool>();
	setting_disableDirectP2BA      = !result["p2"].as<bool>();
	setting_disableDirectP3Tracker = !result["p3"].as<bool>();
	setting_disableDirectVS        = !result["vs"].as<bool>();
	{
		float vsw = result["vs-weight"].as<float>();
		if (vsw >= 0.0f) setting_vsWeight = vsw;  // negative sentinel = keep settings default
	}
	printf("[PHASE_CONFIG] P0=%s P1=%s P2=%s P3=%s VS=%s vsWeight=%.1e\n",
	       setting_useMLForInitialization ? "on":"off",
	       setting_enableDirectP1Bounds   ? "on":"off",
	       !setting_disableDirectP2BA     ? "on":"off",
	       !setting_disableDirectP3Tracker? "on":"off",
	       !setting_disableDirectVS       ? "on":"off",
	       setting_vsWeight);

	if(source.empty() || calib.empty()) { std::cout<< "Path to images or calibration not provided! cannot function without them. exit." << std::endl; return(0);}

	if (debugSaveImages)
	{
		if(42==system("rm -rf images_out")) std::cout<<"system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n"<<std::endl;
		if(42==system("mkdir images_out")) std::cout<<"system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n"<<std::endl;
		if(42==system("rm -rf images_out")) std::cout<<"system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n"<<std::endl;
		if(42==system("mkdir images_out")) std::cout<<"system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n"<<std::endl;
	}

	switch (mode)
	{
	case 1:
		setting_photometricCalibration = 0;
		setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
		setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
		break;
	case 2:
		setting_photometricCalibration = 0;
		setting_affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
		setting_affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
		setting_minGradHistAdd = 3;
		break;
	default:
		break;
	}
	
	DBoW3::Vocabulary* Vocabpnt;
	if(LoopClosure && !vocabPath.empty())
	{
		Vocabpnt = new DBoW3::Vocabulary();
		Vocabpnt->load(vocabPath.c_str());

		printf("Loading Vocabulary from %s!\n", vocabPath.c_str());
		if (Vocabpnt->empty() || Vocabpnt == NULL)
		{
			printf("Failed to load vocabulary! Exit\n");
			exit(1);
		}
	}
	else
	{
		std::cout << "no vocabulary path provided! disabling loop closure." << std::endl;
		LoopClosure = false; 
	}

	if(preset == 0 || preset == 1)
	{
		printf("OPTIMIZED settings:\n"
				"- %s real-time enforcing\n"
				"- %.0f active points (PHASE1: ML-enhanced SLAM optimization)\n"
				"- 5-7 active frames\n"
				"- 1-6 LM iteration each KF\n"
				"- original image resolution\n", 
				preset==0 ? "no " : "1x", setting_desiredPointDensity);

		playbackSpeed = (preset==0 ? 0 : 1);
		// preload = preset==1;
		
		// CRITICAL: Actually set the optimized values (Phase 1 optimization)
		// Balanced: maintain trajectory quality while improving performance
		setting_desiredImmatureDensity = 1500;  // 33% reduction from 1500
		setting_desiredPointDensity = 2000;     // 25% reduction from 2000
	}
	else if(preset == 2 || preset == 3)
	{
		printf("FAST settings:\n"
				"- %s real-time enforcing\n"
				"- 800 active points\n"
				"- 4-6 active frames\n"
				"- 1-4 LM iteration each KF\n"
				"- 424 x 320 image resolution\n", preset==0 ? "no " : "5x");

		playbackSpeed = (preset==2 ? 0 : 5);
		preload = preset==3;
		setting_desiredImmatureDensity = 600;
		setting_desiredPointDensity = 800;
		setting_minFrames = 4;
		setting_maxFrames = 6;
		setting_maxOptIterations=4;
		setting_minOptIterations=1;

		benchmarkSetting_width = 424;
		benchmarkSetting_height = 320;

		setting_logStuff = false;
	}
	

	ImageFolderReader* reader = new ImageFolderReader(source, calib, gammaCalib, vignette, use_16bit, use_colour);
	reader->setGlobalCalibration();
	set_frame_sz(reader->get_undist_width(), reader->get_undist_height());

	if(setting_photometricCalibration > 0 && reader->getPhotometricGamma() == 0)
	{
		printf("ERROR: dont't have photometric calibation. Need to use commandline options mode=1 or mode=2 ");
		exit(1);
	}


	int lstart=startIndex;
	int lend = endIndex;
	int linc = 1;
	if(reverse)
	{
		printf("REVERSE!!!!");
		lstart=endIndex-1;
		if(lstart >= reader->getNumImages())
			lstart = reader->getNumImages()-1;
			
		lend = startIndex;
		linc = -1;
	}



	FullSystem* fullSystem = new FullSystem();
	fullSystem->setGammaFunction(reader->getPhotometricGamma());
	fullSystem->linearizeOperation = (playbackSpeed == 0);
	
	// Apply ML initialization setting from command line
	setting_useMLForInitialization = ml_init_enabled;

	if(LoopClosure)
	{
		fullSystem->setVocab(Vocabpnt);
		printf("Vocabulary Set\n");
	}

	// Initialize ML depth system (Phase 2) if enabled
	if(ml_depth_enabled) {
		printf("Initializing ML depth system (Phase 2)...\n");
		printf("  Model: %s\n", ml_model_path.c_str());
		printf("  Strategy: %s\n", ml_strategy.c_str());
		if(ml_strategy == "snapshot_mode") {
			printf("  Snapshot rate: Every %d keyframes\n", ml_snapshot_interval);
			printf("  Expected ML reduction: %.1f%%\n", 
			       100.0f * (ml_snapshot_interval - 1) / ml_snapshot_interval);
		}
		
		// Create ML configuration using FullSystem's MLConfig
		FullSystem::MLConfig ml_config;
		ml_config.model_path = ml_model_path;
		ml_config.enable_gpu = ml_gpu_enabled;
		ml_config.num_threads = 4;
		
		// GPU-specific configuration
		ml_config.enable_fp16 = ml_fp16_enabled;
		ml_config.gpu_device_id = ml_gpu_device;
		ml_config.gpu_memory_limit_mb = ml_gpu_memory_mb;
		
		ml_config.input_width = ml_input_width;
		ml_config.input_height = ml_input_height;
		ml_config.model_type = ml_model_type_str;
		ml_config.output_scale = ml_output_scale;
		ml_config.benchmark_enabled = ml_benchmark_enabled;
		
		// Ablation study configuration
		ml_config.inference_strategy = ml_strategy;
		ml_config.snapshot_rate = ml_snapshot_interval;
		
		if(ml_gpu_enabled) {
			printf("  GPU Acceleration: Enabled (Device %d, Memory: %zu MB, FP16: %s)\n",
			       ml_gpu_device, ml_gpu_memory_mb, ml_fp16_enabled ? "Yes" : "No");
		} else {
			printf("  GPU Acceleration: Disabled (CPU mode)\n");
		}
		
		// Initialize MLDepthProcessor (synchronous processing)
		if(!fullSystem->initializeMLDepthProcessor(ml_config)) {
			printf("ERROR: Failed to initialize ML depth processor\n");
			return -1;
		}
		printf("ML Depth Processor initialized successfully\n");
		
		// Perform ML warmup with real frame for effective GPU optimization.
		// Use the requested startIndex frame (not hardcoded frame 0) so --startindex N
		// genuinely skips degenerate-motion preludes; warmup-stored mean depth feeds
		// Phase 0 metric scale anchor.
		printf("GPU warmup: starting with frame index %d (one-time GPU initialization)...\n", startIndex);
		auto warmup_start = std::chrono::high_resolution_clock::now();

		try {
			// Obtain a real colour warmup frame. For --associations runs the ImageFolderReader
			// globbed the dataset directory (files[0] may be associations.txt, NOT an image, and
			// reading a text file as colour segfaults) — so source the warmup frame from the first
			// association RGB entry, matching how the RGB-D loop below loads frames (cv::imread).
			cv::Mat first_rgb;
			ImageAndExposure* first_img = nullptr;
			if (!associations.empty()) {
				typedef tum_benchmark::FileReader<RGBDAssociation> WarmupReader;
				WarmupReader warmup_assoc(associations);
				auto wit = warmup_assoc.begin();
				for (int k = 0; k < startIndex && wit != warmup_assoc.end(); ++k) ++wit;
				if (wit != warmup_assoc.end())
					first_rgb = cv::imread(source + "/" + (*wit).rgb_file, cv::IMREAD_COLOR);
			} else {
				// --files path (TUM/KITTI): use the reader's undistorted colour frame
				first_img = reader->getImage(startIndex);
				if (first_img && first_img->useColour && first_img->r_image != nullptr)
					first_rgb = convertUndistortedToRGB(first_img);
			}

			if (!first_rgb.empty()) {
				printf("GPU warmup: processing first real frame (expected ~6000ms)...\n");
				// Process first real frame - this triggers full GPU kernel compilation
				bool warmup_success = fullSystem->performMLWarmup(first_rgb);

				auto warmup_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
					std::chrono::high_resolution_clock::now() - warmup_start).count();

				if (warmup_success) {
					printf("GPU warmup complete in %ldms - subsequent ML inferences ~40ms\n", warmup_duration);
				} else {
					printf("ERROR: Real frame warmup failed - disabling ML depth\n");
					fullSystem->ml_depth_enabled_ = false;
					ml_depth_enabled = false;
				}
			} else {
				printf("ERROR: Could not load first frame for warmup - disabling ML depth\n");
				fullSystem->ml_depth_enabled_ = false;
				ml_depth_enabled = false;
			}
			if (first_img) delete first_img; // Clean up (--files path only)
		} catch (const std::exception& e) {
			printf("ERROR: Exception during ML warmup: %s - disabling ML depth\n", e.what());
			fullSystem->ml_depth_enabled_ = false;
			ml_depth_enabled = false;
		}
		
		// ML processor is ready immediately (synchronous initialization)
		
		// Extract dataset name from source path for results directory
		std::string dataset_name = "unknown";
		size_t last_slash = source.find_last_of('/');
		if (last_slash != std::string::npos) {
			std::string path_part = source.substr(last_slash + 1);
			if (path_part.find("rgbd_dataset_") == 0) {
				dataset_name = path_part.substr(13); // Remove "rgbd_dataset_" prefix
			} else {
				dataset_name = path_part; // Use the directory name as dataset name
			}
		}

		// Benchmark is now integrated into the service automatically
		if (ml_benchmark_enabled) {
			printf("ML performance benchmarking enabled - metrics will be collected during operation\n");
		}

		// DEPTH_DEBUG: Initialize DepthLogger for ML depth debugging (TEMPORARY - for debugging only)
		#ifdef ENABLE_DEPTH_DEBUG
		printf("DEPTH_DEBUG: Initializing DepthLogger for ML depth debugging...\n");
		HSLAM::DepthLogger::initialize("results", dataset_name);
		HSLAM::DepthLogger::setDebugMode(true); // Enable detailed debug logging
		printf("DEPTH_DEBUG: DepthLogger initialized for dataset: %s\n", dataset_name.c_str());
		#endif
	} else {
		printf("ML depth system disabled\n");
		fullSystem->ml_depth_enabled_ = false;
	}

	IOWrap::PangolinDSOViewer* viewer = 0;
	if(!disableAllDisplay)
    {
        // Check for headless environment (no DISPLAY variable)
        const char* display_env = std::getenv("DISPLAY");
        if (display_env == nullptr || strlen(display_env) == 0) {
            printf("WARNING: No DISPLAY environment variable found. Running in headless mode (GUI disabled).\n");
            disableAllDisplay = true;
        } else {
            // Try to initialize GUI with error handling
            try {
                viewer = new IOWrap::PangolinDSOViewer(wG[0],hG[0], false);  // Disable GUI thread (fix GLX error)
                fullSystem->outputWrapper.push_back(viewer);
                printf("GUI viewer initialized successfully\n");
            } catch (const std::exception& e) {
                printf("WARNING: Failed to initialize GUI viewer: %s\n", e.what());
                printf("Continuing in headless mode...\n");
                disableAllDisplay = true;
                viewer = nullptr;
            }
        }
    }



    if(useSampleOutput)
        fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());




	// RGB-D pipeline: Use associations file if provided, otherwise fallback to monocular
    std::thread runthread([&]() {
        struct timeval tv_start;
        gettimeofday(&tv_start, NULL);
        clock_t started = clock();
        double sInitializerOffset=0;
        int processedFrames = 0;  // Declare at function scope
        std::vector<int> idsToPlay;  // Declare at function scope for statistics

        // Initialize FPSLogger for performance monitoring
        std::string dataset_name = "unknown";
        // Extract dataset name from source path
        size_t last_slash = source.find_last_of('/');
        if (last_slash != std::string::npos) {
            std::string path_part = source.substr(last_slash + 1);
            if (path_part.find("rgbd_dataset_") == 0) {
                dataset_name = path_part.substr(13); // Remove "rgbd_dataset_" prefix
            } else {
                dataset_name = path_part; // Use the directory name as dataset name
            }
        }
        if(setting_logStuff) {
            HSLAM::FPSLogger::initialize("results", dataset_name);
            HSLAM::FPSLogger::setDebugMode(false);
        }
        printf("FPSLogger: Initialized for dataset: %s\n", dataset_name.c_str());

        if (!associations.empty()) {
            // RGB-D pipeline using associations.txt
            printf("Using RGB-D pipeline with associations file: %s\n", associations.c_str());
            
            // Initialize DepthLogger for RGB-D pipeline
            HSLAM::DepthLogger::initialize("results", dataset_name);
            HSLAM::DepthLogger::setDebugMode(false); // Disable debug mode by default
            printf("DepthLogger: Initialized for dataset: %s\n", dataset_name.c_str());
            
            // Load associations using tum_benchmark
            typedef tum_benchmark::FileReader<RGBDAssociation> FileReader;
            FileReader reader_assoc(associations);
            
            int frameCount = 0;
            
            for (auto it = reader_assoc.begin(); it != reader_assoc.end(); ++it, ++frameCount) {
                if (frameCount < startIndex) continue;
                if (frameCount >= endIndex) break;
                
                while (Pause) {
                    usleep(5000);
                }
                
                if(!fullSystem->initialized) {
                    gettimeofday(&tv_start, NULL);
                    started = clock();
                    sInitializerOffset = 0;
                }
                
                // Parse association entry
                auto& entry = *it;
                std::string rgb_path = source + "/" + entry.rgb_file;
                std::string depth_path = source + "/" + entry.depth_file;
                
                // Extract timestamp from filename if available, fallback to associations.txt
                double timestamp;
                
                // Try to extract timestamp from RGB filename (TUM RGB-D format: path/timestamp.png)
                std::string rgb_filename = entry.rgb_file;
                
                // Get just the filename part (remove directory path)
                size_t slash_pos = rgb_filename.find_last_of('/');
                if (slash_pos != std::string::npos) {
                    rgb_filename = rgb_filename.substr(slash_pos + 1);
                }
                
                // Extract timestamp from filename
                size_t dot_pos = rgb_filename.find_last_of('.');
                if (dot_pos != std::string::npos) {
                    std::string timestamp_str = rgb_filename.substr(0, dot_pos);
                    try {
                        timestamp = std::stod(timestamp_str);
                        if (!setting_debugout_runquiet) {
                            printf("Extracted timestamp from filename: %.6f\n", timestamp);
                        }
                    } catch (const std::exception& e) {
                        // Fallback to associations.txt timestamp
                        timestamp = entry.rgb_timestamp;
                        if (!setting_debugout_runquiet) {
                            printf("Using associations.txt timestamp: %.6f (filename extraction failed)\n", timestamp);
                        }
                    }
                } else {
                    // Fallback to associations.txt timestamp
                    timestamp = entry.rgb_timestamp;
                    if (!setting_debugout_runquiet) {
                        printf("Using associations.txt timestamp: %.6f (no extension found)\n", timestamp);
                    }
                }
                
                // Load RGB image as color for ML depth processing
                cv::Mat rgb_img_color = cv::imread(rgb_path, cv::IMREAD_COLOR);
                if (rgb_img_color.empty()) {
                    printf("ERROR: Could not load RGB image: %s\n", rgb_path.c_str());
                    continue;
                }
                
                // Create grayscale version for SLAM processing
                cv::Mat rgb_img_raw;
                cv::cvtColor(rgb_img_color, rgb_img_raw, cv::COLOR_BGR2GRAY);
                
                // Convert grayscale to float for SLAM
                cv::Mat rgb_img;
                rgb_img_raw.convertTo(rgb_img, CV_32FC1);
                
                // Convert RGB to proper format for ML
                cv::Mat rgb_for_ml;
                cv::cvtColor(rgb_img_color, rgb_for_ml, cv::COLOR_BGR2RGB);
                
                // Load depth image (16-bit PNG -> CV_32FC1, scaled by 1/5000.0)
                cv::Mat depth_img_raw = cv::imread(depth_path, cv::IMREAD_UNCHANGED);
                if (depth_img_raw.empty()) {
                    printf("ERROR: Could not load depth image: %s\n", depth_path.c_str());
                    continue;
                }
                
                cv::Mat depth_img;
                // Depth-scale divisor: TUM uses 5000, KITTI velodyne-projected we write at ~500.
                // Configurable via --depth-scale CLI flag.
                depth_img_raw.convertTo(depth_img, CV_32FC1, 1.0/depth_scale);
                
                // Debug-only logging for frame processing details
                HSLAM::DepthLogger::logFrameProcessing(processedFrames, rgb_img.cols, rgb_img.rows,
                                                     depth_img.cols, depth_img.rows, timestamp);
                
                // Call TrackRGBD with RGB color for ML and grayscale for SLAM
                fullSystem->TrackRGBD(rgb_for_ml, rgb_img, depth_img, timestamp);
                
                processedFrames++;
                
                if(viewer!=0 && viewer->isDead) break;
                
                if(fullSystem->initFailed || setting_fullResetRequested) {
                    if(processedFrames < 250 || setting_fullResetRequested) {
                        printf("RESETTING!\n");
                        std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
                        for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
                        usleep(20000);
                        if(fullSystem) {
                            delete fullSystem;
                            fullSystem = nullptr;
                        }
                        
                        fullSystem = new FullSystem();
                        fullSystem->setGammaFunction(reader->getPhotometricGamma());
                        fullSystem->linearizeOperation = (playbackSpeed == 0);
                        
                        if(LoopClosure) {
                            fullSystem->setVocab(Vocabpnt);
                            printf("Vocabulary Set\n");
                        }
                        
                        // CRITICAL FIX: Reinitialize ML depth service after reset
                        if(ml_depth_enabled) {
                            printf("Reinitializing ML depth service after SLAM reset...\n");
                            
                            // Recreate ML config (same as original initialization)
                            FullSystem::MLConfig ml_config;
                            ml_config.model_path = ml_model_path;
                            ml_config.enable_gpu = ml_gpu_enabled;
                            ml_config.num_threads = 4;
                            ml_config.enable_fp16 = ml_fp16_enabled;
                            ml_config.gpu_device_id = ml_gpu_device;
                            ml_config.gpu_memory_limit_mb = ml_gpu_memory_mb;
                            ml_config.input_width = ml_input_width;
                            ml_config.input_height = ml_input_height;
                            ml_config.model_type = ml_model_type_str;
                            ml_config.output_scale = ml_output_scale;
                            ml_config.benchmark_enabled = ml_benchmark_enabled;
                            
                            // Ablation study configuration
                            ml_config.inference_strategy = ml_strategy;
                            ml_config.snapshot_rate = ml_snapshot_interval;
                            
                            // Reinitialize MLDepthProcessor
                            if(!fullSystem->initializeMLDepthProcessor(ml_config)) {
                                printf("WARNING: Failed to reinitialize ML depth processor after reset\n");
                            } else {
                                printf("ML depth processor reinitialized successfully\n");
                            }
                        }
                        
                        fullSystem->outputWrapper = wraps;
                        setting_fullResetRequested=false;
                    }
                }
                
                if(fullSystem->isLost) {
                    printf("LOST!!\n");
                    break;
                }
            }
        } else {
            // Fallback to original monocular pipeline
            printf("Using monocular pipeline (no associations file provided)\n");
            
            std::vector<double> timesToPlayAt;
            for(int i=lstart;i>= 0 && i< reader->getNumImages() && linc*i < linc*lend;i+=linc)
            {
                idsToPlay.push_back(i);
                if(timesToPlayAt.size() == 0)
                {
                    timesToPlayAt.push_back((double)0);
                }
                else
                {
                    double tsThis = reader->getTimestamp(idsToPlay[idsToPlay.size()-1]);
                    double tsPrev = reader->getTimestamp(idsToPlay[idsToPlay.size()-2]);
                    // playbackSpeed==0 means "no real-time pacing" (linearize / eval mode). Dividing
                    // by it produced inf schedule entries, which line ~880 then copied into
                    // sInitializerOffset, poisoning MilliSecondsTakenMT and every pipeline-level fps
                    // number downstream (console printed "infms per frame", FPSLogger fell back to a
                    // bogus 0.0 fps). The schedule is unused when playbackSpeed==0, so keep it at 0.
                    timesToPlayAt.push_back(playbackSpeed != 0
                        ? timesToPlayAt.back() + fabs(tsThis-tsPrev)/playbackSpeed
                        : 0.0);
                }
            }

            std::vector<ImageAndExposure*> preloadedImages;
            if(preload)
            {
                printf("LOADING ALL IMAGES!\n");
                for(int ii=0;ii<(int)idsToPlay.size(); ii++)
                {
                    int i = idsToPlay[ii];
                    preloadedImages.push_back(reader->getImage(i));
                }
            }

            for(int ii=0;ii<(int)idsToPlay.size(); ii++)
            {
                while (Pause)
                {
                    usleep(5000);
                }
                    
                if(!fullSystem->initialized)
                {
                    gettimeofday(&tv_start, NULL);
                    started = clock();
                    sInitializerOffset = std::isfinite(timesToPlayAt[ii]) ? timesToPlayAt[ii] : 0.0;
                }

                int i = idsToPlay[ii];

                ImageAndExposure* img;
                if(preload)
                    img = preloadedImages[ii];
                else
                    img = reader->getImage(i);

                bool skipFrame=false;
                if(playbackSpeed!=0)
                {
                    struct timeval tv_now; gettimeofday(&tv_now, NULL);
                    double sSinceStart = sInitializerOffset + ((tv_now.tv_sec-tv_start.tv_sec) + (tv_now.tv_usec-tv_start.tv_usec)/(1000.0f*1000.0f));

                    if(sSinceStart < timesToPlayAt[ii])
                        usleep((int)((timesToPlayAt[ii]-sSinceStart)*1000*1000));
                    else if(sSinceStart > timesToPlayAt[ii]+0.5+0.1*(ii%2))
                    {
                        printf("SKIPFRAME %d (play at %f, now it is %f)!\n", ii, timesToPlayAt[ii], sSinceStart);
                        skipFrame=true;
                    }
                }

                if (!skipFrame) {
                    if(ml_depth_enabled && fullSystem->ml_depth_enabled_) {
                        // Use undistorted RGB if available, otherwise load original
                        cv::Mat rgb_color;
                        
                        // Use DataReader's undistorted RGB channels for all datasets
                        if (img->useColour && img->r_image != nullptr && img->g_image != nullptr && img->b_image != nullptr) {
                            // Universal undistorted RGB processing - trust DataReader architecture
                            rgb_color = convertUndistortedToRGB(img);
                        } else {
                            // Handle case where undistorted color channels are not available
                            printf("Warning: No undistorted color channels available for frame %d\n", i);
                        }
                        
                        // Use ML-enhanced tracking with original ImageAndExposure to preserve both image buffers
                        fullSystem->TrackMonocularWithML(rgb_color, img);
                    } else {
                        // Use standard monocular tracking
                        fullSystem->addActiveFrame(img, i);
                    }
                }

                delete img;
        
                if(viewer!=0)
                    if(viewer->isDead)
                        break;

                if(fullSystem->initFailed || setting_fullResetRequested)
                {
                    if(ii < 250 || setting_fullResetRequested)
                    {
                        printf("RESETTING!\n");
                        std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
                        for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
                        usleep(20000);
                        if(fullSystem)
                        {
                            delete fullSystem;
                            fullSystem = nullptr;
                        }
                            
                        fullSystem = new FullSystem();
                        fullSystem->setGammaFunction(reader->getPhotometricGamma());
                        fullSystem->linearizeOperation = (playbackSpeed == 0);

                        if(LoopClosure)
                        {
                            fullSystem->setVocab(Vocabpnt);
                            printf("Vocabulary Set\n");
                        }

                        // CRITICAL FIX: Reinitialize ML depth service after reset (second path)
                        if(ml_depth_enabled) {
                            printf("Reinitializing ML depth service after SLAM reset...\n");
                            
                            // Recreate ML config (same as original initialization)
                            FullSystem::MLConfig ml_config;
                            ml_config.model_path = ml_model_path;
                            ml_config.enable_gpu = ml_gpu_enabled;
                            ml_config.num_threads = 4;
                            ml_config.enable_fp16 = ml_fp16_enabled;
                            ml_config.gpu_device_id = ml_gpu_device;
                            ml_config.gpu_memory_limit_mb = ml_gpu_memory_mb;
                            ml_config.input_width = ml_input_width;
                            ml_config.input_height = ml_input_height;
                            ml_config.model_type = ml_model_type_str;
                            ml_config.output_scale = ml_output_scale;
                            ml_config.benchmark_enabled = ml_benchmark_enabled;
                            
                            // Ablation study configuration
                            ml_config.inference_strategy = ml_strategy;
                            ml_config.snapshot_rate = ml_snapshot_interval;
                            
                            // Reinitialize MLDepthProcessor
                            if(!fullSystem->initializeMLDepthProcessor(ml_config)) {
                                printf("WARNING: Failed to reinitialize ML depth processor after reset\n");
                            } else {
                                printf("ML depth processor reinitialized successfully\n");
                            }
                        }

                        fullSystem->outputWrapper = wraps;
                        setting_fullResetRequested=false;
                    }
                }

                if(fullSystem->isLost)
                {
                    printf("LOST!!\n");
                    break;
                }
            }
            processedFrames = idsToPlay.size();  // Set for statistics
        }
		// fullSystem->BAatExit();
		fullSystem->blockUntilMappingIsFinished();
		clock_t ended = clock();
        struct timeval tv_end;
        gettimeofday(&tv_end, NULL);

        // End-to-end pipeline wall-clock: first tracked frame -> after the mapping/BA backlog has
        // drained (blockUntilMappingIsFinished above). Unlike track_fps (tracking thread only) this
        // includes mapping, BA, ML inference and loop closure — i.e. the true cost of the pipeline.
        // Deliberately excludes sInitializerOffset, which is a playback-schedule offset, not work.
        double pipelineWallMs = (tv_end.tv_sec  - tv_start.tv_sec) * 1000.0
                              + (tv_end.tv_usec - tv_start.tv_usec) / 1000.0;
        int pipelineFrames = abs(idsToPlay[0] - idsToPlay.back()) + 1;

        fullSystem->printResult("result.txt");
		if (outputPC) fullSystem->printPC("PC.PCD");

		// [MAP_EXPORT] Sprint 0d: ASCII PLY export of marginalized PointHessians
		if (setting_exportMapPly) {
			// Determine output path: use --map-ply-out if set, else result.ply in CWD
			std::string ply_path = setting_mapPlyOut;
			if (ply_path.empty()) ply_path = "result.ply";
			fullSystem->printMapPly(ply_path);
		}

        // Log keyframe statistics
        fullSystem->printKeyframeStats();

        // [PERF_SUMMARY] — machine-parseable performance line for eval summary
        {
            double avg_ml_ms = (ml_depth_enabled && fullSystem->ml_depth_enabled_)
                             ? fullSystem->ml_metrics_.avg_ml_inference_time_ms : -1.0;
            fullSystem->printPerfSummary(avg_ml_ms, pipelineWallMs, pipelineFrames);
        }

        double MilliSecondsTakenSingle = 1000.0f*(ended-started)/(float)(CLOCKS_PER_SEC);
        double MilliSecondsTakenMT = pipelineWallMs;
        
        // Log performance statistics using FPSLogger
        if (!associations.empty()) {
            // RGB-D pipeline statistics
            double avg_ms_per_frame = MilliSecondsTakenMT / (float)processedFrames;
            double fps = 1000.0 / avg_ms_per_frame;
            
            // Log SLAM performance
            HSLAM::FPSLogger::logSlamPerformance(processedFrames, avg_ms_per_frame, fps, "RGB-D");
            
            // Log RGB-D specific statistics using DepthLogger
            HSLAM::DepthLogger::logPerformance(processedFrames, avg_ms_per_frame, fps);
            HSLAM::DepthLogger::finalize();
        } else {
            // Monocular pipeline statistics
            int numFramesProcessed = abs(idsToPlay[0]-idsToPlay.back());
            double numSecondsProcessed = fabs(reader->getTimestamp(idsToPlay[0])-reader->getTimestamp(idsToPlay.back()));
            double avg_ms_per_frame = MilliSecondsTakenMT / (float)numFramesProcessed;
            double fps = 1000.0 / avg_ms_per_frame;
            
            // Determine pipeline type for logging
            std::string pipeline_type = (ml_depth_enabled && fullSystem->ml_depth_enabled_) ? "ML_Enhanced" : "Monocular";
            
            // Log SLAM performance with total keyframes created (not just active sliding window)
            int keyframes = static_cast<int>(fullSystem->allKeyFramesHistory.size());
            HSLAM::FPSLogger::logSlamPerformance(numFramesProcessed, keyframes, avg_ms_per_frame, fps, pipeline_type);
            
            // Log ML performance if enabled
            if (ml_depth_enabled && fullSystem->ml_depth_enabled_) {
                auto ml_metrics = fullSystem->ml_metrics_;
                HSLAM::FPSLogger::logMLPerformance(
                    ml_metrics.ml_keyframes_attempted,
                    ml_metrics.ml_keyframes_successful,
                    ml_metrics.avg_ml_inference_time_ms,
                    ml_metrics.ml_depth_utilization
                );
            }
            
            // Console output for compatibility.
            // NOTE: the first figure is the DATASET capture rate (frames / timestamp span) — a
            // property of the sequence, constant regardless of compute. It is NOT throughput and
            // must never be quoted as achieved fps; use pipeline_fps in [PERF_SUMMARY] for that.
            printf("\n======================"
                    "\n%d Frames (%.1f fps dataset capture rate, NOT throughput)"
                    "\n%.2fms per frame (single core); "
                    "\n%.2fms per frame (multi core); "
                    "\n%.3fx (single core); "
                    "\n%.3fx (multi core); "
                    "\n======================\n\n",
                    numFramesProcessed, numFramesProcessed/numSecondsProcessed,
                    MilliSecondsTakenSingle/numFramesProcessed,
                    MilliSecondsTakenMT / (float)numFramesProcessed,
                    1000 / (MilliSecondsTakenSingle/numSecondsProcessed),
                    1000 / (MilliSecondsTakenMT / numSecondsProcessed));
        }
        
        // Finalize FPSLogger
        if(setting_logStuff) HSLAM::FPSLogger::finalize();
        
        //fullSystem->printFrameLifetimes();
        // time.txt: two-number timing summary, fully redundant with fpsLog.txt and run_log
        // if(setting_logStuff)
        // {
        //     std::ofstream tmlog;
        //     tmlog.open("logs/time.txt", std::ios::trunc | std::ios::out);
        //     if (!associations.empty()) {
        //         tmlog << MilliSecondsTakenSingle/processedFrames << " " << MilliSecondsTakenMT / (float)processedFrames << "\n";
        //     } else {
        //         tmlog << 1000.0f*(ended-started)/(float)(CLOCKS_PER_SEC*reader->getNumImages()) << " "
        //               << ((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f) / (float)reader->getNumImages() << "\n";
        //     }
        //     tmlog.flush();
        //     tmlog.close();
        // }

		if(!pauseEnd){
		for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
		{
			ow->join();
		}}
    });


	if(viewer != 0)
	    viewer->run();

	runthread.join();

	for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
	{
		if(pauseEnd) ow->join();
		delete ow;
	}



	printf("DELETE FULLSYSTEM!\n");
	if (fullSystem)
	{
		delete fullSystem;
		fullSystem = nullptr;
	}

	if(LoopClosure && !vocabPath.empty())
	{
		delete Vocabpnt;
	}

	printf("DELETE READER!\n");
	delete reader;

	printf("EXIT NOW!\n");
	return 0;
}
