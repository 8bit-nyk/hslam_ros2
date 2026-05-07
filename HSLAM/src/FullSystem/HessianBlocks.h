/**
* This file is part of DSO, written by Jakob Engel.
* It has been modified by Georges Younes, Daniel Asmar, John Zelek, and Yan Song Hu
*
* Copyright 2024 University of Waterloo and American University of Beirut.
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/



#pragma once
#define MAX_ACTIVE_FRAMES 100

#include "util/globalCalib.h"
#include "vector"
 
#include <iostream>
#include <fstream>
#include <atomic>
#include <memory>
#include <mutex>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <opencv2/opencv.hpp>
#include "util/NumType.h"
#include "FullSystem/Residuals.h"
#include "util/ImageAndExposure.h"



namespace HSLAM
{

class MapPoint;

inline Vec2 affFromTo(const Vec2 &from, const Vec2 &to)	// contains affine parameters as XtoWorld.
{
	return Vec2(from[0] / to[0], (from[1] - to[1]) / to[0]);
}


struct FrameHessian;
struct PointHessian;

class ImmaturePoint;
class FrameShell;

class EFFrame;
class EFPoint;

#define SCALE_IDEPTH 1.0f		// scales internal value to idepth.
#define SCALE_XI_ROT 1.0f
#define SCALE_XI_TRANS 1.0f
#define SCALE_F 50.0f
#define SCALE_C 50.0f
#define SCALE_W 1.0f
#define SCALE_A 10.0f
#define SCALE_B 1000.0f

#define SCALE_IDEPTH_INVERSE (1.0f / SCALE_IDEPTH)
#define SCALE_XI_ROT_INVERSE (1.0f / SCALE_XI_ROT)
#define SCALE_XI_TRANS_INVERSE (1.0f / SCALE_XI_TRANS)
#define SCALE_F_INVERSE (1.0f / SCALE_F)
#define SCALE_C_INVERSE (1.0f / SCALE_C)
#define SCALE_W_INVERSE (1.0f / SCALE_W)
#define SCALE_A_INVERSE (1.0f / SCALE_A)
#define SCALE_B_INVERSE (1.0f / SCALE_B)


struct FrameFramePrecalc
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	// Static values
	static int instanceCounter;
	FrameHessian* host;	// defines row
	FrameHessian* target;	// defines column

	// Precalc values
	Mat33f PRE_RTll;
	Mat33f PRE_KRKiTll;
	Mat33f PRE_RKiTll;
	Mat33f PRE_RTll_0;

	Vec2f PRE_aff_mode;
	float PRE_b0_mode;

	Vec3f PRE_tTll;
	Vec3f PRE_KtTll;
	Vec3f PRE_tTll_0;

	float distanceLL;


    inline ~FrameFramePrecalc() {}
    inline FrameFramePrecalc() : host(0), target(0), PRE_b0_mode(0.0), distanceLL(0.0) {}
	void set(FrameHessian* host, FrameHessian* target, CalibHessian* HCalib);
};





struct FrameHessian
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EFFrame* efFrame;

	// Constant info & pre-calculated values
	//DepthImageWrap* frame;
	FrameShell* shell;

	Eigen::Vector3f* dI;				 // trace, fine tracking. Used for direction select (not for gradient histograms etc.)
	
	Eigen::Vector3f* dI_c;
	bool colourValid;
	
	Eigen::Vector3f* dIp[PYR_LEVELS];	 // coarse tracking / coarse initializer. NAN in [0] only.
	float* absSquaredGrad[PYR_LEVELS];  // only used for pixel select (histograms etc.). no NAN.






	int frameID;						// incremental ID for keyframes only!
	static int instanceCounter;
	int idx;

	// Photometric Calibration Stuff
	float frameEnergyTH;	// set dynamically depending on tracking residual
	float ab_exposure;

	bool flaggedForMarginalization;

	std::vector<PointHessian*> pointHessians;				// contains all ACTIVE points.
	std::vector<PointHessian*> pointHessiansMarginalized;	// contains all MARGINALIZED points (= fully marginalized, usually because point went OOB.)
	std::vector<PointHessian*> pointHessiansOut;		// contains all OUTLIER points (= discarded.).
	std::vector<ImmaturePoint*> immaturePoints;		// contains all OUTLIER points (= discarded.).

	// ML Depth Integration Fields (Phase 2.7 - Frame-Integrated Architecture)
	std::shared_ptr<cv::Mat> ml_depth_map_;			// ML-estimated depth map (shared ownership)
	std::shared_ptr<cv::Mat> ml_confidence_map_;	// ML confidence map (shared ownership)
	std::shared_ptr<cv::Mat> ml_normal_map_;		// Sprint 1: CV_32FC3 surface normals in camera frame
	std::atomic<bool> has_ml_depth_{false};			// Flag indicating ML depth availability
	std::atomic<bool> has_ml_normals_{false};		// Sprint 1: Flag indicating normal map availability
	std::atomic<bool> ml_pending_{false};			// ML request submitted but not complete
	int ml_request_frame_id_ = -1;					// Tracks ML request by absolute frame ID
	std::atomic<float> ml_confidence_{0.0f};			// ML depth confidence score (thread-safe)
	std::atomic<double> ml_inference_time_ms_{0.0};	// ML inference timing for metrics (thread-safe)
	mutable boost::mutex ml_mutex_;					// Mutex for shared_ptr protection

	Mat66 nullspaces_pose;
	Mat42 nullspaces_affine;
	Vec6 nullspaces_scale;

	// Variable info.
	SE3 worldToCam_evalPT;
	Vec10 state_zero;
	Vec10 state_scaled;
	Vec10 state;	// [0-5: worldToCam-leftEps. 6-7: a,b]
	Vec10 step;
	Vec10 step_backup;
	Vec10 state_backup;


    EIGEN_STRONG_INLINE const SE3 &get_worldToCam_evalPT() const {return worldToCam_evalPT;}
    // The first 6 parameters of state_zero seem to be always 0 (as this part is represented by the worldToCam_evalPT. The last two parameters on the other hand are not zero.
    EIGEN_STRONG_INLINE const Vec10 &get_state_zero() const {return state_zero;} 
    EIGEN_STRONG_INLINE const Vec10 &get_state() const {return state;}
    EIGEN_STRONG_INLINE const Vec10 &get_state_scaled() const {return state_scaled;}
    EIGEN_STRONG_INLINE const Vec10 get_state_minus_stateZero() const {return get_state() - get_state_zero();}


	// precalc values
	SE3 PRE_worldToCam;
	SE3 PRE_camToWorld;
	std::vector<FrameFramePrecalc,Eigen::aligned_allocator<FrameFramePrecalc>> targetPrecalc;
	MinimalImageB3* debugImage;


    inline Vec6 w2c_leftEps() const {return get_state_scaled().head<6>();}
    inline AffLight aff_g2l() const {return AffLight(get_state_scaled()[6], get_state_scaled()[7]);}
    inline AffLight aff_g2l_0() const {return AffLight(get_state_zero()[6]*SCALE_A, get_state_zero()[7]*SCALE_B);}



	void setStateZero(const Vec10 &state_zero);
	inline void setState(const Vec10 &state)
	{

		this->state = state;
		state_scaled.segment<3>(0) = SCALE_XI_TRANS * state.segment<3>(0);
		state_scaled.segment<3>(3) = SCALE_XI_ROT * state.segment<3>(3);
		state_scaled[6] = SCALE_A * state[6];
		state_scaled[7] = SCALE_B * state[7];
		state_scaled[8] = SCALE_A * state[8];
		state_scaled[9] = SCALE_B * state[9];

		PRE_worldToCam = SE3::exp(w2c_leftEps()) * get_worldToCam_evalPT();
		PRE_camToWorld = PRE_worldToCam.inverse();
		//setCurrentNullspace();
	};
	inline void setStateScaled(const Vec10 &state_scaled)
	{

		this->state_scaled = state_scaled;
		state.segment<3>(0) = SCALE_XI_TRANS_INVERSE * state_scaled.segment<3>(0);
		state.segment<3>(3) = SCALE_XI_ROT_INVERSE * state_scaled.segment<3>(3);
		state[6] = SCALE_A_INVERSE * state_scaled[6];
		state[7] = SCALE_B_INVERSE * state_scaled[7];
		state[8] = SCALE_A_INVERSE * state_scaled[8];
		state[9] = SCALE_B_INVERSE * state_scaled[9];

		PRE_worldToCam = SE3::exp(w2c_leftEps()) * get_worldToCam_evalPT();
		PRE_camToWorld = PRE_worldToCam.inverse();
		//setCurrentNullspace();
	};
	inline void setEvalPT(const SE3 &worldToCam_evalPT, const Vec10 &state)
	{

		this->worldToCam_evalPT = worldToCam_evalPT;
		setState(state);
		setStateZero(state);
	};



	inline void setEvalPT_scaled(const SE3 &worldToCam_evalPT, const AffLight &aff_g2l)
	{
		Vec10 initial_state = Vec10::Zero();
		initial_state[6] = aff_g2l.a;
		initial_state[7] = aff_g2l.b;
		this->worldToCam_evalPT = worldToCam_evalPT;
		setStateScaled(initial_state);
		setStateZero(this->get_state());
	};

	void release();

	inline ~FrameHessian()
	{
		assert(efFrame==0);
		release(); instanceCounter--;
		for (int i = 0; i < pyrLevelsUsed; i++)
		{
			if (dIp[i])
				delete[] dIp[i];
			if (absSquaredGrad[i])
				delete[] absSquaredGrad[i];
		}
		if(colourValid)
			delete[] dI_c;

		if(debugImage != 0) delete debugImage;
	};
	inline FrameHessian() :
		ab_exposure(0.0),
		idx(0),
		ml_depth_map_(nullptr),
		ml_confidence_map_(nullptr),
		ml_normal_map_(nullptr),
		has_ml_depth_(false),
		has_ml_normals_(false),
		ml_pending_(false)
	{
		instanceCounter++;
		flaggedForMarginalization=false;
		frameID = -1;
		efFrame = 0;
		frameEnergyTH = 8*8*PATTERNNUM;


		debugImage=0;

		colourValid = false;
	};


    void makeImages(float* color, CalibHessian* HCalib);
	void makeColourImages(float* r, float* g ,float* b);

	inline Vec10 getPrior()
	{
		Vec10 p =  Vec10::Zero();
		if(frameID==0)
		{
			p.head<3>() = Vec3::Constant(setting_initialTransPrior);
			p.segment<3>(3) = Vec3::Constant(setting_initialRotPrior);
			if(setting_solverMode & SOLVER_REMOVE_POSEPRIOR) p.head<6>().setZero();

			p[6] = setting_initialAffAPrior;
			p[7] = setting_initialAffBPrior;
		}
		else
		{
			if(setting_affineOptModeA < 0)
				p[6] = setting_initialAffAPrior;
			else
				p[6] = setting_affineOptModeA;

			if(setting_affineOptModeB < 0)
				p[7] = setting_initialAffBPrior;
			else
				p[7] = setting_affineOptModeB;
		}
		p[8] = setting_initialAffAPrior;
		p[9] = setting_initialAffBPrior;
		return p;
	}


	inline Vec10 getPriorZero()
	{
		return Vec10::Zero();
	}

	// ML Depth Thread-Safe Access Methods
	inline bool hasMLDepth() const { 
		return has_ml_depth_.load() && ml_depth_map_ != nullptr; 
	}

	inline std::shared_ptr<cv::Mat> getMLDepth() const {
		boost::unique_lock<boost::mutex> lock(ml_mutex_);
		return has_ml_depth_.load() ? ml_depth_map_ : nullptr;
	}
	
	inline std::shared_ptr<cv::Mat> getMLConfidenceMap() const {
		boost::unique_lock<boost::mutex> lock(ml_mutex_);
		return has_ml_depth_.load() ? ml_confidence_map_ : nullptr;
	}

	// Sprint 1: accessor for surface normal map (CV_32FC3, camera frame)
	inline std::shared_ptr<cv::Mat> getMLNormalMap() const {
		boost::unique_lock<boost::mutex> lock(ml_mutex_);
		return has_ml_normals_.load() ? ml_normal_map_ : nullptr;
	}

	inline void setMLDepth(const cv::Mat& depth, float confidence, double inference_time,
	                        const cv::Mat& confidence_map = cv::Mat(),
	                        const cv::Mat& normal_map = cv::Mat()) {
		if (!depth.empty()) {
			{
				boost::unique_lock<boost::mutex> lock(ml_mutex_);
				ml_depth_map_ = std::make_shared<cv::Mat>(depth.clone());
				if (!confidence_map.empty()) {
					ml_confidence_map_ = std::make_shared<cv::Mat>(confidence_map.clone());
				}
				// Sprint 1: store normal map if provided
				if (!normal_map.empty()) {
					ml_normal_map_ = std::make_shared<cv::Mat>(normal_map.clone());
					has_ml_normals_.store(true);
				}
			}
			ml_confidence_.store(confidence);
			ml_inference_time_ms_.store(inference_time);
			has_ml_depth_.store(true);
			ml_pending_.store(false);
		}
	}

	inline void clearMLDepth() {
		{
			boost::unique_lock<boost::mutex> lock(ml_mutex_);
			ml_depth_map_.reset();
			ml_confidence_map_.reset();
			ml_normal_map_.reset();      // Sprint 1: also clear normal map
		}
		has_ml_depth_.store(false);
		has_ml_normals_.store(false);   // Sprint 1
		ml_pending_.store(false);
		ml_confidence_.store(0.0f);
		ml_inference_time_ms_.store(0.0);
	}

	inline bool isMLPending() const { return ml_pending_.load(); }
	inline void setMLPending(bool pending) { ml_pending_.store(pending); }
	inline float getMLConfidence() const { return ml_confidence_.load(); }
	inline double getMLInferenceTime() const { return ml_inference_time_ms_.load(); }

};

/**
 * @brief Stores the calibration matrix information
 * 
 * The values here are usually the same as the global K function
 * 
 */
struct CalibHessian
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	static int instanceCounter;

	VecC value_zero;
	VecC value_scaled;
	VecCf value_scaledf;
	VecCf value_scaledi;
	VecC value;
	VecC step;
	VecC step_backup;
	VecC value_backup;
	VecC value_minus_value_zero;

    inline ~CalibHessian() {instanceCounter--;}
	inline CalibHessian()
	{

		VecC initial_value = VecC::Zero();
		// K matrix
		initial_value[0] = fxG[0];
		initial_value[1] = fyG[0];
		initial_value[2] = cxG[0];
		initial_value[3] = cyG[0];

		// Set K matrix
		setValueScaled(initial_value);
		// Set zero point
		value_zero = value;
		value_minus_value_zero.setZero();

		instanceCounter++;
		for(int i=0;i<256;i++)
			Binv[i] = B[i] = i;		// set gamma function to identity
	};


	// normal mode: use the optimized parameters everywhere!
    inline float& fxl() {return value_scaledf[0];}
    inline float& fyl() {return value_scaledf[1];}
    inline float& cxl() {return value_scaledf[2];}
    inline float& cyl() {return value_scaledf[3];}
    inline float& fxli() {return value_scaledi[0];}
    inline float& fyli() {return value_scaledi[1];}
    inline float& cxli() {return value_scaledi[2];}
    inline float& cyli() {return value_scaledi[3];}

	inline Mat33f getCalibMatrix()
	{
		Mat33f calib;
		calib << value_scaledf[0], 0.0f, value_scaledf[2], 0.0f, value_scaledf[1], value_scaledf[3], 0.0f, 0.0f, 1.0f;
		return calib;
	}

	inline Mat33f getInvCalibMatrix()
	{
		Mat33f invcalib;
		invcalib << value_scaledi[0], 0.0f, value_scaledi[2], 0.0f, value_scaledi[1], value_scaledi[3], 0.0f, 0.0f, 1.0f;
		return invcalib;
	}

	inline void setValue(const VecC &value)
	{
		// [0-3: Kl, 4-7: Kr, 8-12: l2r]
		this->value = value;
		// Scaled K matrix
		value_scaled[0] = SCALE_F * value[0];
		value_scaled[1] = SCALE_F * value[1];
		value_scaled[2] = SCALE_C * value[2];
		value_scaled[3] = SCALE_C * value[3];

		// Scaled K matrix as float instead of double
		this->value_scaledf = this->value_scaled.cast<float>();
		//  Inverse scaled K matrix
		this->value_scaledi[0] = 1.0f / this->value_scaledf[0]; // 1/f_x
		this->value_scaledi[1] = 1.0f / this->value_scaledf[1]; // 1/f_y
		this->value_scaledi[2] = - this->value_scaledf[2] / this->value_scaledf[0]; // -c_x/f_x
		this->value_scaledi[3] = - this->value_scaledf[3] / this->value_scaledf[1]; // -c_y/f_y
		this->value_minus_value_zero = this->value - this->value_zero;
	};

	inline void setValueScaled(const VecC &value_scaled)
	{
		this->value_scaled = value_scaled;
		// Scaled K matrix as float instead of double
		this->value_scaledf = this->value_scaled.cast<float>();
		// K matrix with no scaling
		value[0] = SCALE_F_INVERSE * value_scaled[0];
		value[1] = SCALE_F_INVERSE * value_scaled[1];
		value[2] = SCALE_C_INVERSE * value_scaled[2];
		value[3] = SCALE_C_INVERSE * value_scaled[3];

		this->value_minus_value_zero = this->value - this->value_zero;
		// Inverted scaled K matrix
		this->value_scaledi[0] = 1.0f / this->value_scaledf[0]; // 1/f_x
		this->value_scaledi[1] = 1.0f / this->value_scaledf[1]; // 1/f_y
		this->value_scaledi[2] = - this->value_scaledf[2] / this->value_scaledf[0]; // -c_x/f_x
		this->value_scaledi[3] = - this->value_scaledf[3] / this->value_scaledf[1]; // -c_y/f_y
	};


	float Binv[256];
	float B[256];


	EIGEN_STRONG_INLINE float getBGradOnly(float color)
	{
		int c = color+0.5f;
		if(c<5) c=5;
		if(c>250) c=250;
		return B[c+1]-B[c];
	}

	EIGEN_STRONG_INLINE float getBInvGradOnly(float color)
	{
		int c = color+0.5f;
		if(c<5) c=5;
		if(c>250) c=250;
		return Binv[c+1]-Binv[c];
	}
};


// Hessian component associated with one point.
struct PointHessian
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	static int instanceCounter;
	static unsigned long totalInstantCounter;
	unsigned long point_id;
	EFPoint* efPoint;

	// static values
	float color[MAX_RES_PER_POINT];			// colors in host frame
	float weights[MAX_RES_PER_POINT];		// host-weights for respective residuals.


	Eigen::Vector3f colour3[MAX_RES_PER_POINT];
	bool colourValid;

	float u,v;
	int idx;
	float energyTH;
	FrameHessian* host;
	bool hasDepthPrior;          // RESTORED: For indirect MapPoint priors only
	
	// ML Depth Integration Fields (separated from hasDepthPrior)
	bool hasMLDepth;             // NEW: Separate ML depth flag  
	float ml_idepth_reference;   // NEW: ML depth reference value
	float ml_uncertainty;        // NEW: ML depth uncertainty
	float ml_weight;             // NEW: Computed adaptive ML weight

	float my_type;

	float idepth_scaled;
	float idepth_zero_scaled;
	float idepth_zero;
	float idepth;
	float step;
	float step_backup;
	float idepth_backup;

	float nullspaces_scale;
	float idepth_hessian;
	float maxRelBaseline;
	int numGoodResiduals;
	std::weak_ptr<MapPoint> Mp;
	enum PtStatus {ACTIVE=0, INACTIVE, OUTLIER, OOB, MARGINALIZED};
	PtStatus status;

	/**
	 * @brief Set the status of the point
	 * 
	 * Possible values are ACTIVE, INACTIVE, OUTLIER, OOB (out of bounds), MARGINALIZED
	 * 
	 * @param s 
	 */
    inline void setPointStatus(PtStatus s) {status=s;}


	inline void setIdepth(float idepth) {
		this->idepth = idepth;
		this->idepth_scaled = SCALE_IDEPTH * idepth;
    }
	/**
	 * @brief Set the inverse depth value
	 * 
	 * @param idepth_scaled 
	 */
	inline void setIdepthScaled(float idepth_scaled) {
		this->idepth = SCALE_IDEPTH_INVERSE * idepth_scaled;
		this->idepth_scaled = idepth_scaled;
    }
	inline void setIdepthZero(float idepth) {
		idepth_zero = idepth;
		idepth_zero_scaled = SCALE_IDEPTH * idepth;
		nullspaces_scale = -(idepth*1.001 - idepth/1.001)*500;
    }


	std::vector<PointFrameResidual*> residuals;					// only contains good residuals (not OOB and not OUTLIER). Arbitrary order.
	std::pair<PointFrameResidual*, ResState> lastResiduals[2]; 	// contains information about residuals to the last two (!) frames. ([0] = latest, [1] = the one before).


	void release();
	PointHessian(const ImmaturePoint* const rawPoint, CalibHessian* Hcalib);
    inline ~PointHessian() {assert(efPoint==0); release(); instanceCounter--;}


	inline bool isOOB(const std::vector<FrameHessian*>& toKeep, const std::vector<FrameHessian*>& toMarg) const
	{

		int visInToMarg = 0;
		for(PointFrameResidual* r : residuals)
		{
			if(r->state_state != ResState::IN) continue;
			for(FrameHessian* k : toMarg)
				if(r->target == k) visInToMarg++;
		}
		if((int)residuals.size() >= setting_minGoodActiveResForMarg &&
				numGoodResiduals > setting_minGoodResForMarg+10 &&
				(int)residuals.size()-visInToMarg < setting_minGoodActiveResForMarg)
			return true;





		if(lastResiduals[0].second == ResState::OOB) return true;
		if(residuals.size() < 2) return false;
		if(lastResiduals[0].second == ResState::OUTLIER && lastResiduals[1].second == ResState::OUTLIER) return true;
		return false;
	}


	inline bool isInlierNew()
	{
		return (int)residuals.size() >= setting_minGoodActiveResForMarg
                    && numGoodResiduals >= setting_minGoodResForMarg;
	}

};





}

