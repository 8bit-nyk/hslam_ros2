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

 
#include "util/NumType.h"
 
#include "FullSystem/HessianBlocks.h"
namespace HSLAM
{

class MapPoint;

struct ImmaturePointTemporaryResidual
{
public:
	ResState state_state;
	double state_energy;
	ResState state_NewState;
	double state_NewEnergy;
	FrameHessian* target;
};


enum ImmaturePointStatus {
	IPS_GOOD=0,					// Traced is good
	IPS_OOB,					// Out of Bounds: end tracking & marginalize
	IPS_OUTLIER,				// Energy too high, treat as outlier
	IPS_SKIPPED,				// Good enough to not need tracing, use current values
	IPS_BADCONDITION,			// Not traced because of bad condition.
	IPS_UNINITIALIZED};			// Not even traced once


class ImmaturePoint
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	// static values
	float color[MAX_RES_PER_POINT];
	float weights[MAX_RES_PER_POINT];

	std::weak_ptr<MapPoint> Mp;

	Eigen::Vector3f colour3[MAX_RES_PER_POINT];
	bool colourValid;

	Mat22f gradH;
	Vec2f gradH_ev;
	Mat22f gradH_eig;
	float energyTH;
	float u,v;
	FrameHessian* host;
	int idxInImmaturePoints;

	float quality;

	float my_type;

	float idepth_min;
	float idepth_max;
	ImmaturePoint(int u_, int v_, FrameHessian *host_, float type, CalibHessian *HCalib);
	~ImmaturePoint();

	ImmaturePointStatus traceOn(FrameHessian* frame, const Mat33f &hostToFrame_KRKi, const Vec3f &hostToFrame_Kt, const Vec2f &hostToFrame_affine, CalibHessian* HCalib, bool debugPrint=false);

	ImmaturePointStatus lastTraceStatus;
	Vec2f lastTraceUV;
	float lastTracePixelInterval;

	float idepth_GT;
	
	// PHASE 2: ML confidence and uncertainty fields
	float ml_confidence;      // Per-pixel ML confidence [0,1] from normal uncertainty
	float ml_uncertainty_m;   // Uncertainty in meters (for inverse depth transformation)

	// DIAGNOSTIC ONLY (setting_diagTraceStats, default false) — no behavioural effect.
	// The ML bound write at FullSystem.cpp:3166-3167 makes idepth_max finite from birth, so an ML
	// point survives the "never traced successfully" deletion at FullSystem.cpp:859 and can reach
	// activation having never once completed an epipolar search — in which case it is activated at
	// exactly rho_ML. Nobody has ever measured how large that population is: the activation-path
	// diagnostics are dead (`bool print = false`, FullSystemOptPoint.cpp:84) and the trace histogram
	// below traceNewCoarse has been commented out since DSO. These three fields make it countable.
	bool everGood;                          // has this point EVER returned IPS_GOOD?
	bool firstTraceDone;                    // has traceOn been called at least once?
	ImmaturePointStatus firstTraceStatus;   // status of the FIRST trace (recorded by the caller)

	double linearizeResidual(
			CalibHessian *  HCalib, const float outlierTHSlack,
			ImmaturePointTemporaryResidual* tmpRes,
			float &Hdd, float &bd,
			float idepth);
	float getdPixdd(
			CalibHessian *  HCalib,
			ImmaturePointTemporaryResidual* tmpRes,
			float idepth);

	float calcResidual(
			CalibHessian *  HCalib, const float outlierTHSlack,
			ImmaturePointTemporaryResidual* tmpRes,
			float idepth);

private:
};

}

