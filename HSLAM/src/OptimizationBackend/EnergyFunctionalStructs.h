/**
* This file is part of DSO.
* 
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
#include "vector"
#include <math.h>
#include "OptimizationBackend/RawResidualJacobian.h"

namespace HSLAM
{

class PointFrameResidual;
class CalibHessian;
class FrameHessian;
class PointHessian;

class EFResidual;
class EFPoint;
class EFFrame;
class EnergyFunctional;






class EFResidual
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	inline EFResidual(PointFrameResidual* org, EFPoint* point_, EFFrame* host_, EFFrame* target_) :
		data(org), point(point_), host(host_), target(target_)
	{
		isLinearized=false;
		isActiveAndIsGoodNEW=false;
		J = new RawResidualJacobian();
		assert(((long)this)%16==0);
		assert(((long)J)%16==0);
	}
	inline ~EFResidual()
	{
		delete J;
	}


	void takeDataF();


	void fixLinearizationF(EnergyFunctional* ef);


	// structural pointers
	PointFrameResidual* data;
	int hostIDX, targetIDX;
	EFPoint* point;
	EFFrame* host;
	EFFrame* target;
	int idxInAll;

	RawResidualJacobian* J;

	VecNRf res_toZeroF;
	Vec8f JpJdF;


	// status.
	bool isLinearized;

	// if residual is not OOB & not OUTLIER & should be used during accumulations
	bool isActiveAndIsGoodNEW;
	inline const bool &isActive() const {return isActiveAndIsGoodNEW;}
};


enum EFPointStatus {PS_GOOD=0, PS_MARGINALIZE, PS_DROP};

class EFPoint
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EFPoint(PointHessian* d, EFFrame* host_) : data(d),host(host_)
	{
		takeData();
		stateFlag=EFPointStatus::PS_GOOD;
	}
	void takeData();

	PointHessian* data;



	float priorF;          // For indirect MapPoint priors (original DSO)
	float deltaF;
	
	// ML Depth Integration Fields (separated from priorF)
	float ml_priorF;       // NEW: For ML depth priors only
	float ml_reference;    // NEW: ML reference depth (idepth_zero equivalent)
	float ml_sigma;        // ML depth standard deviation (aleatoric uncertainty)

	// Direct.VS: Virtual stereo image-space constraint (Step 4)
	// Computed once at point activation; enters H/b via Schur complement accumulation.
	// vs_h = w * J_rho^2,  vs_b = w * J_rho * r   (J_rho = I_x(u_R) * (-fx * b_vs))
	float vs_h = 0;   // idepth Hessian contribution
	float vs_b = 0;   // idepth gradient contribution


	// constant info (never changes in-between).
	int idxInPoints;
	EFFrame* host;

	// contains all residuals.
	std::vector<EFResidual*> residualsAll;

	// Zero-initialized. The accumulation pass (AccumulatedTopHessian) writes these before the Schur
	// pass (AccumulatedSCHessian) reads them, so in the stock flow the initial value is never
	// observed — but the EFPoint constructor left them indeterminate, so any code that inspects a
	// freshly inserted point's accumulators BEFORE its first accumulation reads garbage. That is
	// undefined behaviour, and it silently produced ~1e19 values when exercised.
	float bdSumF = 0;
	float HdiF = 0;
	float Hdd_accLF = 0;
	VecCf Hcd_accLF = VecCf::Zero();
	float bd_accLF = 0;
	float Hdd_accAF = 0;
	VecCf Hcd_accAF = VecCf::Zero();
	float bd_accAF = 0;


	EFPointStatus stateFlag;
};



class EFFrame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	EFFrame(FrameHessian* d) : data(d)
	{
		takeData();
	}
	void takeData();


	Vec8 prior;				// prior hessian (diagonal)
	Vec8 delta_prior;		// = state-state_prior (E_prior = (delta_prior)' * diag(prior) * (delta_prior)
	Vec8 delta;				// state - state_zero.



	std::vector<EFPoint*> points;
	FrameHessian* data;
	int idx;	// idx in frames.

	int frameID;
};

}

