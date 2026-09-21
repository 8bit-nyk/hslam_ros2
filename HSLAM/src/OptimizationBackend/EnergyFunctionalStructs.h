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
#include "util/settings.h"
#include <algorithm>

namespace HSLAM
{

// WP2c: the explicit Direct.P2 prior weight, shared by EFPoint::takeData (Hessian/gradient),
// AccumulatedSCHessian (gradient) and EnergyFunctional::calcLEnergyF_MT (energy) so the sites cannot
// diverge. The three consumers use ONE convention, valid in both parameterisations:
//     H += w * SCALE_IDEPTH^2          b += w * SCALE_IDEPTH^2 * delta          E  = w * delta^2
// with w = mlPriorWeight(...) and delta = mlPriorDelta(...). w is already the Gauss-Newton weight
// with respect to INVERSE DEPTH (i.e. it carries the residual's Jacobian), so no site needs to know
// which parameterisation is active.
//
//   idepth mode (shipped, setting_mlPriorParam == ML_PRIOR_PARAM_IDEPTH):
//     r = idepth - prior,  delta = r,  dr/d(idepth) = 1
//     w = conf * (1/sigma^2) * exp(-r^2 / (2 tau^2)),  tau = setting_mlSelfGateTau (0.01 1/m),
//     conf = ml_weight / setting_mlDepthWeight (unclamped: init-path points carry ml_weight = 2500);
//     v2 (setting_mlPriorWeightMult != 1 or setting_mlPriorGateK > 0): conf clamped to [0.1, 1],
//     tau_i = k * sigma_i when k > 0, result multiplied by setting_mlPriorWeightMult. sigma is the
//     point's P1 half-width in inverse depth, so w * r^2 is dimensionless.
//
//   log mode (WP2b-log, setting_mlPriorParam == ML_PRIOR_PARAM_LOG): the RELATIVE residual
//     rho = log(idepth / prior) = -log(d / d_ML),  d(rho)/d(idepth) = 1 / idepth,
//     w_rho = conf * (1/sigma_log^2) * gate * mult  with sigma_log = setting_mlPriorSigmaLog, a single
//     DIMENSIONLESS constant (not the P1 box: the box is absolute in inverse depth, so converting it
//     would reintroduce exactly the range dependence this parameterisation exists to remove -- at 40 m
//     the shipped box is ~12 in log units). gate = exp(-rho^2 / (2 (k sigma_log)^2)) when k > 0, else 1.
//     Returned to the caller in inverse-depth form: w = w_rho / idepth^2 and delta = idepth * rho, so
//     w * delta^2 = w_rho * rho^2 and w * delta = w_rho * rho / idepth = the true gradient. Leverage is
//     then uniform over range: a 20 % depth error gives |rho| = 0.18 at 1 m and at 40 m alike.
//     Non-positive idepth or prior => no prior (the log is undefined; the point keeps photometry only).
inline bool mlPriorIsLog()
{
	return setting_mlPriorParam == ML_PRIOR_PARAM_LOG;
}

// The residual expressed in inverse-depth units (see the convention above).
inline float mlPriorDelta(float idepth, float ml_ref)
{
	if (mlPriorIsLog())
	{
		if (idepth <= 1e-6f || ml_ref <= 1e-6f) return 0.0f;
		return idepth * std::log(idepth / ml_ref);
	}
	return idepth - ml_ref;
}

inline float mlPriorWeight(float idepth, float ml_ref, float ml_sigma, float ml_weight, float* self_gate_out = nullptr)
{
	const bool log_mode = mlPriorIsLog();
	const bool v2 = log_mode || (setting_mlPriorWeightMult != 1.0f) || (setting_mlPriorGateK > 0.0f);

	if (log_mode && (idepth <= 1e-6f || ml_ref <= 1e-6f))
	{
		if (self_gate_out) *self_gate_out = 0.0f;
		return 0.0f;
	}

	// residual and its 1-sigma, in the active parameterisation
	const float ml_residual = log_mode ? std::log(idepth / ml_ref) : (idepth - ml_ref);
	const float sigma = log_mode ? std::max(setting_mlPriorSigmaLog, 1e-3f) : ml_sigma;

	float tau;
	if (log_mode)
		tau = (setting_mlPriorGateK > 0.0f) ? setting_mlPriorGateK * sigma : 0.0f;   // 0 = no self-gate
	else
		tau = (v2 && setting_mlPriorGateK > 0.0f) ? setting_mlPriorGateK * ml_sigma : setting_mlSelfGateTau;

	const float self_gate = (tau > 0.0f) ? std::exp(-ml_residual * ml_residual / (2.0f * tau * tau)) : 1.0f;
	if (self_gate_out) *self_gate_out = self_gate;

	float uncertainty_weight = 1.0f / (sigma * sigma);
	float ml_conf = (ml_weight > 0) ? (ml_weight / setting_mlDepthWeight) : 0.5f;
	if (v2) ml_conf = std::min(1.0f, std::max(0.1f, ml_conf));
	float w_ML = ml_conf * uncertainty_weight * self_gate;
	if (v2) w_ML *= setting_mlPriorWeightMult;
	// back to inverse-depth units: d(rho)/d(idepth) = 1/idepth
	if (log_mode) w_ML /= (idepth * idepth);
	return w_ML;
}

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

	// Sprint 10 (E3 / Phase B.4): depth-normal surface-consistency prior. Recomputed once per BA
	// round by FullSystem::computeDepthNormalPriors(); the residual (idepth - dn_target) is then
	// evaluated against the CURRENT idepth each accumulation, mirroring ml_priorF/ml_reference.
	// dn_h = lambda * (Hdd_accAF + Hdd_accLF) — weight tied to photometric stiffness, see settings.h.
	float dn_h = 0;        // idepth Hessian contribution (0 = inactive for this point)
	float dn_target = 0;   // plane-predicted inverse depth (median over neighbours)


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

