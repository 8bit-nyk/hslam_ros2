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

/*
 * KFBuffer.cpp
 *
 *  Created on: Jan 7, 2014
 *      Author: engelj
 */

#include "FullSystem/FullSystem.h"
 
#include "stdio.h"
#include "util/globalFuncs.h"
#include <Eigen/LU>
#include <algorithm>
#include "IOWrapper/ImageDisplay.h"
#include "util/globalCalib.h"

#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include "FullSystem/ImmaturePoint.h"
#include "math.h"

namespace HSLAM
{



/**
 * @brief Do optimization calculations for immature points that are being activated
 * 
 * For all of the active frames
 * - Starts by calculating an optimized depth value
 * - Checks the energy from the optimization to determine if the point should count as visible in a frame
 * - Creates a PointHessian struct for the activated point and frame with the immature point values and new depth
 * 
 * @param point 			List of immature points that are to be optimized
 * @param minObs 
 * @param residuals 		Residual calculations
 * @return PointHessian* 	Output array of activated points
 */
PointHessian* FullSystem::optimizeImmaturePoint(
		ImmaturePoint* point, int minObs,
		ImmaturePointTemporaryResidual* residuals)
{
	int nres = 0;
	for(FrameHessian* fh : frameHessians) // for all active frames
	{
		// Initialize variables for all connected frames
		if(fh != point->host)
		{
			residuals[nres].state_NewEnergy = residuals[nres].state_energy = 0;
			residuals[nres].state_NewState = ResState::OUTLIER;
			residuals[nres].state_state = ResState::IN;
			residuals[nres].target = fh;
			nres++;
		}
	}
	assert(nres == ((int)frameHessians.size())-1);

	bool print = false;

	float lastEnergy = 0;
	float lastHdd=0;
	float lastbd=0;
	
	// ML Depth Integration: Use ML depth as optimization starting point if available
	float currentIdepth;
	if(point->idepth_GT > 0) {
		currentIdepth = point->idepth_GT;  // Start from accurate ML depth estimate
		if(print) printf("USING ML DEPTH as starting point: %.3f (vs midpoint %.3f)\n", 
						 currentIdepth, (point->idepth_max+point->idepth_min)*0.5f);
	} else {
		currentIdepth = (point->idepth_max+point->idepth_min)*0.5f; // Original fallback
	}



	// Initial calculations
	for(int i=0;i<nres;i++)
	{
		// Calculate residual of immature point for all connected frames
		lastEnergy += point->linearizeResidual(&Hcalib, 1000, residuals+i,lastHdd, lastbd, currentIdepth);
		residuals[i].state_state = residuals[i].state_NewState;
		residuals[i].state_energy = residuals[i].state_NewEnergy;
	}

	if(!std::isfinite(lastEnergy) || lastHdd < setting_minIdepthH_act)
	{
		if(print)
			printf("OptPoint: Not well-constrained (%d res, H=%.1f). E=%f. SKIP!\n",
				nres, lastHdd, lastEnergy);
		return 0;
	}

	if(print) printf("Activate point. %d residuals. H=%f. Initial Energy: %f. Initial Id=%f\n" ,
			nres, lastHdd,lastEnergy,currentIdepth);

	// Optimize new values (depth) for activated point
	float lambda = 0.1;
	for(int iteration=0;iteration<setting_GNItsOnPointActivation;iteration++)
	{
		float H = lastHdd;
		H *= 1+lambda;
		float step = (1.0/H) * lastbd;
		float newIdepth = currentIdepth - step;
		
		// CRITICAL FIX: Enforce ML depth bounds during optimization
		if(point->idepth_GT > 0 && setting_preserveMLDepthBounds) {
			// Clamp optimization within ML depth bounds
			if(newIdepth < point->idepth_min) {
				newIdepth = point->idepth_min;
				if(print) printf("OptPoint: Clamped to ML min bound %.3f (was %.3f)\n", 
								newIdepth, currentIdepth - step);
			}
			if(newIdepth > point->idepth_max) {
				newIdepth = point->idepth_max;
				if(print) printf("OptPoint: Clamped to ML max bound %.3f (was %.3f)\n", 
								newIdepth, currentIdepth - step);
			}
		}

		float newHdd=0; float newbd=0; float newEnergy=0;
		for(int i=0;i<nres;i++)
			newEnergy += point->linearizeResidual(&Hcalib, 1, residuals+i,newHdd, newbd, newIdepth);

		if(!std::isfinite(lastEnergy) || newHdd < setting_minIdepthH_act)
		{
			if(print) printf("OptPoint: Not well-constrained (%d res, H=%.1f). E=%f. SKIP!\n",
					nres,
					newHdd,
					lastEnergy);
			return 0;
		}

		if(print) printf("%s %d (L %.2f) %s: %f -> %f (idepth %f)!\n",
				(true || newEnergy < lastEnergy) ? "ACCEPT" : "REJECT",
				iteration,
				log10(lambda),
				"",
				lastEnergy, newEnergy, newIdepth);

		if(newEnergy < lastEnergy) // use new values and increase step
		{
			currentIdepth = newIdepth;
			lastHdd = newHdd;
			lastbd = newbd;
			lastEnergy = newEnergy;
			for(int i=0;i<nres;i++)
			{
				residuals[i].state_state = residuals[i].state_NewState;
				residuals[i].state_energy = residuals[i].state_NewEnergy;
			}

			lambda *= 0.5;
		}
		else // decrease step
		{
			lambda *= 5;
		}

		if(fabsf(step) < 0.0001*currentIdepth)
			break;
	}

	if(!std::isfinite(currentIdepth))
	{
		printf("MAJOR ERROR! point idepth is nan after initialization (%f).\n", currentIdepth);
		return (PointHessian*)((long)(-1));		// yeah I'm like 99% sure this is OK on 32bit systems.
	}


	int numGoodRes=0;
	for(int i=0;i<nres;i++)
		if(residuals[i].state_state == ResState::IN) numGoodRes++;

	if(numGoodRes < minObs)
	{
		if(print) printf("OptPoint: OUTLIER!\n");
		return (PointHessian*)((long)(-1));		// 99% sure this is OK on 32bit systems.
	}



	// Set new PointHessian and PointFrameResidual structs for activated points
	PointHessian* p = new PointHessian(point, &Hcalib);
	if(!std::isfinite(p->energyTH)) {delete p; return (PointHessian*)((long)(-1));}

	// PHASE 2 FIX: Ensure ML confidence values are properly transferred (backup to constructor)
	if(point->idepth_GT > 0 && point->ml_confidence > 0) {
		float adaptive_confidence = std::max(0.1f, std::min(1.0f, point->ml_confidence));
		p->ml_weight = setting_mlDepthWeight * adaptive_confidence;
		p->ml_uncertainty = (point->ml_uncertainty_m > 0) ? point->ml_uncertainty_m : 0.1f;
		
		// PHASE2_DEBUG: Log final adaptive weight for verification  
		static int weight_transfer_count = 0;
		if (weight_transfer_count < 3) {
			printf("PHASE2_DEBUG: Final adaptive weight %d: conf=%.3f -> weight=%.3f (base=%.3f)\n",
			       weight_transfer_count, adaptive_confidence, p->ml_weight, setting_mlDepthWeight);
			weight_transfer_count++;
		}
	}

	p->lastResiduals[0].first = 0;
	p->lastResiduals[0].second = ResState::OOB;
	p->lastResiduals[1].first = 0;
	p->lastResiduals[1].second = ResState::OOB;
	// For ML depth constraints: set idepth_zero to original ML depth, not optimized depth
	if(point->idepth_GT > 0) {
		p->setIdepthZero(point->idepth_GT);  // ML depth as reference for bundle adjustment constraints
		p->setIdepth(currentIdepth);         // Optimized depth as current estimate
	} else {
		p->setIdepthZero(currentIdepth);     // Original behavior for non-ML points
		p->setIdepth(currentIdepth);
	}
	p->setPointStatus(PointHessian::ACTIVE);

	// Do all of the required optimization calculations for the new points
	for(int i=0;i<nres;i++)
		if(residuals[i].state_state == ResState::IN)
		{
			PointFrameResidual* r = new PointFrameResidual(p, p->host, residuals[i].target);
			r->state_NewEnergy = r->state_energy = 0;
			r->state_NewState = ResState::OUTLIER;
			r->setState(ResState::IN);
			p->residuals.push_back(r);

			if(r->target == frameHessians.back())
			{
				p->lastResiduals[0].first = r;
				p->lastResiduals[0].second = ResState::IN;
			}
			else if(r->target == (frameHessians.size()<2 ? 0 : frameHessians[frameHessians.size()-2]))
			{
				p->lastResiduals[1].first = r;
				p->lastResiduals[1].second = ResState::IN;
			}
		}

	if(print) printf("Point activated!\n");

	statistics_numActivatedPoints++;
	return p;
}



}
