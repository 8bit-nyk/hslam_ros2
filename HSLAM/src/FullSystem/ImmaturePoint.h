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
	
	// gd -dev -8june2026 
	// GS integration -> retrieval tracking 
	bool mbRetrived = false;
	void setRetrived(const bool retrived)
	{
		this->mbRetrived = retrived;
	}

	bool isRetrived()
	{
		return this->mbRetrived;
	}

	// GS integration -> host frame ID 
	int getHostFrameID() const
	{
		return host->frameID;
	}

	// GS integration -> average colour3 entries and normalize to [0, 1]
	inline Eigen::Vector3f getColourRGBfloat() const
	{
		if (!colourValid)
			return Eigen::Vector3f::Zero();
		float r = 0.0f, g = 0.0f, b = 0.0f;
		for (unsigned char i = 0; i < MAX_RES_PER_POINT; i++)
		{
			r = (colour3[i][0] + i * r) / (i + 1);
			g = (colour3[i][1] + i * g) / (i + 1);
			b = (colour3[i][2] + i * b) / (i + 1);
		}
		return Eigen::Vector3f(r / 255.0f, g / 255.0f, b / 255.0f);
	}

	// GS integration: back-project pixel to 3D world coordinates
	// depth priority: idepth_GT > midpoint > zero
	inline Eigen::Vector3d getWorldPosition(
		float fxi, float fyi, float cxi, float cyi,
		const SE3 &camToWorld) const
	{
		float idepth = 0.0f;
		if (idepth_GT > 0 && std::isfinite(idepth_GT))
		{
			idepth = idepth_GT;
		}
		else if (idepth_min > 0 && idepth_max > 0 && std::isfinite(idepth_min) && std::isfinite(idepth_max))
		{
			idepth = 0.5f * (idepth_min + idepth_max);
		}
		else
		{
			return Eigen::Vector3d::Zero();
		}
		float z = 1.0f / idepth;
		Eigen::Vector3d pt_cam(
			(fxi * u + cxi) * z,
			(fyi * v + cyi) * z,
			z);
		return camToWorld.matrix3x4() * pt_cam.homogeneous();
	}
	// gd -dev -8june2026 

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

