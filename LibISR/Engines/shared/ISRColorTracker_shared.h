// Copyright 2014-2015 Isis Innovation Limited and the authors of LibISR

#pragma once

#include "../../LibISRDefine.h"
#include "../shared/ISRVoxelAccess_shared.h"

#include "../../Objects/ISRTrackingState.h"
#include "../../Objects/ISRShapeUnion.h"

#include "../../Objects/ISRPose.h"
#include "../../Objects/ISRShape.h"


#ifndef NARROW_BAND
#define NARROW_BAND 20
#endif

#ifndef NARROW_BAND_IN
#define NARROW_BAND_IN -10
#endif

_CPU_AND_GPU_CODE_ inline float castRayToMinSDF(Vector4f &pt_out, int x, int y, const float* voxelData, const Matrix4f& invH, const Vector4f& invIntrinsic, const Vector2f& maxminvalue, float maxvoxelrange)
{

	Vector3f pt_camera_f, pt_block_s, pt_block_e, rayDirection, pt_result;
	bool inside_volume, pt_found; pt_found = false;
	float sdfValue = MAX_SDF, lastsdfValue = MAX_SDF, minSDF = MAX_SDF;
	float totalLength = 0, stepLength, totalLengthMax;

	pt_camera_f.z = maxminvalue.x; // min
	pt_camera_f.x = pt_camera_f.z * ((float(x) *invIntrinsic.x + invIntrinsic.z));
	pt_camera_f.y = pt_camera_f.z * ((float(y) *invIntrinsic.y + invIntrinsic.w));
	pt_block_s = (invH * pt_camera_f);

	pt_camera_f.z = maxminvalue.y; // max
	pt_camera_f.x = pt_camera_f.z * ((float(x) *invIntrinsic.x + invIntrinsic.z));
	pt_camera_f.y = pt_camera_f.z * ((float(y) *invIntrinsic.y + invIntrinsic.w));
	pt_block_e = (invH * pt_camera_f);

	totalLengthMax = length(pt_block_e - pt_block_s);
	rayDirection = (pt_block_e - pt_block_s).normalised();
	pt_result = pt_block_s;

	float step_fine = totalLengthMax * maxvoxelrange;
	float step_coarse = step_fine * 10;

	while (totalLength <= totalLengthMax)
	{
		sdfValue = getSDFValue(pt_result, voxelData, inside_volume);
		if (inside_volume)
		{
			if (fabs(sdfValue) <= 1.0f || (lastsdfValue != MAX_SDF && lastsdfValue*sdfValue < 0))
			{
				minSDF = sdfValue;
				pt_found = true;
				pt_out.x = pt_result.x;
				pt_out.y = pt_result.y;
				pt_out.z = pt_result.z;
				pt_out.w = 1;
				break;
			}
			if (fabs(sdfValue) < minSDF)
			{
				minSDF = fabs(sdfValue);
				pt_out.x = pt_result.x;
				pt_out.y = pt_result.y;
				pt_out.z = pt_result.z;
				pt_out.w = 1;
			}

			lastsdfValue = sdfValue;
			stepLength = sdfValue*step_fine;
		}
		else
		{
			stepLength = step_coarse;
		}

		pt_result += stepLength * rayDirection;
		totalLength += stepLength;
	}

	// if raycasted to a surface, go along ray direction for NARROW_BAND mount for SDF value
	if (pt_found)
	{
		for (int i = 0; i < 2; i++)
		{
			pt_result += step_fine*rayDirection*NARROW_BAND*0.5;
			sdfValue = getSDFValue(pt_result, voxelData, inside_volume);
			if (inside_volume) minSDF = sdfValue < minSDF ? sdfValue : minSDF;
		}
	}
	else if (minSDF > NARROW_BAND)
	{
		minSDF = MAX_SDF;
		pt_out.w = -1;
	}
	else if (minSDF < NARROW_BAND_IN)
	{
		pt_out.w = -1;
	}

	return minSDF;
}


// name is not what it means, it ray cast to a signed distance function now
_CPU_AND_GPU_CODE_ inline void raycastAsHeaviside(float* outHeavisideImg, Vector4f* outPtCloud, int x, int y, const Vector2i& imagesize, const float* voxelData, const Matrix4f& invH, const Vector4f& invIntrinsic, Vector2f minmaxvalue, float maxvoxelrange, Vector4i raycastBB)
{
	int idx = x + y*imagesize.x;
	Vector4f pt_obj(0, 0, 0, -1);

	if (x <= raycastBB.x || x >= raycastBB.z || y <=raycastBB.y || y >= raycastBB.w) outHeavisideImg[idx] = MAX_SDF;
	else
	{
		float minsdf = castRayToMinSDF(pt_obj, x, y, voxelData, invH, invIntrinsic, minmaxvalue, maxvoxelrange);
		outHeavisideImg[idx] = minsdf;
	}

	outPtCloud[idx] = pt_obj;
}


// a working version for now
_CPU_AND_GPU_CODE_ inline float computePerPixelJacobian(float *jacobian, const float* sdfImg, const float* pfImg, const Vector4f* raycastPtImg, const Vector4f &intrinsics, int x, int y, const Vector2i& imagesize, const Matrix4f &H, float& pref)
{
	int idx = y*imagesize.x + x;
	if (raycastPtImg[idx].w==-1) return -1;
	if (raycastPtImg[y*imagesize.x + (x + 1)].w == -1) return -1;
	if (raycastPtImg[y*imagesize.x + (x - 1)].w == -1) return -1;
	if (raycastPtImg[(y + 1)*imagesize.x + x].w == -1) return -1;
	if (raycastPtImg[(y - 1)*imagesize.x + x].w == -1) return -1;

	float dt = sdfImg[idx] * 0.5f;
	float exp_dt = expf(dt);
	float deto = exp_dt + 1.0f;
	float sheaviside = 1.0f / deto;
	float d_heaviside_dt = exp_dt / (deto * deto);

	float pf = pfImg[idx]; //if (pf > 0.6) pf = 1.0f;
	float b = pf*sheaviside + (1 - pf)*(1 - sheaviside);

	//pref = (pf * 2 - 1)/**d_heaviside_dt*/; // linear pool
	pref = (pf * 2 - 1) * d_heaviside_dt / b; //logarithmic pool


	// compute SDF gradient using Scharr operator
	float weight_dx = 10, weight_dy = 10;

	float dt_dx = (sdfImg[y*imagesize.x + (x + 1)] - sdfImg[y*imagesize.x + (x - 1)]) * 10;
	float dt_dy = (sdfImg[(y + 1)*imagesize.x + x] - sdfImg[(y - 1)*imagesize.x + x]) * 10;

	if (raycastPtImg[(y - 1)*imagesize.x + (x - 1)].w != -1 && raycastPtImg[(y - 1)*imagesize.x + (x + 1)].w != -1)
	{
		dt_dx += (sdfImg[(y - 1)*imagesize.x + (x + 1)] - sdfImg[(y - 1)*imagesize.x + (x - 1)]) * 3;
		weight_dx += 3;
	}

	if (raycastPtImg[(y + 1)*imagesize.x + (x - 1)].w != -1 && raycastPtImg[(y + 1)*imagesize.x + (x + 1)].w != -1)
	{
		dt_dx += (sdfImg[(y + 1)*imagesize.x + (x + 1)] - sdfImg[(y + 1)*imagesize.x + (x - 1)]) * 3;
		weight_dx += 3;
	}

	if (raycastPtImg[(y - 1)*imagesize.x + (x - 1)].w != -1 && raycastPtImg[(y + 1)*imagesize.x + (x - 1)].w != -1)
	{
		dt_dy += (sdfImg[(y + 1)*imagesize.x + (x - 1)] - sdfImg[(y - 1)*imagesize.x + (x - 1)]) * 3;
		weight_dy += 3;
	}

	if (raycastPtImg[(y - 1)*imagesize.x + (x + 1)].w != -1 && raycastPtImg[(y + 1)*imagesize.x + (x + 1)].w != -1)
	{
		dt_dy += (sdfImg[(y + 1)*imagesize.x + (x + 1)] - sdfImg[(y - 1)*imagesize.x + (x + 1)]) * 3;
		weight_dy += 3;
	}
	
	dt_dx /= weight_dx; dt_dy /= weight_dy;

	Vector4f pt = H*raycastPtImg[idx];
	float ffx = intrinsics.x / pt.z;
	float ffy = intrinsics.y / pt.z;
	float ccx = intrinsics.z / pt.z;
	float ccy = intrinsics.w / pt.z;

	jacobian[0] = ffx * dt_dx;
	jacobian[1] = ffy * dt_dy;
	jacobian[2] = (-ffx * pt.x / pt.z) * dt_dx + (-ffy * pt.y / pt.z) * dt_dy;
	jacobian[3] = (ffx * pt.x * pt.y / pt.z) * dt_dx + (ffy * (pt.y * pt.y / pt.z + pt.z)) * dt_dy;
	jacobian[4] = -(ffx * (pt.x *pt.x / pt.z + pt.z)) * dt_dx - (ffy * pt.x * pt.y / pt.z) * dt_dy;
	jacobian[5] = ffx * pt.y *dt_dx - ffy * pt.x * dt_dy;
	

	jacobian[3] *= -4;
	jacobian[4] *= -4;
	jacobian[5] *= -4;

	return b;
}

_CPU_AND_GPU_CODE_ inline float computePerPixelJacobian(float *jacobian, const float* sdfImg, const float* pfImg, const Vector4f* raycastPtImg, const Vector4f &intrinsics, int x, int y, const Vector2i& imagesize, const Matrix4f &H, float& GradientPref, float& HessianPref )
{
	int idx = y*imagesize.x + x;
	if (raycastPtImg[idx].w == -1) return -1;
	if (raycastPtImg[y*imagesize.x + (x + 1)].w == -1) return -1;
	if (raycastPtImg[y*imagesize.x + (x - 1)].w == -1) return -1;
	if (raycastPtImg[(y + 1)*imagesize.x + x].w == -1) return -1;
	if (raycastPtImg[(y - 1)*imagesize.x + x].w == -1) return -1;

	float dt = sdfImg[idx] * 0.5f;
	float exp_dt = expf(dt);
	float deto = exp_dt + 1.0f;
	
	
	float sheaviside = 1.0f / deto;
	float dHeaviside = -exp_dt / (deto * deto);
	float ddHeaviside = 2.0f * expf(dt * 2.0f) / (deto * deto * deto) - exp_dt / (deto * deto);

	float pf = pfImg[idx]; //if (pf > 0.6) pf = 1.0f;
	float epp = pf*sheaviside + (1 - pf)*(1 - sheaviside);

	// compute SDF gradient using Scharr operator
	float weight_dx = 10, weight_dy = 10;

	float dt_dx = (sdfImg[y*imagesize.x + (x + 1)] - sdfImg[y*imagesize.x + (x - 1)]) * 10;
	float dt_dy = (sdfImg[(y + 1)*imagesize.x + x] - sdfImg[(y - 1)*imagesize.x + x]) * 10;

	if (raycastPtImg[(y - 1)*imagesize.x + (x - 1)].w != -1 && raycastPtImg[(y - 1)*imagesize.x + (x + 1)].w != -1)
	{
		dt_dx += (sdfImg[(y - 1)*imagesize.x + (x + 1)] - sdfImg[(y - 1)*imagesize.x + (x - 1)]) * 3;
		weight_dx += 3;
	}

	if (raycastPtImg[(y + 1)*imagesize.x + (x - 1)].w != -1 && raycastPtImg[(y + 1)*imagesize.x + (x + 1)].w != -1)
	{
		dt_dx += (sdfImg[(y + 1)*imagesize.x + (x + 1)] - sdfImg[(y + 1)*imagesize.x + (x - 1)]) * 3;
		weight_dx += 3;
	}

	if (raycastPtImg[(y - 1)*imagesize.x + (x - 1)].w != -1 && raycastPtImg[(y + 1)*imagesize.x + (x - 1)].w != -1)
	{
		dt_dy += (sdfImg[(y + 1)*imagesize.x + (x - 1)] - sdfImg[(y - 1)*imagesize.x + (x - 1)]) * 3;
		weight_dy += 3;
	}

	if (raycastPtImg[(y - 1)*imagesize.x + (x + 1)].w != -1 && raycastPtImg[(y + 1)*imagesize.x + (x + 1)].w != -1)
	{
		dt_dy += (sdfImg[(y + 1)*imagesize.x + (x + 1)] - sdfImg[(y - 1)*imagesize.x + (x + 1)]) * 3;
		weight_dy += 3;
	}

	dt_dx /= weight_dx; dt_dy /= weight_dy;

	GradientPref = (pf * 2 - 1) * dHeaviside; //linear pool
	//GradientPref = (pf * 2 - 1) / epp; //logarithmic pool
	HessianPref = (pf * 2 - 1) * (ddHeaviside*(dt_dx*dt_dx + dt_dy*dt_dy));

	Vector4f pt = H*raycastPtImg[idx];
	float ffx = intrinsics.x / pt.z;
	float ffy = intrinsics.y / pt.z;
	float ccx = intrinsics.z / pt.z;
	float ccy = intrinsics.w / pt.z;

	jacobian[0] = ffx * dt_dx;
	jacobian[1] = ffy * dt_dy;
	jacobian[2] = (-ffx * pt.x / pt.z) * dt_dx + (-ffy * pt.y / pt.z) * dt_dy;
	jacobian[3] = (ffx * pt.x * pt.y / pt.z) * dt_dx + (ffy * (pt.y * pt.y / pt.z + pt.z)) * dt_dy;
	jacobian[4] = -(ffx * (pt.x *pt.x / pt.z + pt.z)) * dt_dx - (ffy * pt.x * pt.y / pt.z) * dt_dy;
	jacobian[5] = ffx * pt.y *dt_dx - ffy * pt.x * dt_dy;

	jacobian[3] *= -4.0;
	jacobian[4] *= -4.0f;
	jacobian[5] *= -4.0f;

	return epp;
}



_CPU_AND_GPU_CODE_ inline float computePerPixelEnergy(const float* sdfImg, const float* pfImg, const Vector4f* raycastPtImg, int x, int y, const Vector2i& imagesize)
{
	int idx = y*imagesize.x + x;
	if (raycastPtImg[idx].w == -1) return -1;

	float dt = sdfImg[idx];
	float exp_dt = expf(dt);
	float deto = exp_dt + 1.0f;
	float sheaviside = 1.0f / deto;

	float pf = pfImg[idx];
	return pf*sheaviside + (1 - pf)*(1 - sheaviside);
}
