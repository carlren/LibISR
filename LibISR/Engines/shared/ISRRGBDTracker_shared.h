// Copyright 2014-2015 Isis Innovation Limited and the authors of LibISR

#pragma once

#include "../../LibISRDefine.h"
#include "../shared/ISRVoxelAccess_shared.h"

#include "../../Objects/ISRTrackingState.h"
#include "../../Objects/ISRShapeUnion.h"

#include "../../Objects/ISRPose.h"
#include "../../Objects/ISRShape.h"

// inpt now is in camera coordinates, it need to be transformed by pose invH to object coordinates
// inpt is also been properly scaled to math the voxel resolution
// inpt.w is pf for the point
_CPU_AND_GPU_CODE_ inline float computePerPixelEnergy(const Vector4f &inpt, LibISR::Objects::ISRShape_ptr shapes, LibISR::Objects::ISRPose_ptr poses, int numObj)
{
	if (inpt.w > 0)
	{
		float dt = MAX_SDF, partdt = MAX_SDF;
		int idx;
		float *voxelBlocks;

		for (int i = 0; i < numObj; i++)
		{
			Vector3f objpt = poses[i].getInvH()*Vector3f(inpt.x, inpt.y, inpt.z);
			idx = pt2IntIdx(objpt);
			if (idx >= 0)
			{
				voxelBlocks = shapes[i].getSDFVoxel();
				partdt = voxelBlocks[idx];
				dt = partdt < dt ? partdt : dt; // now use a hard min to approximate
			}
		}

		if (dt == MAX_SDF) return -1.0f;

		float exp_dt = expf(-dt * DTUNE);
		float deto = exp_dt + 1.0f;
		float sheaviside = 1.0f / deto;
		float sdelta = 4.0f* exp_dt * sheaviside * sheaviside;
		float e = inpt.w * sdelta*TMP_WEIGHT + (1 - inpt.w)*sheaviside*(2-TMP_WEIGHT);
		return e;
	}
	else return 0.0f;
}

// inpt now is in camera coordinates, it need to be transformed by pose invH to object coordinates
// inpt is also been properly scaled to math the voxel resolution
// inpt.w is pf for the point
_CPU_AND_GPU_CODE_ inline bool computePerPixelJacobian(float *jacobian, const Vector4f &inpt, LibISR::Objects::ISRShape_ptr shapes, LibISR::Objects::ISRPose_ptr poses, int numObj)
{
	if (inpt.w < 0) return false;

	float dt = MAX_SDF, partdt = MAX_SDF;
	int idx, minidx;
	float *voxelBlocks, *minVoxelBlocks;
	Vector3f pt(inpt.x, inpt.y, inpt.z), minpt;
	Vector3f ddt;
	bool minfound = false, ddtfound = false;

	for (int i = 0; i < numObj; i++)
	{
		Vector3f objpt = poses[i].getInvH()*pt;
		idx = pt2IntIdx(objpt);

		if (idx >= 0)
		{
			voxelBlocks = shapes[i].getSDFVoxel();
			partdt = voxelBlocks[idx];

			if (partdt < dt)
			{
				minidx = i;
				minpt = objpt;
				minfound = true;
				minVoxelBlocks = voxelBlocks;

				dt = partdt;
			}
		}
	}
	if (!minfound) return false;

	ddt = getSDFNormal(minpt, minVoxelBlocks, ddtfound);
	if (!ddtfound) return false;

	float exp_dt = expf(-dt * DTUNE);
	float deto = exp_dt + 1.0f;
	float dbase = exp_dt / (deto * deto);

	float d_heaviside_dt = dbase * DTUNE;
	float d_delta_dt = 8.0f *DTUNE* expf(-2 * DTUNE*dt) / (deto * deto* deto) - 4 * DTUNE * dbase;

	float prefix = inpt.w*d_delta_dt*TMP_WEIGHT + (1 - inpt.w)*d_heaviside_dt*(2-TMP_WEIGHT);

	ddt *= prefix;

	for (int i = 0; i < numObj * 6; i++) jacobian[i] = 0;
	int idxoffset = minidx * 6;

	jacobian[idxoffset + 0] = ddt.x;
	jacobian[idxoffset + 1] = ddt.y;
	jacobian[idxoffset + 2] = ddt.z;
	jacobian[idxoffset + 3] = 4.0f * (ddt.z * minpt.y - ddt.y * minpt.z);
	jacobian[idxoffset + 4] = 4.0f * (ddt.x * minpt.z - ddt.z * minpt.x);
	jacobian[idxoffset + 5] = 4.0f * (ddt.y * minpt.x - ddt.x * minpt.y);

	return true;
}

// inpt now is in camera coordinates, it need to be transformed by pose invH to object coordinates
// inpt is also been properly scaled to math the voxel resolution
// inpt.w is pf for the point
_CPU_AND_GPU_CODE_ inline bool computePerPixelJacobian(float *jacobian, const Vector4f &inpt, LibISR::Objects::ISRShape_ptr shapes, LibISR::Objects::ISRPose_ptr poses, int numObj, float& prefix)
{
	if (inpt.w < 0) return false;

	float dt = MAX_SDF, partdt = MAX_SDF;
	int idx, minidx;
	float *voxelBlocks, *minVoxelBlocks;
	Vector3f pt(inpt.x, inpt.y, inpt.z), minpt;
	Vector3f ddt;
	bool minfound = false, ddtfound = false;

	for (int i = 0; i < numObj; i++)
	{
		Vector3f objpt = poses[i].getInvH()*pt;
		idx = pt2IntIdx(objpt);

		if (idx >= 0)
		{
			voxelBlocks = shapes[i].getSDFVoxel();
			partdt = voxelBlocks[idx];

			if (partdt < dt)
			{
				minidx = i;
				minpt = objpt;
				minfound = true;
				minVoxelBlocks = voxelBlocks;

				dt = partdt;
			}
		}
	}
	if (!minfound) return false;

	ddt = getSDFNormal(minpt, minVoxelBlocks, ddtfound);
	if (!ddtfound) return false;

	float exp_dt = expf(-dt * DTUNE);
	float deto = exp_dt + 1.0f;
	float dbase = exp_dt / (deto * deto);

	float d_heaviside_dt = dbase * DTUNE;
	float d_delta_dt = 8.0f *DTUNE* expf(-2 * DTUNE*dt) / (deto * deto* deto) - 4 * DTUNE * dbase;

	prefix = inpt.w*d_delta_dt*TMP_WEIGHT + (1 - inpt.w)*d_heaviside_dt*(2 - TMP_WEIGHT);

	for (int i = 0; i < numObj * 6; i++) jacobian[i] = 0;
	int idxoffset = minidx * 6;

	jacobian[idxoffset + 0] = ddt.x;
	jacobian[idxoffset + 1] = ddt.y;
	jacobian[idxoffset + 2] = ddt.z;
	jacobian[idxoffset + 3] = 4.0f * (ddt.z * minpt.y - ddt.y * minpt.z);
	jacobian[idxoffset + 4] = 4.0f * (ddt.x * minpt.z - ddt.z * minpt.x);
	jacobian[idxoffset + 5] = 4.0f * (ddt.y * minpt.x - ddt.x * minpt.y);

	return true;
}


// inpt now is in camera coordinates, it need to be transformed by pose invH to object coordinates
// inpt is also been properly scaled to math the voxel resolution
// inpt.w is pf for the point
_CPU_AND_GPU_CODE_ inline float findPerPixelDT(const Vector4f &inpt, LibISR::Objects::ISRShape_ptr shapes, LibISR::Objects::ISRPose_ptr poses, int numObj)
{
	if (inpt.w > 0)
	{
		float dt = MAX_SDF, partdt = MAX_SDF;
		int idx;
		float *voxelBlocks;

		for (int i = 0; i < numObj; i++)
		{
			Vector3f objpt = poses[i].getInvH()*Vector3f(inpt.x, inpt.y, inpt.z);
			idx = pt2IntIdx(objpt);
			if (idx >= 0)
			{
				voxelBlocks = shapes[i].getSDFVoxel();
				partdt = voxelBlocks[idx];
				dt = partdt < dt ? partdt : dt; // now use a hard min to approximate
			}
		}
		return dt;
	}
	else return MAX_SDF;
}
