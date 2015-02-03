#pragma once

#include "../../Utils/LibISRDefine.h"
#include "../Lowlevel/ISRVoxelAccess_DA.h"


#include "../../Objects//Highlevel/ISRTrackingState.h"
#include "../../Objects/Highlevel/ISRShapeUnion.h"


// inpt now is in camera coordinates, it need to be transformed by pose invH to object coordinates
// inpt is also been properly scaled to math the voxel resolution
// inpt.w is pf for the point
_CPU_AND_GPU_CODE_ inline float computePerPixelEnergy(const Vector4f &inpt, LibISR::Objects::ISRShapeUnion* shapeunion, LibISR::Objects::ISRTrackingState* state)
{
	if (inpt.w > 0)
	{
		float dt = MAX_SDF, partdt=MAX_SDF;
		int numObj = state->numPoses();
		int idx;
		float *voxelBlocks;

		for (int i = 0; i < numObj; i++)
		{
			Vector3f objpt = state->getPose(i)->getInvH()*Vector3f(inpt.x, inpt.y, inpt.z);
			idx = pt2IntIdx(objpt);
			if (idx >= 0)
			{
				voxelBlocks = shapeunion->getShape(i)->getSDFVoxel();
				partdt = voxelBlocks[idx];
				dt = partdt < dt ? partdt : dt; // now use a hard min to approximate
			}
		}

		float exp_dt = expf(-dt * DTUNE);
		float deto = exp_dt + 1.0f;
		float sheaviside = 1.0f / deto;
		float sdelta = 4.0f* exp_dt * sheaviside * sheaviside;
		float e = inpt.w * sdelta + (1 - inpt.w)*sheaviside;
		return e;
	}
	else return 0.0f;
}


// inpt now is in camera coordinates, it need to be transformed by pose invH to object coordinates
// inpt is also been properly scaled to math the voxel resolution
// inpt.w is pf for the point
_CPU_AND_GPU_CODE_ inline bool computePerPixelJacobian(float *jacobian, const Vector4f &inpt, LibISR::Objects::ISRShapeUnion* shapeunion, LibISR::Objects::ISRTrackingState* state)
{
	if (inpt.w < 0) return false;
	
	float dt = MAX_SDF, partdt = MAX_SDF;
	int numObj = state->numPoses();
	int idx, minidx;
	float *voxelBlocks, *minVoxelBlocks;
	Vector3f pt(inpt.x, inpt.y, inpt.z), minpt;
	Vector3f ddt;
	bool minfound = false, ddtfound=false;

	for (int i = 0; i < numObj; i++)
	{
		Vector3f objpt = state->getPose(i)->getInvH()*pt;
		idx = pt2IntIdx(objpt);

		if (idx>=0)
		{
			voxelBlocks = shapeunion->getShape(i)->getSDFVoxel();
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
	float d_delta_dt = 4.0f * expf(-dt) / (deto * deto* deto) - 2 * dbase;

	float prefix = inpt.w*d_delta_dt + (1 - inpt.w)*d_heaviside_dt;

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
_CPU_AND_GPU_CODE_ inline float findPerPixelDT(const Vector4f &inpt, LibISR::Objects::ISRShapeUnion* shapeunion, LibISR::Objects::ISRTrackingState* state)
{
	if (inpt.w > 0)
	{
		float dt = MAX_SDF, partdt = MAX_SDF;
		int numObj = state->numPoses();
		int idx;
		float *voxelBlocks;

		for (int i = 0; i < numObj; i++)
		{
			Vector3f objpt = state->getPose(i)->getInvH()*Vector3f(inpt.x, inpt.y, inpt.z);
			idx = pt2IntIdx(objpt);
			if (idx >= 0)
			{
				voxelBlocks = shapeunion->getShape(i)->getSDFVoxel();
				partdt = voxelBlocks[idx];
				dt = partdt < dt ? partdt : dt; // now use a hard min to approximate
			}
		}
		return dt;
	}
	else return 0.0f;
}