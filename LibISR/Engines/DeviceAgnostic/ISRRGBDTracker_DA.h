#pragma once

#include "../../Utils/LibISRDefine.h"

#include "../../Objects/ISRTrackingState.h"
#include "../../Objects/ISRShapeUnion.h"

_CPU_AND_GPU_CODE_ inline int pt2IntIdx(Vector3f pt)
{
	int x = pt.x * VOL_SCALE + DT_VOL_SIZE / 2 - 1;
	int y = pt.y * VOL_SCALE + DT_VOL_SIZE / 2 - 1;
	int z = pt.z * VOL_SCALE + DT_VOL_SIZE / 2 - 1;

	if (x > 0 && x < DT_VOL_SIZE - 1 &&
		y > 0 && y < DT_VOL_SIZE - 1 &&
		z > 0 && z < DT_VOL_SIZE - 1)
		return (z * DT_VOL_SIZE + y) * DT_VOL_SIZE + x;
	else
		return -1;
}

_CPU_AND_GPU_CODE_ inline int pt2IntIdx_offset(Vector3f pt, Vector3i offpt)
{
	int x = pt.x * VOL_SCALE + DT_VOL_SIZE / 2 - 1 + offpt.x;
	int y = pt.y * VOL_SCALE + DT_VOL_SIZE / 2 - 1 + offpt.y;
	int z = pt.z * VOL_SCALE + DT_VOL_SIZE / 2 - 1 + offpt.z;

	if (x > 0 && x < DT_VOL_SIZE - 1 &&
		y > 0 && y < DT_VOL_SIZE - 1 &&
		z > 0 && z < DT_VOL_SIZE - 1)
		return (z * DT_VOL_SIZE + y) * DT_VOL_SIZE + x;
	else
		return -1;
}

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

// this pt_f is already in the object coordinates
_CPU_AND_GPU_CODE_ inline Vector3f computeDDT(const Vector3f &pt_f, float* voxelBlock,  bool &ddtFound)
{
	Vector3f ddt;

	bool isFound; float dt1, dt2;
	int idx;

	idx = pt2IntIdx_offset(pt_f, Vector3i(1, 0, 0)); if (idx == -1) { ddtFound = false; return Vector3f(0.0f); }
	dt1 = voxelBlock[idx];
	idx = pt2IntIdx_offset(pt_f, Vector3i(-1, 0, 0)); if (idx == -1) { ddtFound = false; return Vector3f(0.0f); }
	dt2 = voxelBlock[idx];
	ddt.x = (dt1 - dt2)*0.5f;

	idx = pt2IntIdx_offset(pt_f, Vector3i(0, 1, 0)); if (idx == -1) { ddtFound = false; return Vector3f(0.0f); }
	dt1 = voxelBlock[idx];
	idx = pt2IntIdx_offset(pt_f, Vector3i(0, -1, 0)); if (idx == -1) { ddtFound = false; return Vector3f(0.0f); }
	dt2 = voxelBlock[idx];
	ddt.y = (dt1 - dt2)*0.5f;

	idx = pt2IntIdx_offset(pt_f, Vector3i(0, 0, 1)); if (idx == -1) { ddtFound = false; return Vector3f(0.0f); }
	dt1 = voxelBlock[idx];
	idx = pt2IntIdx_offset(pt_f, Vector3i(0, 0, -1)); if (idx == -1) { ddtFound = false; return Vector3f(0.0f); }
	dt2 = voxelBlock[idx];
	ddt.z = (dt1 - dt2)*0.5f;

	ddtFound = true; return ddt;
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

	ddt = computeDDT(minpt, minVoxelBlocks, ddtfound);
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