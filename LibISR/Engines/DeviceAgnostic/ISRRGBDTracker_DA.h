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

// inpt now is in camera coordinates, it need to be transformed by pose invH to object coordinates
// inpt is also been properly scaled to math the voxel resolution
// inpt.w is pf for the point
_CPU_AND_GPU_CODE_ inline float computePerPixelEnergy(const Vector4f &inpt, LibISR::Objects::ISRShapeUnion* shapeunion, LibISR::Objects::ISRTrackingState* state)
{
	float dt=MAX_SDF, partdt;
	int numObj = state->numPoses();
	int idx;
	float *voxelBlocks;
	Vector3f pt(inpt.x, inpt.y, inpt.z);

	for (int i = 0; i < numObj; i++)
	{
		Vector3f objpt = state->getPose(i)->getInvH()*pt;
		idx = pt2IntIdx(objpt);
		if (idx>=0)
		{
			voxelBlocks = shapeunion->getShape(i)->getSDFVoxel();
			partdt = idx>-1 ? voxelBlocks[idx] : MAX_SDF;
			dt = partdt < dt ? partdt : dt; // now use a hard min to approximate
		}
	}

	float exp_dt = expf(-dt * DTUNE);
	float deto = exp_dt + 1.0f;
	float sheaviside = 1.0f / deto;
	float sdelta = 4.0f* exp_dt * sheaviside * sheaviside;

	return inpt.w * sdelta + (1 - inpt.w)*sheaviside;
}

// this pt_f is already in the object coordinates
_CPU_AND_GPU_CODE_ inline Vector3f computeDDT(const Vector3f &pt_f, float* voxelBlock,  bool &ddtFound)
{
	Vector3f ddt;

	bool isFound; float dt1, dt2;
	int idx;

	idx = pt2IntIdx(pt_f + Vector3f(1, 0, 0)); if (idx == -1) { ddtFound = false; return Vector3f(0.0f); }
	dt1 = voxelBlock[idx];
	idx = pt2IntIdx(pt_f + Vector3f(-1, 0, 0)); if (idx == -1) { ddtFound = false; return Vector3f(0.0f); }
	dt2 = voxelBlock[idx];
	ddt.x = (dt1 - dt2)*0.5f;

	idx = pt2IntIdx(pt_f + Vector3f(0, 1, 0)); if (idx == -1) { ddtFound = false; return Vector3f(0.0f); }
	dt1 = voxelBlock[idx];
	idx = pt2IntIdx(pt_f + Vector3f(0, -1, 0)); if (idx == -1) { ddtFound = false; return Vector3f(0.0f); }
	dt2 = voxelBlock[idx];
	ddt.y = (dt1 - dt2)*0.5f;

	idx = pt2IntIdx(pt_f + Vector3f(0, 0, -1)); if (idx == -1) { ddtFound = false; return Vector3f(0.0f); }
	dt1 = voxelBlock[idx];
	idx = pt2IntIdx(pt_f + Vector3f(0, 0, -1)); if (idx == -1) { ddtFound = false; return Vector3f(0.0f); }
	dt2 = voxelBlock[idx];
	ddt.z = (dt1 - dt2)*0.5f;

	ddtFound = true; return ddt;
}

// inpt now is in camera coordinates, it need to be transformed by pose invH to object coordinates
// inpt is also been properly scaled to math the voxel resolution
// inpt.w is pf for the point
_CPU_AND_GPU_CODE_ inline bool computePerPixelJacobian(float *jacobian, const Vector4f &inpt, LibISR::Objects::ISRShapeUnion* shapeunion, LibISR::Objects::ISRTrackingState* state)
{
	float dt = MAX_SDF, partdt;
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
		voxelBlocks = shapeunion->getShape(i)->getSDFVoxel();
		partdt = idx>-1 ? voxelBlocks[idx] : MAX_SDF;
		if (partdt < dt){ minpt = objpt; minVoxelBlocks = voxelBlocks; dt = partdt; minfound = true; }
	}

	if (!minfound) return false;
	computeDDT(ddt, minVoxelBlocks, ddtfound);
	if (!ddtfound) return false;

	float exp_dt = expf(-dt * DTUNE);
	float deto = exp_dt + 1.0f;
	float dbase = exp_dt / (deto * deto);

	float d_heaviside_dt = dbase * DTUNE;
	float d_delta_dt = 4.0f * expf(-dt) / (deto * deto* deto) - 2 * dbase;

	float prefix = inpt.w*d_delta_dt + (1 - inpt.w)*d_heaviside_dt;

	ddt *= prefix;

	jacobian[0] = ddt.x; jacobian[1] = ddt.y; jacobian[2] = ddt.z;

	jacobian[3] = 4.0f * (ddt.z * minpt.y - ddt.y * minpt.z);
	jacobian[4] = 4.0f * (ddt.x * minpt.z - ddt.z * minpt.x);
	jacobian[5] = 4.0f * (ddt.y * minpt.x - ddt.x * minpt.y);

	return true;
}