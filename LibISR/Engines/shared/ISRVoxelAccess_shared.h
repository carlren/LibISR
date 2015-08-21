// Copyright 2014-2015 Isis Innovation Limited and the authors of LibISR

#pragma once
#include "../../LibISRDefine.h"

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

// this pt_f is already in the object coordinates
_CPU_AND_GPU_CODE_ inline Vector3f getSDFNormal(const Vector3f &pt_f, const float* voxelBlock, bool &ddtFound)
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

// this pt_f is already in the object coordinates
_CPU_AND_GPU_CODE_ inline float getSDFValue(const Vector3f &pt_f, const float* voxelBlock, bool &ddtFound)
{
	int idx = pt2IntIdx(pt_f); 
	if (idx == -1) 
		{ ddtFound = false; return MAX_SDF; }
	else{ ddtFound = true; return voxelBlock[idx]; }
}

