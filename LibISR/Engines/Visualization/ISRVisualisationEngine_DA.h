#pragma once
#include "../../Utils/LibISRDefine.h"


#ifndef LIBISR_VOXEL_ACCESS
#define LIBISR_VOXEL_ACCESS

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

#endif


_CPU_AND_GPU_CODE_ inline bool castRay(Vector3f &pt_out, int x, int y, const float* voxelData, const Matrix4f& invH, const Vector4f& invIntrinsic, const Vector2f& maxminvalue, float maxvoxelrange)
{
	Vector3f pt_camera_f, pt_block_s, pt_block_e, rayDirection, pt_result;
	bool pt_found, inside_volume;
	float sdfValue = MAX_SDF;
	float totalLength=0, stepLength, totalLengthMax;
	int voxidx;

	float step_coarse, step_fine;

	pt_camera_f.z = maxminvalue.x; // min
	pt_camera_f.x = pt_camera_f.z * ((float(x) *invIntrinsic.x + invIntrinsic.z));
	pt_camera_f.y = pt_camera_f.z * ((float(x) *invIntrinsic.y + invIntrinsic.w));
	pt_block_s = (invH * pt_camera_f);

	pt_camera_f.z = maxminvalue.y; // max
	pt_camera_f.x = pt_camera_f.z * ((float(x) *invIntrinsic.x + invIntrinsic.z));
	pt_camera_f.y = pt_camera_f.z * ((float(x) *invIntrinsic.y + invIntrinsic.w));
	pt_block_e = (invH * pt_camera_f);

	totalLengthMax = length(pt_block_e - pt_block_s);
	rayDirection = (pt_block_e - pt_block_s).normalised();
	pt_result = pt_block_s;

	step_fine = totalLengthMax * maxvoxelrange;
	step_coarse = step_fine * 10;

	pt_found = false;

	while (totalLength<=totalLengthMax)
	{
		voxidx = pt2IntIdx(pt_result);
		inside_volume = voxidx >= 0;
		if (inside_volume)
		{
			sdfValue = voxelData[voxidx];
			if (fabs(sdfValue)<=1.0f)
			{
				pt_found = true;
				break;
			}
			else stepLength = sdfValue*step_fine;
		}
		else
		{
			stepLength = step_coarse;
		}

		pt_result += stepLength * rayDirection; 
		totalLength += stepLength;
	}

	pt_out = pt_result;
	return pt_found;
}

_CPU_AND_GPU_CODE_ inline void raycastAndRender(Vector4u *outImg, int x, int y, const Vector2i& imagesize, const float* voxelData, const Matrix4f& invH, const Vector4f& invIntrinsic, const Vector2f *minmaxdata, const Vector3f& lightsource, float maxvoxelrange)
{
	Vector3f pt_obj;
	Vector3f normal_obj;

	float angle;

	int idx = x + y*imagesize.x;

	Vector2f minmaxvalue = minmaxdata[idx];
	castRay(pt_obj, x, y, voxelData, invH, invIntrinsic, minmaxvalue,maxvoxelrange);
}