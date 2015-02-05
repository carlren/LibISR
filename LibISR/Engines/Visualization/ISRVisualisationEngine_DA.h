#pragma once
#include "../../Utils/LibISRDefine.h"
#include "../Lowlevel/ISRVoxelAccess_DA.h"

_CPU_AND_GPU_CODE_ inline bool castRay(Vector3f &pt_out, int x, int y, const float* voxelData, const Matrix4f& invH, const Vector4f& invIntrinsic, const Vector2f& maxminvalue, float maxvoxelrange)
{
	if (maxminvalue.x < 0) return false;
	
	Vector3f pt_camera_f, pt_block_s, pt_block_e, rayDirection, pt_result;
	bool pt_found, inside_volume;
	float sdfValue = MAX_SDF;
	float totalLength=0, stepLength, totalLengthMax;
	int voxidx;

	float step_coarse, step_fine;

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

	step_fine = totalLengthMax * maxvoxelrange;
	step_coarse = step_fine * 10;

	pt_found = false;

	while (totalLength<=totalLengthMax)
	{
		sdfValue = getSDFValue(pt_result, voxelData, inside_volume);
		if (inside_volume)
		{
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

_CPU_AND_GPU_CODE_ inline void computeNormalAndAngle(Vector3f & normal_out, float & angle_out, bool& foundPoint, const Vector3f& pt_in, const float *voxelData, const Vector3f & lightSource)
{
	if (!foundPoint) return;
	normal_out = getSDFNormal(pt_in, voxelData, foundPoint);
	if (!foundPoint) return;

	float normScale = 1.0f / sqrtf(normal_out.x * normal_out.x + normal_out.y * normal_out.y + normal_out.z * normal_out.z);
	normal_out *= normScale;

	angle_out = normal_out.x * lightSource.x + normal_out.y * lightSource.y + normal_out.z * lightSource.z;
	if (!(angle_out > 0.0)) angle_out=0;
}

_CPU_AND_GPU_CODE_ inline void drawRendering(const bool & foundPoint, const float & angle, Vector4u & dest)
{
	if (!foundPoint)
	{
		dest = Vector4u((const uchar&)0);
		return;
	}

	float outRes = (0.6f * angle + 0.4f) * 255.0f;
	dest = Vector4u((uchar)outRes);
}


_CPU_AND_GPU_CODE_ inline void raycastAndRender(Vector4u *outImg, int x, int y, const Vector2i& imagesize, const float* voxelData, const Matrix4f& invH, const Vector4f& invIntrinsic, const Vector2f *minmaxdata, const Vector3f& lightsource, float maxvoxelrange)
{
	Vector3f pt_obj;
	Vector3f normal_obj;

	float angle;

	int idx = x + y*imagesize.x;

	Vector2f minmaxvalue = minmaxdata[idx];
	bool foundpoint = castRay(pt_obj, x, y, voxelData, invH, invIntrinsic, minmaxvalue,maxvoxelrange);
	computeNormalAndAngle(normal_obj, angle, foundpoint, pt_obj, voxelData, lightsource);
	drawRendering(foundpoint, angle, outImg[idx]);
}


_CPU_AND_GPU_CODE_ inline void raycastAndRenderDepth(ushort *outImg, int x, int y, const Vector2i& imagesize, const float* voxelData, const Matrix4f& invH, const Vector4f& invIntrinsic, const Vector2f *minmaxdata, float maxvoxelrange)
{
	Vector3f pt_obj,pt_cam;
	Vector3f normal_obj;

	Matrix4f H, tmpinvH = invH; tmpinvH.inv(H);
	float angle;

	int idx = x + y*imagesize.x;

	Vector2f minmaxvalue = minmaxdata[idx];
	bool foundpoint = castRay(pt_obj, x, y, voxelData, invH, invIntrinsic, minmaxvalue, maxvoxelrange);
	

	if (foundpoint)
	{
		pt_cam = H*pt_obj;
		outImg[idx] = (ushort)(65535*pt_cam.z);
	}	
	else outImg[idx] = 0;
}

_CPU_AND_GPU_CODE_ inline void raycastAndRenderWithDepthAndSurfaceNormal(ushort *outImgD, Vector4u *outImgGray, Vector4u* outImgNormal, int x, int y, const Vector2i& imagesize, const float* voxelData, const Matrix4f& invH, const Vector4f& invIntrinsic, const Vector2f *minmaxdata, const Vector3f& lightsource, float maxvoxelrange)
{
	Vector3f pt_obj, pt_cam;
	Vector3f normal_obj;

	Matrix4f H, tmpinvH = invH; tmpinvH.inv(H);
	float angle;

	int idx = x + y*imagesize.x;

	Vector2f minmaxvalue = minmaxdata[idx];
	bool foundpoint = castRay(pt_obj, x, y, voxelData, invH, invIntrinsic, minmaxvalue, maxvoxelrange);
	
	if (foundpoint)
	{
		pt_cam = H*pt_obj;
		outImgD[idx] = (ushort)(65535 * pt_cam.z);
	}
	else outImgD[idx] = 0;

	computeNormalAndAngle(normal_obj, angle, foundpoint, pt_obj, voxelData, lightsource);
		
	if (foundpoint)
	{
		outImgNormal[idx].r = (uchar)((0.4 + normal_obj.r*0.6)*255.0f);
		outImgNormal[idx].g = (uchar)((0.4 + normal_obj.g*0.6)*255.0f);
		outImgNormal[idx].b = (uchar)((0.4 + normal_obj.b*0.6)*255.0f);
	}
	else outImgNormal[idx] = Vector4u((uchar)0);

	drawRendering(foundpoint, angle, outImgGray[idx]);
}
