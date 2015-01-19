#pragma once

#include "../../Utils/LibISRDefine.h"

#include "../ISRLowlevelEngine.h"

_CPU_AND_GPU_CODE_ inline void unprojectPtWithIntrinsic(const Vector4f &intrinsic, const Vector3f &inpt, Vector4f &outpt)
{
	outpt.x = (inpt.x - inpt.z*intrinsic.z) / intrinsic.x;
	outpt.y = (inpt.y - inpt.z*intrinsic.w) / intrinsic.y;
	outpt.z = inpt.z;
	outpt.w = 1.0f;
}


_CPU_AND_GPU_CODE_ inline float getPf(const Vector4u &pixel, float* histogram, int noBins)
{
	int dim = noBins*noBins*noBins;

	int ru = pixel.r / noBins;
	int gu = pixel.g / noBins;
	int bu = pixel.b / noBins;
	int pidx = ru*noBins*noBins + gu * noBins + bu;

	return histogram[pidx];
}

_CPU_AND_GPU_CODE_ inline void preparePtCouldDataAllInOne
(Vector4f &ptcloud_out, const Vector3f &inpt, const Vector4u *rgb_in, 
const Vector2i imgSize, const Vector4f &intrinsic,
const Matrix3f &H, const Vector3f &T, 
float* histogram, int noBins)
{
	if (inpt.z > 0)
	{
		Vector3f imgPt = H*inpt + T;
		int ix = (int)(imgPt.x / imgPt.z);
		int iy = (int)(imgPt.y / imgPt.z);

		if (ix >= 0 && ix < imgSize.x && iy >= 0 && imgSize.y)
		{
			unprojectPtWithIntrinsic(intrinsic, imgPt, ptcloud_out);
			ptcloud_out.w = getPf(rgb_in[iy * imgSize.x + ix], histogram, noBins);
			return;
		}
	}
	ptcloud_out.x = ptcloud_out.y = ptcloud_out.z = 0; ptcloud_out.w = -1;
}

_CPU_AND_GPU_CODE_ inline void mapRGBDtoRGB(Vector4u &rgb_out, const Vector3f &inpt, const Vector4u *rgb_in, const Vector2i imgSize, const Matrix3f &H, const Vector3f &T)
{
	if (inpt.z>0)
	{
		Vector3f imgPt = H*inpt + T;
		int ix = (int)(imgPt.x / imgPt.z);
		int iy = (int)(imgPt.y / imgPt.z);

		if (ix >= 0 && ix < imgSize.x && iy >= 0 && imgSize.y)
		{
			rgb_out = rgb_in[iy * imgSize.x + ix];
			return;
		}
	}
	rgb_out.r = rgb_out.g = rgb_out.b = 0;
}