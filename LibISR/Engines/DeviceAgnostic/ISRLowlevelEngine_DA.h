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

_CPU_AND_GPU_CODE_ inline float getPf(const Vector4u &pixel, const LibISR::Objects::ISRHistogram* histogram)
{
	int noBins = histogram->noBins;
	int dim = histogram->dim;

	int ru = pixel.r / noBins;
	int gu = pixel.g / noBins;
	int bu = pixel.b / noBins;
	int pidx = ru*noBins*noBins + gu * noBins + bu;

	return histogram->posterior[pidx];
}