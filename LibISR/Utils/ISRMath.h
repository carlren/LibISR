#pragma once
#include <ostream>
#include <stdlib.h>
#include "LibISRDefine.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Legacy from old version of LibISR
//////////////////////////////////////////////////////////////////////////////////////////////////////////

static inline void GetRotationMatrixFromMRP(float *outR, float* r)
{
	float t1 = r[0];
	float t2 = r[1];
	float t3 = r[2];

	float tsq = t1*t1 + t2*t2 + t3*t3;

	float tsum = 1 - tsq;

	outR[0] = 4 * t1*t1 - 4 * t2*t2 - 4 * t3*t3 + tsum*tsum;	outR[1] = 8 * t1*t2 - 4 * t3*tsum;	outR[2] = 8 * t1*t3 + 4 * t2*tsum;
	outR[3] = 8 * t1*t2 + 4 * t3*tsum;	outR[4] = 4 * t2*t2 - 4 * t1*t1 - 4 * t3*t3 + tsum*tsum;	outR[5] = 8 * t2*t3 - 4 * t1*tsum;
	outR[6] = 8 * t1*t3 - 4 * t2*tsum;	outR[7] = 8 * t2*t3 + 4 * t1*tsum;	outR[8] = 4 * t3*t3 - 4 * t2*t2 - 4 * t1*t1 + tsum*tsum;

	for (int i = 0; i<9; i++) outR[i] /= ((1 + tsq)*(1 + tsq));
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Math functions for Heaviside and Delta functions
//////////////////////////////////////////////////////////////////////////////////////////////////////////

_CPU_AND_GPU_CODE_ inline float sHeaviside(float x, float heavisideScale)
{
	return 1.0f / (expf(-x / heavisideScale) + 1.0f);
}

_CPU_AND_GPU_CODE_ inline float sDelta(float x, float heavisideScale)
{
	float exp_dt = expf(-x / heavisideScale);
	float deto = exp_dt + 1.0f;
	return 4.0f* exp_dt / (deto * deto);
}

_CPU_AND_GPU_CODE_ inline float sdHeaviside(float x, float heavisideScale)
{
	float exp_dt = expf(-x / heavisideScale);
	float deto = exp_dt + 1.0f;
	return exp_dt / (deto*deto) / heavisideScale;
}

_CPU_AND_GPU_CODE_ inline float sdDelta(float x, float heavisideScale)
{
	float exp_dt = expf(-x / heavisideScale);
	float deto = exp_dt + 1.0f;
	return 4.0f* exp_dt / (deto * deto);
}


_CPU_AND_GPU_CODE_ inline bool almostZero(float sum)
{
	return(sum*sum<1.0e-25);
}

static inline float fast_log(float val)
{
	int* const exp_ptr = reinterpret_cast<int *> (&val);
	int x = *exp_ptr;
	const int log_2 = ((x >> 23) & 255) - 128;
	x &= ~(255 << 23);
	x += 127 << 23;
	*exp_ptr = x;

	val = ((-1.0f / 3) * val + 2) * val - 2.0f / 3;

	return (val + log_2) * 0.69314718f;
}