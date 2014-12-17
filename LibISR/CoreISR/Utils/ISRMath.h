#pragma once
#include <ostream>
#include <stdlib.h>

typedef unsigned char uchar;
typedef unsigned short ushort;
typedef unsigned int uint;
typedef unsigned long ulong;

#ifndef MIN
#define MIN(a,b) ((a < b) ? a : b)
#endif

#ifndef MAX
#define MAX(a,b) ((a < b) ? b : a)
#endif

#ifndef ABS
#define ABS(a) ((a < 0) ? -a : a)
#endif

#ifndef CLAMP
#define CLAMP(x,a,b) MAX((a), MIN((b), (x)))
#endif

#ifndef ROUND
#define ROUND(x) ((x < 0) ? (x - 0.5f) : (x + 0.5f))
#endif

#ifndef PI
#define PI float(3.1415926535897932384626433832795)
#endif

#ifndef DEGTORAD
#define DEGTORAD float(0.017453292519943295769236907684886)
#endif

#ifndef MY_INF
#define MY_INF 0x7f800000
#endif

#ifndef NULL
#define NULL 0
#endif

#include "ISRVector.h"
#include "ISRMatrix.h"

typedef class CoreISR::Matrix3<float> Matrix3f;
typedef class CoreISR::Matrix4<float> Matrix4f;

typedef class CoreISR::Vector2<short> Vector2s;
typedef class CoreISR::Vector2<int> Vector2i;
typedef class CoreISR::Vector2<float> Vector2f;
typedef class CoreISR::Vector2<double> Vector2d;

typedef class CoreISR::Vector3<short> Vector3s;
typedef class CoreISR::Vector3<double> Vector3d;
typedef class CoreISR::Vector3<int> Vector3i;
typedef class CoreISR::Vector3<uint> Vector3ui;
typedef class CoreISR::Vector3<uchar> Vector3u;
typedef class CoreISR::Vector3<float> Vector3f;

typedef class CoreISR::Vector4<float> Vector4f;
typedef class CoreISR::Vector4<int> Vector4i;
typedef class CoreISR::Vector4<short> Vector4s;
typedef class CoreISR::Vector4<uchar> Vector4u;

inline bool portable_finite(float a)
{
	volatile float temp = a;
	if (temp != a) return false;
	if ((temp - a) != 0.0) return false;
	return true;
}

inline void matmul(const float *A, const float *b, float *x, int numRows, int numCols)
{
	for (int r = 0; r < numRows; ++r)
	{
		float res = 0.0f;
		for (int c = 0; c < numCols; ++c) res += A[r*numCols + c] * b[c];
		x[r] = res;
	}
}


///////////////////// some legacy from ISR ///////////////////////////

static inline void AddToDiagonal(float *dst, float *src, float lambda, float alpha, int dim)
{
	for (int i = 0; i<dim * dim; i++) dst[i] = src[i];
	for (int i = 0; i<dim; i++) dst[i + i * dim] += (lambda * (src[i + i * dim]) + 0.01f) * alpha;
}

static inline void ISRInvMatrix3(float* mIn, float* mOut)
{

	float a00 = mIn[0], a01 = mIn[1], a02 = mIn[2];
	float a10 = mIn[3], a11 = mIn[4], a12 = mIn[5];
	float a20 = mIn[6], a21 = mIn[7], a22 = mIn[8];

	float t2 = a00*a11*a22;
	float t3 = a01*a12*a20;
	float t4 = a02*a10*a21;
	float t7 = a00*a12*a21;
	float t8 = a01*a10*a22;
	float t9 = a02*a11*a20;
	float t5 = t2 + t3 + t4 - t7 - t8 - t9;
	float t6 = 1.0f / t5;

	float reM[9] = { t6*(a11*a22 - a12*a21), -t6*(a01*a22 - a02*a21), t6*(a01*a12 - a02*a11),
		-t6*(a10*a22 - a12*a20), t6*(a00*a22 - a02*a20), -t6*(a00*a12 - a02*a10),
		t6*(a10*a21 - a11*a20), -t6*(a00*a21 - a01*a20), t6*(a00*a11 - a01*a10) };

	memcpy(mOut, reM, 9 * sizeof(float));
}

static inline void UnimindMatrixMul3(float* m1, float* m2, float* mout)
{
	mout[0] = m1[0] * m2[0] + m1[1] * m2[3] + m1[2] * m2[6];
	mout[1] = m1[0] * m2[1] + m1[1] * m2[4] + m1[2] * m2[7];
	mout[2] = m1[0] * m2[2] + m1[1] * m2[5] + m1[2] * m2[8];

	mout[3] = m1[3] * m2[0] + m1[4] * m2[3] + m1[5] * m2[6];
	mout[4] = m1[3] * m2[1] + m1[4] * m2[4] + m1[5] * m2[7];
	mout[5] = m1[3] * m2[2] + m1[4] * m2[5] + m1[5] * m2[8];

	mout[6] = m1[6] * m2[0] + m1[7] * m2[3] + m1[8] * m2[6];
	mout[7] = m1[6] * m2[1] + m1[7] * m2[4] + m1[8] * m2[7];
	mout[8] = m1[6] * m2[2] + m1[7] * m2[5] + m1[8] * m2[8];
}

static inline void UnimindUpdateRT(float* oldR, float* oldT, float* dR, float* dT)
{
	float newR[9];
	float newT[3];

	UnimindMatrixMul3(dR, oldR, newR);

	newT[0] = dR[0] * oldT[0] + dR[1] * oldT[1] + dR[2] * oldT[2] + dT[0];
	newT[1] = dR[3] * oldT[0] + dR[4] * oldT[1] + dR[5] * oldT[2] + dT[1];
	newT[2] = dR[6] * oldT[0] + dR[7] * oldT[1] + dR[8] * oldT[2] + dT[2];

	memcpy(oldR, newR, 9 * sizeof(float));
	memcpy(oldT, newT, 3 * sizeof(float));
}

static inline void UnimindInverseRTtoH(float* R, float* T, float* outH)
{
	float invR[9];

	ISRInvMatrix3(R, invR);

	int idx = 0;

	for (int i = 0; i<3; i++){
		for (int j = 0; j<3; j++, idx++){
			outH[i * 4 + j] = invR[idx];
		}
	}

	outH[3] = -(invR[0] * T[0] + invR[1] * T[1] + invR[2] * T[2]);
	outH[7] = -(invR[3] * T[0] + invR[4] * T[1] + invR[5] * T[2]);
	outH[11] = -(invR[6] * T[0] + invR[7] * T[1] + invR[8] * T[2]);
}

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
// some new function for ISR
//////////////////////////////////////////////////////////////////////////////////////////////////////////

static inline Vector2d ProjectPtToImg(Matrix3f *A, Matrix4f *H, Vector3f *pt)
{
	Vector3f camPt;
	Vector3f imgPt;
	Vector2d retPt;

	camPt.x = H->m00 * pt->x + H->m01 * pt->y + H->m02 * pt->z + H->m03;
	camPt.y = H->m10 * pt->x + H->m11 * pt->y + H->m12 * pt->z + H->m13;
	camPt.z = H->m20 * pt->x + H->m21 * pt->y + H->m22 * pt->z + H->m23;

	imgPt.x = A->m00 * camPt.x + A->m01 * camPt.y + A->m02 * camPt.z;
	imgPt.y = A->m10 * camPt.x + A->m11 * camPt.y + A->m12 * camPt.z;
	imgPt.z = A->m20 * camPt.x + A->m21 * camPt.y + A->m22 * camPt.z;

	retPt.x = (int)(imgPt.x / imgPt.z);
	retPt.y = (int)(imgPt.y / imgPt.z);

	return retPt;
}

static inline float sHeaviside(float x, float heavisideScale)
{
	float exp_dt = expf(-x / heavisideScale);
	return 1.0f / (exp_dt + 1.0f);
}

static inline float sDelta(float x, float heavisideScale)
{
	float exp_dt = expf(-x / heavisideScale);
	float deto = exp_dt + 1.0f;
	return 4.0f* exp_dt / (deto * deto);
}

static inline float sdHeaviside(float x, float heavisideScale)
{
	float exp_dt = expf(-x / heavisideScale);
	float deto = exp_dt + 1.0f;
	return exp_dt / (deto*deto) / heavisideScale;
}

static inline float sdDelta(float x, float heavisideScale)
{
	float exp_dt = expf(-x / heavisideScale);
	float deto = exp_dt + 1.0f;
	return 4.0f* exp_dt / (deto * deto);
}



static inline bool almostZero(float sum)
{
	return(sum*sum<1.0e-25);
}


static inline void UnimindGetMRPfromDegree(float *outR, float *inR)
{
	float rx = inR[0];
	float ry = inR[1];
	float rz = inR[2];

	double rotationX = rx * DEGTORAD;
	double rotationY = ry * DEGTORAD;
	double rotationZ = rz * DEGTORAD;

	double c1 = cos(rotationY / 2);
	double c2 = cos(rotationZ / 2);
	double c3 = cos(rotationX / 2);

	double s1 = sin(rotationY / 2);
	double s2 = sin(rotationZ / 2);
	double s3 = sin(rotationX / 2);

	double c1c2 = c1 * c2;
	double s1s2 = s1 * s2;

	double rotation[4];

	rotation[0] = c1c2*s3 + s1s2*c3;
	rotation[1] = s1*c2*c3 + c1*s2*s3;
	rotation[2] = c1*s2*c3 - s1*c2*s3;
	rotation[3] = c1c2*c3 - s1s2*s3;

	double norm = 1 / sqrt(rotation[0] * rotation[0] + rotation[1] * rotation[1] + rotation[2] * rotation[2] + rotation[3] * rotation[3]);
	for (int i = 0; i<4; i++) rotation[i] *= norm;

	double b0 = rotation[3];
	double b1 = rotation[0];
	double b2 = rotation[1];
	double b3 = rotation[2];

	outR[0] = (float)(b1 / (1 + b0));
	outR[1] = (float)(b2 / (1 + b0));
	outR[2] = (float)(b3 / (1 + b0));
}

static inline void VectorDiff_n(float *dst, float *a, float *b, int dim) { for (int i = 0; i<dim; i++) dst[i] = a[i] + b[i]; }

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