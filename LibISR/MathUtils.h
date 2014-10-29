#ifndef __MATH_UTILS__
#define __MATH_UTILS__

#include <math.h>
#include <memory.h>
#include <stdio.h>

#ifndef MIN
#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef DEGTORAD
#define DEGTORAD (float)(0.017453292519943295769236907684886)
#endif

/// Matrix Types
union _Matrix2f
{
    struct
    {
        float m00, m01;
        float m10, m11;
    };
    float m2[2][2];
    float m[4];
};
typedef union _Matrix2f Matrix2f;

union _Matrix3f
{
    struct
    {
        float m00, m01, m02;
        float m10, m11, m12;
        float m20, m21, m22;
    };
    float m[9];
};
typedef union _Matrix3f Matrix3f;

union _Matrix4f
{
    struct
    {
        float m00, m01, m02, m03;
        float m10, m11, m12, m13;
        float m20, m21, m22, m23;
        float m30, m31, m32, m33;
    };
    float m[16];
};
typedef union _Matrix4f Matrix4f;

/// Vector Types
union _Vector2i
{
    struct { int x, y; };
    struct { int s, t; };
    int v[2];
};
typedef union _Vector2i Vector2i;

union _Vector2f
{
    struct { float x, y; };
    struct { float s, t; };
    float v[2];
};
typedef union _Vector2f Vector2f;

union _Vector2d
{
    struct { double x, y; };
    struct { double s, t; };
    double v[2];
};
typedef union _Vector2d Vector2d;

union _Vector3u
{
    struct { unsigned char x, y, z; };
    struct { unsigned char r, g, b; };
    struct { unsigned char s, t, p; };
    unsigned char v[3];
};
typedef union _Vector3u Vector3u;

union _Vector3f
{
    struct { float x, y, z; };
    struct { float r, g, b; };
    struct { float s, t, p; };
    float v[3];
};
typedef union _Vector3f Vector3f;

union _Vector3d
{
    struct { double x, y, z; };
    struct { double r, g, b; };
    struct { double s, t, p; };
    double v[3];
};
typedef union _Vector3d Vector3d;

union _Vector3i
{
    struct { int x, y, z; };
    struct { int r, g, b; };
    struct { int s, t, p; };
    int v[3];
};
typedef union _Vector3i Vector3i;

union _Vector4f
{
    struct { float x, y, z, w; };
    struct { float r, g, b, a; };
    struct { float s, t, p, q; };
    float v[4];
};
typedef union _Vector4f Vector4f;

union _Vector4u
{
    struct { unsigned char x, y, z, w; };
    struct { unsigned char r, g, b, a; };
    struct { unsigned char s, t, p, q; };
    unsigned char v[4];
};
typedef union _Vector4u Vector4u;

union _Quaternion
{
    struct { Vector3f v; float s; };
    struct { float x, y, z, w; };
    float q[4];
};
typedef union _Quaternion Quaternion;    

#endif

#ifndef __MATH_ENGINE__
#define __MATH_ENGINE__

static inline void MatrixVectorMultiply_3(Vector3f *out, Matrix3f *matrixLeft, Vector3f *vectorRight, Vector3f *work)
{
	work->v[0] = matrixLeft->m[0] * vectorRight->v[0] + matrixLeft->m[3] * vectorRight->v[1] + matrixLeft->m[6] * vectorRight->v[2];
	work->v[1] = matrixLeft->m[1] * vectorRight->v[0] + matrixLeft->m[4] * vectorRight->v[1] + matrixLeft->m[7] * vectorRight->v[2];
	work->v[2] = matrixLeft->m[2] * vectorRight->v[0] + matrixLeft->m[5] * vectorRight->v[1] + matrixLeft->m[8] * vectorRight->v[2];
	out->x = work->x; out->y = work->y; out->z = work->z;
}

static inline void MatrixVectorMultiply_4(Vector4f *out, Matrix4f *matrixLeft, Vector4f *vectorRight, Vector4f *work)
{
	work->v[0] = matrixLeft->m[0] * vectorRight->v[0] + matrixLeft->m[4] * vectorRight->v[1] + 
		matrixLeft->m[8] * vectorRight->v[2] + matrixLeft->m[12] * vectorRight->v[3];
	work->v[1] = matrixLeft->m[1] * vectorRight->v[0] + matrixLeft->m[5] * vectorRight->v[1] +
		matrixLeft->m[9] * vectorRight->v[2] + matrixLeft->m[13] * vectorRight->v[3];
	work->v[2] = matrixLeft->m[2] * vectorRight->v[0] + matrixLeft->m[6] * vectorRight->v[1] + 
		matrixLeft->m[10] * vectorRight->v[2] + matrixLeft->m[14] * vectorRight->v[3];
	work->v[3] = matrixLeft->m[3] * vectorRight->v[0] + matrixLeft->m[7] * vectorRight->v[1] + 
		matrixLeft->m[11] * vectorRight->v[2] + matrixLeft->m[15] * vectorRight->v[3];

	out->x = work->x; out->y = work->y; out->z = work->z; out->w = work->w;
}

static inline void MatrixVectorMultiplyAndNormalise_4(Vector4f *out, Matrix4f *matrixLeft, Vector4f *vectorRight, Vector4f *work)
{
	float norm;

	work->v[0] = matrixLeft->m[0] * vectorRight->v[0] + matrixLeft->m[4] * vectorRight->v[1] + 
		matrixLeft->m[8] * vectorRight->v[2] + matrixLeft->m[12] * vectorRight->v[3];
	work->v[1] = matrixLeft->m[1] * vectorRight->v[0] + matrixLeft->m[5] * vectorRight->v[1] +
		matrixLeft->m[9] * vectorRight->v[2] + matrixLeft->m[13] * vectorRight->v[3];
	work->v[2] = matrixLeft->m[2] * vectorRight->v[0] + matrixLeft->m[6] * vectorRight->v[1] + 
		matrixLeft->m[10] * vectorRight->v[2] + matrixLeft->m[14] * vectorRight->v[3];
	work->v[3] = matrixLeft->m[3] * vectorRight->v[0] + matrixLeft->m[7] * vectorRight->v[1] + 
		matrixLeft->m[11] * vectorRight->v[2] + matrixLeft->m[15] * vectorRight->v[3];

	norm = 1.0f/work->v[3]; work->v[0] *= norm; work->v[1] *= norm; work->v[2] *= norm; work->v[3] *= norm;

	out->x = work->x; out->y = work->y; out->z = work->z; out->w = work->w;
}

static inline void VectorSum_2(Vector2f *out, Vector2f *vectorLeft, Vector2f *vectorRight)
{
	out->x = vectorLeft->x + vectorRight->x;
	out->y = vectorLeft->y + vectorRight->y;
}

static inline void VectorSum_3(Vector3f *out, Vector3f *vectorLeft, Vector3f *vectorRight)
{
	out->x = vectorLeft->x + vectorRight->x;
	out->y = vectorLeft->y + vectorRight->y;
	out->z = vectorLeft->z + vectorRight->z;
}

static inline void VectorDiff_4(Vector4f *out, Vector4f *vectorLeft, Vector4f *vectorRight)
{
	out->x = vectorLeft->x - vectorRight->x;
	out->y = vectorLeft->y - vectorRight->y;
	out->z = vectorLeft->z - vectorRight->z;
	out->w = vectorLeft->w - vectorRight->w;
}

static inline void VectorDiff_3(Vector3f *out, Vector3f *vectorLeft, Vector3f *vectorRight)
{
	out->x = vectorLeft->x - vectorRight->x;
	out->y = vectorLeft->y - vectorRight->y;
	out->z = vectorLeft->z - vectorRight->z;
}

static inline void VectorDiff_2(Vector2f *out, Vector2f *vectorLeft, Vector2f *vectorRight)
{
	out->x = vectorLeft->x - vectorRight->x;
	out->y = vectorLeft->y - vectorRight->y;
}

static inline void VectorMultiplyByScalar_3(Vector3f *out, Vector3f *in, float scalar)
{
	out->x = in->x * scalar;
	out->y = in->y * scalar;
	out->z = in->z * scalar;
}

static inline void VectorMultiplyByVectorTranspose_3(Matrix3f *out, Vector3f *vectorLeft, Vector3f* vectorRight)
{
	out->m00 = vectorLeft->x * vectorRight->x; out->m01 = vectorLeft->x * vectorRight->y; out->m02 = vectorLeft->x * vectorRight->z;
	out->m10 = vectorLeft->y * vectorRight->x; out->m11 = vectorLeft->y * vectorRight->y; out->m12 = vectorLeft->y * vectorRight->z;
	out->m20 = vectorLeft->z * vectorRight->x; out->m21 = vectorLeft->z * vectorRight->y; out->m22 = vectorLeft->z * vectorRight->z;
}

static inline float VectorTransposeMultiplyByVector_3(Vector3f *vectorLeft, Vector3f* vectorRight)
{
	return vectorLeft->x * vectorRight->x + vectorLeft->y * vectorRight->y + vectorLeft->z * vectorRight->z;
}

static inline void MatrixMultiplyByScalar_3(Matrix3f *out, Matrix3f *in, float scalar)
{
	out->m[0] = in->m[0] * scalar; out->m[1] = in->m[1] * scalar; out->m[2] = in->m[2] * scalar;
	out->m[3] = in->m[3] * scalar; out->m[4] = in->m[4] * scalar; out->m[5] = in->m[5] * scalar;
	out->m[6] = in->m[6] * scalar; out->m[7] = in->m[7] * scalar; out->m[8] = in->m[8] * scalar;
}

static inline void MatrixTranspose_3(Matrix3f *out, Matrix3f *in, Matrix3f *work)
{
	work->m[0] = in->m[0]; work->m[1] = in->m[3]; work->m[2] = in->m[6];
	work->m[3] = in->m[1]; work->m[4] = in->m[4]; work->m[5] = in->m[7];
	work->m[6] = in->m[2]; work->m[7] = in->m[5]; work->m[8] = in->m[8];

	out->m[0] = work->m[0]; out->m[1] = work->m[1]; out->m[2] = work->m[2];
	out->m[3] = work->m[3]; out->m[4] = work->m[4]; out->m[5] = work->m[5];
	out->m[6] = work->m[6]; out->m[7] = work->m[7]; out->m[8] = work->m[8];
}

static inline void MatrixTranspose_4(Matrix4f *out, Matrix4f *in, Matrix4f *work)
{
	work->m[0] = in->m[0];	work->m[1] = in->m[4];	work->m[2] = in->m[8];		work->m[3] = in->m[12];
	work->m[4] = in->m[1];	work->m[5] = in->m[5];	work->m[6] = in->m[9];		work->m[7] = in->m[13];
	work->m[8] = in->m[2];	work->m[9] = in->m[6];	work->m[10] = in->m[10];	work->m[11] = in->m[14];
	work->m[12] = in->m[3];	work->m[13] = in->m[7];	work->m[14] = in->m[11];	work->m[15] = in->m[15];

	for (int i=0;i<16;i++) out->m[i]=work->m[i];
	
}

static inline void MatrixSum_3(Matrix3f *out, Matrix3f *matrixLeft, Matrix3f *matrixRight)
{
    out->m[0] = matrixLeft->m[0] + matrixRight->m[0];
    out->m[1] = matrixLeft->m[1] + matrixRight->m[1];
    out->m[2] = matrixLeft->m[2] + matrixRight->m[2];
    
    out->m[3] = matrixLeft->m[3] + matrixRight->m[3];
    out->m[4] = matrixLeft->m[4] + matrixRight->m[4];
    out->m[5] = matrixLeft->m[5] + matrixRight->m[5];
    
    out->m[6] = matrixLeft->m[6] + matrixRight->m[6];
    out->m[7] = matrixLeft->m[7] + matrixRight->m[7];
    out->m[8] = matrixLeft->m[8] + matrixRight->m[8];
}


static inline void MatrixSum_4(Matrix4f *out, Matrix4f *matrixLeft, Matrix4f *matrixRight)
{
    out->m[0] = matrixLeft->m[0] + matrixRight->m[0];
    out->m[1] = matrixLeft->m[1] + matrixRight->m[1];
    out->m[2] = matrixLeft->m[2] + matrixRight->m[2];
    out->m[3] = matrixLeft->m[3] + matrixRight->m[3];

    out->m[4] = matrixLeft->m[4] + matrixRight->m[4];
    out->m[5] = matrixLeft->m[5] + matrixRight->m[5];
    out->m[6] = matrixLeft->m[6] + matrixRight->m[6];
    out->m[7] = matrixLeft->m[7] + matrixRight->m[7];

    out->m[8] = matrixLeft->m[8] + matrixRight->m[8];
	out->m[9] = matrixLeft->m[9] + matrixRight->m[9];
	out->m[10] = matrixLeft->m[10] + matrixRight->m[10];
	out->m[11] = matrixLeft->m[11] + matrixRight->m[11];

	out->m[12] = matrixLeft->m[12] + matrixRight->m[12];
	out->m[13] = matrixLeft->m[13] + matrixRight->m[13];
	out->m[14] = matrixLeft->m[14] + matrixRight->m[14];
	out->m[15] = matrixLeft->m[15] + matrixRight->m[15];
}

static inline void MatrixDiff_3(Matrix3f *out, Matrix3f *matrixLeft, Matrix3f *matrixRight)
{
    out->m[0] = matrixLeft->m[0] - matrixRight->m[0];
    out->m[1] = matrixLeft->m[1] - matrixRight->m[1];
    out->m[2] = matrixLeft->m[2] - matrixRight->m[2];
    
    out->m[3] = matrixLeft->m[3] - matrixRight->m[3];
    out->m[4] = matrixLeft->m[4] - matrixRight->m[4];
    out->m[5] = matrixLeft->m[5] - matrixRight->m[5];
    
    out->m[6] = matrixLeft->m[6] - matrixRight->m[6];
    out->m[7] = matrixLeft->m[7] - matrixRight->m[7];
    out->m[8] = matrixLeft->m[8] - matrixRight->m[8];
}

static inline void SetIdentityMatrix_3(Matrix3f *matrix)
{
	matrix->m[0] = 0.0f; matrix->m[1] = 0.0f; matrix->m[2] = 0.0f;
	matrix->m[3] = 0.0f; matrix->m[4] = 0.0f; matrix->m[5] = 0.0f;
	matrix->m[6] = 0.0f; matrix->m[7] = 0.0f; matrix->m[8] = 0.0f;
	
	matrix->m00 = 1.0f; matrix->m11 = 1.0f; matrix->m22 = 1.0f;
}

static inline void SetZeroMatrix_3(Matrix3f *matrix)
{
	matrix->m[0] = 0.0f; matrix->m[1] = 0.0f; matrix->m[2] = 0.0f;
	matrix->m[3] = 0.0f; matrix->m[4] = 0.0f; matrix->m[5] = 0.0f;
	matrix->m[6] = 0.0f; matrix->m[7] = 0.0f; matrix->m[8] = 0.0f;
}

static inline void SetZeroMatrix_4(Matrix4f *matrix)
{
	matrix->m[0] = 0.0f; matrix->m[1] = 0.0f; matrix->m[2] = 0.0f; matrix->m[3] = 0.0f; 
	matrix->m[4] = 0.0f; matrix->m[5] = 0.0f; matrix->m[6] = 0.0f; matrix->m[7] = 0.0f; 
	matrix->m[8] = 0.0f; matrix->m[9] = 0.0f; matrix->m[10] = 0.0f; matrix->m[11] = 0.0f;
	matrix->m[12] = 0.0f; matrix->m[13] = 0.0f; matrix->m[14] = 0.0f; matrix->m[15] = 0.0f;
}

static inline void SetZeroVector_3(Vector3f *vector)
{
	vector->x = 0; vector->y = 0; vector->z = 0;
}

static inline bool IsEqualVector_3(Vector3f *vectorLeft, Vector3f *vectorRight)
{
	return (vectorLeft->x == vectorRight->x && vectorLeft->y == vectorRight->y && vectorLeft->z == vectorRight->z);
}

static inline bool IsEqualMatrix_3(Matrix3f *matrixLeft, Matrix3f *matrixRight)
{
	return (matrixLeft->m[0] == matrixRight->m[0] && matrixLeft->m[1] == matrixRight->m[1] && matrixLeft->m[2] == matrixRight->m[2] &&
		matrixLeft->m[3] == matrixRight->m[3] && matrixLeft->m[4] == matrixRight->m[4] && matrixLeft->m[5] == matrixRight->m[5] &&
		matrixLeft->m[6] == matrixRight->m[6] && matrixLeft->m[7] == matrixRight->m[7] && matrixLeft->m[8] == matrixRight->m[8]);
}

static inline bool IsEqualVector_2(Vector2f *vectorLeft, Vector2f *vectorRight)
{
	return (vectorLeft->x == vectorRight->x && vectorLeft->y == vectorRight->y);
}

static inline bool IsEqualMatrix_2(Matrix2f *matrixLeft, Matrix2f *matrixRight)
{
	return (matrixLeft->m[0] == matrixRight->m[0] && matrixLeft->m[1] == matrixRight->m[1] && matrixLeft->m[2] == matrixRight->m[2] &&
		matrixLeft->m[3] == matrixRight->m[3]);
}

static inline float MatrixDeterminant_3(Matrix3f *matrix)
{
	float determinant = (matrix->m11*matrix->m22 - matrix->m12*matrix->m21)*matrix->m00 + 
		(matrix->m12*matrix->m20 - matrix->m10*matrix->m22)*matrix->m01 + 
		(matrix->m10*matrix->m21 - matrix->m11*matrix->m20)*matrix->m02;
	return determinant;
}

static inline bool MatrixInvert_3(Matrix3f *out, Matrix3f *in, Matrix3f *work)
{
	float determinant = MatrixDeterminant_3(in);
	if (determinant == 0.0f) return false;

	work->m00 = (in->m11 * in->m22 - in->m12 * in->m21) / determinant; 
	work->m01 = -(in->m01 * in->m22 - in->m02 * in->m21) / determinant; 
	work->m02 = (in->m01 * in->m12 - in->m02 * in->m11) / determinant;
	work->m10 = -(in->m10 * in->m22 - in->m12 * in->m20) / determinant;
	work->m11 = (in->m00 * in->m22 - in->m02 * in->m20) / determinant;
	work->m12 = -(in->m00 * in->m12 - in->m02 * in->m10) / determinant;
	work->m20 = (in->m10 * in->m21 - in->m11 * in->m20) / determinant; 
	work->m21 = -(in->m00 * in->m21 - in->m01 * in->m20) / determinant;
	work->m22 = (in->m00 * in->m11 - in->m01 * in->m10) / determinant;

	out->m[0] = work->m[0]; out->m[1] = work->m[1]; out->m[2] = work->m[2];
	out->m[3] = work->m[3]; out->m[4] = work->m[4]; out->m[5] = work->m[5];
	out->m[6] = work->m[6]; out->m[7] = work->m[7]; out->m[8] = work->m[8];

	return true;
}

static inline bool MatrixInvert_4(Matrix4f *out, Matrix4f *in)
{
	float    tmp[12];
	float    src[16];
	float    det; 

	float *dst = out->m;

	for (int i = 0; i < 4; i++) {
		src[i]        = in->m[i*4];
		src[i + 4]    = in->m[i*4 + 1];
		src[i + 8]    = in->m[i*4 + 2];
		src[i + 12]   = in->m[i*4 + 3];
	}

	tmp[0]  = src[10] * src[15];
	tmp[1]  = src[11] * src[14];
	tmp[2]  = src[9]  * src[15];
	tmp[3]  = src[11] * src[13];
	tmp[4]  = src[9]  * src[14];
	tmp[5]  = src[10] * src[13];
	tmp[6]  = src[8]  * src[15];
	tmp[7]  = src[11] * src[12];
	tmp[8]  = src[8]  * src[14];
	tmp[9]  = src[10] * src[12];
	tmp[10] = src[8]  * src[13];
	tmp[11] = src[9]  * src[12];

	dst[0]  = tmp[0]*src[5] + tmp[3]*src[6] + tmp[4]*src[7];
	dst[0] -= tmp[1]*src[5] + tmp[2]*src[6] + tmp[5]*src[7];
	dst[1]  = tmp[1]*src[4] + tmp[6]*src[6] + tmp[9]*src[7];
	dst[1] -= tmp[0]*src[4] + tmp[7]*src[6] + tmp[8]*src[7];
	dst[2]  = tmp[2]*src[4] + tmp[7]*src[5] + tmp[10]*src[7];
	dst[2] -= tmp[3]*src[4] + tmp[6]*src[5] + tmp[11]*src[7];
	dst[3]  = tmp[5]*src[4] + tmp[8]*src[5] + tmp[11]*src[6];
	dst[3] -= tmp[4]*src[4] + tmp[9]*src[5] + tmp[10]*src[6];
	dst[4]  = tmp[1]*src[1] + tmp[2]*src[2] + tmp[5]*src[3];
	dst[4] -= tmp[0]*src[1] + tmp[3]*src[2] + tmp[4]*src[3];
	dst[5]  = tmp[0]*src[0] + tmp[7]*src[2] + tmp[8]*src[3];
	dst[5] -= tmp[1]*src[0] + tmp[6]*src[2] + tmp[9]*src[3];
	dst[6]  = tmp[3]*src[0] + tmp[6]*src[1] + tmp[11]*src[3];
	dst[6] -= tmp[2]*src[0] + tmp[7]*src[1] + tmp[10]*src[3];
	dst[7]  = tmp[4]*src[0] + tmp[9]*src[1] + tmp[10]*src[2];
	dst[7] -= tmp[5]*src[0] + tmp[8]*src[1] + tmp[11]*src[2];

	tmp[0]  = src[2]*src[7];
	tmp[1]  = src[3]*src[6];
	tmp[2]  = src[1]*src[7];
	tmp[3]  = src[3]*src[5];
	tmp[4]  = src[1]*src[6];
	tmp[5]  = src[2]*src[5];
	tmp[6]  = src[0]*src[7];
	tmp[7]  = src[3]*src[4];
	tmp[8]  = src[0]*src[6];
	tmp[9]  = src[2]*src[4];
	tmp[10] = src[0]*src[5];
	tmp[11] = src[1]*src[4];

	dst[8]  = tmp[0]*src[13] + tmp[3]*src[14] + tmp[4]*src[15];
	dst[8] -= tmp[1]*src[13] + tmp[2]*src[14] + tmp[5]*src[15];
	dst[9]  = tmp[1]*src[12] + tmp[6]*src[14] + tmp[9]*src[15];
	dst[9] -= tmp[0]*src[12] + tmp[7]*src[14] + tmp[8]*src[15];
	dst[10] = tmp[2]*src[12] + tmp[7]*src[13] + tmp[10]*src[15];
	dst[10]-= tmp[3]*src[12] + tmp[6]*src[13] + tmp[11]*src[15];
	dst[11] = tmp[5]*src[12] + tmp[8]*src[13] + tmp[11]*src[14];
	dst[11]-= tmp[4]*src[12] + tmp[9]*src[13] + tmp[10]*src[14];
	dst[12] = tmp[2]*src[10] + tmp[5]*src[11] + tmp[1]*src[9];
	dst[12]-= tmp[4]*src[11] + tmp[0]*src[9] + tmp[3]*src[10];
	dst[13] = tmp[8]*src[11] + tmp[0]*src[8] + tmp[7]*src[10];
	dst[13]-= tmp[6]*src[10] + tmp[9]*src[11] + tmp[1]*src[8];
	dst[14] = tmp[6]*src[9] + tmp[11]*src[11] + tmp[3]*src[8];
	dst[14]-= tmp[10]*src[11] + tmp[2]*src[8] + tmp[7]*src[9];
	dst[15] = tmp[10]*src[10] + tmp[4]*src[8] + tmp[9]*src[9];
	dst[15]-= tmp[8]*src[9] + tmp[11]*src[10] + tmp[5]*src[8];

	det=src[0]*dst[0]+src[1]*dst[1]+src[2]*dst[2]+src[3]*dst[3];

	if (det == 0.0f) return false;

	det = 1/det;
	for (int j = 0; j < 16; j++) dst[j] *= det;

	return true;
}

static inline bool MatrixInvertAndTranspose_3(Matrix3f *out, Matrix3f *in, Matrix3f *work)
{
	float determinant = MatrixDeterminant_3(in);
	if (determinant == 0.0f) return false;
	
	work->m00 = (in->m11 * in->m22 - in->m12 * in->m21) / determinant; 
	work->m10 = -(in->m01 * in->m22 - in->m02 * in->m21) / determinant; 
	work->m20 = (in->m01 * in->m12 - in->m02 * in->m11) / determinant;
	work->m01 = -(in->m10 * in->m22 - in->m12 * in->m20) / determinant;
	work->m11 = (in->m00 * in->m22 - in->m02 * in->m20) / determinant;
	work->m21 = -(in->m00 * in->m12 - in->m02 * in->m10) / determinant;
	work->m02 = (in->m10 * in->m21 - in->m11 * in->m20) / determinant; 
	work->m12 = -(in->m00 * in->m21 - in->m01 * in->m20) / determinant;
	work->m22 = (in->m00 * in->m11 - in->m01 * in->m10) / determinant;

	out->m[0] = work->m[0]; out->m[1] = work->m[1]; out->m[2] = work->m[2];
	out->m[3] = work->m[3]; out->m[4] = work->m[4]; out->m[5] = work->m[5];
	out->m[6] = work->m[6]; out->m[7] = work->m[7]; out->m[8] = work->m[8];

	return true;
}

static inline void MatrixMultiply_4(Matrix4f *out, Matrix4f *matrixLeft, Matrix4f *matrixRight, Matrix4f *work)
{
	int i, j, k;
	for (i = 0; i < 4; i++) for (j = 0; j < 4; j++) 
	{ 
		work->m[i + j*4] = 0; 
		for (k = 0; k < 4; k++) 
			work->m[i + j*4] += matrixLeft->m[i + k*4] * matrixRight->m[k + j*4]; 
	}
	for (i = 0; i < 16; i++) out->m[i] = work->m[i];
}

static inline void MatrixMultiply_3(Matrix3f* out, Matrix3f *matrixLeft, Matrix3f *matrixRight, Matrix3f *work)
{
	work->m[0] = matrixLeft->m[0] * matrixRight->m[0] + matrixLeft->m[3] * matrixRight->m[1] + matrixLeft->m[6] * matrixRight->m[2];
    work->m[3] = matrixLeft->m[0] * matrixRight->m[3] + matrixLeft->m[3] * matrixRight->m[4] + matrixLeft->m[6] * matrixRight->m[5];
    work->m[6] = matrixLeft->m[0] * matrixRight->m[6] + matrixLeft->m[3] * matrixRight->m[7] + matrixLeft->m[6] * matrixRight->m[8];
    
    work->m[1] = matrixLeft->m[1] * matrixRight->m[0] + matrixLeft->m[4] * matrixRight->m[1] + matrixLeft->m[7] * matrixRight->m[2];
    work->m[4] = matrixLeft->m[1] * matrixRight->m[3] + matrixLeft->m[4] * matrixRight->m[4] + matrixLeft->m[7] * matrixRight->m[5];
    work->m[7] = matrixLeft->m[1] * matrixRight->m[6] + matrixLeft->m[4] * matrixRight->m[7] + matrixLeft->m[7] * matrixRight->m[8];
    
    work->m[2] = matrixLeft->m[2] * matrixRight->m[0] + matrixLeft->m[5] * matrixRight->m[1] + matrixLeft->m[8] * matrixRight->m[2];
    work->m[5] = matrixLeft->m[2] * matrixRight->m[3] + matrixLeft->m[5] * matrixRight->m[4] + matrixLeft->m[8] * matrixRight->m[5];
    work->m[8] = matrixLeft->m[2] * matrixRight->m[6] + matrixLeft->m[5] * matrixRight->m[7] + matrixLeft->m[8] * matrixRight->m[8];
    
	out->m[0] = work->m[0]; out->m[1] = work->m[1]; out->m[2] = work->m[2];
	out->m[3] = work->m[3]; out->m[4] = work->m[4]; out->m[5] = work->m[5];
	out->m[6] = work->m[6]; out->m[7] = work->m[7]; out->m[8] = work->m[8];
}

static inline void MatrixMultiplyTranspose_3(Matrix3f* out, Matrix3f *matrixLeft, Matrix3f *matrixRightToTranspose, Matrix3f *work)
{
	Matrix3f matrixRight;
	
	matrixRight.m[0] = matrixRightToTranspose->m[0]; matrixRight.m[1] = matrixRightToTranspose->m[3]; matrixRight.m[2] = matrixRightToTranspose->m[6];
	matrixRight.m[3] = matrixRightToTranspose->m[1]; matrixRight.m[4] = matrixRightToTranspose->m[4]; matrixRight.m[5] = matrixRightToTranspose->m[7];
	matrixRight.m[6] = matrixRightToTranspose->m[2]; matrixRight.m[7] = matrixRightToTranspose->m[5]; matrixRight.m[8] = matrixRightToTranspose->m[8];

	work->m[0] = matrixLeft->m[0] * matrixRight.m[0] + matrixLeft->m[3] * matrixRight.m[1] + matrixLeft->m[6] * matrixRight.m[2];
    work->m[3] = matrixLeft->m[0] * matrixRight.m[3] + matrixLeft->m[3] * matrixRight.m[4] + matrixLeft->m[6] * matrixRight.m[5];
    work->m[6] = matrixLeft->m[0] * matrixRight.m[6] + matrixLeft->m[3] * matrixRight.m[7] + matrixLeft->m[6] * matrixRight.m[8];
    
    work->m[1] = matrixLeft->m[1] * matrixRight.m[0] + matrixLeft->m[4] * matrixRight.m[1] + matrixLeft->m[7] * matrixRight.m[2];
    work->m[4] = matrixLeft->m[1] * matrixRight.m[3] + matrixLeft->m[4] * matrixRight.m[4] + matrixLeft->m[7] * matrixRight.m[5];
    work->m[7] = matrixLeft->m[1] * matrixRight.m[6] + matrixLeft->m[4] * matrixRight.m[7] + matrixLeft->m[7] * matrixRight.m[8];
    
    work->m[2] = matrixLeft->m[2] * matrixRight.m[0] + matrixLeft->m[5] * matrixRight.m[1] + matrixLeft->m[8] * matrixRight.m[2];
    work->m[5] = matrixLeft->m[2] * matrixRight.m[3] + matrixLeft->m[5] * matrixRight.m[4] + matrixLeft->m[8] * matrixRight.m[5];
    work->m[8] = matrixLeft->m[2] * matrixRight.m[6] + matrixLeft->m[5] * matrixRight.m[7] + matrixLeft->m[8] * matrixRight.m[8];
    
	out->m[0] = work->m[0]; out->m[1] = work->m[1]; out->m[2] = work->m[2];
	out->m[3] = work->m[3]; out->m[4] = work->m[4]; out->m[5] = work->m[5];
	out->m[6] = work->m[6]; out->m[7] = work->m[7]; out->m[8] = work->m[8];
}

static inline void SetMatrixColumnsWithSignChange_3(Matrix3f* out, Matrix3f *in, Vector3f *signChange)
{
    out->m[0 * 3 + 0] = signChange->x * in->m[0 * 3 + 0];
    out->m[0 * 3 + 1] = signChange->x * in->m[0 * 3 + 1];
    out->m[0 * 3 + 2] = signChange->x * in->m[0 * 3 + 2];

    out->m[1 * 3 + 0] = signChange->y * in->m[0 * 3 + 0];
    out->m[1 * 3 + 1] = signChange->y * in->m[0 * 3 + 1];
    out->m[1 * 3 + 2] = signChange->y * in->m[0 * 3 + 2];

    out->m[2 * 3 + 0] = signChange->z * in->m[0 * 3 + 0];
    out->m[2 * 3 + 1] = signChange->z * in->m[0 * 3 + 1];
    out->m[2 * 3 + 2] = signChange->z * in->m[0 * 3 + 2];
}

static inline float VectorDotProduct_3(Vector3f *vectorLeft, Vector3f *vectorRight)
{
	return vectorLeft->x * vectorRight->x + vectorLeft->y * vectorRight->y + vectorLeft->z * vectorRight->z;
}

static inline float VectorDotProduct_2(Vector2f *vectorLeft, Vector2f *vectorRight)
{
	return vectorLeft->x * vectorRight->x + vectorLeft->y * vectorRight->y;
}

static inline float VectorDotProduct_4(Vector4f *vectorLeft, Vector4f *vectorRight)
{
	return vectorLeft->x * vectorRight->x + vectorLeft->y * vectorRight->y + vectorLeft->z * vectorRight->z + vectorLeft->w * vectorRight->w;
}

static inline void SetMatrixFromMatrix_3(Matrix3f* out, Matrix3f *in)
{
	out->m[0] = in->m[0]; out->m[1] = in->m[1]; out->m[2] = in->m[2];
	out->m[3] = in->m[3]; out->m[4] = in->m[4]; out->m[5] = in->m[5];
	out->m[6] = in->m[6]; out->m[7] = in->m[7]; out->m[8] = in->m[8];
}

static inline void SetMatrixFromMatrix_4(Matrix4f* out, Matrix4f *in)
{
	out->m[0] = in->m[0]; out->m[1] = in->m[1]; out->m[2] = in->m[2]; out->m[3] = in->m[3]; 
	out->m[4] = in->m[4]; out->m[5] = in->m[5]; out->m[6] = in->m[6]; out->m[7] = in->m[7]; 
	out->m[8] = in->m[8]; out->m[9] = in->m[9]; out->m[10] = in->m[10]; out->m[11] = in->m[11];
	out->m[12] = in->m[12]; out->m[13] = in->m[13]; out->m[14] = in->m[14]; out->m[15] = in->m[15];
}

static inline void SetVectorFromVector_3(Vector3f* out, Vector3f *in)
{
	out->x = in->x; out->y = in->y; out->z = in->z;
}

static inline void SetVectorFromFloat_3(Vector3f* out, float *in)
{
	out->x = in[0]; out->y = in[1]; out->z = in[2];
}

static inline void SetVectorFromVector_4(Vector4f* out, Vector4f *in)
{
	out->x = in->x; out->y = in->y; out->z = in->z; out->w = in->w;
}

static inline float VectorNorm_2(Vector2f *in)
{
    return sqrtf(in->v[0] * in->v[0] + in->v[1] * in->v[1]);
}

static inline float VectorNorm_3(Vector3f *in)
{
    return sqrtf(in->v[0] * in->v[0] + in->v[1] * in->v[1] + in->v[2] * in->v[2]);
}

static inline float VectorNormSq_3(Vector3f *in)
{
    return in->v[0] * in->v[0] + in->v[1] * in->v[1] + in->v[2] * in->v[2];
}

static inline void VectorNormalize_2(Vector2f* out, Vector2f *in)
{
    float scale = 1.0f / sqrtf(in->v[0] * in->v[0] + in->v[1] * in->v[1]);
    out->v[0] = in->v[0] * scale; out->v[1] = in->v[1] * scale;
}

static inline void VectorNormalize_4(Vector4f* out, Vector4f *in)
{
    float scale = 1.0f / sqrtf(in->v[0] * in->v[0] + in->v[1] * in->v[1] + in->v[2] * in->v[2] + in->v[3] * in->v[3]);
    out->v[0] = in->v[0] * scale; out->v[1] = in->v[1] * scale; out->v[2] = in->v[2] * scale; out->v[3] = in->v[3] * scale;
}

static inline void VectorNormalizeHomogeneous_4(Vector4f* out, Vector4f *in)
{
    float scale = 1.0f / in->v[3];
    out->v[0] = in->v[0] * scale; out->v[1] = in->v[1] * scale; out->v[2] = in->v[2] * scale; out->v[3] = 1.0f;
}

static inline void VectorNormalize_3(Vector3f* out, Vector3f *in)
{
    float scale = 1.0f / sqrtf(in->v[0] * in->v[0] + in->v[1] * in->v[1] + in->v[2] * in->v[2]);
    out->v[0] = in->v[0] * scale; out->v[1] = in->v[1] * scale; out->v[2] = in->v[2] * scale;
}

static inline void VectorCrossProduct_3(Vector3f* out, Vector3f* vectorLeft, Vector3f* vectorRight, Vector3f *work)
{
	work->v[0] = vectorLeft->y * vectorRight->z - vectorLeft->z * vectorRight->y;
	work->v[1] = vectorLeft->z * vectorRight->x - vectorLeft->x * vectorRight->z;
	work->v[2] = vectorLeft->x * vectorRight->y - vectorLeft->y * vectorRight->x;
	out->v[0] = work->v[0]; out->v[1] = work->v[1]; out->v[2] = work->v[2];
}

static inline void AddToDiagonal(float *dst, float *src, float lambda, float alpha, int dim)
{
	for (int i=0; i<dim * dim; i++) dst[i] = src[i];
	for (int i=0; i<dim; i++) dst[i + i * dim] += (lambda * (src[i + i * dim]) + 0.01f) * alpha;
}

static inline void ISRInvMatrix3(float* mIn, float* mOut)
{

	float a00=mIn[0], a01=mIn[1], a02=mIn[2];
	float a10=mIn[3], a11=mIn[4], a12=mIn[5];
	float a20=mIn[6], a21=mIn[7], a22=mIn[8];

	float t2 = a00*a11*a22;
	float t3 = a01*a12*a20;
	float t4 = a02*a10*a21;
	float t7 = a00*a12*a21;
	float t8 = a01*a10*a22;
	float t9 = a02*a11*a20;
	float t5 = t2+t3+t4-t7-t8-t9;
	float t6 = 1.0f /t5;

	float reM[9]={t6*(a11*a22-a12*a21),-t6*(a01*a22-a02*a21),t6*(a01*a12-a02*a11),
		-t6*(a10*a22-a12*a20),t6*(a00*a22-a02*a20),-t6*(a00*a12-a02*a10),
		t6*(a10*a21-a11*a20),-t6*(a00*a21-a01*a20),t6*(a00*a11-a01*a10)};

	memcpy(mOut,reM,9*sizeof(float));
}

static inline void UnimindMatrixMul3( float* m1, float* m2, float* mout )
{
	mout[0] = m1[0]*m2[0] + m1[1]*m2[3] + m1[2]*m2[6];
	mout[1] = m1[0]*m2[1] + m1[1]*m2[4] + m1[2]*m2[7];
	mout[2] = m1[0]*m2[2] + m1[1]*m2[5] + m1[2]*m2[8];

	mout[3] = m1[3]*m2[0] + m1[4]*m2[3] + m1[5]*m2[6];
	mout[4] = m1[3]*m2[1] + m1[4]*m2[4] + m1[5]*m2[7];
	mout[5] = m1[3]*m2[2] + m1[4]*m2[5] + m1[5]*m2[8];

	mout[6] = m1[6]*m2[0] + m1[7]*m2[3] + m1[8]*m2[6];
	mout[7] = m1[6]*m2[1] + m1[7]*m2[4] + m1[8]*m2[7];
	mout[8] = m1[6]*m2[2] + m1[7]*m2[5] + m1[8]*m2[8];
}

static inline void UnimindUpdateRT( float* oldR, float* oldT, float* dR, float* dT )
{
	float newR[9];
	float newT[3];

	UnimindMatrixMul3(dR,oldR,newR);

	newT[0]=dR[0]*oldT[0]+dR[1]*oldT[1]+dR[2]*oldT[2]+dT[0];
	newT[1]=dR[3]*oldT[0]+dR[4]*oldT[1]+dR[5]*oldT[2]+dT[1];
	newT[2]=dR[6]*oldT[0]+dR[7]*oldT[1]+dR[8]*oldT[2]+dT[2];

	memcpy(oldR,newR,9*sizeof(float));
	memcpy(oldT,newT,3*sizeof(float));
}

static inline void UnimindInverseRTtoH( float* R, float* T, float* outH )
{
	float invR[9];

	ISRInvMatrix3(R,invR);

	int idx=0;

	for (int i=0;i<3;i++){
		for (int j=0;j<3;j++,idx++){
			outH[i*4+j]=invR[idx];}}

	outH[3]=-(invR[0]*T[0]+invR[1]*T[1]+invR[2]*T[2]);
	outH[7]=-(invR[3]*T[0]+invR[4]*T[1]+invR[5]*T[2]);
	outH[11]=-(invR[6]*T[0]+invR[7]*T[1]+invR[8]*T[2]);
}

static inline void GetRotationMatrixFromMRP( float *outR,float* r )
{
	float t1 = r[0];
	float t2 = r[1];
	float t3 = r[2];

	float tsq = t1*t1 + t2*t2 + t3*t3;

	float tsum = 1-tsq;

	outR[0]=4*t1*t1-4*t2*t2-4*t3*t3+tsum*tsum;	outR[1]=8*t1*t2-4*t3*tsum;	outR[2]=8*t1*t3+4*t2*tsum;
	outR[3]=8*t1*t2+4*t3*tsum;	outR[4]=4*t2*t2-4*t1*t1-4*t3*t3+tsum*tsum;	outR[5]=8*t2*t3-4*t1*tsum;
	outR[6]=8*t1*t3-4*t2*tsum;	outR[7]=8*t2*t3+4*t1*tsum;	outR[8]=4*t3*t3-4*t2*t2-4*t1*t1+tsum*tsum;

	for (int i=0;i<9;i++) outR[i]/=((1+tsq)*(1+tsq));
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


static inline void GetRotationMatrixFromMRP(Matrix3f *outRR, Vector3f* rr)
{
	float *outR = outRR->m;
	float *r = rr->v;

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

static inline void UpdateRT(Matrix3f* oldRR, Vector3f* oldTT, float* dR, float* dT)
{
	float *oldR = oldRR->m;
	float *oldT = oldTT->v;

	float newR[9];
	float newT[3];

	UnimindMatrixMul3(dR, oldR, newR);

	newT[0] = dR[0] * oldT[0] + dR[1] * oldT[1] + dR[2] * oldT[2] + dT[0];
	newT[1] = dR[3] * oldT[0] + dR[4] * oldT[1] + dR[5] * oldT[2] + dT[1];
	newT[2] = dR[6] * oldT[0] + dR[7] * oldT[1] + dR[8] * oldT[2] + dT[2];

	memcpy(oldR, newR, 9 * sizeof(float));
	memcpy(oldT, newT, 3 * sizeof(float));
}




static inline bool almostZero( float sum )
{
	return(sum*sum<1.0e-25);
}

static inline int UnimindCHdecompose(const float *input, int size, float *output)
{
	int i,j,k;
	float sum;
	bool divide;
	int ret = 0;
	for (i=0;i<size;i++) {
		divide=true;
		for (j=0;j<i;j++) output[j*size+i]=0.0;
		for (;j<size;j++) {
			sum=input[i*size+j];
			for (k=i-1;k>=0;k--) sum -= output[i*size+k]*output[j*size+k];
			if (i == j) {
				/* The following applies if A, with rounding errors, is not positive definite.*/
				if (almostZero(sum)) {
					output[i*size+i]=0.0;
					divide = false;
				} else if (sum<0.0) {
					output[i*size+i]=0.0;
					divide = false;
					printf("choldc failed: sqrt(%f)\n", sum);
					ret = -1;
				} else {
					output[i*size+i]=sqrt(sum);
				}
			} else {
				if (!divide) output[j*size+i]=0.0;
				else output[j*size+i]=sum/output[i*size+i];
			}
		}
	}
	return ret;
}

static inline void UnimindCHsolveDecomposed(const float *A, int size, const float *b, float *x, int num=1, int ldb=-1)
{
	int i,k;
	float sum;
	// Solve L y = b, storing y in x.
	for (int eq=0; eq<num; ++eq) {
		for (i=0;i<size;i++) {
			for (sum=b[i+eq*ldb],k=i-1;k>=0;k--) sum -= A[i*size+k]*x[k+eq*ldb];
			x[i+eq*ldb]=sum/A[i*size+i];
		}
	}
	// Solve LT x = y
	for (int eq=0; eq<num; ++eq) {
		for (i=size-1;i>=0;i--) {
			for (sum=x[i+eq*ldb],k=i+1;k<size;k++) sum -= A[k*size+i]*x[k+eq*ldb];
			x[i+eq*ldb]=sum/A[i*size+i];
		}
	}
}

static inline void UnimindGetMRPfromDegree( float *outR, float *inR )
{
	float rx=inR[0];
	float ry=inR[1];
	float rz=inR[2];

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

	double norm = 1 / sqrt(rotation[0]*rotation[0] + rotation[1]*rotation[1] + rotation[2]*rotation[2] + rotation[3]*rotation[3]);
	for(int i=0;i<4;i++) rotation[i] *= norm;

	double b0 = rotation[3];
	double b1 = rotation[0];
	double b2 = rotation[1];
	double b3 = rotation[2];

	outR[0] = (float)(b1 / (1 + b0));
	outR[1] = (float)(b2 / (1 + b0));
	outR[2] = (float)(b3 / (1 + b0));
}

static inline void VectorDiff_n(float *dst, float *a, float *b, int dim) { for (int i=0; i<dim; i++) dst[i] = a[i] + b[i]; }

static inline float fast_log(float val)
{
    int* const exp_ptr = reinterpret_cast<int *> (&val);
	int x = *exp_ptr;
	const int log_2 = ((x >> 23) & 255) - 128;
	x &= ~(255 << 23);
	x += 127 << 23;
	*exp_ptr = x;
    
	val = ((-1.0f/3) * val + 2) * val - 2.0f/3;
    
	return (val + log_2) * 0.69314718f;
}

void inline trilinearInterpolateVolume(float *volOut, float *volIn, int volSizeOut, int volSizeIn)
{
	float sxp0, syp0, szp0;
	int i_sxp1, i_syp1, i_szp1, i_sxp0, i_syp0, i_szp0;

	float srcWidth  = (float)volSizeIn, srcHeight = (float)volSizeIn, srcDepth  = (float)volSizeIn;
	float dstWidth  = (float)volSizeOut, dstHeight = (float)volSizeOut;

	float fScaleX = srcWidth / dstWidth, fScaleY = srcHeight / dstHeight, fScaleZ = srcDepth / dstHeight;

	for(int dz = 0; dz < volSizeOut; dz++) for(int dy = 0; dy < volSizeOut; dy++) for(int dx = 0; dx < volSizeOut; dx++)
	{
		sxp0 = MIN((float)dx * fScaleX, volSizeIn - 1); i_sxp0 = (int)sxp0; i_sxp1 = MIN(i_sxp0 + 1, volSizeIn - 1); 
		syp0 = MIN((float)dy * fScaleY, volSizeIn - 1); i_syp0 = (int)syp0; i_syp1 = MIN(i_syp0 + 1, volSizeIn - 1); 
		szp0 = MIN((float)dz * fScaleZ, volSizeIn - 1); i_szp0 = (int)szp0; i_szp1 = MIN(i_szp0 + 1, volSizeIn - 1); 

		float v000 = volIn[i_sxp0 + i_syp0 * volSizeIn + i_szp0 * volSizeIn * volSizeIn];
		float v100 = volIn[i_sxp1 + i_syp0 * volSizeIn + i_szp0 * volSizeIn * volSizeIn];

		float v001 = volIn[i_sxp0 + i_syp0 * volSizeIn + i_szp1 * volSizeIn * volSizeIn];
		float v101 = volIn[i_sxp1 + i_syp0 * volSizeIn + i_szp1 * volSizeIn * volSizeIn];

		float v010 = volIn[i_sxp0 + i_syp1 * volSizeIn + i_szp0 * volSizeIn * volSizeIn];
		float v110 = volIn[i_sxp1 + i_syp1 * volSizeIn + i_szp0 * volSizeIn * volSizeIn];

		float v011 = volIn[i_sxp0 + i_syp1 * volSizeIn + i_szp1 * volSizeIn * volSizeIn];
		float v111 = volIn[i_sxp1 + i_syp1 * volSizeIn + i_szp1 * volSizeIn * volSizeIn];

		sxp0 = sxp0 - i_sxp0; syp0 = syp0 - i_syp0; szp0 = szp0 - i_szp0;

		float v000_v100 = (v100 - v000) * sxp0 + v000, v001_v101 = (v101 - v001) * sxp0 + v001;
		float v010_v110 = (v110 - v010) * sxp0 + v010, v011_v111 = (v111 - v011) * sxp0 + v011;

		float v000_v100__v010_v110 = (v010_v110 - v000_v100) * syp0 + v000_v100;
		float v001_v101__v011_v111 = (v011_v111 - v001_v101) * syp0 + v001_v101;

		volOut[dx + dy * volSizeOut + dz * volSizeOut * volSizeOut] = 
			(v001_v101__v011_v111 - v000_v100__v010_v110) * szp0 + v000_v100__v010_v110;
	}
}
void inline trilinearInterpolateVolumeRatio128(float *volOut, float *volIn, float fScale,
	int newDstStartX, int newDstStartY, int newDstStartZ, int newDstStopX, int newDstStopY, int newDstStopZ, int dstStartX, int dstStartY, int dstStartZ)
{
	float sxp0, syp0, szp0;
	int i_sxp1, i_syp1, i_szp1, i_sxp0, i_syp0, i_szp0;

	for (int dz = newDstStartZ; dz < newDstStopZ; dz++) for (int dy = newDstStartY; dy < newDstStopY; dy++) for (int dx = newDstStartX; dx < newDstStopX; dx++) 
	{
		sxp0 = MIN((float)(dx - newDstStartX) * fScale + dstStartX, 127); 
		i_sxp0 = (int)sxp0; i_sxp1 = MIN(i_sxp0 + 1, 127); 

		syp0 = MIN((float)(dy - newDstStartY) * fScale + dstStartY, 127); 
		i_syp0 = (int)syp0; i_syp1 = MIN(i_syp0 + 1, 127); 

		szp0 = MIN((float)(dz - newDstStartZ) * fScale + dstStartZ, 127); 
		i_szp0 = (int)szp0; i_szp1 = MIN(i_szp0 + 1, 127); 

		float v000 = volIn[i_sxp0 + (i_syp0 << 7) + (i_szp0 << 14)];
		float v100 = volIn[i_sxp1 + (i_syp0 << 7) + (i_szp0 << 14)];

		float v001 = volIn[i_sxp0 + (i_syp0 << 7) + (i_szp1 << 14)];
		float v101 = volIn[i_sxp1 + (i_syp0 << 7) + (i_szp1 << 14)];

		float v010 = volIn[i_sxp0 + (i_syp1 << 7) + (i_szp0 << 14)];
		float v110 = volIn[i_sxp1 + (i_syp1 << 7) + (i_szp0 << 14)];

		float v011 = volIn[i_sxp0 + (i_syp1 << 7) + (i_szp1 << 14)];
		float v111 = volIn[i_sxp1 + (i_syp1 << 7) + (i_szp1 << 14)];

		sxp0 = sxp0 - i_sxp0; syp0 = syp0 - i_syp0; szp0 = szp0 - i_szp0;

		float v000_v100 = (v100 - v000) * sxp0 + v000, v001_v101 = (v101 - v001) * sxp0 + v001;
		float v010_v110 = (v110 - v010) * sxp0 + v010, v011_v111 = (v111 - v011) * sxp0 + v011;

		float v000_v100__v010_v110 = (v010_v110 - v000_v100) * syp0 + v000_v100;
		float v001_v101__v011_v111 = (v011_v111 - v001_v101) * syp0 + v001_v101;

		volOut[dx + (dy << 7) + (dz << 14)] = 
			(v001_v101__v011_v111 - v000_v100__v010_v110) * szp0 + v000_v100__v010_v110;
	}
}

void inline trilinearInterpolateVolume2FRatio128(Vector2f *volOut, Vector2f *volIn, float fScale,
	int newDstStartX, int newDstStartY, int newDstStartZ, int newDstStopX, int newDstStopY, int newDstStopZ, int dstStartX, int dstStartY, int dstStartZ)
{
	float sxp0, syp0, szp0;
	int i_sxp1, i_syp1, i_szp1, i_sxp0, i_syp0, i_szp0;

	for (int dz = newDstStartZ; dz < newDstStopZ; dz++) for (int dy = newDstStartY; dy < newDstStopY; dy++) for (int dx = newDstStartX; dx < newDstStopX; dx++) 
	{
		sxp0 = MIN((float)(dx - newDstStartX) * fScale + dstStartX, 127); 
		i_sxp0 = (int)sxp0; i_sxp1 = MIN(i_sxp0 + 1, 127); 

		syp0 = MIN((float)(dy - newDstStartY) * fScale + dstStartY, 127); 
		i_syp0 = (int)syp0; i_syp1 = MIN(i_syp0 + 1, 127); 

		szp0 = MIN((float)(dz - newDstStartZ) * fScale + dstStartZ, 127); 
		i_szp0 = (int)szp0; i_szp1 = MIN(i_szp0 + 1, 127); 

		float v000 = volIn[i_sxp0 + (i_syp0 << 7) + (i_szp0 << 14)].x;
		float v100 = volIn[i_sxp1 + (i_syp0 << 7) + (i_szp0 << 14)].x;

		float v001 = volIn[i_sxp0 + (i_syp0 << 7) + (i_szp1 << 14)].x;
		float v101 = volIn[i_sxp1 + (i_syp0 << 7) + (i_szp1 << 14)].x;

		float v010 = volIn[i_sxp0 + (i_syp1 << 7) + (i_szp0 << 14)].x;
		float v110 = volIn[i_sxp1 + (i_syp1 << 7) + (i_szp0 << 14)].x;

		float v011 = volIn[i_sxp0 + (i_syp1 << 7) + (i_szp1 << 14)].x;
		float v111 = volIn[i_sxp1 + (i_syp1 << 7) + (i_szp1 << 14)].x;

		sxp0 = sxp0 - i_sxp0; syp0 = syp0 - i_syp0; szp0 = szp0 - i_szp0;

		float v000_v100 = (v100 - v000) * sxp0 + v000, v001_v101 = (v101 - v001) * sxp0 + v001;
		float v010_v110 = (v110 - v010) * sxp0 + v010, v011_v111 = (v111 - v011) * sxp0 + v011;

		float v000_v100__v010_v110 = (v010_v110 - v000_v100) * syp0 + v000_v100;
		float v001_v101__v011_v111 = (v011_v111 - v001_v101) * syp0 + v001_v101;

		volOut[dx + (dy << 7) + (dz << 14)].x = 
			(v001_v101__v011_v111 - v000_v100__v010_v110) * szp0 + v000_v100__v010_v110;
	}

	for (int dz = newDstStartZ; dz < newDstStopZ; dz++) for (int dy = newDstStartY; dy < newDstStopY; dy++) for (int dx = newDstStartX; dx < newDstStopX; dx++) 
	{
		sxp0 = MIN((float)(dx - newDstStartX) * fScale + dstStartX, 127); 
		i_sxp0 = (int)sxp0; i_sxp1 = MIN(i_sxp0 + 1, 127); 

		syp0 = MIN((float)(dy - newDstStartY) * fScale + dstStartY, 127); 
		i_syp0 = (int)syp0; i_syp1 = MIN(i_syp0 + 1, 127); 

		szp0 = MIN((float)(dz - newDstStartZ) * fScale + dstStartZ, 127); 
		i_szp0 = (int)szp0; i_szp1 = MIN(i_szp0 + 1, 127); 

		float v000 = volIn[i_sxp0 + (i_syp0 << 7) + (i_szp0 << 14)].y;
		float v100 = volIn[i_sxp1 + (i_syp0 << 7) + (i_szp0 << 14)].y;

		float v001 = volIn[i_sxp0 + (i_syp0 << 7) + (i_szp1 << 14)].y;
		float v101 = volIn[i_sxp1 + (i_syp0 << 7) + (i_szp1 << 14)].y;

		float v010 = volIn[i_sxp0 + (i_syp1 << 7) + (i_szp0 << 14)].y;
		float v110 = volIn[i_sxp1 + (i_syp1 << 7) + (i_szp0 << 14)].y;

		float v011 = volIn[i_sxp0 + (i_syp1 << 7) + (i_szp1 << 14)].y;
		float v111 = volIn[i_sxp1 + (i_syp1 << 7) + (i_szp1 << 14)].y;

		sxp0 = sxp0 - i_sxp0; syp0 = syp0 - i_syp0; szp0 = szp0 - i_szp0;

		float v000_v100 = (v100 - v000) * sxp0 + v000, v001_v101 = (v101 - v001) * sxp0 + v001;
		float v010_v110 = (v110 - v010) * sxp0 + v010, v011_v111 = (v111 - v011) * sxp0 + v011;

		float v000_v100__v010_v110 = (v010_v110 - v000_v100) * syp0 + v000_v100;
		float v001_v101__v011_v111 = (v011_v111 - v001_v101) * syp0 + v001_v101;

		volOut[dx + (dy << 7) + (dz << 14)].y = 
			(v001_v101__v011_v111 - v000_v100__v010_v110) * szp0 + v000_v100__v010_v110;
	}
}

void inline trilinearInterpolateVolumeRatio(float *volOut, float *volIn, int volSizeOut, int volSizeIn, float fScale,
	int newDstStartX, int newDstStartY, int newDstStartZ, int newDstStopX, int newDstStopY, int newDstStopZ, int dstStartX, int dstStartY, int dstStartZ)
{
	float sxp0, syp0, szp0;
	int i_sxp1, i_syp1, i_szp1, i_sxp0, i_syp0, i_szp0;

	for (int dz = newDstStartZ; dz < newDstStopZ; dz++) for (int dy = newDstStartY; dy < newDstStopY; dy++) for (int dx = newDstStartX; dx < newDstStopX; dx++) 
	{
		sxp0 = MIN((float)(dx - newDstStartX) * fScale + dstStartX, volSizeIn - 1); 
		i_sxp0 = (int)sxp0; i_sxp1 = MIN(i_sxp0 + 1, volSizeIn - 1); 

		syp0 = MIN((float)(dy - newDstStartY) * fScale + dstStartY, volSizeIn - 1); 
		i_syp0 = (int)syp0; i_syp1 = MIN(i_syp0 + 1, volSizeIn - 1); 

		szp0 = MIN((float)(dz - newDstStartZ) * fScale + dstStartZ, volSizeIn - 1); 
		i_szp0 = (int)szp0; i_szp1 = MIN(i_szp0 + 1, volSizeIn - 1); 

		float v000 = volIn[i_sxp0 + i_syp0 * volSizeIn + i_szp0 * volSizeIn * volSizeIn];
		float v100 = volIn[i_sxp1 + i_syp0 * volSizeIn + i_szp0 * volSizeIn * volSizeIn];

		float v001 = volIn[i_sxp0 + i_syp0 * volSizeIn + i_szp1 * volSizeIn * volSizeIn];
		float v101 = volIn[i_sxp1 + i_syp0 * volSizeIn + i_szp1 * volSizeIn * volSizeIn];

		float v010 = volIn[i_sxp0 + i_syp1 * volSizeIn + i_szp0 * volSizeIn * volSizeIn];
		float v110 = volIn[i_sxp1 + i_syp1 * volSizeIn + i_szp0 * volSizeIn * volSizeIn];

		float v011 = volIn[i_sxp0 + i_syp1 * volSizeIn + i_szp1 * volSizeIn * volSizeIn];
		float v111 = volIn[i_sxp1 + i_syp1 * volSizeIn + i_szp1 * volSizeIn * volSizeIn];

		sxp0 = sxp0 - i_sxp0; syp0 = syp0 - i_syp0; szp0 = szp0 - i_szp0;

		float v000_v100 = (v100 - v000) * sxp0 + v000, v001_v101 = (v101 - v001) * sxp0 + v001;
		float v010_v110 = (v110 - v010) * sxp0 + v010, v011_v111 = (v111 - v011) * sxp0 + v011;

		float v000_v100__v010_v110 = (v010_v110 - v000_v100) * syp0 + v000_v100;
		float v001_v101__v011_v111 = (v011_v111 - v001_v101) * syp0 + v001_v101;

		volOut[dx + dy * volSizeOut + dz * volSizeOut * volSizeOut] = 
			(v001_v101__v011_v111 - v000_v100__v010_v110) * szp0 + v000_v100__v010_v110;
	}
}



#endif