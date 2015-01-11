#pragma once

#include "Vector.h"
#include "Matrix.h"
#include "MathUtils.h"

typedef unsigned char uchar;
typedef unsigned short ushort;
typedef unsigned int uint;
typedef unsigned long ulong;

#ifndef NULL
#define NULL 0
#endif

typedef class ORUtils::Matrix3<float> Matrix3f;
typedef class ORUtils::Matrix4<float> Matrix4f;

typedef class ORUtils::Vector2<short> Vector2s;
typedef class ORUtils::Vector2<int> Vector2i;
typedef class ORUtils::Vector2<float> Vector2f;
typedef class ORUtils::Vector2<double> Vector2d;

typedef class ORUtils::Vector3<short> Vector3s;
typedef class ORUtils::Vector3<double> Vector3d;
typedef class ORUtils::Vector3<int> Vector3i;
typedef class ORUtils::Vector3<uint> Vector3ui;
typedef class ORUtils::Vector3<uchar> Vector3u;
typedef class ORUtils::Vector3<float> Vector3f;

typedef class ORUtils::Vector4<float> Vector4f;
typedef class ORUtils::Vector4<int> Vector4i;
typedef class ORUtils::Vector4<short> Vector4s;
typedef class ORUtils::Vector4<uchar> Vector4u;

