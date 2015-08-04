#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//////////////////////////////////////////////////////////////////////////
// Some settings for LibISR, some of them will be moved to an setting class
//////////////////////////////////////////////////////////////////////////

#define MAX_OBJECT_COUNT 2
#define HISTOGRAM_BIN 32
#define MAX_IMG_PTS 307200
#define DT_VOL_SIZE 200
#define VOL_SCALE 1000
#define BLOCK_SIZE_SDF 8
#define BLOCK_SIZE_IMG 16
#define KINECT_PLAYER_INDEX_SHIFT          3
#define DTUNE 0.2f
#define MAX_SDF 128.0f
#define NUM_OBJ 2

#define  TMP_WEIGHT 1.3f

//------------------------------------------------------
// 
// math defines
//
//------------------------------------------------------

typedef unsigned char uchar;
typedef unsigned short ushort;
typedef unsigned int uint;
typedef unsigned long ulong;

#include "../../ORUtils/PlatformIndependence.h"
#include "../../ORUtils/Vector.h"
#include "../../ORUtils/Matrix.h"

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


#include "../../ORUtils/Image.h"
#include "../../ORUtils/MathUtils.h"

//////////////////////////////////////////////////////////////////////////
// Types for mask images
//////////////////////////////////////////////////////////////////////////

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef BLACK
#define BLACK 0
#endif

#ifndef WHITE
#define WHITE 255
#endif

#ifndef GREY1
#define GREY1 127
#endif

#ifndef GREY2
#define GREY2 128
#endif

#ifndef HIST_FG_PIXEL
#define HIST_FG_PIXEL -2
#endif

#ifndef HIST_BG_PIXEL
#define HIST_BG_PIXEL -3
#endif

#ifndef HIST_USELESS_PIXEL
#define HIST_USELESS_PIXEL -1
#endif

//////////////////////////////////////////////////////////////////////////
// Image types used by LibISR
//////////////////////////////////////////////////////////////////////////

#ifndef FloatImage
#define FloatImage ORUtils::Image<float>
#endif

#ifndef Float2Image
#define Float2Image ORUtils::Image<Vector2f>
#endif

#ifndef Float4Image
#define Float4Image ORUtils::Image<Vector4f>
#endif

#ifndef ShortImage
#define ShortImage ORUtils::Image<short>
#endif

#ifndef Short3Image
#define Short3Image ORUtils::Image<Vector3s>
#endif

#ifndef Short4Image
#define Short4Image ORUtils::Image<Vector4s>
#endif

#ifndef UShortImage
#define UShortImage ORUtils::Image<ushort>
#endif

#ifndef UIntImage
#define UIntImage ORUtils::Image<uint>
#endif

#ifndef IntImage
#define IntImage ORUtils::Image<int>
#endif

#ifndef UCharImage
#define UCharImage ORUtils::Image<uchar>
#endif

#ifndef UChar4Image
#define UChar4Image ORUtils::Image<Vector4u>
#endif

#ifndef BoolImage
#define BoolImage ORUtils::Image<bool>
#endif


//////////////////////////////////////////////////////////////////////////
// Ramdom functions
//////////////////////////////////////////////////////////////////////////

#ifndef VOLVAL
#define VOLVAL(vol,x,y,z) (vol[((z) * DT_VOL_SIZE + (y)) * DT_VOL_SIZE + (x)])
#endif

#ifndef DEBUGBREAK
#define DEBUGBREAK \
{ \
	int ryifrklaeybfcklarybckyar=0; \
	ryifrklaeybfcklarybckyar++; \
}
#endif