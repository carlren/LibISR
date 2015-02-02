#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "../../ORUtils/MathDefine.h"
#include "../../ORUtils/Image.h"
#include "../../ORUtils/MathUtils.h"

//////////////////////////////////////////////////////////////////////////
// Some settings for LibISR, some of them will be moved to an setting class
//////////////////////////////////////////////////////////////////////////

#define MAX_OBJECT_COUNT 2
#define HISTOGRAM_BIN 16
#define MAX_IMG_PTS 307200
#define DT_VOL_SIZE 200
#define VOL_SCALE 1000
#define BLOCK_SIZE_SDF 8
#define BLOCK_SIZE_IMG 16
#define KINECT_PLAYER_INDEX_SHIFT          3
#define DTUNE 0.5f
#define MAX_SDF 121.0f

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

#ifndef ISRFloatImage
#define ISRFloatImage ORUtils::Image<float>
#endif

#ifndef ISRFloat2Image
#define ISRFloat2Image ORUtils::Image<Vector2f>
#endif

#ifndef ISRFloat4Image
#define ISRFloat4Image ORUtils::Image<Vector4f>
#endif

#ifndef ISRShortImage
#define ISRShortImage ORUtils::Image<short>
#endif

#ifndef ISRShort3Image
#define ISRShort3Image ORUtils::Image<Vector3s>
#endif

#ifndef ISRShort4Image
#define ISRShort4Image ORUtils::Image<Vector4s>
#endif

#ifndef ISRUShortImage
#define ISRUShortImage ORUtils::Image<ushort>
#endif

#ifndef ISRUIntImage
#define ISRUIntImage ORUtils::Image<uint>
#endif

#ifndef ISRIntImage
#define ISRIntImage ORUtils::Image<int>
#endif

#ifndef ISRUCharImage
#define ISRUCharImage ORUtils::Image<uchar>
#endif

#ifndef ISRUChar4Image
#define ISRUChar4Image ORUtils::Image<Vector4u>
#endif

#ifndef ISRBoolImage
#define ISRBoolImage ORUtils::Image<bool>
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