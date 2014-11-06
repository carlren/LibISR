#pragma once

#include <stdlib.h>
#include <math.h>

#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
#define _CPU_AND_GPU_CODE_ __device__	// for CUDA device code
#else
#define _CPU_AND_GPU_CODE_ 
#endif

#include "ISRMath.h"

#ifndef MAX_OBJECT_COUNT
#define MAX_OBJECT_COUNT 2
#endif

#ifndef HISTOGRAM_BIN
#define HISTOGRAM_BIN 16
#endif

#ifndef MAX_IMG_PTS
#define MAX_IMG_PTS 307200
#endif

#ifndef DT_VOL_SIZE
#define DT_VOL_SIZE 200
#endif

#ifndef BLOCK_SIZE_IMG
#define BLOCK_SIZE_IMG 16
#endif

#ifndef HIST_BINS
#define HIST_BINS 32
#endif

#ifndef MAX_BLOCK_SIZE
#define MAX_BLOCK_SIZE 1024
#endif

#ifndef VOL_SCALE
#define VOL_SCALE 1000
#endif


//debug
#ifndef DEBUGBREAK
#define DEBUGBREAK \
{ \
	int ryifrklaeybfcklarybckyar=0; \
	ryifrklaeybfcklarybckyar++; \
}
#endif

//types

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


#ifndef ISRFloatImage
#define ISRFloatImage ISRImage<float>
#endif

#ifndef ISRFloat2Image
#define ISRFloat2Image ISRImage<Vector2f>
#endif

#ifndef ISRFloat4Image
#define ISRFloat4Image ISRImage<Vector4f>
#endif

#ifndef ISRShortImage
#define ISRShortImage ISRImage<short>
#endif

#ifndef ISRShort3Image
#define ISRShort3Image ISRImage<Vector3s>
#endif

#ifndef ISRShort4Image
#define ISRShort4Image ISRImage<Vector4s>
#endif

#ifndef ISRUShortImage
#define ISRUShortImage ISRImage<ushort>
#endif

#ifndef ISRUIntImage
#define ISRUIntImage ISRImage<uint>
#endif

#ifndef ISRIntImage
#define ISRIntImage ISRImage<int>
#endif

#ifndef ISRUCharImage
#define ISRUCharImage ISRImage<uchar>
#endif

#ifndef ISRUChar4Image
#define ISRUChar4Image ISRImage<Vector4u>
#endif

#ifndef ISRBoolImage
#define ISRBoolImage ISRImage<bool>
#endif


#ifndef VOLVAL
#define VOLVAL(vol,x,y,z) (vol[((z) * DT_VOL_SIZE + (y)) * DT_VOL_SIZE + (x)])
#endif


#ifndef KINECT_PLAYER_INDEX_SHIFT
#define KINECT_PLAYER_INDEX_SHIFT          3
#endif

#ifndef  USHORT
typedef unsigned short USHORT;
#endif

