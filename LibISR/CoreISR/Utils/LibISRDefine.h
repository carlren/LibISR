#ifndef __ISR_DEFINES__
#define __ISR_DEFINES__

#include <stdlib.h>
#include <math.h>

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
#define CLAMP(x,a,b) ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))
#endif

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

#ifndef DEGTORAD
#define DEGTORAD (float)(0.017453292519943295769236907684886)
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
#ifndef NULL
#define NULL 0
#endif

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


#ifndef ISRUCharImage
#define ISRUCharImage ISRImage<unsigned char>
#endif

#ifndef ISRFloatImage
#define ISRFloatImage ISRImage<float>
#endif

#ifndef ISRFloat2Image
#define ISRFloat2Image ISRImage<Vector2f>
#endif

#ifndef ISRUChar4Image
#define ISRUChar4Image ISRImage<Vector4u>
#endif

#ifndef ISRUShortImage
#define ISRUShortImage ISRImage<unsigned short>
#endif

#ifndef ISRInt3Image
#define ISRInt3Image ISRImage<Vector3i>
#endif

#ifndef ISRFloat3Image
#define ISRFloat3Image ISRImage<Vector3f>
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


#endif