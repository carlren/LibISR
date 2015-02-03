#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <fstream>


#include "../LibISR/Utils/LibISRDefine.h"

void SaveImageToFile(const ISRUChar4Image* image, const char* fileName, bool flipVertical = false);
void SaveImageToFile(const ISRShortImage* image, const char* fileName);
void SaveImageToFile(const ISRFloatImage* image, const char* fileName);
bool ReadImageFromFile(ISRUChar4Image* image, const char* fileName);
bool ReadImageFromFile(ISRShortImage *image, const char *fileName);

template <typename T> void ReadFromBIN(T *data, int dataSize, const char *fileName)
{
	FILE *f = fopen(fileName, "rb");
	fread(data, dataSize * sizeof(T), 1, f);
	fclose(f);
}

template <typename T> void WriteToBIN(const T *data, int dataSize, const char *fileName)
{
	FILE *f = fopen(fileName, "wb");
	fwrite(data, dataSize * sizeof(T), 1, f);
	fclose(f);
}

///////////////////////////////////////////////////////////////////////////
// Some Random Functions for Debug
///////////////////////////////////////////////////////////////////////////

template<typename T>
static inline void PrintOneChannelToFile(char* fileName, T *data, int nCount)
{
	FILE* fid = fopen(fileName, "w");

	for (int i = 0; i < nCount; i++)
	{
		fprintf(fid, "%f\n", data[i].y);
	}
	fclose(fid);
}


static inline void PrintArrayToFile(char* fileName, const float *data, int nCount)
{
	FILE* fid = fopen(fileName, "w");

	for (int i = 0; i < nCount; i++)
	{
		fprintf(fid, "%f\n", data[i]);
	}
	fclose(fid);
}

static inline void PrintPointListToFile(char* fileName, Vector3f *data, int nCount)
{
	FILE* fid = fopen(fileName, "w");

	for (int i = 0; i < nCount; i++)
	{
		fprintf(fid, "%f\t", data[i].x);
		fprintf(fid, "%f\t", data[i].y);
		fprintf(fid, "%f\n", data[i].z);
	}
	fclose(fid);
}

static inline void PrintPointListToFile(char* fileName, Vector4f *data, int nCount)
{
	FILE* fid = fopen(fileName, "w");

	for (int i = 0; i < nCount; i++)
	{
		//if (data[i].w!=-1)
		{
			fprintf(fid, "%f\t", data[i].x);
			fprintf(fid, "%f\t", data[i].y);
			fprintf(fid, "%f\t", data[i].z);
			fprintf(fid, "%f\n", data[i].w);
		}
	}
	fclose(fid);
}


template <typename T> 
inline void WriteMatlabTXTImg(char* fileName, T *imgData, int w, int h)
{
	std::ofstream fid;
	fid.open(fileName);

	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < w; j++)
		{
			int idx = i*w + j;
			fid << imgData[idx] << '\t';
		}
		fid << '\n';
	}
	fid.close();
}


static inline void WritePPMimage(char* fileName, Vector4u *imgData, int w, int h)
{
	Vector3u *tmpData = new Vector3u[w*h];
	for (int i = 0; i < w*h; i++) tmpData[i] = imgData[i].toVector3();
	

	FILE* fid = fopen(fileName, "w");
	fprintf(fid, "P6\n");
	fprintf(fid, "%d %d\n", w, h);
	fprintf(fid, "255\n");
	fclose(fid);

	fid = fopen(fileName, "ab+");
	fwrite(tmpData, sizeof(char), w*h * 3, fid);
	fclose(fid);
}
