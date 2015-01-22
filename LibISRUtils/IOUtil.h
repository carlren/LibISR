#pragma once

#include <stdio.h>
#include <stdlib.h>

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
		if (data[i].w!=-1)
		{
			fprintf(fid, "%f\t", data[i].x);
			fprintf(fid, "%f\t", data[i].y);
			fprintf(fid, "%f\t", data[i].z);
			fprintf(fid, "%f\n", data[i].w);
		}
	}
	fclose(fid);
}
