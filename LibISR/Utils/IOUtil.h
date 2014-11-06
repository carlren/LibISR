#pragma once

#include <stdio.h>
#include <stdlib.h>
#include "..//CoreISR//LibISR.h"

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

///////////////////////////////////////////////////////////////////////////////////////

static inline void WritePGMimage(char* fileName, unsigned short *imgData, int w, int h)
{
	FILE* fid = fopen(fileName, "w");
	fprintf(fid, "P5\n");
	fprintf(fid, "%d %d\n", w, h);
	fprintf(fid, "65535\n");
	fclose(fid);

	fid = fopen(fileName, "ab+");
	fwrite(imgData, sizeof(unsigned short), w*h, fid);
	fclose(fid);
}

static inline void WritePPMimage(char* fileName, char *imgData, int w, int h)
{
	FILE* fid = fopen(fileName, "w");
	fprintf(fid, "P6\n");
	fprintf(fid, "%d %d\n", w, h);
	fprintf(fid, "255\n");
	fclose(fid);

	fid = fopen(fileName, "ab+");
	fwrite(imgData, sizeof(char), w*h*3, fid);
	fclose(fid);
}

static inline void WriteMatlabTXTImg(char* fileName, float *imgData, int w, int h)
{
	FILE* fid = fopen(fileName, "w");

	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < w; j++)
		{
			int idx = i*w + j;
			fprintf(fid, "%f\t", imgData[idx]);
		}
		fprintf(fid, "\n");
	}
	fclose(fid);
}

static inline void WriteMatlabTXTImg(char* fileName, unsigned char *imgData, int w, int h)
{
	FILE* fid = fopen(fileName, "w");

	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < w; j++)
		{
			int idx = i*w + j;
			fprintf(fid, "%d\t", (int)imgData[idx]);
		}
		fprintf(fid, "\n");
	}
	fclose(fid);
}


static inline void PrintArrayToFile(char* fileName, float *data, int nCount)
{
	FILE* fid = fopen(fileName, "w");

	for (int i = 0; i < nCount; i++)
	{
		fprintf(fid, "%f\t", data[i]);
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

