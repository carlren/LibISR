// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "ISRCalibIO.h"

#include <fstream>

using namespace LibISR::Objects;

bool LibISR::Objects::readIntrinsics(std::istream & src, ISRIntrinsics & dest)
{
	float sizeX, sizeY;
	float focalLength[2], centerPoint[2];

	src >> sizeX >> sizeY;
	src >> focalLength[0] >> focalLength[1];
	src >> centerPoint[0] >> centerPoint[1];
	if (src.fail()) return false;

	dest.SetFrom(focalLength[0], focalLength[1], centerPoint[0], centerPoint[1], sizeX, sizeY);
	return true;
}

bool LibISR::Objects::readIntrinsics(const char *fileName, ISRIntrinsics & dest)
{
	std::ifstream f(fileName);
	return LibISR::Objects::readIntrinsics(f, dest);
}

bool LibISR::Objects::readExtrinsics(std::istream & src, ISRExtrinsics & dest)
{
	Matrix4f calib;
	src >> calib.m00 >> calib.m10 >> calib.m20 >> calib.m30;
	src >> calib.m01 >> calib.m11 >> calib.m21 >> calib.m31;
	src >> calib.m02 >> calib.m12 >> calib.m22 >> calib.m32;
	calib.m03 = 0.0f; calib.m13 = 0.0f; calib.m23 = 0.0f; calib.m33 = 1.0f;
	if (src.fail()) return false;

	dest.SetFrom(calib);
	return true;
}

bool LibISR::Objects::readExtrinsics(const char *fileName, ISRExtrinsics & dest)
{
	std::ifstream f(fileName);
	return LibISR::Objects::readExtrinsics(f, dest);
}

bool LibISR::Objects::readHomographyCalib(std::istream & src, ISRExHomography & dest)
{
	Matrix3f calib;
	src >> calib.m00 >> calib.m10 >> calib.m20;
	src >> calib.m01 >> calib.m11 >> calib.m21;
	src >> calib.m02 >> calib.m12 >> calib.m22;
	
	Vector3f T;
	src >> T.x >> T.y >> T.z;

	dest.SetFrom(calib, T);

	if (src.fail()) return false;
	return true;
}

bool LibISR::Objects::readHomographyCalib(const char *fileName, ISRExHomography & dest)
{
	std::ifstream f(fileName);
	return LibISR::Objects::readHomographyCalib(f, dest);
}

bool LibISR::Objects::readDisparityCalib(std::istream & src, ISRDisparityCalib & dest)
{
	float a, b;
	src >> a >> b;
	if (src.fail()) return false;

	dest.SetFrom(a, b);
	return true;
}

bool LibISR::Objects::readDisparityCalib(const char *fileName, ISRDisparityCalib & dest)
{
	std::ifstream f(fileName);
	return LibISR::Objects::readDisparityCalib(f, dest);
}

bool LibISR::Objects::readRGBDCalib(std::istream & src, ISRCalib & dest)
{
	if (!LibISR::Objects::readIntrinsics(src, dest.intrinsics_rgb)) return false;
	if (!LibISR::Objects::readIntrinsics(src, dest.intrinsics_d)) return false;
	if (!LibISR::Objects::readExtrinsics(src, dest.trafo_rgb_to_depth)) return false;
	if (!LibISR::Objects::readDisparityCalib(src, dest.disparityCalib)) return false;
	if (!LibISR::Objects::readHomographyCalib(src, dest.homo_depth_to_color)) return false;
	return true;
}

bool LibISR::Objects::readRGBDCalib(const char *fileName, ISRCalib & dest)
{
	std::ifstream f(fileName);
	return LibISR::Objects::readRGBDCalib(f, dest);
}

bool LibISR::Objects::readRGBDCalib(const char *rgbIntrinsicsFile, const char *depthIntrinsicsFile, const char *disparityCalibFile, const char *extrinsicsFile, ISRCalib & dest)
{
	bool ret = true;
	ret &= LibISR::Objects::readIntrinsics(rgbIntrinsicsFile, dest.intrinsics_rgb);
	ret &= LibISR::Objects::readIntrinsics(depthIntrinsicsFile, dest.intrinsics_d);
	ret &= LibISR::Objects::readExtrinsics(extrinsicsFile, dest.trafo_rgb_to_depth);
	ret &= LibISR::Objects::readDisparityCalib(disparityCalibFile, dest.disparityCalib);
	return ret;
}

