#pragma once

#include "../Objects/ISRCalib.h"

#include <iostream>

namespace CoreISR
{
	namespace Objects
	{
		bool readIntrinsics(std::istream & src, ISRIntrinsics & dest);
		bool readIntrinsics(const char *fileName, ISRIntrinsics & dest);
		bool readExtrinsics(std::istream & src, ISRExtrinsics & dest);
		bool readExtrinsics(const char *fileName, ISRExtrinsics & dest);
		bool readDisparityCalib(std::istream & src, ISRDisparityCalib & dest);
		bool readDisparityCalib(const char *fileName, ISRDisparityCalib & dest);
		bool readHomographyCalib(std::istream & src, ISRExHomography & dest);
		bool readHomographyCalib(const char *fileName, ISRExHomography & dest);
		bool readRGBDCalib(std::istream & src, ISRCalib & dest);
		bool readRGBDCalib(const char *fileName, ISRCalib & dest);

		bool readRGBDCalib(const char *rgbIntrinsicsFile, const char *depthIntrinsicsFile, const char *disparityCalibFile, const char *extrinsicsFile, ISRCalib & dest);
	}
}

