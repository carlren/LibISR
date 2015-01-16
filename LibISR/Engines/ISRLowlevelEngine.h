#pragma once

#include "../Objects/ISRFrame.h"

namespace LibISR
{
	namespace Engine
	{
		class ISRLowlevelEngine
		{
		public:
			ISRLowlevelEngine(){}
			~ISRLowlevelEngine(){}

			virtual void createForgroundProbabilityMap(ISRFloatImage *pfmap_out, ISRUChar4Image *rgb_in, Objects::ISRHistogram *histogram) = 0;

			//virtual void createCamCordPointCloud(ISRFloat4Image *ptcloud_out,ISRFloatImage *depth_in, Vector4f instrinsic) = 0;
			virtual void createCamCordPointCloud(ISRFloat4Image *ptcloud_out, ISRShortImage *raw_depth_in, Vector4f instrinsic) = 0;

			//virtual void createAlignedRGBImage(ISRUChar4Image *rgb_out, ISRFloatImage *depth_in, ISRUChar4Image *rgb_in, Objects::ISRExHomography *home) = 0;
			virtual void createAlignedRGBImage(ISRUChar4Image *rgb_out, ISRShortImage *raw_depth_in, ISRUChar4Image *rgb_in, Objects::ISRExHomography *home) = 0;
			
			//virtual void createPointCloudWithPf(ISRFloat4Image *ptcloud_out, ISRFloatImage *depth_in, ISRUChar4Image *rgb_in, Vector4f intrinsic, Objects::ISRHistogram *histogram) = 0;
			virtual void createPointCloudWithPf(ISRFloat4Image *ptcloud_out, ISRShortImage *raw_depth_in, ISRUChar4Image *rgb_in, Vector4f intrinsic, Objects::ISRHistogram *histogram) = 0;

			virtual void preparePointCloudForRGBDTrackerAllInOne(ISRFloat4Image *ptcloud_out, ISRShortImage *raw_depth_in, ISRUChar4Image *rgb_in, Objects::ISRCalib* calib, Objects::ISRHistogram *histogram) = 0;
		};
	}

}