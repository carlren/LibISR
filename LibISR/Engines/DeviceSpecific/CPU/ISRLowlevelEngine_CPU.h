#pragma once

#include "../../ISRLowlevelEngine.h"

namespace LibISR
{
	namespace Engine
	{
		class ISRLowlevelEngine_CPU:public ISRLowlevelEngine
		{
		public:

			void createForgroundProbabilityMap(ISRFloatImage *pfmap_out, ISRUChar4Image *rgb_in, Objects::ISRHistogram *histogram);

			void createCamCordPointCloud(ISRFloat4Image *ptcloud_out, ISRFloatImage *depth_in, Vector4f instrinsic);
			void createCamCordPointCloud(ISRFloat4Image *ptcloud_out, ISRShortImage *raw_depth_in, Vector4f instrinsic);

			void createAlignedRGBImage(ISRUChar4Image *rgb_out, ISRFloatImage *depth_in, ISRUChar4Image *rgb_in, Objects::ISRExHomography *home);
			void createAlignedRGBImage(ISRUChar4Image *rgb_out, ISRShortImage *raw_depth_in, ISRUChar4Image *rgb_in, Objects::ISRExHomography *home);

			void createPointCloudWithPf(ISRFloat4Image *ptcloud_out, ISRFloatImage *depth_in, ISRUChar4Image *rgb_in, Vector4f intrinsic, Objects::ISRHistogram *histogram);
			void createPointCloudWithPf(ISRFloat4Image *ptcloud_out, ISRShortImage *raw_depth_in, ISRUChar4Image *rgb_in, Vector4f intrinsic, Objects::ISRHistogram *histogram);

			void preparePointCloudForRGBDTrackerAllInOne(ISRFloat4Image *ptcloud_out, ISRShortImage *raw_depth_in, ISRUChar4Image *rgb_in, Objects::ISRCalib* calib, Objects::ISRHistogram *histogram);

			ISRLowlevelEngine_CPU(){}
			~ISRLowlevelEngine_CPU(){}
		};

	}
}