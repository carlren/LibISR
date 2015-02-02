#pragma once

#include "ISRLowlevelEngine.h"

namespace LibISR
{
	namespace Engine
	{
		class ISRLowlevelEngine_CPU:public ISRLowlevelEngine
		{
		public:

			void createForgroundProbabilityMap(ISRFloatImage *pfmap_out, ISRUChar4Image *rgb_in, Objects::ISRHistogram *histogram);
			
			void createCamCordPointCloud(ISRFloat4Image *ptcloud_out, ISRShortImage *raw_depth_in, const Vector4f &instrinsic);
			
			void createAlignedRGBImage(ISRUChar4Image *rgb_out, ISRShortImage *raw_depth_in, ISRUChar4Image *rgb_in, Objects::ISRExHomography *home);

			void createPointCloudWithPf(ISRFloat4Image *ptcloud_out, ISRShortImage *raw_depth_in, ISRUChar4Image *rgb_in, Vector4f intrinsic, Objects::ISRHistogram *histogram);

			void preparePointCloudForRGBDTrackerAllInOne(ISRFloat4Image *ptcloud_out, ISRShortImage *raw_depth_in, ISRUChar4Image *rgb_in, Objects::ISRCalib* calib, Objects::ISRHistogram *histogram, const Vector4i& boundingbox);

			//////////////////////////////////////////////////////////////////////////
			// Below are the functions that are currently used rgbd tracker
			//////////////////////////////////////////////////////////////////////////

			void prepareAlignedRGBDData(ISRFloat4Image *outimg, ISRShortImage *raw_depth_in, ISRUChar4Image *rgb_in, Objects::ISRExHomography *home);

			void subsampleImageRGBDImage(ISRFloat4Image *outimg, ISRFloat4Image *inimg);

			void preparePointCloudFromAlignedRGBDImage(ISRFloat4Image *ptcloud_out, ISRFloat4Image *inimg, Objects::ISRHistogram *histogram, const Vector4f &intrinsic, const Vector4i &boundingbox);

			ISRLowlevelEngine_CPU(){}
			~ISRLowlevelEngine_CPU(){}
		};

	}
}