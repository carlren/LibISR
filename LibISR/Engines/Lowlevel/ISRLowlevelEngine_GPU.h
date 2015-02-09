#pragma once

#include "ISRLowlevelEngine.h"

namespace LibISR
{
	namespace Engine
	{
		class ISRLowlevelEngine_GPU:public ISRLowlevelEngine
		{
		public:
			void computepfImageFromHistogram(ISRUChar4Image *rgb_in, Objects::ISRHistogram *histogram);

			void prepareAlignedRGBDData(ISRFloat4Image *outimg, ISRShortImage *raw_depth_in, ISRUChar4Image *rgb_in, Objects::ISRExHomography *home);

			void subsampleImageRGBDImage(ISRFloat4Image *outimg, ISRFloat4Image *inimg);

			void preparePointCloudFromAlignedRGBDImage(ISRFloat4Image *ptcloud_out, ISRFloat4Image *inimg, Objects::ISRHistogram *histogram, const Vector4f &intrinsic, const Vector4i &boundingbox);

			ISRLowlevelEngine_GPU(){}
			~ISRLowlevelEngine_GPU(){}
		};

	}
}