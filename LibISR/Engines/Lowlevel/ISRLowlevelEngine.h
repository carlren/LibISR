#pragma once

#include "../../Objects/Highlevel/ISRFrame.h"
#include "../../Objects/Highlevel/ISRTrackingState.h"

namespace LibISR
{
	namespace Engine
	{
		class ISRLowlevelEngine
		{
		public:

			Vector4i findBoundingBoxFromCurrentState(const Objects::ISRTrackingState* state, const Matrix3f& K, const Vector2i& imgsize);

			//////////////////////////////////////////////////////////////////////////
			//// virtual functions that are implemented both on CPU and on GPU
			//////////////////////////////////////////////////////////////////////////

			virtual void prepareAlignedRGBDData(ISRFloat4Image *outimg, ISRShortImage *raw_depth_in, ISRUChar4Image *rgb_in, Objects::ISRExHomography *home) = 0;
			
			virtual void subsampleImageRGBDImage(ISRFloat4Image *outimg, ISRFloat4Image *inimg) = 0;

			virtual void preparePointCloudFromAlignedRGBDImage(ISRFloat4Image *ptcloud_out, ISRFloat4Image *inimg, Objects::ISRHistogram *histogram, const Vector4f &intrinsic  ,const Vector4i &boundingbox) = 0;
		};
	}

}