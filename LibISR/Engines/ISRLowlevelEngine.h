// Copyright 2014-2015 Isis Innovation Limited and the authors of LibISR

#pragma once

#include "../Objects/ISRFrame.h"
#include "../Objects/ISRTrackingState.h"

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

			virtual void computepfImageFromHistogram(UChar4Image *rgb_in, Objects::ISRHistogram *histogram) = 0;

			virtual void prepareAlignedRGBDData(Float4Image *outimg, ShortImage *raw_depth_in, UChar4Image *rgb_in, Objects::ISRExHomography *home) = 0;
			
			virtual void subsampleImageRGBDImage(Float4Image *outimg, Float4Image *inimg) = 0;

			virtual void preparePointCloudFromAlignedRGBDImage(Float4Image *ptcloud_out, Float4Image *inimg, Objects::ISRHistogram *histogram, const Vector4f &intrinsic  ,const Vector4i &boundingbox) = 0;
		};
	}

}
