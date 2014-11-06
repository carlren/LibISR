#pragma once

#include <stdio.h>
#include <stdlib.h>

#include "..//Utils//LibISRDefine.h"
#include "..//Utils//MathUtils.h"

#include "..//Objects//ISRHistogram.h"
#include "..//Objects//ISRImage.h"
#include "../Objects//ISRCalib.h"

using namespace CoreISR::Objects;

namespace CoreISR
{
	namespace Objects
	{

		class ISRView
		{
		public:
			enum InputDepthType
			{
				ISR_DISPARITY_DEPTH,
				ISR_SHORT_DEPTH,
				ISR_FLOAT_DEPTH,
			};

			enum InputImageType
			{
				ISR_DEPTH_ONLY,
				ISR_RGBD_EXTRINSIC,
				ISR_RGBD_HOMOGRAPHY,
			};

			InputDepthType inputDepthType;
			InputImageType inputImageType;

			ISRCalib *calib;

			ISRUChar4Image *rgb;
			ISRShortImage *rawDepth;

			ISRFloatImage *depth;
			ISRUChar4Image *alignedRgb;


			ISRView(const ISRCalib &calib, Vector2i rgb_size, Vector2i d_size, bool useGPU)
			{
				this->calib = new ISRCalib(calib);
				this->rgb = new ISRUChar4Image(rgb_size, useGPU);
				this->rawDepth = new ISRShortImage(d_size, useGPU);
				this->depth = new ISRFloatImage(d_size, useGPU);
				this->alignedRgb = new ISRUChar4Image(d_size, useGPU);
			}


			~ISRView()
			{
				delete calib;
				delete rgb;
				delete rawDepth;
				delete depth;
				delete alignedRgb;
			}

			ISRView(const ISRView&);
			ISRView& operator=(const ISRView&);
		};
	}
}

