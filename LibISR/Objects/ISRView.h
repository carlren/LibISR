#pragma once
#include "../LibISRDefine.h"

#include "ISRHistogram.h"
#include "ISRCalib.h"

namespace LibISR
{
	/**
	\brief
		view carries the basic image information
		including RGB, depth, raw depth and aligned RGB
		also includes the calibration parameters

		refactored: Jan/13/2015
	*/

	namespace Objects
	{

		class ISRView
		{
		public:
			enum InputDepthType
			{
				ISR_DISPARITY_DEPTH,
				ISR_SHORT_DEPTH,
			};

			InputDepthType inputDepthType;

			ISRCalib *calib;

			UChar4Image *rgb;
			ShortImage *rawDepth;

			FloatImage *depth;
			UChar4Image *alignedRgb;

			ISRView(const ISRCalib &calib, Vector2i rgb_size, Vector2i d_size, bool  useGPU = false)
			{
				this->calib = new ISRCalib(calib);
				this->rgb = new UChar4Image(rgb_size, true,useGPU);
				this->rawDepth = new ShortImage(d_size, true, useGPU);
				this->depth = new FloatImage(d_size, true, useGPU);
				this->alignedRgb = new UChar4Image(d_size, true, useGPU);
			}

			~ISRView()
			{
				delete calib;

				delete rgb;
				delete depth;

				delete rawDepth;
				delete alignedRgb;
			}

			ISRView(const ISRView&);
			ISRView& operator=(const ISRView&);
		};
	}
}

