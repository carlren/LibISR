#pragma once

#include <stdio.h>
#include <stdlib.h>

#include "../Utils/LibISRDefine.h"

#include "../Objects/ISRHistogram.h"
#include "../Objects/ISRView.h"

namespace LibISR
{
	namespace Objects
	{
		/** \brief
		Represents intermediat data for a RGB-D frame, 
		including occlusion map, per-pixel forground map

		refactored: Jan/13/2015
		*/
		class ISRFrame
		{
		public:

			int height;
			int width;

			Vector2i depth_size;
			Vector2i rgb_size;

			ISRView* view;

			ISRHistogram *histogram;

			ISRUCharImage *occMap;
			ISRFloatImage *pfImage;
			ISRFloatImage *idxImage;

			ISRFrame(const ISRCalib &calib, Vector2i color_size, Vector2i d_size, bool  useGPU = false)
			{
				depth_size = d_size;
				rgb_size = color_size;

				occMap = new ISRUCharImage(d_size, useGPU);
				occMap->Clear(1);

				pfImage = new ISRFloatImage(d_size,useGPU);
				idxImage = new ISRFloatImage(d_size, useGPU);
				view = new ISRView(calib, color_size, d_size, useGPU);
			}


			~ISRFrame()
			{
				delete this->occMap;
				delete this->pfImage;
				delete this->idxImage;
				delete this->view;

			}
		};
	}
}

