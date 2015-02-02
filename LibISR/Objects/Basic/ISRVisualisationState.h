#pragma once
#include "../../Utils/LibISRDefine.h"

namespace LibISR
{
	namespace Objects
	{
		class ISRVisualisationState {
		public:
			ISRFloat2Image *minmaxImage;
			ISRUChar4Image *outputImage;

			ISRVisualisationState(const Vector2i & imgSize, bool allocateGPU)
			{
				minmaxImage = new ISRFloat2Image(imgSize, allocateGPU);
				outputImage = new ISRUChar4Image(imgSize, allocateGPU);
			}

			virtual ~ISRVisualisationState(void)
			{
				delete minmaxImage;
				delete outputImage;
			}
		};
	}
}
