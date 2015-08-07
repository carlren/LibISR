#pragma once
#include "../LibISRDefine.h"

namespace LibISR
{
	namespace Objects
	{
		class ISRVisualisationState {
		public:
			Float2Image *minmaxImage;
			UChar4Image *outputImage;

			ISRVisualisationState(const Vector2i & imgSize, bool allocateGPU)
			{
				minmaxImage = new Float2Image(imgSize, true, allocateGPU);
				outputImage = new UChar4Image(imgSize, true, allocateGPU);
			}

			virtual ~ISRVisualisationState(void)
			{
				delete minmaxImage;
				delete outputImage;
			}
		};
	}
}
