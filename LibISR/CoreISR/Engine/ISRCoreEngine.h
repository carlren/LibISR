#pragma once

#include "../Utils/ISRLibSettings.h"
#include "../Objects/ISRFrame.h"
#include "../Objects/ISRView.h"
#include "../Objects/ISRHistogram.h"

namespace CoreISR
{
	namespace Engine
	{

		class ISRCoreEngine
		{
		private:

			ISRLibSettings *settings;

		public:
			
			ISRFrame *frame;
			ISRHistogram **histograms;

			ISRView* GetView(){ return frame->view;};

			void ProcessFrame(void);

			ISRCoreEngine(const ISRLibSettings *settings, const ISRCalib *calib, Vector2i d_dize, Vector2i rgb_size);
			
			~ISRCoreEngine()
			{
				delete this->frame;
				for (int i = 0; i < this->settings->noHistogramDim; i++) 
					delete this->histograms[i];
			}
		};
	}
}

