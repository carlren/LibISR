#pragma once

#include "../Utils/ISRLibSettings.h"
#include "../Objects/ISRFrame.h"
#include "../Objects/ISRView.h"
#include "../Objects/ISRHistogram.h"
#include "../Objects/ISRShapeUnion.h"
#include "../Objects/ISROptimizationHelper.h"


namespace LibISR
{
	namespace Engine
	{

		class ISRCoreEngine
		{
		private:

			ISRLibSettings *settings;

		public:
			
			Objects::ISRFrame *frame;
			Objects::ISRShapeUnion* shapeUnion;
			Objects::ISROptimizationHelper *optimizationHelper;

			Objects::ISRView* GetView(){ return frame->view;};

			void ProcessFrame(void);

			ISRCoreEngine(const ISRLibSettings *settings, const Objects::ISRCalib *calib, Vector2i d_dize, Vector2i rgb_size);
			
			~ISRCoreEngine()
			{
				delete this->frame;
				delete this->shapeUnion;
				delete this->optimizationHelper;
			}
		};
	}
}

