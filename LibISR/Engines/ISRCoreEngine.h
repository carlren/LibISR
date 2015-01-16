#pragma once

#include "../Utils/ISRLibSettings.h"

#include "../Objects/ISRFrame.h"
#include "../Objects/ISRShapeUnion.h"
#include "../Objects/ISRTrackingState.h"


namespace LibISR
{
	namespace Engine
	{

		class ISRCoreEngine
		{
		private:

			Objects::ISRLibSettings *settings;

		public:
			
			Objects::ISRFrame *frame;
			Objects::ISRShapeUnion* shapeUnion;
			Objects::ISRTrackingState* trackingState;

			Objects::ISRView* GetView(){ return frame->view;};

			void ProcessFrame(void);

			ISRCoreEngine(const Objects::ISRLibSettings *settings, const Objects::ISRCalib *calib, Vector2i d_dize, Vector2i rgb_size);
			
			~ISRCoreEngine()
			{
				delete this->frame;
				delete this->shapeUnion;
			}
		};
	}
}

