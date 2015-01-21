#pragma once

#include "../Utils/ISRLibSettings.h"

#include "../Objects/ISRFrame.h"
#include "../Objects/ISRShapeUnion.h"
#include "../Objects/ISRTrackingState.h"

#include "ISRLowlevelEngine.h"
#include "DeviceAgnostic/ISRLowlevelEngine_DA.h"
#include "DeviceSpecific/CPU/ISRLowlevelEngine_CPU.h"

#include "ISRRGBDTracker.h"
#include "DeviceAgnostic/ISRRGBDTracker_DA.h"
#include "DeviceSpecific/CPU/ISRRGBDTracker_CPU.h"


namespace LibISR
{
	namespace Engine
	{

		class ISRCoreEngine
		{
		private:

			Objects::ISRLibSettings *settings;

			Engine::ISRLowlevelEngine* lowLevelEngine;
			Engine::ISRTracker* tracker;

		public:

			Objects::ISRFrame *frame;
			Objects::ISRShapeUnion* shapeUnion;
			Objects::ISRTrackingState* trackingState;

			Objects::ISRView* getView(){ return frame->view;};
			Objects::ISRTrackingState* getTrackingState(){ return trackingState; }

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

