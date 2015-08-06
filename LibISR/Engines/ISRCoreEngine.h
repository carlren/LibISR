#pragma once

#include "../Objects/ISRLibSettings.h"

#include "../Objects/ISRFrame.h"
#include "../Objects/ISRShapeUnion.h"
#include "../Objects/ISRTrackingState.h"

#include "ISRLowlevelEngine.h"
#include "shared/ISRLowlevelEngine_shared.h"
#include "CPU/ISRLowlevelEngine_CPU.h"
#include "GPU/ISRLowlevelEngine_GPU.h"

#include "ISRRGBDTracker.h"
#include "shared/ISRRGBDTracker_shared.h"
#include "CPU/ISRRGBDTracker_CPU.h"
#include "GPU/ISRRGBDTracker_GPU.h"

#include "ISRColorTracker.h"
#include "shared/ISRColorTracker_shared.h"
#include "CPU/ISRColorTracker_CPU.h"

#include "ISRJointTracker.h"
#include "CPU/ISRColorTracker_CPU.h"

#include "ISRVisualisationEngine.h"
#include "shared/ISRVisualisationEngine_shared.h"
#include "CPU/ISRVisualisationEngine_CPU.h"
#include "GPU/ISRVisualisationEngine_GPU.h"

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
			Engine::ISRVisualisationEngine* visualizationEngine;

			Engine::ISRColorTracker_CPU *tmpTracker;

			float maxposediff;

		public:

			bool needStarTracker;

			Objects::ISRFrame *frame;
			Objects::ISRShapeUnion *shapeUnion;
			Objects::ISRTrackingState *trackingState;

			Objects::ISRView *getView(){ return frame->view;};
			Objects::ISRImageHierarchy *getImageHierarchy(){ return frame->imgHierarchy; }
			Objects::ISRVisualisationState *getRenderingState(){ return frame->rendering; }
			float getEnergy(){ return trackingState->energy; };

			void processFrame(void);

			ISRCoreEngine(const Objects::ISRLibSettings *settings, const Objects::ISRCalib *calib, Vector2i d_dize, Vector2i rgb_size);
			
			~ISRCoreEngine()
			{
				delete this->frame;
				delete this->shapeUnion;
			}
		};
	}
}

