#pragma once

#include"../Objects/ISRFrame.h"
#include"../Objects/ISRShapeUnion.h"
#include"../Objects/ISRTrackingState.h"

namespace LibISR
{
	namespace Engine
	{
		/** \brief
		Basic interface to any sort of trackers that will track
		a single or multiple objects
		*/
		class ISRTracker
		{
		public:
			/** Track the 6-DoF poses of all the objects in the shape union.
			*/
			virtual void TrackObjects(Objects::ISRFrame *frame, Objects::ISRShapeUnion *shapeUnion, Objects::ISRTrackingState *trackerState, bool updateappearance = true) = 0;

			virtual ~ISRTracker(void) {}
		};
	}
}
