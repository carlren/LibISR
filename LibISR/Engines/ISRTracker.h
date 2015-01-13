#pragma once

#include "../Utils/LibISRDefine.h"

#include "../Objects/ISRFrame.h"
#include "../Objects/ISRShapeUnion.h"

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
			virtual void TrackObjects(LibISR::Objects::ISRFrame &frame, LibISR::Objects::ISRShapeUnion &shapeUnion) = 0;

			virtual ~ISRTracker(void) {}
		};
	}
}

