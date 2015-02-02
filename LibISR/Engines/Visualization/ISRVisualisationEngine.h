#pragma once

#include "../../Objects/Basic/ISRVisualisationState.h"

#include "../../Objects/Highlevel/ISRFrame.h"
#include "../../Objects/Highlevel/ISRTrackingState.h"
#include "../../Objects/Highlevel/ISRShapeUnion.h"

namespace LibISR
{
	namespace Engine
	{
		class ISRVisualisationEngine
		{
		public:
			ISRVisualisationEngine(){}
			~ISRVisualisationEngine(){}

			virtual void renderImage(Objects::ISRVisualisationState* rendering, const Objects::ISRTrackingState* state, const Objects::ISRShapeUnion* shapes, const Vector4f& intrinsic) = 0;
			virtual void renderObject(Objects::ISRVisualisationState* rendering, const Matrix4f& invH, const Objects::ISRShape_ptr shape, const Vector4f& intrinsic) = 0;
		};
	}
}