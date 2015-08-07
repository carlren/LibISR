#pragma once

#include "../Objects/ISRVisualisationState.h"

#include "../Objects/ISRFrame.h"
#include "../Objects/ISRTrackingState.h"
#include "../Objects/ISRShapeUnion.h"

namespace LibISR
{
	namespace Engine
	{
		class ISRVisualisationEngine
		{
		public:
			ISRVisualisationEngine(){}
			~ISRVisualisationEngine(){}

			//////////////////////////////////////////////////////////////////////////
			//// virtual functions that are implemented both on CPU and on GPU
			//////////////////////////////////////////////////////////////////////////

			virtual void renderObject(Objects::ISRVisualisationState* rendering, const Matrix4f& invH, const Objects::ISRShape_ptr shape, const Vector4f& intrinsic) = 0;
			virtual void renderDepth(UShortImage* renderedDepth, Objects::ISRVisualisationState* rendering, const Matrix4f& invH, const Objects::ISRShape_ptr shape, const Vector4f& intrinsic) = 0;
			virtual void renderDepthNormalAndObject(UShortImage* renderedDepth,UChar4Image* renderNormal, Objects::ISRVisualisationState* rendering, const Matrix4f& invH, const Objects::ISRShape_ptr shape, const Vector4f& intrinsic) = 0;

			virtual void renderAsSDF(FloatImage* SDFImage, Float4Image* ptCloud, Objects::ISRVisualisationState* rendering, const Matrix4f& invH, const Objects::ISRShape_ptr shape, const Vector4f& intrinsic) = 0;

			void updateMinmaxmImage(Float2Image* minmaximg, const Matrix4f& H, const Matrix3f& K, const Vector2i& imgsize);
		};
	}
}
