// Copyright 2014-2015 Isis Innovation Limited and the authors of LibISR

#pragma once
#include "../ISRVisualisationEngine.h"

namespace LibISR
{
	namespace Engine
	{
		class ISRVisualisationEngine_CPU: public ISRVisualisationEngine
		{
		public:

			void renderObject(Objects::ISRVisualisationState* rendering, const Matrix4f& invH, const Objects::ISRShape_ptr shape, const Vector4f& intrinsic);
			void renderDepth(UShortImage* renderedDepth, Objects::ISRVisualisationState* rendering, const Matrix4f& invH, const Objects::ISRShape_ptr shape, const Vector4f& intrinsic);
			void renderDepthNormalAndObject(UShortImage* renderedDepth, UChar4Image* renderNormal, Objects::ISRVisualisationState* rendering, const Matrix4f& invH, const Objects::ISRShape_ptr shape, const Vector4f& intrinsic);

			void renderAsSDF(FloatImage* SDFImage, Float4Image* ptCloud, Objects::ISRVisualisationState* rendering, const Matrix4f& invH, const Objects::ISRShape_ptr shape, const Vector4f& intrinsic);
		};

	}

}
