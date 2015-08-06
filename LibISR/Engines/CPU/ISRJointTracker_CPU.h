#pragma once

#include "../ISRJointTracker.h"

namespace LibISR
{
	namespace Engine
	{
		class ISRJointTracker_CPU :public ISRJointTracker
		{
		protected:

			void initializeTracker(const Vector2i& imageSize);

			void updatePfImage();

			void raycastTo2DHeaviside(Objects::ISRTrackingState *trackingState);

			// evaluate the energy given current poses and shapes
			// the poses are always taken from tmpPoses
			float evaluateEnergy(Objects::ISRTrackingState * trackerState);

			// compute the Hessian and the Jacobian given the current poses and shape
			// the poses are always taken from tmpPoses
			float computeJacobianAndHessian(float *gradient, float *hessian, Objects::ISRTrackingState * trackerState);
			
			void lableForegroundPixels(Objects::ISRTrackingState * trackerState);

		public:
			ISRJointTracker_CPU(int nObjs, Vector2i imageSize);
			~ISRJointTracker_CPU();
		};

	}
}
