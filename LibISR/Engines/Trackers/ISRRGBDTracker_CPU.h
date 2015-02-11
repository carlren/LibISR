#pragma once

#include "ISRRGBDTracker.h"

namespace LibISR
{
	namespace Engine
	{
		class ISRRGBDTracker_CPU :public ISRRGBDTracker
		{
		protected:
			// evaluate the energy given current poses and shapes
			// the poses are always taken from tmpPoses
			void evaluateEnergy(float *energy, Objects::ISRTrackingState * trackerState);

			// compute the Hessian and the Jacobian given the current poses and shape
			// the poses are always taken from tmpPoses
			void computeJacobianAndHessian(float *gradient, float *hessian, Objects::ISRTrackingState * trackerState) const;

			void lableForegroundPixels(Objects::ISRTrackingState * trackerState);

			void lableForegroundPixels(Objects::ISRTrackingState * trackerState, Vector4i bb);
		public:
			ISRRGBDTracker_CPU(int nObjs);
			~ISRRGBDTracker_CPU();


		};

	}
}