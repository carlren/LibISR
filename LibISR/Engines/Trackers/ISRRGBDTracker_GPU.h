#pragma once

#include "ISRRGBDTracker.h"

namespace LibISR
{
	namespace Engine
	{
		class ISRRGBDTracker_GPU :public ISRRGBDTracker
		{
		private:
			float *energy_host, *gradient_host, *hessian_host;
			float *energy_dvic, *gradient_divc, *hessian_divc;

		protected:
			// evaluate the energy given current poses and shapes
			// the poses are always taken from tmpPoses
			void evaluateEnergy(float *energy, Objects::ISRTrackingState * trackerState);

			// compute the Hessian and the Jacobian given the current poses and shape
			// the poses are always taken from tmpPoses
			void computeJacobianAndHessian(float *gradient, float *hessian, Objects::ISRTrackingState * trackerState) const;

			void lableForegroundPixels(Objects::ISRTrackingState * trackerState);
		public:
			ISRRGBDTracker_GPU(int nObjs, const Vector2i& imgSize);
			~ISRRGBDTracker_GPU();


		};

	}
}