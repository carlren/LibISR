#pragma once

#include "ISRRGBDTracker.h"

namespace LibISR
{
	namespace Engine
	{
		class ISRRGBDTracker_GPU :public ISRRGBDTracker
		{
		private:
			float *gradient_host, *hessian_host;
			float *gradient_divc, *hessian_divc;
			Vector3f *energy_dvic, *energy_host;

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
			ISRRGBDTracker_GPU(int nObjs, const Vector2i& imgSize);
			~ISRRGBDTracker_GPU();


		};

	}
}