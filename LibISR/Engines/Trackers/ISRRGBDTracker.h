#pragma once
#include "ISRTracker.h"

namespace LibISR
{
	namespace Engine
	{
		class ISRRGBDTracker : public ISRTracker
		{
		protected:

			// the current accepted tracker's state
			// incremental change of poses will always
			// be applied to this state
			Objects::ISRTrackingState * accpetedState;

			// temp tracker state after applying the incremental pose change
			// energy function is always evaluated on this set of poses
			Objects::ISRTrackingState * tempState;

			// pointer to the current set of shapes
			Objects::ISRShapeUnion *shapeUnion;

			// pointer to the current frame
			Objects::ISRFrame *frame;

			// number of objects
			int nObjects;

			// size of the gradient
			int ATb_Size; // (6*nObjects)

			// size of the Hessian
			int ATA_size; // (Atb_size^2)

			// Hessian approximated with JTJ
			float* ATA_host;

			// gradient
			float* ATb_host;

			// evaluate the energy given current poses and shapes
			// the poses are always taken from tmpPoses
			virtual void evaluateEnergy(float *energy, Objects::ISRTrackingState * trackerState) = 0;

			// compute the Hessian and the Jacobian given the current poses and shape
			// the poses are always taken from tmpPoses
			virtual void computeJacobianAndHessian(float *gradient, float *hessian, Objects::ISRTrackingState * trackerState) const = 0;

			// back-project pixels into object coordinates to see whether they are on the object surface
			virtual void lableForegroundPixels(Objects::ISRTrackingState * trackerState) = 0;

			virtual void lableForegroundPixels(Objects::ISRTrackingState * trackerState, Vector4i bb) = 0;

		public:

			int numParameters() const { return ATb_Size; }
			int numObjects() const { return nObjects;  };

			void  TrackObjects(Objects::ISRFrame *frame, Objects::ISRShapeUnion *shapeUnion, Objects::ISRTrackingState *trackerState, bool updateappearance=true);

			void fastReinitialize(float& oldenergy);

			ISRRGBDTracker(int nObjs, bool useGPU);
			~ISRRGBDTracker();
		};

	}
}

