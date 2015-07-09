#pragma once
#include "ISRTracker.h"

namespace LibISR
{
	namespace Engine
	{
		class ISRColorTracker : public ISRTracker
		{
		private:

			void computeSingleStepLM(float *step, float *ATA, float *ATb, float lambda, int dim);
			void computeSingleStepGN(float *step, float *ATA, float *ATb, int dim);

		protected:
			bool initialized;

			// raycast to the SDF for a smoothed heavisideImage
			FloatImage *HeavisideImage;

			// the corresponding 3D points
			Float4Image *raycastPointCloud;

			// pf image
			FloatImage *pfImage;

			// the current boundingbox used for raycast
			Vector4i raycastBoundingBox;

			// min-max range of the raycast
			Vector2f raycastMinmax;

			// a list of volume corners
			Vector3f cornerList[8];

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

			virtual void initializeTracker(const Vector2i& imageSize) = 0;

			// update the minmax value for raycast search
			void computeMinmaxForRayCast(const Matrix4f& H, const Matrix3f& K, const Vector2i& imgsize);
			
			// load pfImage based on the histogram in frame
			virtual void updatePfImage() = 0;
			
			// do a raycast to the current SDF to generate a 2D Heaviside function to guide the tracking
			virtual void raycastTo2DHeaviside(Objects::ISRTrackingState *trackingState) = 0;

			// evaluate the energy given current poses and shapes
			// the poses are always taken from tmpPoses
			virtual float evaluateEnergy(Objects::ISRTrackingState * trackerState) = 0;

			// compute the Hessian and the Jacobian given the current poses and shape
			// the poses are always taken from tmpPoses
			virtual float computeJacobianAndHessian(float *gradient, float *hessian, Objects::ISRTrackingState * trackerState) = 0;

		public:

			int numParameters() const { return ATb_Size; }
			int numObjects() const { return nObjects;  };

			void  TrackObjects(Objects::ISRFrame *frame, Objects::ISRShapeUnion *shapeUnion, Objects::ISRTrackingState *trackerState, bool updateappearance=true);

			ISRColorTracker(int nObjs, Vector2i imageSize, bool useGPU);
			~ISRColorTracker();
		};

	}
}

