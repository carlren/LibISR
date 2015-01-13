#pragma once
#include "ISRTracker.h"

namespace LibISR
{
	namespace Engine
	{
		class ISRRGBDTracker : public ISRTracker
		{
		private:

			// the current accepted set of pose
			// increamental change of pose will always
			// be applied to this pose
			Objects::ISRPose **acceptedPoses;

			// temp poses after applying the increamental pose change
			// energy fucntion is always evaluated on this set of poses
			Objects::ISRPose **tmpPoses;

			// copy the pose from the shape union to 
			void loadPosesFromShapeUnion();

			// copy poses back to the shape union
			void copyPosesToShapeUnion();

			// number of objects
			int nObjects;

			// size of the gradient
			int ATb_Size;

			// size of the Hessian
			int ATA_size;

		protected:

			// Hessian procximated with JTJ
			float* ATA_host;

			// gradient
			float* ATb_host;

			// evaluate the energy given current poses and shapes
			// the poses are always taken from tmpPoses
			virtual void evaluateEnergy(float *energy, LibISR::Objects::ISRShapeUnion &shapes, LibISR::Objects::ISRFrame &frame) = 0;

			// compute the Hesssian and the Jacobian given the current poses and shape
			// the poses are always taken from tmpPoses
			virtual void computeJacobianAndHessian(float *gradient, float *hessian, LibISR::Objects::ISRShapeUnion &shapes, LibISR::Objects::ISRFrame &frame) const = 0;

		public:

			// apply a increamental pose change to current set of pose
			// d_pose * acceptedPoses -> tmpPoses
			void applyPoseChange(const float* d_pose) const;


			// evaluation point for LM optimization
			class EvaluationPoint
			{
			protected:
				float cacheEnergy;
				float *cacheNabla;
				float *cacheHessian;

				Objects::ISRPose *mPoses;
				const ISRRGBDTracker *mParent;

				void computeGradients(bool requiresHessian);
				
			public:
				float energy(){ return cacheEnergy; };
				
				const float* nabla_energy(){if (cacheNabla == NULL) computeGradients(false); return cacheNabla; }
				const float* hessian_GN() { if (cacheHessian == NULL) computeGradients(true); return cacheHessian; }
			
				const Objects::ISRPose* getPoses() const { return mPoses; }

				EvaluationPoint(Objects::ISRPose* pos, const ISRRGBDTracker *f_parent);
				~EvaluationPoint(void)
				{
					delete mPoses;
					if (cacheNabla != NULL) delete[] cacheNabla;
					if (cacheHessian != NULL) delete[] cacheHessian;
				}
			};

			EvaluationPoint* evaluateAt(Objects::ISRPose *poses) const
			{
				return new EvaluationPoint(poses, this);
			}

			void TrackObjects(LibISR::Objects::ISRFrame &frame, LibISR::Objects::ISRShapeUnion &shapes) = 0;

			ISRRGBDTracker(int nObjs, bool useGPU);
			~ISRRGBDTracker();
		};

	}
}

