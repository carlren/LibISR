#ifndef __ISR_TRACKING__
#define __ISR_TRACKING__

#include "ISRHistogram.h"
#include "ISRShape.h"
#include "ISRShapeUnion.h"
#include "ISRImage.h"
#include "ISRPose.h"
#include "ISRFrame.h"
#include "ISRPoints.h"
#include "ISROptimizationHelper.h"

#include "IOUtil.h"
#include "Timer.h"

#include <stdlib.h>

using namespace LibISR::Objects;

namespace LibISR
{
	namespace Engine
	{
		class ISRTrackingEngine
		{

		private:
			static ISRTrackingEngine *instance;

			//unproject image pixels into CAMERA cooridnates
			int		UnprojectRGBDImgPts(ISRFrame *frame, ISRShapeUnion *shapes);

			// apply a Euclidean transform to points
			void	TransformRT(ISRPoints *inPoints, ISRPoints *outPoints, ISRPose *pose);

			// evaulate energy function
			float	EvaluateEnergyFunction(ISRShapeUnion *shapeUnion);

			// compute the hessian and jacobian
			void ComputeJacobianAndHessian(ISRShapeUnion *shapeUnion, ISROptimizationHelper *helper);

			// update the appearnce model based on points
			void UpdateHistogramFromPoints(ISRHistogram *histogram, ISRShapeUnion *shapeUnion);
		public:
			static ISRTrackingEngine* Instance(void){
				if (instance == NULL) 		
					instance = new ISRTrackingEngine();
				return instance;}

			ISRTrackingEngine();
			~ISRTrackingEngine();

			void TrackFrame(ISRFrame* frame, ISRShapeUnion *shapes, ISROptimizationHelper *ophelper);

			void	GetPfMap(ISRFrame* frame);
		};

	}
}

#endif