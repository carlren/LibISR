//#pragma once
//
//#include "..//Objects//ISRHistogram.h"
//#include "..//Objects//ISRShape.h"
//#include "..//Objects//ISRShapeUnion.h"
//#include "..//Objects//ISRImage.h"
//#include "..//Objects//ISRPose.h"
//#include "..//Objects//ISRFrame.h"
//#include "..//Objects//ISRPoints.h"
//#include "..//Objects//ISROptimizationHelper.h"
//
//#include <stdlib.h>
//
//using namespace CoreISR::Objects;
//
//namespace CoreISR
//{
//	namespace Engine
//	{
//		class ISRTrackingEngine
//		{
//
//		private:
//			static ISRTrackingEngine *instance;
//
//			//unproject image pixels into CAMERA cooridnates
//			int	UnprojectRGBDImgPts(ISRFrame *frame, ISRShapeUnion *shapes);
//
//			// apply a Euclidean transform to points
//			void TransformRT(ISRPoints *inPoints, ISRPoints *outPoints, ISRPose *pose);
//
//			// evaulate energy function
//			float EvaluateEnergyFunction(ISRShapeUnion *shapeUnion);
//
//			// compute the hessian and jacobian
//			void ComputeJacobianAndHessian(ISRShapeUnion *shapeUnion, ISROptimizationHelper *helper);
//
//			// update the appearnce model based on points
//			void UpdateHistogramFromPoints(ISRHistogram *histogram, ISRShapeUnion *shapeUnion);
//		public:
//			static ISRTrackingEngine* Instance(void){
//				if (instance == NULL) 		
//					instance = new ISRTrackingEngine();
//				return instance;}
//
//			ISRTrackingEngine();
//			~ISRTrackingEngine();
//
//			void TrackFrame(ISRFrame* frame, ISRShapeUnion *shapes, ISROptimizationHelper *ophelper);
//
//			void	GetPfMap(ISRFrame* frame);
//		};
//
//	}
//}
//
