//#pragma once
//
//#include "../Objects/ISRHistogram.h"
//#include "../Objects/ISRShapeUnion.h"
//#include "../Objects/ISRFrame.h"
//#include "../Objects/ISROptimizationHelper.h"
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
//			int	UnprojectRGBDImgPts(const ISRFrame &frame, ISRShapeUnion &shapes);
//
//			// apply a Euclidean transform to points
//			void TransformRT(const ISRPoints &inPoints, ISRPoints &outPoints, const ISRPose &pose);
//
//			// evaulate energy function
//			float EvaluateEnergyFunction(const ISRShapeUnion &shapeUnion);
//
//			// compute the hessian and jacobian
//			void ComputeJacobianAndHessian(const ISRShapeUnion &shapeUnion, ISROptimizationHelper &helper);
//
//			// update the appearnce model based on points
//			void UpdateHistogramFromPoints(ISRHistogram &histogram, const ISRShapeUnion &shapeUnion);
//
//			// update the pf pb map of frame using histogram
//			void GetPfMap(ISRFrame &frame);
//
//		public:
//			static ISRTrackingEngine* Instance(void){
//				if (instance == NULL) 		
//					instance = new ISRTrackingEngine();
//				return instance;}
//
//			ISRTrackingEngine();
//			~ISRTrackingEngine();
//
//			void TrackFrame(ISRFrame &frame, ISRShapeUnion &shapes, ISROptimizationHelper &ophelper);
//		};
//
//	}
//}
//
