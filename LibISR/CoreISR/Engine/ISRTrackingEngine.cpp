//#include "ISRTrackingEngine.h"
//
//using namespace CoreISR::Engine;
//
//ISRTrackingEngine *ISRTrackingEngine::instance;
//
//ISRTrackingEngine::ISRTrackingEngine(){}
//ISRTrackingEngine::~ISRTrackingEngine(){}
//
//int tmpcount = 0;
//
//void ISRTrackingEngine::TrackFrame(ISRFrame* frame, ISRShapeUnion *shapeUnion, ISROptimizationHelper *ophelper)
//{
//	float lastEnergy=0;
//	float currentEnergy=0;
//	ISRPose *currentPose = new ISRPose();
//	float lambda = 1000.0f;
//	bool converged = false;
//	Matrix3f tempR3; Vector3f tempT3;
//	ISRPose* tmpPose = new ISRPose();
//
//	
//	shapeUnion->UpdateOccMapOnFrame(frame);
//	
//
//	this->GetPfMap(frame);
//	
//
//	//char tmpName[100];
//	//sprintf(tmpName, "d:\\tmp\\posterior_%05d.txt", tmpcount);
//	//WriteMatlabTXTImg(tmpName, frame->pfImage->data,640, 480);
//	//tmpcount++;
//
//	
//	this->UnprojectRGBDImgPts(frame, shapeUnion);
//
//	//PrintPointListToFile("d:\\tmp\\camPtList.txt", shapeUnion->sharedCamPoints->data, shapeUnion->nPtCount);
//
//	for (int i = 0; i < shapeUnion->nShapesCount; i++)
//	{
//
//		TransformRT(shapeUnion->sharedCamPoints, shapeUnion->shapes[i]->unprojectedPts, shapeUnion->shapes[i]->pose);
//		shapeUnion->shapes[i]->tmpUnprojectPts->CopyFrom(shapeUnion->shapes[i]->unprojectedPts);
//
//	}
//	//PrintPointListToFile("d:\\tmp\\unPtList.txt", shapeUnion->shapes[0]->unprojectedPts->data, shapeUnion->nPtCount);
//
//	//myTimer.restart();
//	lastEnergy = this->EvaluateEnergyFunction(shapeUnion);
//	//myTimer.check("energy:");
//
//	for (int iter = 0; iter < 200; iter++)
//	{
//		
//		this->ComputeJacobianAndHessian(shapeUnion,ophelper);
//		
//
//		while (true)
//		{
//			if (lambda>(float)1e6){ converged = true; break; }
//			ophelper->SolveLM(lambda);
//
//			for (int i = 0; i < shapeUnion->nShapesCount; i++)
//			{
//				tmpPose->SetFromStep(&(ophelper->finalStep[i*6]));
//				TransformRT(shapeUnion->shapes[i]->unprojectedPts, shapeUnion->shapes[i]->tmpUnprojectPts,tmpPose);
//			}
//			currentEnergy = this->EvaluateEnergyFunction(shapeUnion);
//
//			if (currentEnergy>lastEnergy)
//			{
//				for (int i = 0; i < shapeUnion->nShapesCount; i++)
//				{
//					shapeUnion->shapes[i]->unprojectedPts->CopyFrom(shapeUnion->shapes[i]->tmpUnprojectPts);
//					lastEnergy = currentEnergy;
//					lambda /= 10;
//					shapeUnion->shapes[i]->pose->UpdateFromStep(&(ophelper->finalStep[i*6]));
//				}
//				break;
//			}
//			else lambda *= 10.0f;
//		}
//
//		if (converged) break;
//	}
//
//	//myTimer.restart();
//	UpdateHistogramFromPoints(frame->histogram, shapeUnion);
//	//myTimer.check("histogram update:");
//}
//
//
//static inline int Pt2IntIdx(Vector3f pt)
//{
//	int x = pt.x * VOL_SCALE + DT_VOL_SIZE / 2 - 1;
//	int y = pt.y * VOL_SCALE + DT_VOL_SIZE / 2 - 1;
//	int z = pt.z * VOL_SCALE + DT_VOL_SIZE / 2 - 1;
//	
//	if (x > 0 && x < DT_VOL_SIZE - 1 &&
//		y > 0 && y < DT_VOL_SIZE - 1 &&
//		z > 0 && z < DT_VOL_SIZE - 1)
//		return (z * DT_VOL_SIZE + y) * DT_VOL_SIZE + x;
//	else
//		return -1;
//}
//
//// done!
//int	ISRTrackingEngine::UnprojectRGBDImgPts(ISRFrame *frame, ISRShapeUnion *shapes)
//{
//	unsigned char *occMap = frame->occMap->data;
//	float *depthMap = frame->depthImage->data;
//	float *pfMap = frame->pfImage->data;
//	float *colorIdxMap = frame->idxImage->data;
//
//	float *invA = frame->intrinsics->invA;
//
//	float x, y, z;
//
//	Vector3f *camPoints = shapes->sharedCamPoints->data;
//	float *pfPoints = shapes->pfList;
//	int *colorIdx = shapes->colorIdxList;
//
//	int pixCount = 0;
//
//	int sRate = frame->samplingRate;
//
//	for (int j = 0; j < frame->width; j+=sRate)
//	for (int i = 0; i < frame->height; i+=sRate)
//	{
//		int idx = i*frame->width + j;
//		z = depthMap[idx];
//
//		if (occMap[idx]>0 && z>0)
//		{
//			x = (j + 1)*z;
//			y = (i + 1)*z;
//
//			camPoints[pixCount].x = invA[0] * x + invA[1] * y + invA[2] * z;
//			camPoints[pixCount].y = invA[3] * x + invA[4] * y + invA[5] * z;
//			camPoints[pixCount].z = invA[6] * x + invA[7] * y + invA[8] * z;
//
//			pfPoints[pixCount] = pfMap[idx];
//			colorIdx[pixCount] = (int)colorIdxMap[idx];
//
//			pixCount++;
//		}
//	}
//	shapes->nPtCount = pixCount;
//	shapes->sharedCamPoints->count = pixCount;
//	return pixCount;
//}
//
//// done!
//void ISRTrackingEngine::TransformRT(ISRPoints *inPoints, ISRPoints *outPoints, ISRPose *pose)
//{
//	Vector3f *outPts = outPoints->data;
//	Vector3f *inPts = inPoints->data;
//
//	Matrix4f *H = pose->H;
//
//	for (int i = 0; i < inPoints->count; i++)
//	{
//		outPts[i].x = H->m00 * inPts[i].x + H->m01 * inPts[i].y + H->m02 * inPts[i].z + H->m03;
//		outPts[i].y = H->m10 * inPts[i].x + H->m11 * inPts[i].y + H->m12 * inPts[i].z + H->m13;
//		outPts[i].z = H->m20 * inPts[i].x + H->m21 * inPts[i].y + H->m22 * inPts[i].z + H->m23;
//	}
//
//	outPoints->count = inPoints->count;
//}
//
//// done!
//float ISRTrackingEngine::EvaluateEnergyFunction(ISRShapeUnion *shapeUnion)
//{
//	ISRShape *currentShape;
//	Vector3f* currentPtList;
//	
//	//Vector3i tmpIdx;
//	int tmpIdx;
//
//	float energy=0;
//	
//	float dt;
//	float partDt;
//	float pf;
//
//	float exp_dt,deto,sdelta,sheaviside;
//
//	int nPtCount = shapeUnion->nPtCount;
//
//	for (int i = 0; i < nPtCount; i++)
//	{
//		dt = 121.0f; // max dt for 200*200*200 vol
//		pf = shapeUnion->pfList[i];
//
//		for (int j = 0; j < shapeUnion->nShapesCount; j++)
//		{
//			currentShape = shapeUnion->shapes[j];
//			currentPtList = currentShape->tmpUnprojectPts->data;
//			
//			tmpIdx = Pt2IntIdx(currentPtList[i]);
//			partDt = tmpIdx>-1 ? currentShape->model_dt[tmpIdx] : 121.0f;
//
//			dt = partDt < dt ? partDt : dt;
//		}
//		
//		exp_dt = expf(-dt / 2.0f);
//		deto = exp_dt + 1.0f;
//		sheaviside = 1.0f / deto;
//		sdelta = 4.0f* exp_dt * sheaviside * sheaviside;
//
//		energy += pf * sdelta + (1 - pf)*sheaviside;
//
//		if (dt <= 2 && dt >= -2)
//			shapeUnion->surfHashList[i] = 1;
//		else
//			shapeUnion->surfHashList[i] = 0;
//
//	}
//
//	return energy / shapeUnion->nPtCount;
//}
//
//
//void ISRTrackingEngine::ComputeJacobianAndHessian(ISRShapeUnion *shapeUnion, ISROptimizationHelper *opHelper)
//{
//
//	ISRShape *currentShape;
//	Vector3f* currentPtList;
//
//	float dt;
//	float partDt;
//	float pf;
//
//	Vector3f dDt;
//	float dDelta;
//	float dHeaviside;
//	Vector3f pt;
//
//	int minIdx;
//	int DoF = opHelper->nDof;
//
//	float prefix;
//	float jacobian[6];
//
//	bool minFound;
//
//	//Vector3i tmpIdx;
//	int tmpIdx;
//
//	int nPtCount = shapeUnion->nPtCount;
//
//	opHelper->Clear();
//
//	for (int i = 0; i < nPtCount; i++)
//	{
//		dt = 121.0f; // max dt for 200*200*200 vol
//		minFound = false;
//
//		pf = shapeUnion->pfList[i];
//
//		for (int j = 0; j < shapeUnion->nShapesCount; j++)
//		{
//			currentShape = shapeUnion->shapes[j];
//			currentPtList = currentShape->unprojectedPts->data;
//
//			tmpIdx = Pt2IntIdx(currentPtList[i]);
//			partDt = tmpIdx>-1 ? currentShape->model_dt[tmpIdx] : 121.0f;
//
//			if (partDt < dt)
//			{
//				minIdx = j;
//				dt = partDt;
//				pt = currentPtList[i];
//
//				dDt = currentShape->d_dt[tmpIdx];
//				dDelta = currentShape->d_delta_dt[tmpIdx];
//				dHeaviside = currentShape->d_heaviside_dt[tmpIdx];
//
//				minFound = true;
//			}		
//		}
//
//		if (minFound)
//		{
//			prefix = pf*dDelta + (1 - pf)*dHeaviside;
//			
//			jacobian[0] = dDt.x * prefix;
//			jacobian[1] = dDt.y * prefix;
//			jacobian[2] = dDt.z * prefix;
//			jacobian[3] = (4 * dDt.z* pt.y - 4 * dDt.y * pt.z) * prefix;
//			jacobian[4] = (4 * dDt.x* pt.z - 4 * dDt.z * pt.x) * prefix;
//			jacobian[5] = (4 * dDt.y* pt.x - 4 * dDt.x * pt.y) * prefix;
//
//			for (int a = 0; a < 6; a++) for (int b = 0; b < 6; b++)
//				opHelper->Hessian[(a + minIdx * 6) + (b + minIdx * 6)*DoF] += jacobian[a] * jacobian[b];
//
//			//WriteMatlabTXTImg("d:\\tmp\\hessianNew.txt", opHelper->Hessian, 6, 6);
//
//			for (int a = 0; a < 6; a++)
//				opHelper->step[a + minIdx * 6] -= jacobian[a];
//
//		}
//	}
//
//	//WriteMatlabTXTImg("d:\\tmp\\hessianNew.txt", opHelper->Hessian, 12, 12);
//
//}
//
//// done!
//void ISRTrackingEngine::GetPfMap(ISRFrame *frame)
//{
//	frame->pfImage->Clear();
//	Vector4u *pixels = (Vector4u*)frame->alignedColorImage->data;
//	
//	float* pfMap = frame->pfImage->data;
//	float* idxMap = frame->idxImage->data;
//
//	ISRHistogram* histogram = frame->histogram;
//	int noBins = histogram->noBins;
//	int dim = histogram->dim;
//	int ru, gu, bu;
//	int pidx;
//
//	for (int i = 0; i < frame->height; i++) 
//		for (int j = 0; j < frame->width; j++)
//		{
//			int idx = i*frame->width + j;
//
//			if (frame->occMap->data[idx]>0)
//			{
//				ru = pixels[idx].r / noBins;
//				gu = pixels[idx].g / noBins;
//				bu = pixels[idx].b / noBins;
//				pidx = ru*noBins*noBins + gu * noBins + bu;
//
//				idxMap[idx] = pidx;
//				pfMap[idx] = histogram->posterior[pidx];
//			}
//		}
//}
//
//
//void ISRTrackingEngine::UpdateHistogramFromPoints(ISRHistogram* histogram, ISRShapeUnion *shapeUnion)
//{
//	int* idxList = shapeUnion->colorIdxList;
//	int* surfHashList = shapeUnion->surfHashList;
//
//	ISRHistogram* tmpHistogram = new ISRHistogram(HISTOGRAM_BIN);
//
//	float sumHistogramForeground = 0;
//	float sumHistogramBackground = 0;
//
//	for (int i = 0; i < shapeUnion->nPtCount; i++)
//	{
//		int pidx = idxList[i];
//		if (surfHashList[i] == 1)
//		{
//			tmpHistogram->data_notnormalised[pidx].x++;
//			sumHistogramForeground++;
//		}
//		else 
//		{ 
//			tmpHistogram->data_notnormalised[pidx].y++;
//			sumHistogramBackground++;
//		}
//	}
//
//	sumHistogramForeground = (sumHistogramForeground != 0) ? 1.0f / sumHistogramForeground : 0;
//	sumHistogramBackground = (sumHistogramBackground != 0) ? 1.0f / sumHistogramBackground : 0;
//
//	for (int i = 0; i < histogram->dim; i++)
//	{
//		tmpHistogram->data_normalised[i].x = tmpHistogram->data_notnormalised[i].x * sumHistogramForeground + 0.0001f;
//		tmpHistogram->data_normalised[i].y = tmpHistogram->data_notnormalised[i].y * sumHistogramBackground + 0.0001f;
//	}
//
//	histogram->UpdateHistogram(tmpHistogram, 0.05f, 0.3f);
//}