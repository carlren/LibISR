#include "ISRRGBDTracker.h"

#include "../../ORUtils/Cholesky.h"
#include <math.h>
#include <stdio.h>

#include "../../LibISRUtils/IOUtil.h"

using namespace LibISR::Engine;
using namespace LibISR::Objects;


static inline bool minimizeLM(const ISRRGBDTracker &tracker, ISRTrackingState *initialization);

LibISR::Engine::ISRRGBDTracker::ISRRGBDTracker(int nObjs, bool useGPU)
{
	nObjects = nObjs;
	ATb_Size = nObjs * 6;
	ATA_size = ATb_Size*ATb_Size;

	ATb_host = new float[ATb_Size];
	ATA_host = new float[ATA_size];

	accpetedState = new Objects::ISRTrackingState(nObjs);
	tempState = new Objects::ISRTrackingState(nObjs);
}

LibISR::Engine::ISRRGBDTracker::~ISRRGBDTracker()
{
	free(ATb_host);
	free(ATA_host);

	delete accpetedState;
	delete tempState;
}


void computeSingleStep(float *step, float *ATA, float *ATb, float lambda, int dim)
{
	float *tmpATA = new float[dim*dim];
	for (int i = 0; i < dim*dim; i++) tmpATA[i] = ATA[i];


	for (int i = 0; i < dim * dim; i += (dim + 1))
	{
		float &ele = tmpATA[i];
		if (!(fabs(ele) < 1e-15f)) ele *= (1.0f + lambda); else ele = lambda*1e-10f;
	}
	
	ORUtils::Cholesky cholA(tmpATA, dim);
	cholA.Backsub(step, ATb);

	for (int i = 0; i < 6; i++) step[i] = -step[i];
}

double stepQuality(float actual_reduction, const float *step, const float *grad, const float *hessian, int numPara)
{
	float predicted_reduction = 0.0;
	float *tmp = new float[numPara];

	matmul(hessian, step, tmp, numPara, numPara);
	for (int i = 0; i < numPara; i++) predicted_reduction -= grad[i] * step[i] + 0.5*step[i] * tmp[i];
	delete[] tmp;

	if (predicted_reduction < 0) return actual_reduction / fabs(predicted_reduction);
	return actual_reduction / predicted_reduction;
}

void LibISR::Engine::ISRRGBDTracker::TrackObjects(ISRFrame *frame, ISRShapeUnion *shapeUnion, ISRTrackingState *trackerState)
{
	this->frame = frame;
	this->shapeUnion = shapeUnion;

	this->accpetedState->setFrom(*trackerState);
	this->tempState->setFrom(*trackerState);

	float *cacheNabla = new float[ATb_Size];	

	float lastenergy = 0;
	float currentenergy = 0;

	bool converged = false;
	float lambda = 1000.0f;

	// These are some sensible default parameters for Levenberg Marquardt.
	// The first three control the convergence criteria, the others might
	// impact convergence speed.
	static const int MAX_STEPS = 100;
	static const float MIN_STEP = 0.00005f;
	static const float MIN_DECREASE = 0.00001f;
	static const float TR_QUALITY_GAMMA1 = 0.75f;
	static const float TR_QUALITY_GAMMA2 = 0.25f;
	static const float TR_REGION_INCREASE = 2.0f;
	static const float TR_REGION_DECREASE = 0.3f;

	evaluateEnergy(&lastenergy, accpetedState);

	//ISRFloat4Image *tmpPtCloud = new ISRFloat4Image(frame->depth_size, false);
	//Vector4f* inptr = frame->ptCloud->GetData(false);
	//Vector4f* outptr = tmpPtCloud->GetData(false);
	//Matrix4f tmpH = tempState->getPose(0)->getInvH();

	////PrintArrayToFile("E:/LibISR/debug/invH_debug.txt", tmpH.m, 16);

	//for (int i = 0; i < frame->ptCloud->dataSize; i++)
	//{
	//	if (inptr[i].w > 0)
	//	{
	//		Vector3f outpt = tmpH*inptr[i].toVector3();
	//		outptr[i].x = outpt.x;
	//		outptr[i].y = outpt.y;
	//		outptr[i].z = outpt.z;
	//	}
	//	else outptr[i].w = -1;
	//}

	//PrintPointListToFile("E:/LibISR/debug/objcloud_debug.txt", outptr, frame->ptCloud->dataSize);

	for (int iter = 0; iter < MAX_STEPS; iter++)
	{
		computeJacobianAndHessian(ATb_host, ATA_host, tempState);

		//PrintArrayToFile("e:/LibISR/debug/ATA_debug.txt", ATA_host, ATA_size);
		//PrintArrayToFile("e:/LibISR/debug/ATB_debug.txt", ATb_host, ATb_Size);

		while (true)
		{
			computeSingleStep(cacheNabla, ATA_host, ATb_host, lambda, ATb_Size);
			
			// check if step size is very small
			float MAXnorm = 0.0;
			for (int i = 0; i<ATb_Size; i++) { float tmp = fabs(cacheNabla[i]); if (tmp>MAXnorm) MAXnorm = tmp; }
			if (MAXnorm < MIN_STEP) break;


			//PrintArrayToFile("e:/LibISR/debug/step_debug.txt", cacheNabla, ATb_Size);
			//Matrix4f tmpm = tempState->getPose(1)->getInvH();
			//PrintArrayToFile("E:/LibISR/invH_before_debug.txt", tmpm.m , 16);

			tempState->applyIncrementalPoseChangesToInvH(cacheNabla);

			//tmpm = tempState->getPose(1)->getInvH();
			//PrintArrayToFile("E:/LibISR/invH_after_debug.txt", tmpm.m, 16);

			evaluateEnergy(&currentenergy, tempState);

			{ // Olaf's extra step to validate convergence

				float rho = stepQuality(lastenergy - currentenergy, cacheNabla, ATb_host, ATA_host, ATb_Size);

				if (rho > TR_QUALITY_GAMMA1) // step quality is good, accpet the step
				{
					lambda = lambda / TR_REGION_INCREASE;
					lastenergy = currentenergy;
					accpetedState->setFrom(*tempState);

					if (!(currentenergy < (lastenergy - fabs(lastenergy) * MIN_DECREASE))) converged = true;
					break;
				}
				else if (rho <= TR_QUALITY_GAMMA2)  // step quality is bad, do not accept the step 
				{
					lambda /= TR_REGION_DECREASE;
					tempState->setFrom(*accpetedState);
					continue;
				}
			}

			// // Carl's very simple convergence criteria
			//if (currentenergy<lastenergy)
			//{
			//	lastenergy = currentenergy;
			//	lambda /= 30.0f;
			//	accpetedState->setFrom(*tempState);
			//	break;
			//}
			//else
			//{
			//	lambda *= 10.0f;
			//	tempState->setFrom(*accpetedState);
			//}
			//if (lambda>1e6){ converged = true; break; }
		}
		if (converged) break;
	}

	trackerState->setFrom(*accpetedState);
	//Matrix4f tmpm = tempState->getPose(0)->getInvH();
	//PrintArrayToFile("E:/LibISR/debug/invH_debug2.txt", tmpm.m, 16);

}
