#include "ISRRGBDTracker.h"

#include <math.h>
#include <stdio.h>
#include <iostream>

#include "../../ORUtils/Cholesky.h"
#include "../Utils/IOUtil.h"

using namespace LibISR::Engine;
using namespace LibISR::Objects;


LibISR::Engine::ISRRGBDTracker::ISRRGBDTracker(int nObjs, bool useGPU)
{
	nObjects = nObjs;
	ATb_Size = nObjs * 6;
	ATA_size = ATb_Size*ATb_Size;

	ATb_host = new float[ATb_Size];
	ATA_host = new float[ATA_size];

	accpetedState = new Objects::ISRTrackingState(nObjs,useGPU);
	tempState = new Objects::ISRTrackingState(nObjs,useGPU);
}

LibISR::Engine::ISRRGBDTracker::~ISRRGBDTracker()
{
	delete[] ATb_host;
	delete[] ATA_host;

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
}


void LibISR::Engine::ISRRGBDTracker::fastReinitialize(float& oldenergy)
{
	float e = oldenergy, tmpe = 0;
	Vector3f bigestR(0.0f);

	int R_SEP = 6;
	float R_DEGREE = 360.0f / R_SEP;

	for (int i = 1; i < R_SEP; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			Vector3f tmpV(0.0f);
			tmpV.v[j] = i * R_DEGREE;
			this->tempState->setFrom(*this->accpetedState);
			this->tempState->getPose(0)->applyIncrementalRotationToInvHInDegree(tmpV);
			this->tempState->updatePoseDeviceFromHost();
			evaluateEnergy(&tmpe, this->tempState);

			if (tmpe > e)
			{
				e = tmpe;
				bigestR = tmpV;
			}
		}

	}

	this->tempState->setFrom(*this->accpetedState);

	if (e-oldenergy>=0.05)
	{
		printf("Reinitialized, energy increase: %f \t rotation: (%d,%d,%d)\n", e - oldenergy, (int)bigestR.x, (int)bigestR.y, (int)bigestR.z);
		this->tempState->getPose(0)->applyIncrementalRotationToInvHInDegree(bigestR);
		this->tempState->updatePoseDeviceFromHost();
		this->accpetedState->setFrom(*this->tempState);
		oldenergy = e;
	}
}

void LibISR::Engine::ISRRGBDTracker::TrackObjects(ISRFrame *frame, ISRShapeUnion *shapeUnion, ISRTrackingState *trackerState, bool updateappearance)
{
	this->frame = frame;
	this->shapeUnion = shapeUnion;

	this->accpetedState->setFrom(*trackerState);
	this->tempState->setFrom(*trackerState);

	float *cacheNabla = new float[ATb_Size];	

	float lastenergy = 0;
	float currentenergy = 0;

	bool converged = false;
	float lambda = 10000.0f;

	evaluateEnergy(&lastenergy, accpetedState);
	if (lastenergy < 0.1f) { trackerState->energy = 0; return; }

//	//--------------------------------------------------------------------------
//	//
//	// Gauss Newton
//	//
//	//--------------------------------------------------------------------------

//	for (int i = 0; i < 20;i++)
//	{
//		computeJacobianAndHessian(ATb_host, ATA_host, accpetedState);
//		computeSingleStep(cacheNabla, ATA_host, ATb_host, lambda, ATb_Size);
//		accpetedState->applyIncrementalPoseChangesToInvH(cacheNabla);
//		evaluateEnergy(&lastenergy, accpetedState);
//	}
	
//	--------------------------------------------------------------------------
	
//	 LM
	
//	--------------------------------------------------------------------------

	// These are some sensible default parameters for Levenberg Marquardt.
	// The first three control the convergence criteria, the others might
	// impact convergence speed.
	static const int MAX_STEPS = 100;
	static const float MIN_STEP = 0.00005f;
	static const float MIN_DECREASE = 0.0001f;
	static const float TR_REGION_INCREASE = 0.10f;
	static const float TR_REGION_DECREASE = 10.0f;

	{// minimalist LM main loop
		for (int iter = 0; iter < MAX_STEPS; iter++)
		{
			computeJacobianAndHessian(ATb_host, ATA_host, tempState);
            
			while (true)
			{
				computeSingleStep(cacheNabla, ATA_host, ATb_host, lambda, ATb_Size);

				// check if step size is very small, if so, converge.
				float MAXnorm = 0.0;
				for (int i = 0; i<ATb_Size; i++) { float tmp = fabs(cacheNabla[i]); if (tmp>MAXnorm) MAXnorm = tmp; }
				if (MAXnorm < MIN_STEP) { converged = true; break; }

				tempState->applyIncrementalPoseChangesToInvH(cacheNabla);
                
				evaluateEnergy(&currentenergy, tempState);

				if (currentenergy > lastenergy)
				{
					// check if energy decrease is too small, if so, converge.
					if (abs(currentenergy - lastenergy) / abs(lastenergy) < MIN_DECREASE) {converged = true;}
					lastenergy = currentenergy;
					lambda *= TR_REGION_INCREASE;
					accpetedState->setFrom(*tempState);
					break;
				}
				else
				{
					lambda *= TR_REGION_DECREASE;
					tempState->setFrom(*accpetedState);
				}
			}
			if (converged) break;
		}

	}
	
    

	// after convergence, the w channel of ptcloud is recycled for histogram update
	if (lastenergy>=0.5f && updateappearance)
	{
		lableForegroundPixels(accpetedState);
		frame->currentLevel->rgbd->UpdateHostFromDevice();
		frame->histogram->updateHistogramFromLabeledRGBD(frame->currentLevel->rgbd, 0.3f, 0.1f);
	}

	//if (trackerState->numPoses() == 1 && lastenergy > 0.3f && lastenergy < 0.7f)
	//{
	//	fastReinitialize(lastenergy);
	//}
	
	trackerState->setFrom(*accpetedState);
	trackerState->energy = lastenergy;
	//printf("\tEnergy:%f", lastenergy);
}
