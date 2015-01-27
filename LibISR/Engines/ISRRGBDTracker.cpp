#include "ISRRGBDTracker.h"

#include <math.h>
#include <stdio.h>

#include "../../ORUtils/Cholesky.h"
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
	static const float MIN_DECREASE = 0.0001f;
	static const float TR_REGION_INCREASE = 0.10f;
	static const float TR_REGION_DECREASE = 10.0f;

	{// minimalist LM main loop
		evaluateEnergy(&lastenergy, accpetedState);

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
	
	lableForegroundPixels(accpetedState);
	frame->histogram->updateHistogramFromLabeledRGBD(frame->currentLevel->rgbd, 0.05f, 0.3f);

	trackerState->setFrom(*accpetedState);

	printf("\tEnergy:%f", lastenergy);
}
