#include "ISRRGBDTracker.h"

#include "../../ORUtils/Cholesky.h"
#include <math.h>
#include <stdio.h>

using namespace LibISR::Engine;
using namespace LibISR::Objects;


static inline bool minimizeLM(const ISRRGBDTracker &tracker, ISRTrackingState *initialization);

LibISR::Engine::ISRRGBDTracker::ISRRGBDTracker(int nObjs, bool useGPU)
{
	nObjects = nObjs;
	ATb_Size = nObjs * 6;
	ATA_size = ATb_Size*ATb_Size;

	//ATb_host = (float*)malloc(ATb_Size*sizeof(float));
	//ATA_host = (float*)malloc(ATA_size*sizeof(float));

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

void LibISR::Engine::ISRRGBDTracker::TrackObjects(ISRFrame *frame, ISRShapeUnion *shapeUnion, ISRTrackingState *trackerState)
{
	minimizeLM(*this, trackerState);
}

LibISR::Engine::ISRRGBDTracker::EvaluationPoint::EvaluationPoint(ISRTrackingState * trackerState, const ISRRGBDTracker *f_parent)
{
	mState = trackerState;
	mParent = f_parent;

	ISRRGBDTracker *parent = (ISRRGBDTracker*)mParent;
	parent->evaluateEnergy(&cacheEnergy, mState);

	cacheHessian = NULL; cacheNabla = NULL;
}


static inline double stepQuality(ISRRGBDTracker::EvaluationPoint *x, ISRRGBDTracker::EvaluationPoint *x2, const float *step, const float *grad, const float *B, int numPara)
{
	double actual_reduction = x->energy() - x2->energy();
	double predicted_reduction = 0.0;
	float *tmp = new float[numPara];

	matmul(B, step, tmp, numPara, numPara);
	for (int i = 0; i < numPara; i++) predicted_reduction -= grad[i] * step[i] + 0.5*step[i] * tmp[i];
	delete[] tmp;

	if (predicted_reduction < 0) return actual_reduction / fabs(predicted_reduction);
	return actual_reduction / predicted_reduction;
}

static inline bool minimizeLM(const ISRRGBDTracker &tracker, ISRTrackingState* initialization)
{
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

	int numPara = tracker.numParameters();
	
	float *d = new float[numPara];
	float lambda = 1000.0f;
	int step_counter = 0;

	ISRRGBDTracker::EvaluationPoint *x = tracker.evaluateAt(initialization);
	ISRRGBDTracker::EvaluationPoint *x2 = NULL;

	if (!portable_finite(x->energy())) { delete[] d; delete x; return false; }

	do
	{
		const float *grad;
		const float *B;

		grad = x->nabla_energy();
		B = x->hessian_GN();



		bool success;
		{
			float *A = new float[numPara*numPara];
			for (int i = 0; i < numPara*numPara; ++i) A[i] = B[i];
			for (int i = 0; i < numPara; ++i)
			{
				float & ele = A[i*(numPara + 1)];
				if (!(fabs(ele) < 1e-15f)) ele *= (1.0f + lambda); else ele = lambda*1e-10f;
			}

			ORUtils::Cholesky cholA(A, numPara);
			cholA.Backsub(&(d[0]), grad);
			// TODO: if Cholesky failed, set success to false!

			success = true;
			delete[] A;
		}



		if (success)
		{
			float MAXnorm = 0.0;
			for (int i = 0; i<numPara; i++) { float tmp = fabs(d[i]); if (tmp>MAXnorm) MAXnorm = tmp; }

			if (MAXnorm < MIN_STEP) break;
			for (int i = 0; i < numPara; i++) d[i] = -d[i];

			
			ISRTrackingState tmpState = *x->getState();
			tmpState.applyIncrementalPoseChangesToInvH(d);
			
			x2 = tracker.evaluateAt(&tmpState);

			// check whether step reduces error function and
			// compute a new value of lambda
			//x2 = tracker.evaluateAt(tracker);

			double rho = stepQuality(x, x2, d, grad, B, numPara);
			if (rho > TR_QUALITY_GAMMA1)
				lambda = lambda / TR_REGION_INCREASE;
			else if (rho <= TR_QUALITY_GAMMA2)
			{
				success = false;
				lambda = lambda / TR_REGION_DECREASE;
			}
		}
		else
		{
			x2 = NULL;
			// can't compute a step quality here...
			lambda = lambda / TR_REGION_DECREASE;
		}

		if (success)
		{
			// accept step
			bool continueIteration = true;
			if (!(x2->energy() < (x->energy() - fabs(x->energy()) * MIN_DECREASE))) continueIteration = false;


			delete x;
			x = x2;

			if (!continueIteration) break;
		}
		else if (x2 != NULL) delete x2;
		if (step_counter++ >= MAX_STEPS - 1) break;
	} while (true);

	*initialization = *x->getState();
	delete x;

	delete[] d;

	return true;
}

