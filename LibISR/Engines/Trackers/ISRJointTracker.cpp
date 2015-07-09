#include "ISRJointTracker.h"

#include <math.h>
#include <stdio.h>

#include "../../../ORUtils/Cholesky.h"
#include "../../../LibISRUtils/IOUtil.h"

using namespace LibISR::Engine;
using namespace LibISR::Objects;

ISRJointTracker::ISRJointTracker(int nObjs, Vector2i imageSize, bool useGPU)
{
	nObjects = nObjs;
	ATb_Size = nObjs * 6;
	ATA_size = ATb_Size*ATb_Size;

	ATb_host = new float[ATb_Size];
	ATA_host = new float[ATA_size];

	// precompute the corner list
	for (int i = -1, idx = 0; i <= 1; i += 2)
		for (int j = -1; j <= 1; j += 2)
			for (int k = -1; k <= 1; k += 2, idx++)
				cornerList[idx] = Vector3f(i*0.1f, j*0.1f, k*0.1f);

	initialized = false;

	accpetedState = new Objects::ISRTrackingState(nObjs, useGPU);
	tempState = new Objects::ISRTrackingState(nObjs, useGPU);
}

ISRJointTracker::~ISRJointTracker()
{
	delete[] ATb_host;
	delete[] ATA_host;

	delete accpetedState;
	delete tempState;
}


void ISRJointTracker::computeSingleStepLM(float *step, float *ATA, float *ATb, float lambda, int dim)
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
	//for (int i = 3; i < 6; i++) step[i] *= 3; // this is a hack for now
	for (int i = 0; i < 6; i++) step[i] *= 100;
}

void ISRJointTracker::computeSingleStepGN(float *step, float *ATA, float *ATb, int dim)
{
	for (int i = 0; i < dim * dim; i += (dim + 1))  if (fabs(ATA[i])<1.0f) ATA[i] += 1.0f;
	ORUtils::Cholesky cholA(ATA, dim);
	cholA.Backsub(step, ATb);
}


void ISRJointTracker::TrackObjects(ISRFrame *frame, ISRShapeUnion *shapeUnion, ISRTrackingState *trackerState, bool updateappearance)
{
	if (!initialized)
	{
		initializeTracker(frame->currentLevel->rgbd->noDims);
		initialized = true;
	}

	this->frame = frame;
	this->shapeUnion = shapeUnion;

	float lastenergy;

	updatePfImage();
	float step[6];


	//Gauss Newton
	for (int i = 0; i < 50; i++)
	{
		lastenergy = computeJacobianAndHessian(ATb_host, ATA_host, trackerState);
		computeSingleStepLM(step, ATA_host, ATb_host, 10, 6);
		trackerState->getPose(0)->applyIncrementalChangeToH(step);
	}

	if (lastenergy >= 0.5f && updateappearance)
	{
		lableForegroundPixels(accpetedState);
		frame->currentLevel->rgbd->UpdateHostFromDevice();
		frame->histogram->updateHistogramFromLabeledRGBD(frame->currentLevel->rgbd, 0.3f, 0.1f);
	}

}

void ISRJointTracker::computeMinmaxForRayCast(const Matrix4f& H, const Matrix3f& K, const Vector2i& imgsize)
{
	Vector3f ipts[8];
	raycastBoundingBox = Vector4i(imgsize.x, imgsize.y, 0, 0);
	float maxz = -99999.9f, minz = 99999.9f;

	for (int idx = 0; idx < 8; idx++)
	{
		ipts[idx] = K*(H*cornerList[idx]);
		ipts[idx].x /= ipts[idx].z;
		ipts[idx].y /= ipts[idx].z;

		raycastBoundingBox.x = ipts[idx].x < raycastBoundingBox.x ? ipts[idx].x : raycastBoundingBox.x;
		raycastBoundingBox.y = ipts[idx].y < raycastBoundingBox.y ? ipts[idx].y : raycastBoundingBox.y;
		raycastBoundingBox.z = ipts[idx].x > raycastBoundingBox.z ? ipts[idx].x : raycastBoundingBox.z;
		raycastBoundingBox.w = ipts[idx].y > raycastBoundingBox.w ? ipts[idx].y : raycastBoundingBox.w;

		maxz = ipts[idx].z > maxz ? ipts[idx].z : maxz;
		minz = ipts[idx].z < minz ? ipts[idx].z : minz;
	}

	raycastBoundingBox.x = raycastBoundingBox.x < 0 ? 0 : raycastBoundingBox.x;
	raycastBoundingBox.y = raycastBoundingBox.y < 0 ? 0 : raycastBoundingBox.y;
	raycastBoundingBox.z = raycastBoundingBox.z > imgsize.x ? imgsize.x : raycastBoundingBox.z;
	raycastBoundingBox.w = raycastBoundingBox.w > imgsize.y ? imgsize.y : raycastBoundingBox.w;

	raycastMinmax.x = minz;
	raycastMinmax.y = maxz;
}
