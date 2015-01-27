
#include "ISRRGBDTracker_CPU.h"
#include "../../DeviceAgnostic/ISRRGBDTracker_DA.h"

#include "../../../../LibISRUtils/IOUtil.h"

using namespace LibISR::Engine;
using namespace LibISR::Objects;

LibISR::Engine::ISRRGBDTracker_CPU::ISRRGBDTracker_CPU(int nObjs) :ISRRGBDTracker(nObjs, false){}
LibISR::Engine::ISRRGBDTracker_CPU::~ISRRGBDTracker_CPU(){}

void LibISR::Engine::ISRRGBDTracker_CPU::evaluateEnergy(float *energy, Objects::ISRTrackingState * trackerState)
{
	int count = this->frame->ptCloud->dataSize;
	Vector4f* ptcloud_ptr = this->frame->ptCloud->GetData(false);

	float e = 0, es=0;
	int totalpix = 0;

	for (int i = 0; i < count; i++)
	{
		es = computePerPixelEnergy(ptcloud_ptr[i], this->shapeUnion, trackerState);
		if (es != 0){ e += es; totalpix++; }
	}
		
	energy[0] = e/totalpix;
}

void LibISR::Engine::ISRRGBDTracker_CPU::computeJacobianAndHessian(float *gradient, float *hessian, Objects::ISRTrackingState * trackerState) const
{
	int count = this->frame->ptCloud->dataSize;
	Vector4f* ptcloud_ptr = this->frame->ptCloud->GetData(false);

	int noPara = trackerState->numPoses() * 6;
	int noParaSQ = noPara*noPara;

	float *globalGradient = new float[noPara];
	float *globalHessian = new float[noParaSQ];
	float *jacobian = new float[noPara];

	for (int i = 0; i < noPara; i++) globalGradient[i] = 0.0f;
	for (int i = 0; i < noParaSQ; i++) globalHessian[i] = 0.0f;

	for (int i = 0; i < count; i++)
	{
		if (computePerPixelJacobian(jacobian, ptcloud_ptr[i], shapeUnion, trackerState))
		{
			for (int a = 0, counter = 0; a < noPara; a++) 	
			{
				globalGradient[a] += jacobian[a];
				for (int b = 0; b < noPara; b++, counter++) globalHessian[counter] += jacobian[a] * jacobian[b];
			}
		}
	}
	
	for (int r = 0; r < noPara; ++r) gradient[r] = globalGradient[r];
	for (int r = 0; r < noParaSQ; ++r) hessian[r] = globalHessian[r];
}

void LibISR::Engine::ISRRGBDTracker_CPU::lableForegroundPixels(Objects::ISRTrackingState * trackerState)
{
	int count = this->frame->ptCloud->dataSize;
	Vector4f* ptcloud_ptr = this->frame->ptCloud->GetData(false);
	Vector4f* rgbd_ptr = this->frame->currentLevel->rgbd->GetData(false);

	float dt;
	int totalpix = 0;

	for (int i = 0; i < count; i++)
	{
		if (ptcloud_ptr[i].w > 0) // in the bounding box and have depth
		{
			dt = findPerPixelDT(ptcloud_ptr[i], this->shapeUnion, trackerState);
			if (fabs(dt) <= 2) { rgbd_ptr[i].w = HIST_FG_PIXEL; }
			else { rgbd_ptr[i].w = HIST_BG_PIXEL; }
		}
	}	
}


