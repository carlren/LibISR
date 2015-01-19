
#include "ISRRGBDTracker_CPU.h"
#include "../../DeviceAgnostic/ISRRGBDTracker_DA.h"

using namespace LibISR::Engine;
using namespace LibISR::Objects;

LibISR::Engine::ISRRGBDTracker_CPU::ISRRGBDTracker_CPU(int nObjs) :ISRRGBDTracker(nObjs, false){}
LibISR::Engine::ISRRGBDTracker_CPU::~ISRRGBDTracker_CPU(){}

void LibISR::Engine::ISRRGBDTracker_CPU::evaluateEnergy(float *energy, Objects::ISRTrackingState * trackerState)
{
	int count = this->frame->ptCloud->dataSize;
	Vector4f* ptcloud_ptr = this->frame->ptCloud->GetData(false);

	float e = 0;

	for (int i = 0; i < count; i++) e += computePerPixelEnergy(ptcloud_ptr[i], this->shapeUnion, trackerState);
	energy[0] = -e;
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
			for (int r = 0, counter = 0; r < noPara; r++)
			{
				globalGradient[r] -= jacobian[r];
				for (int c = 0; c <= r; c++, counter++) globalHessian[counter] += jacobian[r] * jacobian[c];
			}
		}
	}
	for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++) hessian[r + c * 6] = globalHessian[counter];
	for (int r = 0; r < noPara; ++r) for (int c = r + 1; c < noPara; c++) hessian[r + c * 6] = hessian[c + r * 6];
	for (int r = 0; r < noPara; ++r) gradient[r] = globalGradient[r];
}


