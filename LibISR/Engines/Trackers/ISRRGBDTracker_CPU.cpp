#include "ISRRGBDTracker_CPU.h"
#include "ISRRGBDTracker_DA.h"

#include "../../../LibISRUtils/IOUtil.h"

using namespace LibISR::Engine;
using namespace LibISR::Objects;

LibISR::Engine::ISRRGBDTracker_CPU::ISRRGBDTracker_CPU(int nObjs) :ISRRGBDTracker(nObjs, false){}
LibISR::Engine::ISRRGBDTracker_CPU::~ISRRGBDTracker_CPU(){}

void LibISR::Engine::ISRRGBDTracker_CPU::evaluateEnergy(float *energy, Objects::ISRTrackingState * trackerState)
{
	int count = this->frame->ptCloud->dataSize;
	Vector4f* ptcloud_ptr = this->frame->ptCloud->GetData(false);

	ISRShape_ptr shapes = this->shapeUnion->getShapeList(false);
	ISRPose_ptr poses = trackerState->getPoseList(false);
	int objCount = trackerState->numPoses();

	float e = 0, es=0;
	int totalpix = 0;
	int totalpfpix = 0;

	for (int i = 0; i < count; i++)
	{
		es = computePerPixelEnergy(ptcloud_ptr[i], shapes, poses, objCount);
		if (es > 0)
		{ 
			e += es; totalpix++; 
			if (ptcloud_ptr[i].w > 0.5) totalpfpix++;
		}
		
	}
		
	energy[0] = totalpfpix>100 ? e / totalpix : 0.0f;
}

void LibISR::Engine::ISRRGBDTracker_CPU::computeJacobianAndHessian(float *gradient, float *hessian, Objects::ISRTrackingState * trackerState) const
{
	int count = this->frame->ptCloud->dataSize;
	Vector4f* ptcloud_ptr = this->frame->ptCloud->GetData(false);

	ISRShape_ptr shapes = this->shapeUnion->getShapeList(false);
	ISRPose_ptr poses = trackerState->getPoseList(false);
	int objCount = trackerState->numPoses();

	int noPara = trackerState->numPoses() * 6;
	int noParaSQ = noPara*noPara;

	float *globalGradient = new float[noPara];
	float *globalHessian = new float[noParaSQ];
	float *jacobian = new float[noPara];

	for (int i = 0; i < noPara; i++) globalGradient[i] = 0.0f;
	for (int i = 0; i < noParaSQ; i++) globalHessian[i] = 0.0f;

	for (int i = 0; i < count; i++)
	{
		if (computePerPixelJacobian(jacobian, ptcloud_ptr[i], shapes, poses, objCount))
		{
			for (int a = 0, counter = 0; a < noPara; a++) 	
			{
				globalGradient[a] += jacobian[a];
				for (int b = 0; b <= a; b++, counter++) globalHessian[counter] += jacobian[a] * jacobian[b];
			}
		}
	}
	
	for (int r = 0; r < noPara; ++r) gradient[r] = globalGradient[r];

	for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++) hessian[r + c * noPara] = globalHessian[counter];
	for (int r = 0; r < noPara; ++r) for (int c = r + 1; c < noPara; c++) hessian[r + c * noPara] = hessian[c + r*noPara];
}

void LibISR::Engine::ISRRGBDTracker_CPU::lableForegroundPixels(Objects::ISRTrackingState * trackerState)
{
	int count = this->frame->ptCloud->dataSize;
	Vector4f* ptcloud_ptr = this->frame->ptCloud->GetData(false);
	Vector4f* rgbd_ptr = this->frame->currentLevel->rgbd->GetData(false);

	ISRShape_ptr shapes = this->shapeUnion->getShapeList(false);
	ISRPose_ptr poses = trackerState->getPoseList(false);
	int objCount = trackerState->numPoses();

	float dt;
	int totalpix = 0;

	for (int i = 0; i < count; i++)
	{
		if (ptcloud_ptr[i].w > 0) // in the bounding box and have depth
		{
			dt = findPerPixelDT(ptcloud_ptr[i], shapes, poses,objCount);
			if (fabs(dt) <= 5) { rgbd_ptr[i].w = HIST_FG_PIXEL; }
			else { rgbd_ptr[i].w = HIST_BG_PIXEL; }
		}
	}	
}

void LibISR::Engine::ISRRGBDTracker_CPU::lableForegroundPixels(Objects::ISRTrackingState * trackerState, Vector4i bb)
{

}


