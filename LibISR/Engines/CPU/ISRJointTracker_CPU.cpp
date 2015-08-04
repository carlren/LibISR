#include "ISRJointTracker_CPU.h"
#include "ISRColorTracker_DA.h"
#include "ISRRGBDTracker_DA.h"

#include "../../../LibISRUtils/IOUtil.h"
#include "../Lowlevel/ISRLowlevelEngine_DA.h"

using namespace LibISR::Engine;
using namespace LibISR::Objects;

ISRJointTracker_CPU::ISRJointTracker_CPU(int nObjs, Vector2i imageSize) :ISRJointTracker(nObjs, imageSize, false)
{
}
ISRJointTracker_CPU::~ISRJointTracker_CPU()
{
	if (initialized)
	{
		delete HeavisideImage;
		delete raycastPointCloud;
		delete pfImage;
	}
	
}

float ISRJointTracker_CPU::evaluateEnergy(ISRTrackingState * trackerState)
{
	raycastTo2DHeaviside(trackerState);

	Vector4f* ptcloud_ptr = raycastPointCloud->GetData(MEMORYDEVICE_CPU);
	float* pf_ptr = pfImage->GetData(MEMORYDEVICE_CPU);
	float* sdf_ptr = HeavisideImage->GetData(MEMORYDEVICE_CPU);

	Vector2i imgsize = frame->currentLevel->rgbd->noDims;

	float energy = 0;
	float count = 0;

	for (int y = raycastBoundingBox.y + 1; y < raycastBoundingBox.w - 1; y++)
		for (int x = raycastBoundingBox.x + 1; x < raycastBoundingBox.z - 1; x++)
		{
			float epp = computePerPixelEnergy(sdf_ptr, pf_ptr, ptcloud_ptr, x, y, imgsize);
			if (epp > 0){ energy += epp; count++; }
		}
	if (count>0) energy /= count;
	return energy;
}

float ISRJointTracker_CPU::computeJacobianAndHessian(float *gradient, float *hessian, ISRTrackingState * trackerState)
{
	raycastTo2DHeaviside(trackerState);
	
	Vector4f* ptcloud_ptr = raycastPointCloud->GetData(MEMORYDEVICE_CPU);
	float* pf_ptr = pfImage->GetData(MEMORYDEVICE_CPU);
	float* sdf_ptr = HeavisideImage->GetData(MEMORYDEVICE_CPU);

	int noPara = 6;
	int noParaSQ = noPara*noPara;

	float *globalGradient = new float[noPara];
	float *globalHessian = new float[noParaSQ];
	float *jacobian = new float[noPara];

	for (int i = 0; i < noPara; i++) globalGradient[i] = 0.0f;
	for (int i = 0; i < noParaSQ; i++) globalHessian[i] = 0.0f;

	Vector4f intrinsics = frame->currentLevel->intrinsic;
	Vector2i imgsize = frame->currentLevel->rgbd->noDims;

	Matrix4f H = trackerState->getPose(0)->getH();

	float energy = 0;
	float count = 0;

	for (int y = raycastBoundingBox.y + 1; y < raycastBoundingBox.w - 1; y++)
		for (int x = raycastBoundingBox.x + 1; x < raycastBoundingBox.z - 1; x++)
		{
			float pref;
			float epp = computePerPixelJacobian(jacobian, sdf_ptr, pf_ptr, ptcloud_ptr, intrinsics, x, y, imgsize, H, pref);
			if (epp > 0)
			{
				energy += epp;
				count++;

				for (int a = 0, counter = 0; a < noPara; a++)
				{
					globalGradient[a] += jacobian[a] * pref;
					for (int b = 0; b <= a; b++, counter++) globalHessian[counter] += jacobian[a] * jacobian[b];
				}
			}
		}
	
	////// yeah! this is a working version!
	//for (int y = raycastBoundingBox.y + 1; y < raycastBoundingBox.w - 1; y++)
	//	for (int x = raycastBoundingBox.x + 1; x < raycastBoundingBox.z - 1; x++)
	//	{
	//		float gradPrif, hessianPrif;
	//		float epp = computePerPixelJacobian(jacobian, sdf_ptr, pf_ptr, ptcloud_ptr, intrinsics, x, y, imgsize, H, gradPrif, hessianPrif);
	//		if (epp > 0)
	//		{
	//			energy += epp;
	//			count++;

	//			for (int a = 0, counter = 0; a < noPara; a++)
	//			{
	//				globalGradient[a] -= jacobian[a] * gradPrif;
	//				for (int b = 0; b <= a; b++, counter++) globalHessian[counter] += jacobian[a] * jacobian[b] * gradPrif * gradPrif;
	//			}
	//		}
	//	}

	for (int r = 0; r < noPara; ++r) gradient[r] = globalGradient[r];
	for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++) hessian[r + c * noPara] = globalHessian[counter];
	for (int r = 0; r < noPara; ++r) for (int c = r + 1; c < noPara; c++) hessian[r + c * noPara] = hessian[c + r*noPara];

	return energy / count;
}

void ISRJointTracker_CPU::raycastTo2DHeaviside(ISRTrackingState *trackingState)
{
	const float *voxelData = shapeUnion->getShape(0,false)->getSDFVoxel();
	float *outSDF_ptr = HeavisideImage->GetData(MEMORYDEVICE_CPU);
	Vector4f *outPtCloud_ptr = raycastPointCloud->GetData(MEMORYDEVICE_CPU);


	Vector2i imgSize = frame->currentLevel->rgbd->noDims;
	Vector4f& intrinsic = frame->currentLevel->intrinsic;
	Matrix4f invH = trackingState->getPose(0)->getInvH();
	Matrix4f H = trackingState->getPose(0)->getH();
	Matrix3f K; K.setZeros(); 
	K.m00 = intrinsic.x; K.m20 = intrinsic.z; K.m11 = intrinsic.y; K.m21 = intrinsic.w; K.m22 = 1;
	computeMinmaxForRayCast(H, K, imgSize);

	Vector4f invIntrinsic;
	invIntrinsic.x = 1 / intrinsic.x; invIntrinsic.y = 1 / intrinsic.y;
	invIntrinsic.z = -intrinsic.z*invIntrinsic.x; invIntrinsic.w = -intrinsic.w*invIntrinsic.y;

	float one_on_top_of_maxVoxelRange = 1 / sqrtf(DT_VOL_SIZE*DT_VOL_SIZE + DT_VOL_SIZE*DT_VOL_SIZE + DT_VOL_SIZE*DT_VOL_SIZE);

#pragma omp parallel 
	{
#pragma omp for
		for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		{
			raycastAsHeaviside(outSDF_ptr, outPtCloud_ptr, x, y, imgSize, voxelData, invH, invIntrinsic, raycastMinmax, one_on_top_of_maxVoxelRange, raycastBoundingBox);
		}
	}

}

void ISRJointTracker_CPU::updatePfImage()
{		
	float *pf_ptr = pfImage->GetData(MEMORYDEVICE_CPU);
	Vector4f *inimg_ptr = frame->currentLevel->rgbd->GetData(MEMORYDEVICE_CPU);

	float *posterior = frame->histogram->posterior;
	int noBins = frame->histogram->noBins;

	for (int i = 0; i < pfImage->dataSize;i++)
		pf_ptr[i] = getPf(inimg_ptr[i], posterior, noBins);
}

void ISRJointTracker_CPU::initializeTracker(const Vector2i& imageSize)
{
	HeavisideImage = new FloatImage(imageSize, MEMORYDEVICE_CPU);
	raycastPointCloud = new Float4Image(imageSize, MEMORYDEVICE_CPU);
	pfImage = new FloatImage(imageSize, MEMORYDEVICE_CPU);
}


