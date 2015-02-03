#include "ISRRGBDTracker_GPU.h"
#include "ISRRGBDTracker_DA.h"

#include "../../Utils/ISRCUDAUtils.h"

#include "../../../LibISRUtils/IOUtil.h"
#include "../../../ORUtils/CUDADefines.h"


using namespace LibISR::Engine;
using namespace LibISR::Objects;

__global__ void evaluateEnergy_device(float* e_device, Vector4f* ptcloud_ptr, ISRShapeUnion* shapeunion, ISRTrackingState* state, int count);



LibISR::Engine::ISRRGBDTracker_GPU::ISRRGBDTracker_GPU(int nObjs, const Vector2i& imgSize) :ISRRGBDTracker(nObjs, true)
{
	Vector2i gridSize((imgSize.x + 15) / 16, (imgSize.y + 15) / 16);

	int e_size = gridSize.x*gridSize.y;
	int g_size = ATb_Size*e_size;
	int h_size = ATA_size*e_size;

	energy_host = new float[e_size];
	gradient_host = new float[g_size];
	hessian_host = new float[h_size];

	ORcudaSafeCall(cudaMalloc((void**)&energy_dvic,sizeof(float)*e_size));
	ORcudaSafeCall(cudaMalloc((void**)&gradient_divc, sizeof(float)*g_size));
	ORcudaSafeCall(cudaMalloc((void**)&hessian_divc, sizeof(float)*h_size));

}
LibISR::Engine::ISRRGBDTracker_GPU::~ISRRGBDTracker_GPU()
{
	delete[] energy_host;
	delete[] gradient_host;
	delete[] hessian_host;

	ORcudaSafeCall(cudaFree(energy_dvic));
	ORcudaSafeCall(cudaFree(gradient_divc));
	ORcudaSafeCall(cudaFree(hessian_divc));
}



void LibISR::Engine::ISRRGBDTracker_GPU::evaluateEnergy(float *energy, Objects::ISRTrackingState * trackerState)
{
	int count = this->frame->ptCloud->dataSize;

	dim3 blockSize(256, 1);
	dim3 gridSize((int)ceil((float)count / (float)blockSize.x), 1);

	Vector4f* ptcloud_ptr = this->frame->ptCloud->GetData(true);

	ORcudaSafeCall(cudaMemset(energy_dvic, 0, sizeof(float)*gridSize.x));

	evaluateEnergy_device <<<gridSize, blockSize >>> (energy_dvic, ptcloud_ptr, shapeUnion, trackerState, count);
	
	ORcudaSafeCall(cudaMemcpy(energy_host,energy_dvic,sizeof(float)*gridSize.x,cudaMemcpyDeviceToHost));

	float e = 0;
	
	for (int i = 0; i < gridSize.x; i++) e += energy_host[i];
	
	energy[0] = e ;
}

void LibISR::Engine::ISRRGBDTracker_GPU::computeJacobianAndHessian(float *gradient, float *hessian, Objects::ISRTrackingState * trackerState) const
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

void LibISR::Engine::ISRRGBDTracker_GPU::lableForegroundPixels(Objects::ISRTrackingState * trackerState)
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


__global__ void evaluateEnergy_device(float* e_device, Vector4f* ptcloud_ptr, ISRShapeUnion* shapeunion, ISRTrackingState* state, int count)
{
	int locId_global = threadIdx.x + blockIdx.x * blockDim.x, locId_local = threadIdx.x;

	__shared__ float dim_shared[256];

	dim_shared[locId_local] = 0.0f;

	if (locId_global < count)
	{
		Vector4f inpt = ptcloud_ptr[locId_global];
		if (inpt.w > -1.0f) dim_shared[locId_local] = computePerPixelEnergy(inpt, shapeunion, state);
	}

	{ //reduction for e_device
		__syncthreads();

		if (locId_local < 128) dim_shared[locId_local] += dim_shared[locId_local + 128];
		__syncthreads();
		if (locId_local < 64) dim_shared[locId_local] += dim_shared[locId_local + 64];
		__syncthreads();

		if (locId_local < 32) warpReduce(dim_shared, locId_local);

		if (locId_local == 0) e_device[blockIdx.x] = dim_shared[locId_local];
	}
}
