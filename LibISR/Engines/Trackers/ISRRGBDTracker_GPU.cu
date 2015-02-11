#include "ISRRGBDTracker_GPU.h"
#include "ISRRGBDTracker_DA.h"

#include "../../Utils/ISRCUDAUtils.h"

#include "../../../LibISRUtils/IOUtil.h"
#include "../../../ORUtils/CUDADefines.h"


using namespace LibISR::Engine;
using namespace LibISR::Objects;

__global__ void evaluateEnergy_device(Vector3f* e_device, Vector4f* ptcloud_ptr, ISRShape_ptr shapes, ISRPose_ptr poses, int count, int objCount);
__global__ void computeJacobianAndHessian_device(float* g_device, float* h_device, Vector4f* ptcloud_ptr, ISRShape_ptr shapes, ISRPose_ptr poses, int count, int objCount);
__global__ void lableForegroundPixels_device(Vector4f* ptcloud_ptr, Vector4f* rgbd_ptr, ISRShape_ptr shapes, ISRPose_ptr poses, int count, int objCount);


LibISR::Engine::ISRRGBDTracker_GPU::ISRRGBDTracker_GPU(int nObjs, const Vector2i& imgSize) :ISRRGBDTracker(nObjs, true)
{
	Vector2i gridSize((imgSize.x + 15) / 16, (imgSize.y + 15) / 16);

	int e_size = gridSize.x*gridSize.y;
	int g_size = ATb_Size*e_size;
	int h_size = ATA_size*e_size;

	energy_host = new Vector3f[e_size];
	gradient_host = new float[g_size];
	hessian_host = new float[h_size];

	ORcudaSafeCall(cudaMalloc((void**)&energy_dvic, sizeof(Vector3f)*e_size));
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
	int ptCount = this->frame->ptCloud->dataSize;
	int objCount = trackerState->numPoses();
	Vector4f* ptcloud_ptr = this->frame->ptCloud->GetData(true);
	ISRShape_ptr shapes = shapeUnion->getShapeList(true);
	ISRPose_ptr poses = trackerState->getPoseList(true);

	dim3 blockSize(256, 1);
	dim3 gridSize((int)ceil((float)ptCount / (float)blockSize.x), 1);

	ORcudaSafeCall(cudaMemset(energy_dvic, 0, sizeof(Vector3f)*gridSize.x));

	evaluateEnergy_device << <gridSize, blockSize >> > (energy_dvic, ptcloud_ptr, shapes, poses, ptCount, objCount);

	ORcudaSafeCall(cudaMemcpy(energy_host, energy_dvic, sizeof(Vector3f)*gridSize.x, cudaMemcpyDeviceToHost));

 	Vector3f e(0.0f);
	for (int i = 0; i < gridSize.x; i++) e += energy_host[i];
	energy[0] = e.z < 100 ? 0.0f : e.x/e.y;
}

void LibISR::Engine::ISRRGBDTracker_GPU::computeJacobianAndHessian(float *gradient, float *hessian, Objects::ISRTrackingState * trackerState) const
{
	int ptCount = this->frame->ptCloud->dataSize;
	int objCount = trackerState->numPoses();
	int noPara = objCount * 6;
	int noParaSQ = noPara*noPara;

	Vector4f* ptcloud_ptr = this->frame->ptCloud->GetData(true);
	ISRShape_ptr shapes = shapeUnion->getShapeList(true);
	ISRPose_ptr poses = trackerState->getPoseList(true);

	dim3 blockSize(256, 1);
	dim3 gridSize((int)ceil((float)ptCount / (float)blockSize.x), 1);

	ORcudaSafeCall(cudaMemset(gradient_divc, 0, sizeof(float)*gridSize.x*noPara));
	ORcudaSafeCall(cudaMemset(hessian_divc, 0, sizeof(float)*gridSize.x*noParaSQ));

	computeJacobianAndHessian_device << <gridSize, blockSize >> > (gradient_divc, hessian_divc, ptcloud_ptr, shapes, poses, ptCount,objCount);

	ORcudaSafeCall(cudaMemcpy(gradient_host, gradient_divc, sizeof(float)*gridSize.x*noPara, cudaMemcpyDeviceToHost));
	ORcudaSafeCall(cudaMemcpy(hessian_host, hessian_divc, sizeof(float)*gridSize.x*noParaSQ, cudaMemcpyDeviceToHost));

	float *globalGradient = new float[noPara];
	float *globalHessian = new float[noParaSQ];

	for (int i = 0; i < noPara; i++) globalGradient[i] = 0.0f;
	for (int i = 0; i < noParaSQ; i++) globalHessian[i] = 0.0f;

	for (int i = 0; i < gridSize.x; i++)
	{
		for (int p = 0; p < noPara; p++)  globalGradient[p] += gradient_host[i * noPara + p];
		for (int p = 0; p < noParaSQ; p++)  globalHessian[p] += hessian_host[i * noParaSQ + p];
	}


	for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++) hessian[r + c * noPara] = globalHessian[counter];
	for (int r = 0; r < noPara; ++r) for (int c = r + 1; c < noPara; c++) hessian[r + c * noPara] = hessian[c + r * noPara];
	for (int r = 0; r < noPara; ++r) gradient[r] = globalGradient[r];
	
}

void LibISR::Engine::ISRRGBDTracker_GPU::lableForegroundPixels(Objects::ISRTrackingState * trackerState)
{
	int ptCount = this->frame->ptCloud->dataSize;
	int objCount = trackerState->numPoses();
	Vector4f* ptcloud_ptr = this->frame->ptCloud->GetData(true);
	Vector4f* rgbd_ptr = this->frame->currentLevel->rgbd->GetData(true);
	ISRShape_ptr shapes = shapeUnion->getShapeList(true);
	ISRPose_ptr poses = trackerState->getPoseList(true);

	dim3 blockSize(256, 1);
	dim3 gridSize((int)ceil((float)ptCount / (float)blockSize.x), 1);

	lableForegroundPixels_device << <gridSize, blockSize >> >(ptcloud_ptr, rgbd_ptr, shapes, poses, ptCount, objCount);
}

void LibISR::Engine::ISRRGBDTracker_GPU::lableForegroundPixels(Objects::ISRTrackingState * trackerState, Vector4i bb)
{

}


__global__ void evaluateEnergy_device(Vector3f* e_device, Vector4f* ptcloud_ptr, ISRShape_ptr shapes, ISRPose_ptr poses, int count, int objCount)
{
	int locId_global = threadIdx.x + blockIdx.x * blockDim.x, locId_local = threadIdx.x;

	__shared__ float dim_shared[256], count_shared[256], pfcount_shared[256];

	dim_shared[locId_local] = 0.0f;
	count_shared[locId_local] = 0.0f;
	pfcount_shared[locId_local] = 0.0f;

	if (locId_global < count)
	{
		Vector4f inpt = ptcloud_ptr[locId_global];
		if (inpt.w > -1.0f)
		{
			dim_shared[locId_local] = computePerPixelEnergy(inpt, shapes, poses, objCount);
			if (dim_shared[locId_local] > 0.0f)
			{
				count_shared[locId_local] = 1.0f;
				if (inpt.w > 0.5f) pfcount_shared[locId_local] = 1.0f;
			}
			else dim_shared[locId_local] = 0.0f;

		}
	}

	{ //reduction for e_device
		__syncthreads();
		if (locId_local < 128)
		{
			dim_shared[locId_local] += dim_shared[locId_local + 128];
			count_shared[locId_local] += count_shared[locId_local + 128];
			pfcount_shared[locId_local] += pfcount_shared[locId_local + 128];
		}
		__syncthreads();
		if (locId_local < 64) 
		{
			dim_shared[locId_local] += dim_shared[locId_local + 64];
			count_shared[locId_local] += count_shared[locId_local + 64];
			pfcount_shared[locId_local] += pfcount_shared[locId_local + 64];
		}
		__syncthreads();
		if (locId_local < 32)
		{
			warpReduce(dim_shared, locId_local);
			warpReduce(count_shared, locId_local);
			warpReduce(pfcount_shared, locId_local);
		}
		if (locId_local == 0)
		{
			e_device[blockIdx.x].x = dim_shared[locId_local];
			e_device[blockIdx.x].y = count_shared[locId_local];
			e_device[blockIdx.x].z = pfcount_shared[locId_local];
		}
	}
}

__global__ void computeJacobianAndHessian_device(float* g_device, float* h_device, Vector4f* ptcloud_ptr, ISRShape_ptr shapes, ISRPose_ptr poses, int count, int objCount)
{
	int locId_global = threadIdx.x + blockIdx.x * blockDim.x, locId_local = threadIdx.x;

	__shared__ float dim_shared[256];
	__shared__ bool shouldAdd; bool hasValidData = false;

	float localGradient[MAX_OBJECT_COUNT * 6], localHessian[MAX_OBJECT_COUNT*MAX_OBJECT_COUNT * 36];
	int noPara = objCount * 6;
	int noParaSQ = noPara*noPara;

	shouldAdd = false;

	__syncthreads();

	if (locId_global < count)
	{
		Vector4f cPt = ptcloud_ptr[locId_global];
		if (cPt.w > -1.0f)
		{
			if (computePerPixelJacobian(localGradient, cPt, shapes, poses, objCount))
			{
				shouldAdd = true; hasValidData = true;

				for (int r = 0, counter = 0; r < noPara; r++) for (int c = 0; c <= r; c++, counter++)
					localHessian[counter] = localGradient[r] * localGradient[c];
			}
		}
	}

	__syncthreads();

	if (!hasValidData && shouldAdd)
	{
		for (int i = 0; i < noPara; i++) localGradient[i] = 0.0f;
		for (int i = 0; i < noParaSQ; i++) localHessian[i] = 0.0f;
	}

	if (shouldAdd)
	{
		//reduction for gradient
		for (int paraId = 0; paraId < noPara; paraId++)
		{
			dim_shared[locId_local] = localGradient[paraId];
			__syncthreads();

			if (locId_local < 128) dim_shared[locId_local] += dim_shared[locId_local + 128];
			__syncthreads();
			if (locId_local < 64) dim_shared[locId_local] += dim_shared[locId_local + 64];
			__syncthreads();

			if (locId_local < 32) warpReduce(dim_shared, locId_local);

			if (locId_local == 0) g_device[blockIdx.x * noPara + paraId] = dim_shared[locId_local];
		}

		__syncthreads();

		//reduction for hessian
		for (int paraId = 0; paraId < noParaSQ; paraId++)
		{
			dim_shared[locId_local] = localHessian[paraId];
			__syncthreads();

			if (locId_local < 128) dim_shared[locId_local] += dim_shared[locId_local + 128];
			__syncthreads();
			if (locId_local < 64) dim_shared[locId_local] += dim_shared[locId_local + 64];
			__syncthreads();

			if (locId_local < 32) warpReduce(dim_shared, locId_local);

			if (locId_local == 0) h_device[blockIdx.x * noParaSQ + paraId] = dim_shared[locId_local];
		}
	}

}

__global__ void lableForegroundPixels_device(Vector4f* ptcloud_ptr, Vector4f* rgbd_ptr, ISRShape_ptr shapes, ISRPose_ptr poses, int count, int objCount)
{
	int locId_global = threadIdx.x + blockIdx.x * blockDim.x, locId_local = threadIdx.x;

	if (locId_global < count)
	{
		if (ptcloud_ptr[locId_global].w > 0.0f)
		{
			float dt = findPerPixelDT(ptcloud_ptr[locId_global], shapes, poses,objCount);
			
			if (fabs(dt) <= 5) { rgbd_ptr[locId_global].w = HIST_FG_PIXEL; }
			else { rgbd_ptr[locId_global].w = HIST_BG_PIXEL; }
		}
	}
}
