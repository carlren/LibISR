#pragma once
#include "../../Utils/LibISRDefine.h"
#include "../Basic/ISRShape.h"

namespace LibISR
{
	namespace Objects
	{
		/**
		\brief
		Stores a set of shapes and their corresponding poses
		Very simple now, should be more useful in the future

		refactored: Jan/13/2015
		*/

		class ISRShapeUnion
		{
		private:

			ISRShape_ptr shapes_device;
			ISRShape_ptr shapes_host;

		public:

			int nObjs;
			bool useGPU;

			_CPU_AND_GPU_CODE_ const ISRShape_ptr getShapeList(bool fromgpu=false) const { if (fromgpu) return shapes_device; else return shapes_host; }

			_CPU_AND_GPU_CODE_ const ISRShape_ptr getShape(int id, bool fromgpu=false) const { if (fromgpu) return &shapes_device[id]; else return &shapes_host[id]; }
			_CPU_AND_GPU_CODE_ ISRShape_ptr getShape(int id, bool fromgpu=false) { if (fromgpu) return &shapes_device[id]; else return &shapes_host[id]; }


			// these functions will only be called from CPU
			void loadShapeFromFile(const char* fileName, Vector3i size, int id)
			{
				shapes_host[id].loadShapeFromFile(fileName, size);
				if (useGPU) ORcudaSafeCall(cudaMemcpy(shapes_device, shapes_host, sizeof(ISRShape)*nObjs, cudaMemcpyHostToDevice));
			}

			void loadShapeFromExistingShape(const ISRShape &shape, int id)
			{
				shapes_host[id].loadShapeFromExistingShape(shape);
				if (useGPU) ORcudaSafeCall(cudaMemcpy(shapes_device, shapes_host, sizeof(ISRShape)*nObjs, cudaMemcpyHostToDevice));
			}
			
			void shareSDFWithExistingShape(ISRShape& shape, int id)
			{
				shapes_host[id].shareSDFWithExistingShape(shape);
				if (useGPU) ORcudaSafeCall(cudaMemcpy(shapes_device, shapes_host, sizeof(ISRShape)*nObjs, cudaMemcpyHostToDevice));
			}

			ISRShapeUnion(int count, bool usegpu)
			{
				nObjs = count;
				useGPU = usegpu;

				shapes_host = new ISRShape[nObjs];				
				for (int i = 0; i < nObjs; i++) shapes_host[i].initialize(useGPU, i);

				if (useGPU)
				{
					ORcudaSafeCall(cudaMalloc((void**)&shapes_device, sizeof(ISRShape)*nObjs));
					ORcudaSafeCall(cudaMemcpy(shapes_device, shapes_host, sizeof(ISRShape)*nObjs, cudaMemcpyHostToDevice));
				}
			}

			~ISRShapeUnion()
			{
				if (useGPU)  ORcudaSafeCall(cudaFree(shapes_device));
				delete shapes_host;
			}

		};
	}
}

