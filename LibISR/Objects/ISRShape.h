#pragma once
#include <stdio.h>

#include "../Utils/LibISRDefine.h"
#include "../../ORUtils/CUDADefines.h"

#include "../Objects/ISRPose.h"

namespace LibISR
{
	namespace Objects
	{
		/**
		\brief
			object SDF
			this version don't contain any pose
			radiant of SDF are computed on the fly,
			so no gradient volume is kept

			refactored: Jan/13/2015
		*/
		class ISRShape
		{
		private:

			float *dt;

		public:

			int objectId;
			Vector3i volSize;
			int allocatedSize;

			bool useGPU;
			bool modelShared;
			bool modelLoaded;
			
			_CPU_AND_GPU_CODE_ inline float* getSDFVoxel(){ return dt; }
			_CPU_AND_GPU_CODE_ inline const float* getSDFVoxel() const { return dt; }

			void  loadShapeFromFile(char* fileName, Vector3i size)
			{
				volSize = size;
				allocatedSize = size.x*size.y*size.z;

				float *dt_host = (float*)malloc(sizeof(float) * allocatedSize);
				
				FILE* f;
				f = fopen(fileName, "rb");
				fread(dt, sizeof(float) * this->allocatedSize, 1, f);
				fclose(f);

				if (useGPU)
				{
					ORcudaSafeCall(cudaMalloc((void**)&dt,allocatedSize*sizeof(float)));
					ORcudaSafeCall(cudaMemcpy(dt, dt_host, allocatedSize*sizeof(float), cudaMemcpyHostToDevice));

					free(dt_host);
				}
				else
				{
					dt = dt_host;
				}

				modelLoaded = true;
				modelShared = false;
			}

			void  loadShapeFromExistingShape(const ISRShape &shape)
			{
				volSize = shape.volSize;
				allocatedSize = shape.allocatedSize;
				useGPU = shape.useGPU;

				if (useGPU)
				{
					ORcudaSafeCall(cudaMalloc((void**)&dt, allocatedSize*sizeof(float)));
					ORcudaSafeCall(cudaMemcpy(dt, shape.getSDFVoxel(), allocatedSize*sizeof(float), cudaMemcpyHostToDevice));
				}
				else
				{
					dt = (float*)malloc(sizeof(float) * allocatedSize);
					memcpy(dt, shape.getSDFVoxel(), allocatedSize*sizeof(float));
				}

				modelLoaded = true;
				modelShared = false;
			}

			void  shareSDFWithExistingShape(ISRShape &shape)
			{
				volSize = shape.volSize;
				allocatedSize = shape.allocatedSize;
				useGPU = shape.useGPU;

				dt = shape.getSDFVoxel();

				modelLoaded = true;
				modelShared = true;
			}

			ISRShape(bool useGPU)
			{
				objectId = objectId;
				this->useGPU = useGPU;

				modelLoaded = false;
				modelShared = false;
			}

			~ISRShape()
			{ 
				if (modelLoaded && !modelShared)
					if (useGPU) ORcudaSafeCall(cudaFree(dt)); else free(dt);
			}
		};

		typedef ISRShape* ISRShape_ptr;
	}
}
