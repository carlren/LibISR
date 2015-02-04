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

			ISRShape_ptr shapes;

		public:

			int nObjs;
			bool useGPU;

			_CPU_AND_GPU_CODE_ const ISRShape_ptr getShapeList() const { return shapes; }

			_CPU_AND_GPU_CODE_ const ISRShape_ptr getShape(int id) const { return &shapes[id]; }
			_CPU_AND_GPU_CODE_ ISRShape_ptr getShape(int id) { return &shapes[id]; }

			void loadShapeFromFile(const char* fileName, Vector3i size, int id)
			{
				if (useGPU)
				{
				}
				else
				{
					shapes[id].loadShapeFromFile(fileName, size);
				}
			}

			ISRShapeUnion(int count, bool usegpu)
			{
				nObjs = count;
				useGPU = usegpu;

				ISRShape_ptr shapes_host = new ISRShape[nObjs];				
				for (int i = 0; i < nObjs; i++) shapes_host[i].initialize(useGPU, i);

				if (useGPU)
				{
					ORcudaSafeCall(cudaMalloc((void**)&shapes, sizeof(ISRShape)*nObjs));
					ORcudaSafeCall(cudaMemcpy(shapes, shapes_host, sizeof(ISRShape)*nObjs, cudaMemcpyHostToDevice));
					
					free(shapes_host);
				}
				else shapes = shapes_host;
			}

			~ISRShapeUnion()
			{
				if (useGPU)  ORcudaSafeCall(cudaFree(shapes)); else delete shapes;
			}

		};
	}
}

