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

			ISRShapeUnion(int count, bool usegpu)
			{
				nObjs = count;
				shapes = new ISRShape[nObjs];
				useGPU = usegpu;
				for (int i = 0; i < nObjs; i++) shapes[i].initialize(useGPU, i);
			}

			~ISRShapeUnion()
			{
				delete shapes;
				//for (int i = 0; i < nObjs; i++) delete &shapes[i];
			}

		};
	}
}

