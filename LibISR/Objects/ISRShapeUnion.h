#pragma once

#include <stdio.h>

#include "../Utils/LibISRDefine.h"
#include "../Objects/ISRShape.h"
#include "../Objects/ISRFrame.h"

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

			const ISRShape_ptr getShapeList() const { return shapes; }

			const ISRShape_ptr getShape(int id) const { return &shapes[id]; }
			ISRShape_ptr getShape(int id) { return &shapes[id]; }

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

