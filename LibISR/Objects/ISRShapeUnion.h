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

			ISRShape_ptr *shapes;

		public:

			int nObjs;
			bool useGPU;

			const ISRShape_ptr* getShapeList() const { return shapes; }

			const ISRShape_ptr getShape(int id) const { return shapes[id]; }
			ISRShape_ptr getShape(int id) { return shapes[id]; }


			ISRShapeUnion(int count, bool useGPU)
			{
				nObjs = count;
				shapes = (ISRShape_ptr*)malloc(nObjs*sizeof(ISRShape_ptr));
				for (int i = 0; i < nObjs; i++) shapes[i] = new ISRShape(useGPU);
			}

			~ISRShapeUnion()
			{
				for (int i = 0; i < nObjs; i++)
				{
					delete shapes[i];
				}
			}

		};
	}
}

