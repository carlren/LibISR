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
			Stores a set of shapes and their corrisponding poses
			Very simple now, should be more useful in the future

			refactored: Jan/13/2015
		*/

		class ISRShapeUnion
		{

		public:

			ISRShape **shapes;
			ISRPose **poses;

			int nObjs;

			ISRShapeUnion(int count)
			{
				nObjs = count;
				shapes = (ISRShape**)malloc(count*sizeof(ISRShape*));
				poses = (ISRPose**)malloc(count*sizeof(ISRPose*));
			}


			~ISRShapeUnion()
			{
				for (int i = 0; i < nObjs; i++)
				{
					delete shapes[i];
					delete poses[i];
				}
			}

		};
	}
}

