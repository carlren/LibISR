#pragma once

#include "../Utils/LibISRDefine.h"
#include "ISRPose.h"

namespace LibISR
{
	namespace Objects
	{
		/**
		\brief
			this class contains a set of object poses
			trackers will take this as input for cost 
			function evaluation
		*/

		class ISRPoseSet
		{
		public:

			ISRPose_ptr *poses;
			int nPose;

			ISRPoseSet(int num)
			{
				nPose = num;
				poses = (ISRPose_ptr*)malloc(num*sizeof(ISRPose_ptr*));

				for (int i = 0; i < num; i++) poses[i] = new ISRPose();
			}

			~ISRPoseSet()
			{

			}
		};
	}
}