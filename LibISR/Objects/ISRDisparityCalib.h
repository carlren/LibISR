#pragma once

#include "../Utils/LibISRDefine.h"

#include <stdlib.h>

namespace CoreISR
{
	namespace Objects
	{
		/** \brief
		Represents the calibration information to compute a depth
		image from the disparity image typically received from a
		Kinect.
		*/
		class ISRDisparityCalib
		{
		public:
			/** These are the actual parameters. */
			Vector2f params;

			/** Setup from given arguments. */
			void SetFrom(float a, float b)
			{
				params.x = a; params.y = b;
			}

			ISRDisparityCalib(void)
			{
				// standard calibration parameters, not very accurate...
				params.x = 1090.f; params.y = 0.075f;
			}
		};
	}
}
