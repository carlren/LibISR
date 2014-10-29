#pragma once

#include "ISRIntrinsics.h"

namespace CoreISR
{
	namespace Objects
	{
		/** \brief
		Represents the joint RGBD calibration parameters
		*/
		class ISRCalib
		{
		public:
			/// Intrinsic parameters of the RGB camera.
			ISRIntrinsics intrinsics_rgb;
			/// Intrinsic parameters of the depth camera.
			ISRIntrinsics intrinsics_d;
			/** @brief
			Extrinsic calibration between RGB and depth
			cameras.

			This transformation takes points from the RGB
			camera coordinate system to the depth camera
			coordinate system.
			*/
			ITMExtrinsics trafo_rgb_to_depth;
			/// Calibration information to compute depth from disparity images.
			ITMDisparityCalib disparityCalib;
		};
	}
}
