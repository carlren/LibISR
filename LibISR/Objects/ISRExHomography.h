#pragma once

#include "../Utils/LibISRDefine.h"

#include <stdio.h>

namespace CoreISR
{
	namespace Objects
	{
		/** \brief
		Represents the extrinsic calibration between RGB and depth
		cameras
		*/
		class ISRExHomography
		{
		public:
			/** The transformation matrix representing the
			extrinsic calibration data.
			*/
			Matrix3f H;
			/** Inverse of the above. */
			Vector3f T;

			/** Setup from a given 4x4 matrix, where only the upper
			three rows are used. More specifically, m00...m22
			are expected to contain a rotation and m30...m32
			contain the translation.
			*/

			void SetFrom(const Matrix3f & srcH, const Vector3f & srcT)
			{
				this->H = srcH;
				this->T = srcT;
			}

			ISRExHomography()
			{
				H.setZeros();
				H.m00 = H.m11 = H.m22 = 1.0f;
				T=Vector3f(0.0f);
			}
		};
	}
}
