#pragma once
#include "../LibISRDefine.h"



namespace LibISR
{
	namespace Objects
	{
		/** \brief
		Represents the parameters for projection with a projective
		camera
		*/
		class ISRIntrinsics
		{
		public:
			/** The actual intrinsic calibration parameters. */
			struct ProjectionParamsSimple
			{
				Vector4f all;
				float fx, fy, px, py;
			} projectionParamsSimple;

			Matrix3f A;
			Matrix3f invA;

			int width, height;

			/** Setup all the internal members of this class from
			the given parameters. Everything is in pixel
			coordinates.
			@param fx Focal length in x direction
			@param fy Focal length in y direction
			@param cx Principal point in x direction
			@param cy Principal point in y direction
			@param sizeX Image size in x direction
			@param sizeY Image size in y direction
			*/
			void SetFrom(float fx, float fy, float cx, float cy)
			{
				projectionParamsSimple.fx = fx; projectionParamsSimple.fy = fy;
				projectionParamsSimple.px = cx; projectionParamsSimple.py = cy;
				projectionParamsSimple.all = Vector4f(fx, fy, cx, cy);

				A.setZeros();
				A.m00 = fx; A.m20 = cx; A.m11 = fy; A.m21 = cy; A.m22 = 1;
				A.inv(invA);
			}

			void SetFrom(float fx, float fy, float cx, float cy, int w, int h)
			{
				projectionParamsSimple.fx = fx; projectionParamsSimple.fy = fy;
				projectionParamsSimple.px = cx; projectionParamsSimple.py = cy;
				projectionParamsSimple.all = Vector4f(fx, fy, cx, cy);

				A.setZeros();
				A.m00 = fx; A.m20 = cx; A.m11 = fy; A.m21 = cy; A.m22 = 1;
				A.inv(invA);

				this->width = w;
				this->height = h;
			}

			const Vector4f& getParam() const { return projectionParamsSimple.all; }

			ISRIntrinsics(void)
			{
				SetFrom(580, 580, 320, 240, 640, 480);

			}

		};
	}
}
