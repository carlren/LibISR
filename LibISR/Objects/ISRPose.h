#pragma once

#include "../Utils/LibISRDefine.h"
#include "../Utils/MathUtils.h"

// ok, we use the openGL column major matrix now

namespace CoreISR
{
	namespace Objects
	{
		class ISRPose
		{
		private:

			Matrix3f GetRotationMatrixFromMRP(const Vector3f &r)
			{
				Matrix3f outR;

				float t1 = r.x;
				float t2 = r.y;
				float t3 = r.z;

				float tsq = t1*t1 + t2*t2 + t3*t3;

				float tsum = 1 - tsq;

				outR.m00 = 4 * t1*t1 - 4 * t2*t2 - 4 * t3*t3 + tsum*tsum;	outR.m10 = 8 * t1*t2 - 4 * t3*tsum;							outR.m20 = 8 * t1*t3 + 4 * t2*tsum;
				outR.m01 = 8 * t1*t2 + 4 * t3*tsum;							outR.m11 = 4 * t2*t2 - 4 * t1*t1 - 4 * t3*t3 + tsum*tsum;	outR.m21 = 8 * t2*t3 - 4 * t1*tsum;
				outR.m02 = 8 * t1*t3 - 4 * t2*tsum;							outR.m12 = 8 * t2*t3 + 4 * t1*tsum;							outR.m22 = 4 * t3*t3 - 4 * t2*t2 - 4 * t1*t1 + tsum*tsum;

				for (int i = 0; i<9; i++) outR.m[i] /= ((1 + tsq)*(1 + tsq));

				return outR;
			}


			void UpdateHfromCurrentRT()
			{
				int idx = 0;

				H.m00 = R.m00; H.m10 = R.m10; H.m20 = R.m20; H.m30 = t[0];
				H.m01 = R.m01; H.m11 = R.m11; H.m21 = R.m21; H.m31 = t[1];
				H.m02 = R.m02; H.m12 = R.m12; H.m22 = R.m22; H.m32 = t[2];
				H.m03 = 0.0f;  H.m13 = 0.0f;  H.m23 = 0.0f;  H.m33 = 1.0f;

				H.inv(invH);
			}

			void ApplyIncreamentalPoseChange(float *pose)
			{
				Vector3f tt, rr;
				tt.x = pose[0]; tt.y = pose[1]; tt.z = pose[2];
				rr.x = pose[3]; rr.y = pose[4]; rr.z = pose[5];

				Matrix3f tmpR = GetRotationMatrixFromMRP(rr);
				
				R = tmpR*R;
				t = tmpR*t + tt;
			}

		public:

			float scale;
			Vector3f r; // angle in rad
			Vector3f mrp; // mrp parameter
			Matrix3f R; // c->o rotation matrix
			Vector3f t; // c->o traslation
			
			Matrix4f H;// transformation c->o
			Matrix4f invH;// inverse transforamtion o->c

			ISRPose(void) { } 
			~ISRPose(void) { }

			// [t,r]^T, the rotation is already in MRP
			void SetFromStep(float* step)
			{
				t.x = step[0]; t.y = step[1]; t.z = step[2];
				mrp.x = step[3]; mrp.y = step[4]; mrp.z = step[5];
				
				R = GetRotationMatrixFromMRP(mrp);
				UpdateHfromCurrentRT();
			}

			// [t,r]^T, the rotation is already in MRP
			void UpdateFromStep(float* step)
			{
				ApplyIncreamentalPoseChange(step);
				UpdateHfromCurrentRT();
			}


		};
	}
}

