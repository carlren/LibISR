#pragma once

#include "../Utils/LibISRDefine.h"

namespace LibISR
{
	namespace Objects
	{
		/**
		\brief
		To keep it simple and minimalist, our pose only keep
		the 6-DoF transformation from object coordinates to 
		camera coordinates in 4x4 matrix H and invH
		
		refactored: Jan/13/2015
		*/
		class ISRPose
		{

		private:

			Matrix4f H;// transformation c->o
			Matrix4f invH;// inverse transformation o->c

		private:

			// rotation matrix here is already column major
			Matrix3f getRotationMatrixFromMRP(const Vector3f &r)
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

			// the transformation matrix here is already column major
			Matrix4f getProjectionMatrixFromRT(const Vector3f &r, const Vector3f &t)
			{
				Matrix3f outR = getRotationMatrixFromMRP(r);
				
				Matrix4f M;
				M.m00 = outR.m00; M.m01 = outR.m01; M.m02 = outR.m02; M.m03 = 0;
				M.m10 = outR.m10; M.m11 = outR.m11; M.m12 = outR.m12; M.m13 = 0;
				M.m20 = outR.m20; M.m21 = outR.m21; M.m22 = outR.m22; M.m23 = 0;
				M.m30 = t.x;	  M.m31 = t.y;		M.m32 = t.z;	  M.m33 = 1;

				return M;
			}

			// get the transformation matrix from pose parameters step = [t' r']
			Matrix4f getProjectionMatrixFromParam(const float* step)
			{
				Vector3f dt(step), dr(&step[3]);
				return getProjectionMatrixFromRT(dr, dt);
			}

		public:

			// set values
			void setFromH(const Matrix4f &M){ H = M; H.inv(invH); }
			void setFromInvH(const Matrix4f &M){ invH = M; invH.inv(H); }
			void setFromRT(const Vector3f &r, const Vector3f &t){ H = getProjectionMatrixFromRT(r, t); H.inv(invH);}
			void setFromParam(const float* param){ H = getProjectionMatrixFromParam(param); H.inv(invH);}

			//get values
			Matrix4f* getH(){ return &H;}
			Matrix4f* getInvH() { return &invH; }


			// apply incremental change to back projection matrix
			void applyIncrementalChangeToInvH(const Vector3f &dr, const Vector3f &dt)
			{
				Matrix4f deltaM = getProjectionMatrixFromRT(dr, dt);
				invH = deltaM*invH; invH.inv(H);
			}

			void applyIncrementalChangeToInvH(const float* step)
			{
				Matrix4f deltaM = getProjectionMatrixFromParam(step);
				invH = deltaM*invH; invH.inv(H);
			}

			// apply incremental change to projection matrix
			void applyIncrementalChangeToH(const Vector3f &dr, const Vector3f &dt)
			{
				Matrix4f deltaM = getProjectionMatrixFromRT(dr, dt);
				H = deltaM*H; H.inv(invH);
			}

			void applyIncrementalChangeToH(const float* step)
			{
				Matrix4f deltaM = getProjectionMatrixFromParam(step);
				H = deltaM*H; H.inv(invH);
			}

			ISRPose(void) { H.setZeros(); invH.setZeros(); }
			~ISRPose(void) { }

		};

		typedef ISRPose* ISRPose_ptr;
}
}

