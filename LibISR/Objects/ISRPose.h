#pragma once
#include "../../Utils/LibISRDefine.h"

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

			Matrix4f H;// transformation obj->cam
			Matrix4f invH;// inverse transformation cam->obj

		private:

			// rotation matrix here is already column major
			_CPU_AND_GPU_CODE_ Matrix3f getRotationMatrixFromMRP(const Vector3f &r)
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
			_CPU_AND_GPU_CODE_ Matrix4f getProjectionMatrixFromRT(const Vector3f &r, const Vector3f &t)
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
			_CPU_AND_GPU_CODE_ Matrix4f getProjectionMatrixFromParam(const float* step)
			{
				Vector3f dt(step), dr(&step[3]);
				return getProjectionMatrixFromRT(dr, dt);
			}

			_CPU_AND_GPU_CODE_ Vector3f getMRPfromDegree(Vector3f r)
			{
				float rotationX = r.x * DEGTORAD;
				float rotationY = r.y * DEGTORAD;
				float rotationZ = r.z * DEGTORAD;

				float c1 = cos(rotationY / 2);
				float c2 = cos(rotationZ / 2);
				float c3 = cos(rotationX / 2);

				float s1 = sin(rotationY / 2);
				float s2 = sin(rotationZ / 2);
				float s3 = sin(rotationX / 2);

				float c1c2 = c1 * c2;
				float s1s2 = s1 * s2;

				float rotation1 = c1c2*s3 + s1s2*c3;
				float rotation2 = s1*c2*c3 + c1*s2*s3;
				float rotation3 = c1*s2*c3 - s1*c2*s3;
				float rotation4 = c1c2*c3 - s1s2*s3;

				float normal = 1 / sqrt(rotation1 *rotation1 + rotation2 * rotation2 + rotation3 * rotation3 + rotation4 * rotation4);

				float b0 = rotation4 * normal;
				float b1 = rotation1 * normal;
				float b2 = rotation2 * normal;
				float b3 = rotation3 * normal;

				Vector3f theta;

				theta.x = b1 / (1 + b0);
				theta.y = b2 / (1 + b0);
				theta.z = b3 / (1 + b0);

				return theta;
			}

		public:

			// set values
			_CPU_AND_GPU_CODE_ void setFromH(const Matrix4f &M){ H = M; H.inv(invH); }
			_CPU_AND_GPU_CODE_ void setFromInvH(const Matrix4f &M){ invH = M; invH.inv(H); }

			_CPU_AND_GPU_CODE_ void setHFromRT(const Vector3f &r, const Vector3f &t){ H = getProjectionMatrixFromRT(r, t); H.inv(invH); }
			_CPU_AND_GPU_CODE_ void setHFromParam(const float* param){ H = getProjectionMatrixFromParam(param); H.inv(invH); }

			_CPU_AND_GPU_CODE_ void setInvHFromRT(const Vector3f &r, const Vector3f &t){ invH = getProjectionMatrixFromRT(r, t); invH.inv(H); }
			_CPU_AND_GPU_CODE_ void setInvHFromParam(const float* param){ invH = getProjectionMatrixFromParam(param); invH.inv(H); }

			//get values
			_CPU_AND_GPU_CODE_ const Matrix4f& getH() const { return H; }
			_CPU_AND_GPU_CODE_ const Matrix4f& getInvH() const { return invH; }

			// apply incremental change to back projection matrix
			_CPU_AND_GPU_CODE_ void applyIncrementalChangeToInvH(const Vector3f &dr, const Vector3f &dt)
			{
				Matrix4f deltaM = getProjectionMatrixFromRT(dr, dt);
				invH = deltaM*invH; invH.inv(H);
			}

			_CPU_AND_GPU_CODE_ void applyIncrementalChangeToInvH(const float* step)
			{
				Matrix4f deltaM = getProjectionMatrixFromParam(step);
				invH = deltaM*invH; invH.inv(H);
			}

			_CPU_AND_GPU_CODE_ void applyIncrementalRotationToInvHInDegree(const Vector3f& dr)
			{
				Vector3f r = getMRPfromDegree(dr);
				applyIncrementalChangeToInvH(r, Vector3f(0.0f));
			}

			// apply incremental change to projection matrix
			_CPU_AND_GPU_CODE_ void applyIncrementalChangeToH(const Vector3f &dr, const Vector3f &dt)
			{
				Matrix4f deltaM = getProjectionMatrixFromRT(dr, dt);
				H = deltaM*H; H.inv(invH);
			}

			_CPU_AND_GPU_CODE_ void applyIncrementalChangeToH(const float* step)
			{
				Matrix4f deltaM = getProjectionMatrixFromParam(step);
				H = deltaM*H;
				H.inv(invH);
			}

			ISRPose(void) { H.setZeros(); invH.setZeros(); }

		};

		typedef ISRPose* ISRPose_ptr;
}
}

