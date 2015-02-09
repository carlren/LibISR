#pragma once
#include "../../Utils/LibISRDefine.h"
#include "../Basic/ISRPose.h"

namespace LibISR
{
	namespace Objects
	{
		/**
		\brief
		this class contains the current state
		of the multi-object object tracker.
		Energy function is evaluated given this class.
		*/

		class ISRTrackingState
		{
		private:

			int nPose;
			
			ISRPose_ptr poses;
			ISRPose_ptr poses_device;

		public:

			float energy;
			bool useGPU;

			ISRPose_ptr getPoseList(bool fromGPU=false) 
			{
				if (fromGPU) return poses_device;
				else return poses;
			}


			const ISRPose_ptr getPose(int id) const { return &poses[id]; }
			ISRPose_ptr getPose(int id) { return &poses[id]; }

			int numPoses() const { return nPose; }

			void inline applyIncrementalPoseChangesToInvH(float* step)
			{
				for (int i = 0, j = 0; i < nPose; i++, j += 6)
				{
					poses[i].applyIncrementalChangeToInvH(&step[j]);
				}

				if (useGPU) ORcudaSafeCall(cudaMemcpy(poses_device, poses, nPose*sizeof(ISRPose), cudaMemcpyHostToDevice));
			}

			void inline applyIncrementalPoseChangesToH(float* step)
			{
				for (int i = 0, j = 0; i < nPose; i++, j += 6)
				{
					poses[i].applyIncrementalChangeToH(&step[j]);
				}

				if (useGPU) ORcudaSafeCall(cudaMemcpy(poses_device, poses, nPose*sizeof(ISRPose), cudaMemcpyHostToDevice));
			}

			void setFrom(const ISRTrackingState &inposes)
			{
				int count = inposes.numPoses();
				for (int i = 0; i < count; i++)
				{
					this->getPose(i)->setFromH(inposes.getPose(i)->getH());
				}

				if (useGPU) ORcudaSafeCall(cudaMemcpy(poses_device, poses, nPose*sizeof(ISRPose), cudaMemcpyHostToDevice));
			}

			void inline setHFromParam(float *param, int id)
			{
				poses[id].setHFromParam(param);
				if (useGPU) ORcudaSafeCall(cudaMemcpy(poses_device, poses, nPose*sizeof(ISRPose), cudaMemcpyHostToDevice));
			}

			void inline setInvHFromParam(float *param, int id)
			{
				poses[id].setInvHFromParam(param);
				if (useGPU) ORcudaSafeCall(cudaMemcpy(poses_device, poses, nPose*sizeof(ISRPose), cudaMemcpyHostToDevice));
			}

			void inline updatePoseDeviceFromHost()
			{
				if (useGPU) ORcudaSafeCall(cudaMemcpy(poses_device, poses, nPose*sizeof(ISRPose), cudaMemcpyHostToDevice));
			}

			ISRTrackingState(int num, bool usegpu=false)
			{
				energy = 0;
				nPose = num;
				poses = new ISRPose[num];
				useGPU = usegpu;
				if (useGPU)	ORcudaSafeCall(cudaMalloc((void**)&poses_device, num*sizeof(ISRPose)));
			
			}

			~ISRTrackingState()
			{
				delete[] poses;
				if (useGPU) ORcudaSafeCall(cudaFree(poses_device));
			}

		};
	}
}