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

		public:
			_CPU_AND_GPU_CODE_ ISRPose_ptr getPoseList() { return poses; }

			_CPU_AND_GPU_CODE_ const ISRPose_ptr getPose(int id) const { return &poses[id]; }
			_CPU_AND_GPU_CODE_ ISRPose_ptr getPose(int id) { return &poses[id]; }

			_CPU_AND_GPU_CODE_ int numPoses() const { return nPose; }

			_CPU_AND_GPU_CODE_ void inline applyIncrementalPoseChangesToInvH(float* step)
			{
				for (int i = 0, j = 0; i < nPose; i++, j += 6)
					poses[i].applyIncrementalChangeToInvH(&step[j]);
			}

			_CPU_AND_GPU_CODE_ void inline applyIncrementalPoseChangesToIH(float* step)
			{
				for (int i = 0, j = 0; i < nPose; i++, j += 6)
					poses[i].applyIncrementalChangeToH(&step[j]);
			}

			_CPU_AND_GPU_CODE_ void setFrom(const ISRTrackingState &inposes)
			{
				int count = inposes.numPoses();
				for (int i = 0; i < count; i++)
				{
					this->getPose(i)->setFromH(inposes.getPose(i)->getH());
				}
			}

			ISRTrackingState(int num)
			{
				nPose = num;
				poses = new ISRPose[num];
			}

			~ISRTrackingState()
			{
				free(poses);
			}

		};
	}
}