#pragma once

#include "../Utils/LibISRDefine.h"
#include "ISRPose.h"

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

			ISRPose_ptr *poses;
			int nPose;

		public:

			ISRPose_ptr* getPoseList() { return poses; }
			
			const ISRPose_ptr getPose(int id) const { return poses[id]; }
			ISRPose_ptr getPose(int id) { return poses[id]; }

			int numPoses() const { return nPose; }

			void inline applyIncrementalPoseChangesToInvH(float* step)
			{
				for (int i = 0, j = 0; i < nPose; i++, j += 6)
					poses[i]->applyIncrementalChangeToInvH(&step[j]);
			}

			void inline applyIncrementalPoseChangesToIH(float* step)
			{
				for (int i = 0, j = 0; i < nPose; i++, j += 6)
					poses[i]->applyIncrementalChangeToH(&step[j]);
			}

			void setFrom(const ISRTrackingState &inposes)
			{
				int count = inposes.numPoses();
				for (int i = 0; i < count;i++)
				{
					this->getPose(i)->setFromH(inposes.getPose(i)->getH());
				}
			}

			ISRTrackingState(int num)
			{
				nPose = num;
				poses = new ISRPose_ptr[num];
				for (int i = 0; i < num; i++) poses[i] = new ISRPose();
			}

			~ISRTrackingState()
			{
				for (int i = 0; i < nPose; i++)	delete poses[i];
				free(poses);
			}

			//ISRTrackingState& operator= (const ISRTrackingState& rhs)
			//{	
			//	int numObj = rhs.numPoses();
			//	ISRTrackingState tmpState(numObj);
			//	for (int i = 0; i < numObj;i++)
			//		*tmpState.getPose(i) = *rhs.getPose(i);
			//	return tmpState;
			//}

		};
	}
}