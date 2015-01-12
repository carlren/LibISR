#pragma once

#include <stdio.h>

#include "../Utils/LibISRDefine.h"
#include "../Objects/ISRShape.h"
#include "../Objects/ISRFrame.h"

using namespace CoreISR::Objects;

namespace LibISR
{
	namespace Objects
	{
		class ISRShapeUnion
		{
		private:

			Vector4f GetBoundingBox(const ISRPose &pose, const ISRIntrinsics &intrinsic)
			{
				Vector4f BB;
				BB.w = 639.0f; BB.x = 1.0f; BB.y = 479.0f; BB.z = 1.0f;

				Vector3f cornerPts[8], imgPts[8];
				float halfSize = 0.1f;
				int idx = 0;
				for (int i = -1; i <= 1; i += 2) for (int j = -1; j <= 1; j += 2) for (int k = -1; k <= 1; k += 2)
				{
					cornerPts[idx].x = halfSize * i;
					cornerPts[idx].y = halfSize * j;
					cornerPts[idx].z = halfSize * k;
				}

				Matrix4f invH = pose.invH;
				Matrix3f A = intrinsic.A;

				for (int i = 0; i<8; i++)
				{
					imgPts[i] = A*(invH*cornerPts[8]);

					if (imgPts[i].z != 0)
					{
						imgPts[i].x /= imgPts[i].z;
						imgPts[i].y /= imgPts[i].z;
					}

					BB.w = imgPts[i].x < BB.w ? imgPts[i].x : BB.w;
					BB.x = imgPts[i].x > BB.x ? imgPts[i].x : BB.x;
					BB.y = imgPts[i].y < BB.y ? imgPts[i].y : BB.y;
					BB.z = imgPts[i].z > BB.z ? imgPts[i].z : BB.z;

				}

				BB.w = BB.w < 0 ? 0 : BB.w;
				BB.x = BB.x > 639 ? 639 : BB.x;
				BB.y = BB.y < 0 ? 0 : BB.y;
				BB.z = BB.z > 479 ? 479 : BB.z;
			}

		public:

			ISRShape **shapes;
			int nShapesCount;

			ISRPoints *sharedCamPoints;
			float *pfList;
			int *colorIdxList;
			int *surfHashList;

			int tmpSkipCount;


			int nPtCount;

			ISRShapeUnion(int count)
			{
				this->nShapesCount = count;
				this->shapes = (ISRShape**)malloc(count*sizeof(ISRShape*));
				this->sharedCamPoints = new ISRPoints(MAX_IMG_PTS);
				this->pfList = (float*)malloc(MAX_IMG_PTS * sizeof(float));
				this->colorIdxList = (int*)malloc(MAX_IMG_PTS * sizeof(int));
				this->surfHashList = (int*)malloc(MAX_IMG_PTS * sizeof(int));
				this->nPtCount = 0;
			}

			void UpdateOccMapOnFrame(ISRFrame *frame)
			{
				Vector4f BB;

				tmpSkipCount = 0;

				frame->occMap->Clear();
				uchar* occMap = frame->occMap->GetData(false);

				for (int i = 0; i < nShapesCount; i++)
				{
					Vector4f BB = GetBoundingBox(shapes[i]->pose, frame->view->calib->intrinsics_d);

					for (int j = BB.y; j < BB.z; j++)
						for (int k = BB.w; k < BB.x; k++)
						{
							int idx = j*frame->width + k;

							tmpSkipCount += (1 - occMap[idx]);
							occMap[idx] = 1;
						}
				}

				frame->samplingRate = (int)sqrtf(tmpSkipCount / 6000);
			}

			~ISRShapeUnion()
			{
				for (int i = 0; i < this->nShapesCount; i++)
					delete this->shapes[i];
				delete this->sharedCamPoints;
				free(this->pfList);
				free(this->colorIdxList);
				free(this->surfHashList);
			}

		};
	}
}

