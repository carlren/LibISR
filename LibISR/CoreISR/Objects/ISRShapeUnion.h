#pragma once

#include <stdio.h>

#include "..//Utils//LibISRDefine.h"
#include "..//Utils//MathUtils.h"
#include "..//Objects//ISRShape.h"
#include "..//Objects//ISRFrame.h"

using namespace CoreISR::Objects;

namespace CoreISR
{
	namespace Objects
	{
		class ISRShapeUnion
		{
		private:

			void GetBoundingBox(Vector4f *BB, ISRPose* pose, ISRIntrinsics* intrinsic)
			{
				BB->w = 639.0f; BB->x = 1.0f; BB->y = 479.0f; BB->z = 1.0f;

				float halfSize = 0.1f;

				float cornerPts[24] = { halfSize, halfSize, halfSize, 
										halfSize, halfSize, -halfSize, 
										halfSize, -halfSize, halfSize,
										halfSize, -halfSize, -halfSize, 
										-halfSize, halfSize, halfSize, 
										-halfSize, halfSize, -halfSize,
										-halfSize, -halfSize, halfSize, 
										-halfSize, -halfSize, -halfSize };

				float imgPts[24];

				float* R = pose->R->m;
				float* T = pose->t->v;

				float *invH = pose->invH->m;
				float* A = intrinsic->A;

				for (int i = 0; i<8; i++)
				{
					int offsetIdx = i * 3;

					imgPts[offsetIdx] = invH[0] * cornerPts[offsetIdx] + invH[1] * cornerPts[offsetIdx + 1] + invH[2] * cornerPts[offsetIdx + 2] + invH[3];
					imgPts[offsetIdx+1] = invH[4] * cornerPts[offsetIdx] + invH[5] * cornerPts[offsetIdx + 1] + invH[6] * cornerPts[offsetIdx + 2] + invH[7];
					imgPts[offsetIdx+2] = invH[8] * cornerPts[offsetIdx] + invH[9] * cornerPts[offsetIdx + 1] + invH[10] * cornerPts[offsetIdx + 2] + invH[11];
					
					if (imgPts[offsetIdx + 2] != 0)
					{
						imgPts[offsetIdx] = A[0] * (imgPts[offsetIdx] / imgPts[offsetIdx + 2]) + A[2];
						imgPts[offsetIdx + 1] = A[4] * (imgPts[offsetIdx + 1] / imgPts[offsetIdx + 2]) + A[5];
					}

					BB->w = imgPts[offsetIdx] < BB->w ? imgPts[offsetIdx] : BB->w;
					BB->x = imgPts[offsetIdx] > BB->x ? imgPts[offsetIdx] : BB->x;
					BB->y = imgPts[offsetIdx + 1] < BB->y ? imgPts[offsetIdx + 1] : BB->y;
					BB->z = imgPts[offsetIdx + 1] > BB->z ? imgPts[offsetIdx + 1] : BB->z;

				}

				BB->w = BB->w < 0 ? 0 : BB->w;
				BB->x = BB->x > 639 ? 639 : BB->x;
				BB->y = BB->y < 0 ? 0 : BB->y;
				BB->z = BB->z > 479 ? 479 : BB->z;

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
				for (int i = 0; i < nShapesCount; i++)
				{
					GetBoundingBox(&BB, shapes[i]->pose, frame->intrinsics);

					for (int j = BB.y; j < BB.z; j++)
						for (int k = BB.w; k < BB.x; k++)
						{
							int idx = j*frame->width + k;

							tmpSkipCount += (1 - frame->occMap->data[idx]);
							frame->occMap->data[idx] = 1;
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

