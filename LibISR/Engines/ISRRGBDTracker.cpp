#include "ISRRGBDTracker.h"

#include "../../ORUtils/Cholesky.h"
#include <math.h>
#include <stdio.h>

using namespace LibISR::Engine;
using namespace LibISR::Objects;

ISRRGBDTracker::ISRRGBDTracker(int nObjs, bool useGPU)
{
	nObjects = nObjs;
	ATb_Size = nObjs * 6;
	ATA_size = ATb_Size*ATb_Size;

	ATb_host = (float*)malloc(ATb_Size*sizeof(float));
	ATA_host = (float*)malloc(ATA_size*sizeof(float));

	acceptedPoses = (ISRPose**)malloc(nObjects*sizeof(ISRPose**));
	tmpPoses = (ISRPose**)malloc(nObjects*sizeof(ISRPose**));
}

ISRRGBDTracker::~ISRRGBDTracker()
{
	free(ATb_host);
	free(ATA_host);

	free(acceptedPoses);
	free(tmpPoses);
}



static inline bool minimizeLM(const ISRRGBDTracker *tracker, ISRPose** initialization);


static inline void GetRotationMatrixFromMRP(float *outR, const float* r)
{
	float t1 = r[0], t2 = r[1], t3 = r[2];

	float tsq = t1*t1 + t2*t2 + t3*t3;

	float tsum = 1 - tsq;

	outR[0] = 4 * t1*t1 - 4 * t2*t2 - 4 * t3*t3 + tsum*tsum;	outR[1] = 8 * t1*t2 - 4 * t3*tsum;	outR[2] = 8 * t1*t3 + 4 * t2*tsum;
	outR[3] = 8 * t1*t2 + 4 * t3*tsum;	outR[4] = 4 * t2*t2 - 4 * t1*t1 - 4 * t3*t3 + tsum*tsum;	outR[5] = 8 * t2*t3 - 4 * t1*tsum;
	outR[6] = 8 * t1*t3 - 4 * t2*tsum;	outR[7] = 8 * t2*t3 + 4 * t1*tsum;	outR[8] = 4 * t3*t3 - 4 * t2*t2 - 4 * t1*t1 + tsum*tsum;

	for (int i = 0; i<9; i++) outR[i] /= ((1 + tsq)*(1 + tsq));
}
