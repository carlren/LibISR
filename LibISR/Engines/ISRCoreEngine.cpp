#include "ISRCoreEngine.h"
#include "../../LibISRUtils/IOUtil.h"
#include "../../LibISRUtils/Timer.h"
#include "../../LibISRUtils/NVTimer.h"

using namespace LibISR::Engine;
using namespace LibISR::Objects;

LibISR::Engine::ISRCoreEngine::ISRCoreEngine(const ISRLibSettings *settings, const ISRCalib *calib, Vector2i d_dize, Vector2i rgb_size)
{
	this->settings = new ISRLibSettings(*settings);
	this->shapeUnion = new ISRShapeUnion(settings->noTrackingObj, settings->useGPU);
	this->trackingState = new ISRTrackingState(settings->noTrackingObj);

	this->lowLevelEngine = new ISRLowlevelEngine_CPU();
	this->tracker = new ISRRGBDTracker_CPU(settings->noTrackingObj);

	this->frame = new ISRFrame(*calib, d_dize, rgb_size);
	this->frame->histogram = new ISRHistogram(settings->noHistogramDim);
}

void LibISR::Engine::ISRCoreEngine::processFrame(void)
{
	ISRView* myview = getView();
	ISRImageHierarchy* myhierarchy = getImageHierarchy();
	

	//StopWatchInterface* timer;
	//sdkCreateTimer(&timer);
	//float timeused;

	myhierarchy->levels[0].boundingbox = lowLevelEngine->findBoundingBoxFromCurrentState(trackingState, myview->calib->intrinsics_d.A, myview->depth->noDims);
	myhierarchy->levels[0].intrinsic = myview->calib->intrinsics_d.getParam();

	//sdkResetTimer(&timer); sdkStartTimer(&timer);
	lowLevelEngine->prepareAlignedRGBDData(myhierarchy->levels[0].rgbd, myview->rawDepth, myview->rgb, &myview->calib->homo_depth_to_color);
	//timeused = sdkGetTimerValue(&timer);
	//printf("Align Data:%f\n", timeused);

	//sdkResetTimer(&timer); sdkStartTimer(&timer);
	for (int i = 1; i < myhierarchy->noLevels;i++)
	{
		myhierarchy->levels[i].intrinsic = myhierarchy->levels[i - 1].intrinsic / 2;
		myhierarchy->levels[i].boundingbox = myhierarchy->levels[i - 1].boundingbox / 2;
		lowLevelEngine->subsampleImageRGBDImage(myhierarchy->levels[i].rgbd, myhierarchy->levels[i - 1].rgbd);
	}
	//timeused = sdkGetTimerValue(&timer);
	//printf("Build Hierarchy:%f\n", timeused);
	

	ISRImageHierarchy::ImageLevel& lastLevel = myhierarchy->levels[myhierarchy->noLevels - 1];
	//ISRImageHierarchy::ImageLevel& lastLevel = myhierarchy->levels[2];
	frame->currentLevel = &lastLevel;

	//sdkResetTimer(&timer); sdkStartTimer(&timer);
	lowLevelEngine->preparePointCloudFromAlignedRGBDImage(frame->ptCloud, lastLevel.rgbd, frame->histogram, lastLevel.intrinsic, lastLevel.boundingbox);
	//timeused = sdkGetTimerValue(&timer);
	//printf("Prepare Point Cloud:%f\n", timeused);

	//sdkResetTimer(&timer); sdkStartTimer(&timer); 
	tracker->TrackObjects(frame, shapeUnion, trackingState);
	//timeused = sdkGetTimerValue(&timer);
	//printf("Track Object:%f\n\n\n\n\n", timeused);

}