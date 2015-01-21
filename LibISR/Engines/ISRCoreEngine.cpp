#include "ISRCoreEngine.h"
#include "../../LibISRUtils/IOUtil.h"

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

void LibISR::Engine::ISRCoreEngine::ProcessFrame(void)
{
	ISRView* myview = getView();

	lowLevelEngine->createAlignedRGBImage(myview->alignedRgb, myview->rawDepth, myview->rgb, &myview->calib->homo_depth_to_color);
	lowLevelEngine->preparePointCloudForRGBDTrackerAllInOne(frame->ptCloud, myview->rawDepth, myview->rgb, myview->calib, frame->histogram);

	//PrintPointListToFile("E:/LibISR/debug/ptcloud_debug.txt",frame->ptCloud->GetData(false), frame->ptCloud->dataSize);
	//PrintArrayToFile("E:/LibISR/histogram_debug.txt", frame->histogram->posterior, frame->histogram->dim);

	tracker->TrackObjects(frame, shapeUnion, trackingState);
	
}