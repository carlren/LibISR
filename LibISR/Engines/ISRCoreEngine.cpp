#include "ISRCoreEngine.h"

using namespace LibISR::Engine;
using namespace LibISR::Objects;

LibISR::Engine::ISRCoreEngine::ISRCoreEngine(const ISRLibSettings *insettings, const ISRCalib *calib, Vector2i d_dize, Vector2i rgb_size)
{
	settings = new ISRLibSettings(*insettings);
	shapeUnion = new ISRShapeUnion(settings->noTrackingObj, settings->useGPU);
	trackingState = new ISRTrackingState(settings->noTrackingObj);


	lowlevelEngine = new ISRLowlevelEngine_CPU();
	tracker = new ISRRGBDTracker_CPU(settings->noTrackingObj);

	frame = new ISRFrame(*calib, d_dize, rgb_size);
	frame->histogram = new ISRHistogram(settings->noHistogramDim);
}

void LibISR::Engine::ISRCoreEngine::ProcessFrame(void)
{
	ISRView *myview = getView();

	if (myview->inputDepthType == ISRView::InputDepthType::ISR_SHORT_DEPTH)
	{
		lowlevelEngine->createAlignedRGBImage(myview->alignedRgb, myview->rawDepth, myview->rgb, &myview->calib->homo_depth_to_color);
	}

	
}