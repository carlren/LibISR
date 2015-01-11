#include "ISRCoreEngine.h"

using namespace CoreISR::Engine;

ISRCoreEngine::ISRCoreEngine(const ISRLibSettings *settings, const ISRCalib *calib, Vector2i d_dize, Vector2i rgb_size)
{
	this->settings = new ISRLibSettings(*settings);
	this->shapeUnion = new ISRShapeUnion(settings->noTrackingObj);
	this->optimizationHelper = new ISROptimizationHelper(settings->noTrackingObj);


	this->frame = new ISRFrame(*calib, d_dize, rgb_size);
	this->frame->histogram = new ISRHistogram(settings->noHistogramDim);
}

void ISRCoreEngine::ProcessFrame(void)
{

}