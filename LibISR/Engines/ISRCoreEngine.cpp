#include "ISRCoreEngine.h"

using namespace LibISR::Engine;
using namespace LibISR::Objects;

LibISR::Engine::ISRCoreEngine::ISRCoreEngine(const ISRLibSettings *settings, const ISRCalib *calib, Vector2i d_dize, Vector2i rgb_size)
{
	this->settings = new ISRLibSettings(*settings);
	this->shapeUnion = new ISRShapeUnion(settings->noTrackingObj, settings->useGPU);


	this->frame = new ISRFrame(*calib, d_dize, rgb_size);
	this->frame->histogram = new ISRHistogram(settings->noHistogramDim);
}

void LibISR::Engine::ISRCoreEngine::ProcessFrame(void)
{

}