#include "ISRCoreEngine.h"

using namespace CoreISR::Engine;

ISRCoreEngine::ISRCoreEngine(const ISRLibSettings *settings, const ISRCalib *calib, Vector2i d_dize, Vector2i rgb_size)
{
	this->settings = new ISRLibSettings(*settings);
	this->frame = new ISRFrame(*calib, d_dize, rgb_size);
	
	int histoCount=1;
	if (!settings->singleAappearanceModel) histoCount = settings->noTrackingObj;
	histograms = (ISRHistogram**)malloc(histoCount*sizeof(ISRHistogram*));
	for (int i = 0; i < settings->noTrackingObj; i++)
		this->histograms[i] = new ISRHistogram(settings->noHistogramDim);
}

void ISRCoreEngine::ProcessFrame(void)
{

}