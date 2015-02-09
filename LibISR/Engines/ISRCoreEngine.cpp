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
	this->trackingState = new ISRTrackingState(settings->noTrackingObj,settings->useGPU);
	this->frame = new ISRFrame(*calib, d_dize, rgb_size, settings->useGPU);
	this->frame->histogram = new ISRHistogram(settings->noHistogramDim, settings->useGPU);

	if (settings->useGPU)
	{
		this->lowLevelEngine = new ISRLowlevelEngine_GPU();
		this->visualizationEngine = new ISRVisualisationEngine_GPU();
		this->tracker = new ISRRGBDTracker_GPU(settings->noTrackingObj,d_dize);
	}
	else
	{
		this->lowLevelEngine = new ISRLowlevelEngine_CPU();
		this->visualizationEngine = new ISRVisualisationEngine_CPU();
		this->tracker = new ISRRGBDTracker_CPU(settings->noTrackingObj);
	}
	
}

void LibISR::Engine::ISRCoreEngine::processFrame(void)
{
	//StopWatchInterface *timer;
	//sdkCreateTimer(&timer);


	ISRView* myview = getView();
	ISRImageHierarchy* myhierarchy = getImageHierarchy();
	

	myhierarchy->levels[0].boundingbox = lowLevelEngine->findBoundingBoxFromCurrentState(trackingState, myview->calib->intrinsics_d.A, myview->depth->noDims);
	myhierarchy->levels[0].intrinsic = myview->calib->intrinsics_d.getParam();

	if (settings->useGPU)
	{
		myview->rawDepth->UpdateDeviceFromHost();
		myview->rgb->UpdateDeviceFromHost();
	}

	//sdkResetTimer(&timer); sdkStartTimer(&timer);
	lowLevelEngine->prepareAlignedRGBDData(myhierarchy->levels[0].rgbd, myview->rawDepth, myview->rgb, &myview->calib->homo_depth_to_color);
	//sdkStopTimer(&timer); printf("Prepare Aligned Image Time : [%f] ms\n", sdkGetTimerValue(&timer));

	//sdkResetTimer(&timer); sdkStartTimer(&timer);
	for (int i = 1; i < myhierarchy->noLevels; i++)
	{
		myhierarchy->levels[i].intrinsic = myhierarchy->levels[i - 1].intrinsic / 2;
		myhierarchy->levels[i].boundingbox = myhierarchy->levels[i - 1].boundingbox / 2;
		lowLevelEngine->subsampleImageRGBDImage(myhierarchy->levels[i].rgbd, myhierarchy->levels[i - 1].rgbd);
	}
	//sdkStopTimer(&timer); printf("Image Hierarchy Image Time : [%f] ms\n", sdkGetTimerValue(&timer));

	int lvlnum; lvlnum = settings->useGPU ? 2 : myhierarchy->noLevels - 1;
	ISRImageHierarchy::ImageLevel& lastLevel = myhierarchy->levels[lvlnum];
	frame->currentLevel = &lastLevel;

	//sdkResetTimer(&timer); sdkStartTimer(&timer);
	lowLevelEngine->preparePointCloudFromAlignedRGBDImage(frame->ptCloud, lastLevel.rgbd, frame->histogram, lastLevel.intrinsic, lastLevel.boundingbox);
	//sdkStopTimer(&timer); printf("Prepare Point Cloud Time : [%f] ms\n", sdkGetTimerValue(&timer));

	//sdkResetTimer(&timer); sdkStartTimer(&timer);
	tracker->TrackObjects(frame, shapeUnion, trackingState);
	//sdkStopTimer(&timer); printf("Track Object Time : [%f] ms\n", sdkGetTimerValue(&timer));

	myview->alignedRgb->SetFrom(myview->rgb);
	lowLevelEngine->computepfImageFromHistogram(myview->rgb, frame->histogram);

	ISRVisualisationState* myrendering = getRenderingState();

	ISRVisualisationState** tmprendering = new ISRVisualisationState*[settings->noTrackingObj];

	for (int i = 0; i < settings->noTrackingObj; i++)
	{
		tmprendering[i] = new ISRVisualisationState(myview->rawDepth->noDims, settings->useGPU);
		tmprendering[i]->outputImage->Clear(0);
		visualizationEngine->updateMinmaxmImage(tmprendering[i]->minmaxImage, trackingState->getPose(i)->getH(), myview->calib->intrinsics_d.A, myview->depth->noDims);
		tmprendering[i]->minmaxImage->UpdateDeviceFromHost();
		visualizationEngine->renderObject(tmprendering[i], trackingState->getPose(i)->getInvH(), shapeUnion->getShape(0), myview->calib->intrinsics_d.getParam());
	}

	myrendering->outputImage->Clear(0);
	
	for (int i = 0; i < myrendering->outputImage->dataSize; i++)
	{
		for (int j = 0; j < settings->noTrackingObj; j++)
		{
			myrendering->outputImage->GetData(false)[i] += tmprendering[j]->outputImage->GetData(false)[i] / settings->noTrackingObj;
		}

		if (myrendering->outputImage->GetData(false)[i] != Vector4u(0, 0, 0, 0))
		{
			myview->alignedRgb->GetData(false)[i] = myrendering->outputImage->GetData(false)[i];
		}
	}

	for (int i = 0; i < settings->noTrackingObj; i++) delete tmprendering[i];
	delete[] tmprendering;

}

