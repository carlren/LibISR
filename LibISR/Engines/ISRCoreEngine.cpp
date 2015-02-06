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

	if (settings->useGPU)
	{
		this->lowLevelEngine = new ISRLowlevelEngine_GPU();
		this->visualizationEngine = new ISRVisualisationEngine_GPU();
	}
	else
	{
		this->lowLevelEngine = new ISRLowlevelEngine_CPU();
		this->visualizationEngine = new ISRVisualisationEngine_CPU();
	}
	

	this->tracker = new ISRRGBDTracker_CPU(settings->noTrackingObj);
	
	this->frame = new ISRFrame(*calib, d_dize, rgb_size, settings->useGPU);
	this->frame->histogram = new ISRHistogram(settings->noHistogramDim, settings->useGPU);
}

void LibISR::Engine::ISRCoreEngine::processFrame(void)
{
	ISRView* myview = getView();
	ISRImageHierarchy* myhierarchy = getImageHierarchy();

	myhierarchy->levels[0].boundingbox = lowLevelEngine->findBoundingBoxFromCurrentState(trackingState, myview->calib->intrinsics_d.A, myview->depth->noDims);
	myhierarchy->levels[0].intrinsic = myview->calib->intrinsics_d.getParam();

	if (settings->useGPU)
	{
		myview->rawDepth->UpdateDeviceFromHost();
		myview->rgb->UpdateDeviceFromHost();
	}

	lowLevelEngine->prepareAlignedRGBDData(myhierarchy->levels[0].rgbd, myview->rawDepth, myview->rgb, &myview->calib->homo_depth_to_color);

	for (int i = 1; i < myhierarchy->noLevels; i++)
	{
		myhierarchy->levels[i].intrinsic = myhierarchy->levels[i - 1].intrinsic / 2;
		myhierarchy->levels[i].boundingbox = myhierarchy->levels[i - 1].boundingbox / 2;
		lowLevelEngine->subsampleImageRGBDImage(myhierarchy->levels[i].rgbd, myhierarchy->levels[i - 1].rgbd);
	}

	ISRImageHierarchy::ImageLevel& lastLevel = myhierarchy->levels[myhierarchy->noLevels - 1];
	frame->currentLevel = &lastLevel;

	lowLevelEngine->preparePointCloudFromAlignedRGBDImage(frame->ptCloud, lastLevel.rgbd, frame->histogram, lastLevel.intrinsic, lastLevel.boundingbox);

	// this is temp
	if (settings->useGPU) frame->ptCloud->UpdateHostFromDevice();

	tracker->TrackObjects(frame, shapeUnion, trackingState);

	ISRVisualisationState* myrendering = getRenderingState();
	ISRVisualisationState* rendering0 = new ISRVisualisationState(myview->rawDepth->noDims, true);
	ISRVisualisationState* rendering1 = new ISRVisualisationState(myview->rawDepth->noDims, true);


	visualizationEngine->updateMinmaxmImage(rendering1->minmaxImage, trackingState->getPose(0)->getH(), myview->calib->intrinsics_d.A, myview->depth->noDims);
	rendering1->minmaxImage->UpdateDeviceFromHost();
	visualizationEngine->renderObject(rendering1, trackingState->getPose(0)->getInvH(), shapeUnion->getShape(0), myview->calib->intrinsics_d.getParam());

	visualizationEngine->updateMinmaxmImage(rendering0->minmaxImage, trackingState->getPose(1)->getH(), myview->calib->intrinsics_d.A, myview->depth->noDims);
	rendering0->minmaxImage->UpdateDeviceFromHost();
	visualizationEngine->renderObject(rendering0, trackingState->getPose(1)->getInvH(), shapeUnion->getShape(1), myview->calib->intrinsics_d.getParam());


	for (int i = 0; i < myrendering->outputImage->dataSize;i++)
	{
		if (rendering0->outputImage->GetData(false)[i]!=Vector4u(0,0,0,0))
		{
			if (rendering1->outputImage->GetData(false)[i] != Vector4u(0, 0, 0, 0))
			{
				myrendering->outputImage->GetData(false)[i] = (rendering0->outputImage->GetData(false)[i] / 2 + rendering1->outputImage->GetData(false)[i] / 2);
			}
			else myrendering->outputImage->GetData(false)[i] = rendering0->outputImage->GetData(false)[i];
		}
		else myrendering->outputImage->GetData(false)[i] = rendering1->outputImage->GetData(false)[i];
	}
}