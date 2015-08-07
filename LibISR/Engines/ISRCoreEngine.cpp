#include "ISRCoreEngine.h"
#include "../Utils/IOUtil.h"
#include "../Utils/NVTimer.h"

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

		this->tmpTracker = new Engine::ISRColorTracker_CPU(settings->noTrackingObj, rgb_size);
	}
	
	maxposediff = 0;
	needStarTracker = false;
}

void LibISR::Engine::ISRCoreEngine::processFrame(void)
{
	StopWatchInterface *timer;
	sdkCreateTimer(&timer);

	ISRView* myview = getView();
	ISRImageHierarchy* myhierarchy = getImageHierarchy();
	
	myhierarchy->levels[0].boundingbox = lowLevelEngine->findBoundingBoxFromCurrentState(trackingState, myview->calib->intrinsics_d.A, myview->depth->noDims);
	myhierarchy->levels[0].intrinsic = myview->calib->intrinsics_d.getParam();


	if (settings->useGPU)
	{
		myview->rawDepth->UpdateDeviceFromHost();
		myview->rgb->UpdateDeviceFromHost();
	}

	// align colour image with depth image if need to
	sdkResetTimer(&timer); sdkStartTimer(&timer);
	lowLevelEngine->prepareAlignedRGBDData(myhierarchy->levels[0].rgbd, myview->rawDepth, myview->rgb, &myview->calib->homo_depth_to_color);
	sdkStopTimer(&timer); printf("\rAlign:[%.2f] ", sdkGetTimerValue(&timer));

	// build image hierarchy
	sdkResetTimer(&timer); sdkStartTimer(&timer);
	for (int i = 1; i < myhierarchy->noLevels; i++)
	{
		myhierarchy->levels[i].intrinsic = myhierarchy->levels[i - 1].intrinsic / 2;
		myhierarchy->levels[i].boundingbox = myhierarchy->levels[i - 1].boundingbox / 2;
		lowLevelEngine->subsampleImageRGBDImage(myhierarchy->levels[i].rgbd, myhierarchy->levels[i - 1].rgbd);
	}
	sdkStopTimer(&timer); printf("Hier:[%.2f] ", sdkGetTimerValue(&timer));

	trackingState->boundingBox = myhierarchy->levels[0].boundingbox;

	int lvlnum; lvlnum = settings->useGPU ? 1 : myhierarchy->noLevels - 1;
	ISRImageHierarchy::ImageLevel& lastLevel = myhierarchy->levels[lvlnum];
	frame->currentLevel = &lastLevel;


	sdkResetTimer(&timer); sdkStartTimer(&timer);
	lowLevelEngine->preparePointCloudFromAlignedRGBDImage(frame->ptCloud, lastLevel.rgbd, frame->histogram, lastLevel.intrinsic, lastLevel.boundingbox);
	sdkStopTimer(&timer); printf("Prep:[%.2f] ", sdkGetTimerValue(&timer));


	if (needStarTracker)
	{
//		Matrix4f H = trackingState->getPose(0)->getInvH();
//		tmpTracker->TrackObjects(frame, shapeUnion, trackingState);
//		Matrix4f newH = trackingState->getPose(0)->getInvH();
//		trackingState->getPose(0)->setFromInvH(H);
		tracker->TrackObjects(frame, shapeUnion, trackingState);
		
//		if (trackingState->energy<0.3)
//		{
//			trackingState->getPose(0)->setFromInvH(newH);
//		}

	}


	if (settings->useGPU) myview->alignedRgb->SetFrom(myview->rgb, ORUtils::MemoryBlock<Vector4u>::CUDA_TO_CPU);
	else myview->alignedRgb->SetFrom(myview->rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);

	lowLevelEngine->computepfImageFromHistogram(myview->rgb, frame->histogram);

	ISRVisualisationState* myrendering = getRenderingState();
	ISRVisualisationState** tmprendering = new ISRVisualisationState*[settings->noTrackingObj];

	// raycast
	sdkResetTimer(&timer); sdkStartTimer(&timer);
	for (int i = 0; i < settings->noTrackingObj; i++)
	{
		tmprendering[i] = new ISRVisualisationState(myview->rawDepth->noDims, settings->useGPU);
		tmprendering[i]->outputImage->Clear(0);
		visualizationEngine->updateMinmaxmImage(tmprendering[i]->minmaxImage, trackingState->getPose(i)->getH(), myview->calib->intrinsics_d.A, myview->depth->noDims);
		tmprendering[i]->minmaxImage->UpdateDeviceFromHost();
		visualizationEngine->renderObject(tmprendering[i], trackingState->getPose(i)->getInvH(), shapeUnion->getShape(0), myview->calib->intrinsics_d.getParam());
		//visualizationEngine->renderAsSDF(tmpSDFImage, tmpPtCloud, tmprendering[i], trackingState->getPose(i)->getInvH(), shapeUnion->getShape(0), myview->calib->intrinsics_d.getParam());
	}
	sdkStopTimer(&timer); printf("Render:[%.2f] ", sdkGetTimerValue(&timer));

	//tmpSDFImage->UpdateHostFromDevice();
	//tmpPtCloud->UpdateHostFromDevice();
	//PrintPointListToFile("e:/libisr/ptcloud.txt", tmpPtCloud->GetData(MEMORYDEVICE_CPU), tmpPtCloud->dataSize);
	//WriteMatlabTXTImg("e:/libisr/sdf.txt", tmpSDFImage->GetData(MEMORYDEVICE_CPU), 640, 480);


	sdkResetTimer(&timer); sdkStartTimer(&timer);
	myrendering->outputImage->Clear(0);
	for (int i = 0; i < myrendering->outputImage->dataSize; i++)
	{
		for (int j = 0; j < settings->noTrackingObj; j++)
		{
			myrendering->outputImage->GetData(MEMORYDEVICE_CPU)[i] += tmprendering[j]->outputImage->GetData(MEMORYDEVICE_CPU)[i] / settings->noTrackingObj;
		}

		if (myrendering->outputImage->GetData(MEMORYDEVICE_CPU)[i] != Vector4u(0, 0, 0, 0))
		{
			/*myview->alignedRgb->GetData(MEMORYDEVICE_CPU)[i] = myrendering->outputImage->GetData(MEMORYDEVICE_CPU)[i] / 3 * 2 + myview->alignedRgb->GetData(MEMORYDEVICE_CPU)[i] / 3;*/
			myview->alignedRgb->GetData(MEMORYDEVICE_CPU)[i] = myrendering->outputImage->GetData(MEMORYDEVICE_CPU)[i];

		}
	}

	for (int i = 0; i < settings->noTrackingObj; i++) delete tmprendering[i];
	delete[] tmprendering;
	sdkStopTimer(&timer); printf("Rest:[%.2f]", sdkGetTimerValue(&timer));
}

