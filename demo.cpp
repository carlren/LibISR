// Copyright 2014-2015 Isis Innovation Limited and the authors of LibISR
#include <stdio.h>

#include"LibISR/LibISR.h"
#include"LibISR/UI/ImageSourceEngine.h"
#include"LibISR/UI/OpenNIEngine.h"
#include"LibISR/UI/UIEngine.h"

#include"LibISR/Utils/IOUtil.h"
#include"LibISR/Utils/NVTimer.h"

using namespace LibISR::Engine;
using namespace LibISR::Objects;
using namespace LibISRUtils;
using namespace std;

int main(int argc, char** argv)
{

	//////////////////////////////////////////////////////////////////////////
	// setup for live demo
	//////////////////////////////////////////////////////////////////////////

    if (argc !=3){
        std::cout<<"Usage: ./demo <path to SDF model> <path to calib file>"<<std::endl;
        return -1;
    }

    const char *sdfFile = argv[1];
	const char *calibFile = argv[2];
    
//	const char *sdfFile = "/home/carl/Work/Code/github/LibISR/Data/teacan.bin";
//	const char *calibFile = "/home/carl/Work/Code/github/LibISR/Data/calib_reg.txt";
    
    
	ImageSourceEngine *imageSource = new OpenNIEngine(calibFile, NULL, true);
    
	ISRLibSettings isrSettings;
	isrSettings.noHistogramDim = HISTOGRAM_BIN;
	isrSettings.noTrackingObj = 1;
	isrSettings.singleAappearanceModel = true;
	isrSettings.useGPU = true;

	ISRCoreEngine *coreEngine = new ISRCoreEngine(&isrSettings, &imageSource->calib, imageSource->getDepthImageSize(), imageSource->getRGBImageSize());
    
    coreEngine->shapeUnion->loadShapeFromFile(sdfFile, Vector3i(DT_VOL_SIZE, DT_VOL_SIZE, DT_VOL_SIZE), 0);
	for (int i = 1; i < isrSettings.noTrackingObj; i++)
		coreEngine->shapeUnion->shareSDFWithExistingShape(*coreEngine->shapeUnion->getShape(0), i);
    
    
    // initial pose for the tracker
	float poses[6] = { 0.0f, 0.0f, 0.8f, -PI/2 , 0, 0 };
	coreEngine->trackingState->setHFromParam(poses, 0);

    
	///////////////////////////////////////////////////////////////////////////
	// run it!
	///////////////////////////////////////////////////////////////////////////
	UIEngine::Instance()->Initialise(argc, argv, imageSource, coreEngine, "./");
	UIEngine::Instance()->Run();
	UIEngine::Instance()->Shutdown();

	return 0;
}
