#include <stdio.h>

#include "../LibISR/LibISR.h"


#include "ImageSourceEngine.h"
#include "OpenNIEngine.h"
#include "UIEngine.h"

#include "../LibISRUtils/IOUtil.h"
#include "../LibISRUtils/Timer.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

#pragma comment( lib, "opencv_core2410.lib" )
#pragma comment( lib, "opencv_highgui2410.lib" )

using namespace LibISR::Engine;
using namespace LibISRUtils;

int main(int argc, char** argv)
{
	const char *calibFile = "../Data/calib.txt";
	const char *colorImgSource = "../Data/K1_cut/c-%04i.ppm";
	const char *depthImgSource = "../Data/K1_cut/d-%04i.pgm";

	ImageSourceEngine *imageSource = new ImageFileReader(calibFile, colorImgSource, depthImgSource);
	//ImageSourceEngine *imageSource = new OpenNIEngine(calibFile,NULL,true);

	ISRLibSettings isrSettings;
	isrSettings.noHistogramDim = 16;
	isrSettings.noTrackingObj = 2;
	isrSettings.singleAappearanceModel = true;
	isrSettings.useGPU = false;

	ISRCoreEngine *coreEngine = new ISRCoreEngine(&isrSettings, &imageSource->calib,imageSource->getDepthImageSize(),imageSource->getRGBImageSize());


	UIEngine::Instance()->Initialise(argc, argv, imageSource, coreEngine, " ");
	UIEngine::Instance()->Run();
	UIEngine::Instance()->Shutdown();
}