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
using namespace LibISR::Objects;
using namespace LibISRUtils;

int main(int argc, char** argv)
{
	const char *calibFile = "../Data/calib.txt";
	const char *histogramFile = "../Data/histogram.txt";

	//const char *colorImgSource = "../Data/K1_cut/c-%04i.ppm";
	//const char *depthImgSource = "../Data/K1_cut/d-%04i.pgm";

	const char *colorImgSource = "E:/Data/k1_cut/c-%04i.ppm";
	const char *depthImgSource = "E:/Data/k1_cut/d-%04i.pgm";

	ImageSourceEngine *imageSource = new ImageFileReader(calibFile, colorImgSource, depthImgSource);
	//ImageSourceEngine *imageSource = new OpenNIEngine(calibFile,NULL,true);

	ISRLibSettings isrSettings;
	isrSettings.noHistogramDim = 16;
	isrSettings.noTrackingObj = 2;
	isrSettings.singleAappearanceModel = true;
	isrSettings.useGPU = false;

	ISRCoreEngine *coreEngine = new ISRCoreEngine(&isrSettings, &imageSource->calib,imageSource->getDepthImageSize(),imageSource->getRGBImageSize());
	
	///////////////////////////////////////////////////////////////////////////
	// some manual initialization
	///////////////////////////////////////////////////////////////////////////

	coreEngine->frame->histogram->loadPosteriorFromFile(histogramFile);

	float pose1[6] = { 0.5119f, -0.1408f, 0.7854f, 0.0f, -0.637070260807493f, 0.0f };
	float pose2[6] = { 0.6687f, 0.5081f, 0.1909f, 0.5469f, 0.9473f, -0.9473f };
	coreEngine->getTrackingState()->getPose(0)->setFromParam(pose1);
	coreEngine->getTrackingState()->getPose(1)->setFromParam(pose2);
	
	///////////////////////////////////////////////////////////////////////////

	UIEngine::Instance()->Initialise(argc, argv, imageSource, coreEngine, " ");
	UIEngine::Instance()->Run();
	UIEngine::Instance()->Shutdown();
}