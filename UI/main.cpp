#include <stdio.h>

#include "../LibISR/LibISR.h"

#include "ImageSourceEngine.h"
#include "OpenNIEngine.h"
#include "UIEngine.h"

#include "../LibISRUtils/IOUtil.h"
#include "../LibISRUtils/Timer.h"

using namespace LibISR::Engine;
using namespace LibISR::Objects;
using namespace LibISRUtils;

int main(int argc, char** argv)
{
	//const char *colorImgSource = "../Data/K1_cut/c-%04i.ppm";
	//const char *depthImgSource = "../Data/K1_cut/d-%04i.pgm";
	//const char *calibFile = "../Data/Calib_kinect1.txt";

	//const char *colorImgSource = "E:/Data/k1_cut/c-%04i.ppm";
	//const char *depthImgSource = "E:/Data/k1_cut/d-%04i.pgm";
	//const char *calibFile = "../Data/Calib_kinect1.txt";

	//const char *colorImgSource = "E:/Libisr/k1_cut/cr0-%04i.ppm";
	//const char *depthImgSource = "E:/Libisr/k1_cut/d-%04i.pgm";
	//const char *calibFile = "../Data/calib.txt";

	const char *sdfFile = "../Data/newCut.bin";

	const char* histogram_rgb = "../Data/color.ppm";
	const char* histogram_mask = "../Data/mask.ppm";

	//ImageSourceEngine *imageSource = new ImageFileReader(calibFile, colorImgSource, depthImgSource);
	
	const char *calibFile = "../Data/calib_reg.txt";
	ImageSourceEngine *imageSource = new OpenNIEngine(calibFile,NULL,true);
	

	ISRLibSettings isrSettings;
	isrSettings.noHistogramDim = HISTOGRAM_BIN;
	isrSettings.noTrackingObj = 1;
	isrSettings.singleAappearanceModel = true;
	isrSettings.useGPU = true;

	ISRCoreEngine *coreEngine = new ISRCoreEngine(&isrSettings, &imageSource->calib, imageSource->getDepthImageSize(), imageSource->getRGBImageSize());	
	coreEngine->shapeUnion->loadShapeFromFile(sdfFile, Vector3i(DT_VOL_SIZE, DT_VOL_SIZE, DT_VOL_SIZE), 0);
	for (int i = 1; i < isrSettings.noTrackingObj; i++)
		coreEngine->shapeUnion->shareSDFWithExistingShape(*coreEngine->shapeUnion->getShape(0), i);

	///////////////////////////////////////////////////////////////////////////
	// some manual initialization
	///////////////////////////////////////////////////////////////////////////

	// initialize color histogram from mask image
	ISRUChar4Image *histogramimage = new ISRUChar4Image(imageSource->getDepthImageSize(), false);
	ISRUChar4Image *histogrammask = new ISRUChar4Image(imageSource->getDepthImageSize(), false);
	if (!ReadImageFromFile(histogramimage, histogram_rgb)) { printf("histogram initialization error!\n"); return 0; }
	if (!ReadImageFromFile(histogrammask, histogram_mask)) { printf("histogram initialization error!\n"); return 0; }
	coreEngine->frame->histogram->buildHistogram(histogramimage, histogrammask);

	/// initialized poses are [T' R']'
	//float pose1[6] = { 0.5119f, -0.1408f, 0.7854f, 0.0f, -0.637070260807493f, 0.0f };
	//float pose2[6] = { 0.6687f, 0.5081f, 0.1909f, 0.5469f, 0.9473f, -0.9473f };
	//float poses[12] = { 0.5119f, -0.1408f, 0.7854f, 0.0f, -0.637070260807493f, 0.0f, 0.6687f, 0.5081f, 0.1909f, 0.5469f, 0.9473f, -0.9473f };
	//for (int i = 0; i < isrSettings.noTrackingObj; i++)	coreEngine->trackingState->setInvHFromParam(&poses[6*i], i);

	float poses[6] = { 0.0f, 0.0f, 0.9f, 0.2f, 0.0f, 0.0f };
	coreEngine->trackingState->setHFromParam(poses,0);

	///////////////////////////////////////////////////////////////////////////

	UIEngine::Instance()->Initialise(argc, argv, imageSource, coreEngine, "E:/Data/LibISR_out/");
	UIEngine::Instance()->Run();
	UIEngine::Instance()->Shutdown();

	return 0;
}