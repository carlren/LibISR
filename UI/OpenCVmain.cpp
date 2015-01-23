
#include "../LibISR/LibISR.h"

#include "ImageSourceEngine.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

#pragma comment( lib, "opencv_core2410.lib" )
#pragma comment( lib, "opencv_highgui2410.lib" )

#include "../LibISRUtils/IOUtil.h"
#include "../LibISRUtils/Timer.h"

using namespace LibISR::Engine;
using namespace LibISR::Objects;
using namespace LibISRUtils;

void main(int argc, char** argv)
{
	const char *colorImgSource = "../Data/K1_cut/c-%04i.ppm";
	const char *depthImgSource = "../Data/K1_cut/d-%04i.pgm";

	//const char *colorImgSource = "E:/Data/k1_cut/c-%04i.ppm";
	//const char *depthImgSource = "E:/Data/k1_cut/d-%04i.pgm";

	const char *calibFile = "../Data/calib.txt";
	const char *histogramFile = "../Data/histogram.txt";

	const char *sdfFile = "../Data/newCut.bin";

	ImageSourceEngine *imageSource = new ImageFileReader(calibFile, colorImgSource, depthImgSource);
	//ImageSourceEngine *imageSource = new OpenNIEngine(calibFile,NULL,true);

	ISRLibSettings isrSettings;
	isrSettings.noHistogramDim = 16;
	isrSettings.noTrackingObj = 2;
	isrSettings.singleAappearanceModel = true;
	isrSettings.useGPU = false;

	ISRCoreEngine *coreEngine = new ISRCoreEngine(&isrSettings, &imageSource->calib, imageSource->getDepthImageSize(), imageSource->getRGBImageSize());

	coreEngine->shapeUnion->getShape(0)->loadShapeFromFile(sdfFile, Vector3i(DT_VOL_SIZE, DT_VOL_SIZE, DT_VOL_SIZE));
	coreEngine->shapeUnion->getShape(1)->shareSDFWithExistingShape(*coreEngine->shapeUnion->getShape(0));

	coreEngine->frame->histogram->loadPosteriorFromFile(histogramFile);

	float pose1[6] = { 0.5119f, -0.1408f, 0.7854f, 0.0f, -0.637070260807493f, 0.0f };
	float pose2[6] = { 0.6687f, 0.5081f, 0.1909f, 0.5469f, 0.9473f, -0.9473f };
	coreEngine->getTrackingState()->getPose(0)->setInvHFromParam(pose1);
	coreEngine->getTrackingState()->getPose(1)->setInvHFromParam(pose2);

	cvNamedWindow("Depth", 0);
	IplImage* depthFrame = cvCreateImage(cvSize(640, 480), 8, 4);

	//depthFrame->imageData = (char*)coreEngine->getView()->alignedRgb->GetData(false);

	char inputColorName[100];
	char inputDepthName[200];

	Vector3f cpt[6], ipt[6];
	CvPoint cvpt[6];

	for (int i = -1, c = 0; i <= 1; i += 2, c++)
	{
		cpt[0 + c] = Vector3f(i / 10.0f, 0, 0);
		cpt[2 + c] = Vector3f(0, i / 10.0f, 0);
		cpt[4 + c] = Vector3f(0, 0, i / 10.0f);
	}
		

	CvScalar color = CV_RGB(255, 0, 255);
	CvScalar color2 = CV_RGB(0, 255, 255);
	int key;
	int count = 0;

	Matrix3f A = coreEngine->getView()->calib->intrinsics_d.A;

	imageSource->getImages(coreEngine->getView());

	while ((key = cvWaitKey(10)) != 27 && count < 250)
	{
		if (!imageSource->hasMoreImages()) return;
		
		coreEngine->ProcessFrame();
		memcpy(depthFrame->imageData, (char*)coreEngine->getView()->alignedRgb->GetData(false), 640 * 480 * sizeof(char) * 4);

		Matrix4f M = coreEngine->trackingState->getPose(0)->getH();
		
		for (int i = 0; i < 6; i++)
		{
			ipt[i] = A*(M*cpt[i]);
			cvpt[i].x = ipt[i].x / ipt[i].z;
			cvpt[i].y = ipt[i].y / ipt[i].z;
		}
		
		for (int i = 0; i < 3; i++) cvDrawLine(depthFrame, cvpt[i*2], cvpt[i*2+1], color, 2);
			
		cvShowImage("Depth", depthFrame);
		count++;
	}
	cvDestroyAllWindows();
}