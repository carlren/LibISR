#include <stdio.h>

#include "CoreISR/CoreISR.h"

#include "Engine/ImageSourceEngine.h"
#include "Engine/OpenNIEngine.h"
#include "Engine/UIEngine.h"


#include "Utils/IOUtil.h"
#include "Utils/Timer.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

#pragma comment( lib, "opencv_core2410.lib" )
#pragma comment( lib, "opencv_highgui2410.lib" )

using namespace LibISR::Engine;

int main(int argc, char** argv)
{
	const char *calibFile = "../Data/calib.txt";
	const char *colorImgSource = "../Data/K1_cut/c-%04i.ppm";
	const char *depthImgSource = "../Data/K1_cut/d-%04i.pgm";

	//ImageSourceEngine *imageSource = new ImageFileReader(calibFile, colorImgSource, depthImgSource);
	ImageSourceEngine *imageSource = new OpenNIEngine(calibFile,NULL,true);

	ISRLibSettings isrSettings;
	isrSettings.noHistogramDim = 16;
	isrSettings.noTrackingObj = 2;
	isrSettings.singleAappearanceModel = true;
	isrSettings.useGPU = false;

	ISRCoreEngine *coreEngine = new ISRCoreEngine(&isrSettings, &imageSource->calib,imageSource->getDepthImageSize(),imageSource->getRGBImageSize());

	UIEngine::Instance()->Initialise(argc, argv, imageSource, coreEngine, " ");
	UIEngine::Instance()->Run();
	UIEngine::Instance()->Shutdown();

	//cvNamedWindow("Color", 0);
	//cvNamedWindow("Depth", 0);

	//IplImage* colorFrame = cvCreateImage(cvSize(640, 480), 8, 4);
	//IplImage* depthFrame = cvCreateImage(cvSize(640, 480), 16, 1);

	//Timer myTimer;

	//myTimer.start();

	//int key;
	//while ((key = cvWaitKey(1)) != 27)
	//{
	//	myTimer.restart();
	//	imageSource->getImages(coreEngine->GetView());
	//	myTimer.check();

	//	memcpy(depthFrame->imageData, coreEngine->GetView()->rawDepth->GetData(false), 640 * 480 * sizeof(short));
	//	memcpy(colorFrame->imageData, coreEngine->GetView()->rgb->GetData(false), 640 * 480 * sizeof(Vector4u));

	//	cvShowImage("Color", colorFrame);
	//	cvShowImage("Depth", depthFrame);

	//}

	//cvDestroyAllWindows();
	//cvReleaseImage(&colorFrame);
	//cvReleaseImage(&depthFrame);

	//ISRFrame *myFrame = new ISRFrame(640, 480);
	//Vector3d myVolSize; myVolSize.x = 200; myVolSize.y = 200; myVolSize.z = 200;

	//int objCount = 2;

	//ISRShapeUnion *myShapeUnion = new ISRShapeUnion(objCount);
	//ISROptimizationHelper *myHelper = new ISROptimizationHelper(objCount);

	//for (int i = 0; i < objCount; i++)
	//{
	//	myShapeUnion->shapes[i] = new ISRShape(myVolSize, 2, 0);
	//	myShapeUnion->shapes[i]->LoadShapeFromFile("../Data/newCut.bin");
	//}

	//float step1[6] = { 0.5119f, -0.1408f, 0.7854f, 0.0f, - 0.637070260807493f, 0.0f };
	//myShapeUnion->shapes[0]->pose->SetFromStep(step1);

	//float step2[6] = { 0.6687f,    0.5081f,    0.1909f, 0.5469f, 0.9473f, -0.9473f };
	//myShapeUnion->shapes[1]->pose->SetFromStep(step2);
	//
	//cvNamedWindow("Color", 0);
	//cvNamedWindow("Depth", 0);

	//IplImage* colorFrame = cvCreateImage(cvSize(640, 480), 8, 4);
	//IplImage* depthFrame = cvCreateImage(cvSize(640, 480), 8, 4);

	//colorFrame->imageData = (char*)myFrame->alignedColorImage->data;
	//depthFrame->imageData = (char*)myFrame->displayDepthImage->data;

	//char inputColorName[100];
	//char inputDepthName[200];

	//int key;
	//int count = 0;

	//ISRUChar4Image *tColor = new ISRUChar4Image(640, 480);
	//ISRUShortImage *tDepth = new ISRUShortImage(640, 480);

	//ISRUChar4Image *histColor = new ISRUChar4Image(640, 480);
	//ISRUCharImage *histMask = new ISRUCharImage(640, 480);


	//loadISRU4Image("..\\Data\\color.jpg",histColor);
	//IplImage *tmpMask = cvLoadImage("..\\Data\\mask.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	//memcpy(histMask->data, tmpMask->imageData, 640 * 480 * sizeof(unsigned char));
	//myFrame->histogram->BuildHistogram(histColor, histMask);

	//float ctPt1[3] = { 0, 0.1, 0 };
	//float ctPt2[3] = { 0, -0.1, 0 };
	//float ctPt3[3] = { 0.1, 0, 0 };
	//float ctPt4[3] = { -0.1, 0, 0 };
	//float ctPt5[3] = { 0, 0, 0.1 };
	//float ctPt6[3] = { 0, 0, -0.1 };

	//CvScalar color = CV_RGB(255, 0, 255);
	//CvScalar color2 = CV_RGB(0, 255, 255);

	//Timer *mytimer = new Timer();

	//while ((key = cvWaitKey(10)) != 27 && count<1480)
	//{
	//	sprintf(inputColorName, "..\\Data\\k1_cut\\c0-%04d.jpg", count);
	//	sprintf(inputDepthName, "..\\Data\\k1_cut\\d-%04d.pgm", count);

	//	loadISRU4Image(inputColorName, tColor);
	//	IplImage *tmpDepth = cvLoadImage(inputDepthName, CV_LOAD_IMAGE_UNCHANGED);
	//	memcpy(tDepth->data, tmpDepth->imageData, 640 * 480 * sizeof(unsigned short));

	//	myFrame->loadColorAndDepthFrame(tColor,tDepth,true);

	//	mytimer->restart();
	//	ISRTrackingEngine::Instance()->TrackFrame(myFrame, myShapeUnion, myHelper);	
	//	mytimer->check();


	//	Vector2d iPt1 = ProjectPtToImg((Matrix3f*) myFrame->intrinsics->A, myShapeUnion->shapes[0]->pose->invH, (Vector3f*)ctPt1);
	//	Vector2d iPt2 = ProjectPtToImg((Matrix3f*)myFrame->intrinsics->A, myShapeUnion->shapes[0]->pose->invH, (Vector3f*)ctPt2);
	//	Vector2d iPt3 = ProjectPtToImg((Matrix3f*)myFrame->intrinsics->A, myShapeUnion->shapes[0]->pose->invH, (Vector3f*)ctPt3);
	//	Vector2d iPt4 = ProjectPtToImg((Matrix3f*)myFrame->intrinsics->A, myShapeUnion->shapes[0]->pose->invH, (Vector3f*)ctPt4);
	//	Vector2d iPt5 = ProjectPtToImg((Matrix3f*)myFrame->intrinsics->A, myShapeUnion->shapes[0]->pose->invH, (Vector3f*)ctPt5);
	//	Vector2d iPt6 = ProjectPtToImg((Matrix3f*)myFrame->intrinsics->A, myShapeUnion->shapes[0]->pose->invH, (Vector3f*)ctPt6);


	//	CvPoint pt1 = { iPt1.x, iPt1.y };
	//	CvPoint pt2 = { iPt2.x, iPt2.y };
	//	CvPoint pt3 = { iPt3.x, iPt3.y };
	//	CvPoint pt4 = { iPt4.x, iPt4.y };
	//	CvPoint pt5 = { iPt5.x, iPt5.y };
	//	CvPoint pt6 = { iPt6.x, iPt6.y };

	//	cvDrawLine(colorFrame, pt1, pt2, color, 2);
	//	cvDrawLine(colorFrame, pt3, pt4, color, 2);
	//	cvDrawLine(colorFrame, pt5, pt6, color, 2);

	//	iPt1 = ProjectPtToImg((Matrix3f*)myFrame->intrinsics->A, myShapeUnion->shapes[1]->pose->invH, (Vector3f*)ctPt1);
	//	iPt2 = ProjectPtToImg((Matrix3f*)myFrame->intrinsics->A, myShapeUnion->shapes[1]->pose->invH, (Vector3f*)ctPt2);
	//	iPt3 = ProjectPtToImg((Matrix3f*)myFrame->intrinsics->A, myShapeUnion->shapes[1]->pose->invH, (Vector3f*)ctPt3);
	//	iPt4 = ProjectPtToImg((Matrix3f*)myFrame->intrinsics->A, myShapeUnion->shapes[1]->pose->invH, (Vector3f*)ctPt4);
	//	iPt5 = ProjectPtToImg((Matrix3f*)myFrame->intrinsics->A, myShapeUnion->shapes[1]->pose->invH, (Vector3f*)ctPt5);
	//	iPt6 = ProjectPtToImg((Matrix3f*)myFrame->intrinsics->A, myShapeUnion->shapes[1]->pose->invH, (Vector3f*)ctPt6);

	//	CvPoint pt11 = { iPt1.x, iPt1.y };
	//	CvPoint pt21 = { iPt2.x, iPt2.y };
	//	CvPoint pt31 = { iPt3.x, iPt3.y };
	//	CvPoint pt41 = { iPt4.x, iPt4.y };
	//	CvPoint pt51 = { iPt5.x, iPt5.y };
	//	CvPoint pt61 = { iPt6.x, iPt6.y };

	//	cvDrawLine(colorFrame, pt11, pt21, color2, 2);
	//	cvDrawLine(colorFrame, pt31, pt41, color2, 2);
	//	cvDrawLine(colorFrame, pt51, pt61, color2, 2);


	//	cvShowImage("Color", colorFrame);
	//	cvShowImage("Depth", depthFrame);

	//	cvReleaseImage(&tmpDepth);

	//	count++;
	//}

	//cvDestroyAllWindows();
	//cvReleaseImage(&colorFrame);
	//cvReleaseImage(&depthFrame);

}