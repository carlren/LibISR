#include <stdio.h>

#include "IOUtil.h"
#include "Timer.h"

#include"ISRHistogram.h"
#include"ISRShape.h"
#include"ISRImage.h"
#include"ISRPose.h"
//#include"ISRKinectCapture.h"
#include"ISRFrame.h"
#include"ISRTrackingEngine.h"


#include "../Dependency/OpenCV/opencv2/highgui/highgui.hpp"
#include "../Dependency/OpenCV/opencv2/core/core.hpp"

using namespace LibISR::Objects;
using namespace LibISR::Engine;

//int main(void
//{
//	ISRKinectCapture *myCapture = new ISRKinectCapture();
//	ISRFrame *myFrame = new ISRFrame(640,480);
//	Vector3d myVolSize; myVolSize.x = 200; myVolSize.y = 200; myVolSize.z = 200;
//	ISRShape *myShape = new ISRShape(myVolSize,2,0);
//	myShape->LoadShapeFromFile("../Data/newCut.bin");
//
//
//	cvNamedWindow("Color",0);
//	cvNamedWindow("Depth",0);
//
//	IplImage* colorFrame = cvCreateImage(cvSize(640, 480), 8, 4);
//	IplImage* depthFrame = cvCreateImage(cvSize(640,480),8,4);
//
//	colorFrame->imageData = (char*)myFrame->alignedColorImage->data;
//	depthFrame->imageData = (char*)myFrame->displayDepthImage->data;
//
//	int key;
//	while((key = cvWaitKey(10))!=27)
//	{
//		myCapture->pullColorFrame();
//		myCapture->pullDepthFrame();
//		myFrame->loadColorAndDepthFrame(myCapture->colorFrame,myCapture->depthFrame,true);
//
//		cvShowImage("Color",colorFrame);
//		cvShowImage("Depth",depthFrame);
//
//	}
//
//	cvDestroyAllWindows();
//	
//	cvReleaseImage(&colorFrame);
//	cvReleaseImage(&depthFrame);
//
//}



void loadISRU4Image(char* fileName, ISRUChar4Image* outImage)
{
	IplImage* tmpImg = cvLoadImage(fileName, CV_LOAD_IMAGE_UNCHANGED);

	for (int i = 0; i < 480; i++) for (int j = 0; j < 640; j++)
	{
		int idx = i * 640 + j;
		outImage->data[idx].x = tmpImg->imageData[idx * 3];
		outImage->data[idx].y = tmpImg->imageData[idx * 3 + 1];
		outImage->data[idx].z = tmpImg->imageData[idx * 3 + 2];
	}

	cvReleaseImage(&tmpImg);
}

int main(void)
{
	ISRFrame *myFrame = new ISRFrame(640, 480);
	Vector3d myVolSize; myVolSize.x = 200; myVolSize.y = 200; myVolSize.z = 200;

	//ISRShapeUnion *myShapeUnion = new ISRShapeUnion(1);
	//myShapeUnion->shapes[0] = new ISRShape(myVolSize, 2, 0);
	//myShapeUnion->shapes[0]->LoadShapeFromFile("../Data/newCut.bin");

	//ISRShapeUnion *myShapeUnion = new ISRShapeUnion(2);

	//myShapeUnion->shapes[0] = new ISRShape(myVolSize, 2, 0);
	//myShapeUnion->shapes[1] = new ISRShape(myVolSize, 2, 0);

	//myShapeUnion->shapes[0]->LoadShapeFromFile("../Data/newCut.bin");
	//myShapeUnion->shapes[1]->LoadShapeFromFile("../Data/newCut.bin");

	float A[9] = { 592.8291f, 0.0f, 321.1862f,
		0.0f, 596.1464f, 236.3956f,
		0.0f, 0.0f, 1.0000f };
	myFrame->intrinsics->SetFrom(A);

	int objCount = 2;

	ISRShapeUnion *myShapeUnion = new ISRShapeUnion(objCount);
	ISROptimizationHelper *myHelper = new ISROptimizationHelper(objCount);

	//myShapeUnion->shapes[0] = new ISRShape(myVolSize, 2, 0);
	//myShapeUnion->shapes[0]->LoadShapeFromFile("../Data/newCut.bin");

	for (int i = 0; i < objCount; i++)
	{
		myShapeUnion->shapes[i] = new ISRShape(myVolSize, 2, 0);
		myShapeUnion->shapes[i]->LoadShapeFromFile("../Data/newCut.bin");
	}

	float step1[6] = { 0.5119f, -0.1408f, 0.7854f, 0.0f, - 0.637070260807493f, 0.0f };
	myShapeUnion->shapes[0]->pose->SetFromStep(step1);

	float step2[6] = { 0.6687f,    0.5081f,    0.1909f, 0.5469f, 0.9473f, -0.9473f };
	for (int i = 1; i < objCount; i++)
	{
		myShapeUnion->shapes[1]->pose->SetFromStep(step2);
	}
	
	cvNamedWindow("Color", 0);
	cvNamedWindow("Depth", 0);

	IplImage* colorFrame = cvCreateImage(cvSize(640, 480), 8, 4);
	IplImage* depthFrame = cvCreateImage(cvSize(640, 480), 8, 4);

	colorFrame->imageData = (char*)myFrame->alignedColorImage->data;
	depthFrame->imageData = (char*)myFrame->displayDepthImage->data;

	char inputColorName[100];
	char inputDepthName[200];

	int key;
	int count = 0;

	ISRUChar4Image *tColor = new ISRUChar4Image(640, 480);
	ISRUShortImage *tDepth = new ISRUShortImage(640, 480);

	ISRUChar4Image *histColor = new ISRUChar4Image(640, 480);
	ISRUCharImage *histMask = new ISRUCharImage(640, 480);


	loadISRU4Image("Files\\color.jpg",histColor);
	IplImage *tmpMask = cvLoadImage("Files\\mask.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	memcpy(histMask->data, tmpMask->imageData, 640 * 480 * sizeof(unsigned char));
	myFrame->histogram->BuildHistogram(histColor, histMask);

	//PrintArrayToFile("d:\\hist.txt", myFrame->histogram->posterior, myFrame->histogram->dim);

	float ctPt1[3] = { 0, 0.1, 0 };
	float ctPt2[3] = { 0, -0.1, 0 };
	float ctPt3[3] = { 0.1, 0, 0 };
	float ctPt4[3] = { -0.1, 0, 0 };
	float ctPt5[3] = { 0, 0, 0.1 };
	float ctPt6[3] = { 0, 0, -0.1 };

	CvScalar color = CV_RGB(255, 0, 255);
	CvScalar color2 = CV_RGB(0, 255, 255);

	Timer *mytimer = new Timer();

	//char outTimerName[100];
	//sprintf(outTimerName, "d:\\Timer_%04d.txt", objCount);
	//FILE* fid = fopen(outTimerName, "w");

	while ((key = cvWaitKey(10)) != 27 && count<1480)
	{
		sprintf(inputColorName, "Files\\k1_cut\\c0-%04d.jpg", count);
		sprintf(inputDepthName, "Files\\k1_cut\\d-%04d.pgm", count);

		loadISRU4Image(inputColorName, tColor);
		IplImage *tmpDepth = cvLoadImage(inputDepthName, CV_LOAD_IMAGE_UNCHANGED);
		memcpy(tDepth->data, tmpDepth->imageData, 640 * 480 * sizeof(unsigned short));

		myFrame->loadColorAndDepthFrame(tColor,tDepth,true);
		
		//WriteMatlabTXTImg("d:\\tmp\\depth.txt", myFrame->depthImage->data, 640, 480);

		mytimer->restart();
		ISRTrackingEngine::Instance()->TrackFrame(myFrame, myShapeUnion, myHelper);	
		//fprintf(fid, "%f\n", mytimer->elapsed_time());
		mytimer->check();

		Vector2d iPt1 = ProjectPtToImg((Matrix3f*) myFrame->intrinsics->A, myShapeUnion->shapes[0]->pose->invH, (Vector3f*)ctPt1);
		Vector2d iPt2 = ProjectPtToImg((Matrix3f*)myFrame->intrinsics->A, myShapeUnion->shapes[0]->pose->invH, (Vector3f*)ctPt2);
		Vector2d iPt3 = ProjectPtToImg((Matrix3f*)myFrame->intrinsics->A, myShapeUnion->shapes[0]->pose->invH, (Vector3f*)ctPt3);
		Vector2d iPt4 = ProjectPtToImg((Matrix3f*)myFrame->intrinsics->A, myShapeUnion->shapes[0]->pose->invH, (Vector3f*)ctPt4);
		Vector2d iPt5 = ProjectPtToImg((Matrix3f*)myFrame->intrinsics->A, myShapeUnion->shapes[0]->pose->invH, (Vector3f*)ctPt5);
		Vector2d iPt6 = ProjectPtToImg((Matrix3f*)myFrame->intrinsics->A, myShapeUnion->shapes[0]->pose->invH, (Vector3f*)ctPt6);


		CvPoint pt1 = { iPt1.x, iPt1.y };
		CvPoint pt2 = { iPt2.x, iPt2.y };
		CvPoint pt3 = { iPt3.x, iPt3.y };
		CvPoint pt4 = { iPt4.x, iPt4.y };
		CvPoint pt5 = { iPt5.x, iPt5.y };
		CvPoint pt6 = { iPt6.x, iPt6.y };

		cvDrawLine(colorFrame, pt1, pt2, color, 2);
		cvDrawLine(colorFrame, pt3, pt4, color, 2);
		cvDrawLine(colorFrame, pt5, pt6, color, 2);

		iPt1 = ProjectPtToImg((Matrix3f*)myFrame->intrinsics->A, myShapeUnion->shapes[1]->pose->invH, (Vector3f*)ctPt1);
		iPt2 = ProjectPtToImg((Matrix3f*)myFrame->intrinsics->A, myShapeUnion->shapes[1]->pose->invH, (Vector3f*)ctPt2);
		iPt3 = ProjectPtToImg((Matrix3f*)myFrame->intrinsics->A, myShapeUnion->shapes[1]->pose->invH, (Vector3f*)ctPt3);
		iPt4 = ProjectPtToImg((Matrix3f*)myFrame->intrinsics->A, myShapeUnion->shapes[1]->pose->invH, (Vector3f*)ctPt4);
		iPt5 = ProjectPtToImg((Matrix3f*)myFrame->intrinsics->A, myShapeUnion->shapes[1]->pose->invH, (Vector3f*)ctPt5);
		iPt6 = ProjectPtToImg((Matrix3f*)myFrame->intrinsics->A, myShapeUnion->shapes[1]->pose->invH, (Vector3f*)ctPt6);

		CvPoint pt11 = { iPt1.x, iPt1.y };
		CvPoint pt21 = { iPt2.x, iPt2.y };
		CvPoint pt31 = { iPt3.x, iPt3.y };
		CvPoint pt41 = { iPt4.x, iPt4.y };
		CvPoint pt51 = { iPt5.x, iPt5.y };
		CvPoint pt61 = { iPt6.x, iPt6.y };

		cvDrawLine(colorFrame, pt11, pt21, color2, 2);
		cvDrawLine(colorFrame, pt31, pt41, color2, 2);
		cvDrawLine(colorFrame, pt51, pt61, color2, 2);


		cvShowImage("Color", colorFrame);
		cvShowImage("Depth", depthFrame);

		cvReleaseImage(&tmpDepth);

		count++;
	}

	//fclose(fid);
	cvDestroyAllWindows();

	cvReleaseImage(&colorFrame);
	cvReleaseImage(&depthFrame);

}