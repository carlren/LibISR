
#include "../LibISR/LibISR.h"

#include "ImageSourceEngine.h"
#include "OpenNIEngine.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

#pragma comment( lib, "opencv_core2410.lib" )
#pragma comment( lib, "opencv_highgui2410.lib" )


#include "../LibISRUtils/IOUtil.h"
#include "../LibISRUtils/Timer.h"
#include "../LibISRUtils/NVTimer.h"

using namespace LibISR::Engine;
using namespace LibISR::Objects;
using namespace LibISRUtils;

void inline updateHistogramFromRendering(UChar4Image* rendering, UChar4Image* rgb, LibISR::Objects::ISRHistogram* hist)
{
	Vector4u* imgptr = rendering->GetData(MEMORYDEVICE_CPU);
	Vector4u bpix((uchar)0);
	for (int i = 0; i < rendering->dataSize; i++)
		if (imgptr[i] != bpix) imgptr[i] = Vector4u(255, 255, 255, 255);
		else imgptr[i] = Vector4u(100, 100, 100, 100);

		hist->buildHistogram(rgb, rendering);

}


void copydataISR2OpenCV(IplImage* outimg, UChar4Image* inimg)
{
	uchar* outimg_ptr = (uchar*)outimg->imageData;
	Vector4u* inimg_ptr = inimg->GetData(MEMORYDEVICE_CPU);

	for (int i = 0; i < inimg->dataSize;i++)
	{
		outimg_ptr[i * 4 + 0] = inimg_ptr[i].b;
		outimg_ptr[i * 4 + 1] = inimg_ptr[i].g;
		outimg_ptr[i * 4 + 2] = inimg_ptr[i].r;
	}
}

void main__(int argc, char** argv)
{
	//const char *sdfFile = "../Data/newCut.bin";
	const char *sdfFile = "../Data/teacan.bin";
	//const char *sdfFile = "../Data/ball.bin";
	const char *calibFile = "../Data/calib_reg.txt";

	ImageSourceEngine *imageSource = new OpenNIEngine(calibFile, NULL, true);

	ISRLibSettings isrSettings;
	isrSettings.noHistogramDim = HISTOGRAM_BIN;
	isrSettings.noTrackingObj = 1;
	isrSettings.singleAappearanceModel = true;
	isrSettings.useGPU = false;

	ISRCoreEngine *coreEngine = new ISRCoreEngine(&isrSettings, &imageSource->calib, imageSource->getDepthImageSize(), imageSource->getRGBImageSize());
	coreEngine->shapeUnion->loadShapeFromFile(sdfFile, Vector3i(DT_VOL_SIZE, DT_VOL_SIZE, DT_VOL_SIZE), 0);
	for (int i = 1; i < isrSettings.noTrackingObj; i++)
		coreEngine->shapeUnion->shareSDFWithExistingShape(*coreEngine->shapeUnion->getShape(0), i);

	float poses[6] = { 0.0f, 0.0f, 0.7f, 0.1f, 0.0f, 0.0f };
	coreEngine->trackingState->setHFromParam(poses, 0);

	//////////////////////////////////////////////////////////////////////////
	// opencv interface stuff
	//////////////////////////////////////////////////////////////////////////

	cvNamedWindow("LibISR", 0);
	IplImage* depthFrame = cvCreateImage(cvSize(640, 480), 8, 4);


	Vector3f cpt[4];	cpt[0] = Vector3f(0, 0, 0);	cpt[1] = Vector3f(0.1, 0, 0);	cpt[2] = Vector3f(0, 0.1, 0);	cpt[3] = Vector3f(0, 0, 0.1);
	CvPoint cvpt[4];Vector3f ipt[4];
	CvScalar color[3];	color[0] = CV_RGB(0, 0, 255);	color[1] = CV_RGB(0, 255, 0);	color[2] = CV_RGB(255, 0, 0);
	CvScalar bbcolor = CV_RGB(255,255,0);

	int key=0;
	int count = 0;

	Matrix3f A = coreEngine->getView()->calib->intrinsics_d.A;
	Matrix3f H = coreEngine->getView()->calib->homo_depth_to_color.H;
	Vector3f T = coreEngine->getView()->calib->homo_depth_to_color.T*0.001;

	StopWatchInterface *timer;
	sdkCreateTimer(&timer);
	float processedTime = 0;

	while (key != 27)
	{
		key = cvWaitKey(10);
		if (!imageSource->hasMoreImages()) return;
		imageSource->getImages(coreEngine->getView());
		
		//if (key=='r') updateHistogramFromRendering(coreEngine->getRenderingState()->outputImage, coreEngine->getView()->rgb, coreEngine->frame->histogram);

		coreEngine->processFrame();

		//Vector4i bb = coreEngine->frame->imgHierarchy->levels[0].boundingbox;
		copydataISR2OpenCV(depthFrame, coreEngine->getView()->alignedRgb);

		//// draw the axis on object
		//for (int o = 0; o < isrSettings.noTrackingObj; o++)
		//{
		//	Matrix4f M = coreEngine->trackingState->getPose(o)->getH();
		//	for (int i = 0; i < 4; i++)
		//	{
		//		ipt[i] = H*(A*(M*cpt[i])) + T;
		//		cvpt[i].x = ipt[i].x / ipt[i].z;
		//		cvpt[i].y = ipt[i].y / ipt[i].z;
		//	}
		//	for (int i = 0; i < 3; i++) cvDrawLine(depthFrame, cvpt[0], cvpt[i + 1], color[i], 2);
		//}

		//// draw the bounding box
		//CvPoint p1, p2, p3, p4;
		//p1.x = bb.x; p1.y = bb.y; 
		//p2.x = bb.z; p2.y = bb.y;
		//p3.x = bb.z; p4.y = bb.w;
		//p4.x = bb.x, p3.y = bb.w;
		//
		//cvDrawLine(depthFrame, p1, p2, bbcolor, 2);
		//cvDrawLine(depthFrame, p2, p3, bbcolor, 2);
		//cvDrawLine(depthFrame, p3, p4, bbcolor, 2);
		//cvDrawLine(depthFrame, p4, p1, bbcolor, 2);

		cvShowImage("LibISR", depthFrame);

		//char tmpchar[200];
		//sprintf(tmpchar, outName, count);
		//cvSaveImage(tmpchar, depthFrame);

		count++;
	}
	cvDestroyAllWindows();
}