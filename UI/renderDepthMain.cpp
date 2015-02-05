
#include "../LibISR/LibISR.h"

#include "ImageSourceEngine.h"

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


Vector3f getMRPfromDegree(Vector3f r)
{
	float rotationX = r.x * DEGTORAD;
	float rotationY = r.y * DEGTORAD;
	float rotationZ = r.z * DEGTORAD;

	float c1 = cos(rotationY / 2);
	float c2 = cos(rotationZ / 2);
	float c3 = cos(rotationX / 2);

	float s1 = sin(rotationY / 2);
	float s2 = sin(rotationZ / 2);
	float s3 = sin(rotationX / 2);

	float c1c2 = c1 * c2;
	float s1s2 = s1 * s2;

	float rotation1 = c1c2*s3 + s1s2*c3;
	float rotation2 = s1*c2*c3 + c1*s2*s3;
	float rotation3 = c1*s2*c3 - s1*c2*s3;
	float rotation4 = c1c2*c3 - s1s2*s3;

	float normal = 1 / sqrt(rotation1 *rotation1 + rotation2 * rotation2 + rotation3 * rotation3 + rotation4 * rotation4);

	float b0 = rotation4 * normal;
	float b1 = rotation1 * normal;
	float b2 = rotation2 * normal;
	float b3 = rotation3 * normal;

	Vector3f theta;

	theta.x = b1 / (1 + b0);
	theta.y = b2 / (1 + b0);
	theta.z = b3 / (1 + b0);

	return theta;
}

void main(int argc, char** argv)
{
	const char *outNameD = "../Data/out/depth_%04i.png";
	const char *outNameV = "../Data/out/surface_%04i.jpg";
	const char *outNameN= "../Data/out/normal_%04i.jpg";
	const char *sdfFile = "../Data/couch.bin";
	char tmpname[200];

	Vector2i imgsize(640, 480);
	ISRPose_ptr pose = new ISRPose();
	ISRShape_ptr shape = new ISRShape(); shape->initialize(false, 0); shape->loadShapeFromFile(sdfFile, Vector3i(DT_VOL_SIZE, DT_VOL_SIZE, DT_VOL_SIZE));
	ISRIntrinsics* intrinsic = new ISRIntrinsics(); intrinsic->SetFrom(582.624481, 582.691032, 313.044758, 238.443896);

	ISRVisualisationEngine* renderingengine = new ISRVisualisationEngine_CPU();
	ISRVisualisationState* vstate = new ISRVisualisationState(imgsize, false);
	ISRUShortImage* dimg = new ISRUShortImage(imgsize, false);
	ISRUChar4Image* nimg = new ISRUChar4Image(imgsize, false);

	StopWatchInterface *timer;
	sdkCreateTimer(&timer);
	float processedTime = 0;
	sdkResetTimer(&timer); sdkStartTimer(&timer);



	float xzRot, tilt, zmove;
	Matrix3f Robj, Rcam, Rfinal;
	Vector3f T(0.0f, 0.1, 0.6f), Tfinal;
	Matrix4f Hfinal;

	Vector3f basemrp = getMRPfromDegree(Vector3f(180, -90, 0));

	for (int i = 0; i < 20; i++)
	{
		pose->setHFromRT(basemrp, Vector3f(0, 0, 0));

		xzRot = (((float)rand() / RAND_MAX) * 2 - 1)*PI;
		tilt = -(float)rand() * 0.02 * PI / RAND_MAX;
		zmove = (((float)rand() / RAND_MAX) - 0.5) * 0.3;

		Robj.setIdentity();
		Robj.m00 = cos(xzRot);	Robj.m20 = -sin(xzRot);
		Robj.m02 = sin(xzRot);	Robj.m22 = cos(xzRot);

		Rcam.setIdentity();
		Rcam.m11 = cos(tilt); Rcam.m21 = -sin(tilt);
		Rcam.m12 = sin(tilt); Rcam.m22 = cos(tilt);

		Rfinal = Rcam*Robj; Tfinal = Rcam*(T + Vector3f(0, 0, zmove));

		Hfinal.setIdentity();
		Hfinal.m00 = Rfinal.m00; Hfinal.m10 = Rfinal.m10; Hfinal.m20 = Rfinal.m20; Hfinal.m30 = Tfinal.x;
		Hfinal.m01 = Rfinal.m01; Hfinal.m11 = Rfinal.m11; Hfinal.m21 = Rfinal.m21; Hfinal.m31 = Tfinal.y;
		Hfinal.m02 = Rfinal.m02; Hfinal.m12 = Rfinal.m12; Hfinal.m22 = Rfinal.m22; Hfinal.m32 = Tfinal.z;

		Matrix4f tmpH = Hfinal*pose->getH();
		pose->setFromH(tmpH);

		renderingengine->updateMinmaxmImage(vstate->minmaxImage, pose->getH(), intrinsic->A, imgsize);
		
		renderingengine->renderDepthNormalAndObject(dimg, nimg, vstate, pose->getInvH(), shape, intrinsic->getParam());

		//renderingengine->renderObject(vstate, pose->getInvH(), shape, intrinsic->getParam());
		//renderingengine->renderDepth(dimg, vstate, pose->getInvH(), shape, intrinsic->getParam());

		IplImage* renderframe = cvCreateImage(cvSize(640, 480), 8, 4);
		IplImage* writeframe = cvCreateImage(cvSize(640, 480), 16, 1);
		IplImage* normalframe = cvCreateImage(cvSize(640, 480), 8, 4);

		memcpy(renderframe->imageData, vstate->outputImage->GetData(false), 640 * 480 * sizeof(char) * 4);
		memcpy(normalframe->imageData, nimg->GetData(false), 640 * 480 * sizeof(char) * 4);
		memcpy(writeframe->imageData, dimg->GetData(false), 640 * 480 * sizeof(ushort));

		sprintf_s(tmpname, outNameV, i);
		cvSaveImage(tmpname, renderframe);
		sprintf_s(tmpname, outNameD, i);
		cvSaveImage(tmpname, writeframe);
		sprintf_s(tmpname, outNameN, i);
		cvSaveImage(tmpname, normalframe);


		processedTime = sdkGetTimerValue(&timer);
		printf("\rAverage Rendering Time : [%f] ms = [%d] fps", processedTime / i, (int)(i * 1000 / processedTime));
	}
}