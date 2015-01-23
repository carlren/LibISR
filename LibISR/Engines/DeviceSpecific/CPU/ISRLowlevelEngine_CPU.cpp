#include "ISRLowlevelEngine_CPU.h"
#include "../../DeviceAgnostic/ISRLowlevelEngine_DA.h"

#include "../../../../LibISRUtils/IOUtil.h"

using namespace LibISR;
using namespace LibISR::Engine;
using namespace LibISR::Objects;

void LibISR::Engine::ISRLowlevelEngine_CPU::createCamCordPointCloud(ISRFloat4Image *ptcloud_out, ISRShortImage *raw_depth_in, const Vector4f &instrinsic)
{
	int w = raw_depth_in->noDims.width;
	int h = raw_depth_in->noDims.height;

	short* depth_ptr = raw_depth_in->GetData(false);
	Vector4f* ptcloud_ptr = ptcloud_out->GetData(false);

	for (int i = 0; i < h; i++) for (int j = 0; j < w; j++)
	{
		int idx = i * w + j;
		ushort rawdepth = depth_ptr[idx];
		float z = rawdepth == 65535 ? 0 : ((float)rawdepth) / 1000.0f;
		if (z>0) unprojectPtWithIntrinsic(instrinsic, Vector3f(j*z, i*z, z), ptcloud_ptr[idx]);
		else ptcloud_ptr[idx] = Vector4f(0, 0, 0, -1);
	}
}

void LibISR::Engine::ISRLowlevelEngine_CPU::createForgroundProbabilityMap(ISRFloatImage *pfmap_out, ISRUChar4Image *rgb_in, Objects::ISRHistogram *histogram)
{

	int w = rgb_in->noDims.width;
	int h = rgb_in->noDims.height;

	float* pf_ptr = pfmap_out->GetData(false);
	Vector4u* rgb_ptr = rgb_in->GetData(false);

	for (int i = 0; i < h; i++) for (int j = 0; j < w; j++)
	{
		int idx = i * w + j;
		pf_ptr[idx] = getPf(rgb_ptr[idx],histogram->posterior,histogram->noBins);
	}
}

void LibISR::Engine::ISRLowlevelEngine_CPU::createAlignedRGBImage(ISRUChar4Image *rgb_out, ISRShortImage *raw_depth_in, ISRUChar4Image *rgb_in, Objects::ISRExHomography *home)
{
	int w = raw_depth_in->noDims.width;
	int h = raw_depth_in->noDims.height;

	short* depth_ptr = raw_depth_in->GetData(false);
	Vector4u* rgb_in_ptr = rgb_in->GetData(false);
	Vector4u* rgb_out_ptr = rgb_out->GetData(false);

	for (int i = 0; i < h; i++) for (int j = 0; j < w; j++)
	{
		int idx = i * w + j;
		ushort rawdepth = depth_ptr[idx];
		float z = rawdepth == 65535 ? 0 : ((float)rawdepth) / 1000.0f;

		mapRGBDtoRGB(rgb_out_ptr[idx], Vector3f(j*z, i*z, z), rgb_in_ptr, raw_depth_in->noDims, home->H, home->T);
	}
}

void LibISR::Engine::ISRLowlevelEngine_CPU::createPointCloudWithPf(ISRFloat4Image *ptcloud_out, ISRShortImage *raw_depth_in, ISRUChar4Image *rgb_in, Vector4f intrinsic, Objects::ISRHistogram *histogram)
{
	int w = raw_depth_in->noDims.width;
	int h = raw_depth_in->noDims.height;

	short* depth_ptr = raw_depth_in->GetData(false);
	Vector4f* ptcloud_ptr = ptcloud_out->GetData(false);
	Vector4u* rgb_ptr = rgb_in->GetData(false);

	for (int i = 0; i < h; i++) for (int j = 0; j < w; j++)
	{
		int idx = i * w + j;
		ushort rawdepth = depth_ptr[idx];
		float z = rawdepth == 65535 ? 0 : ((float)rawdepth) / 1000.0f;

		if (z > 0)
		{
			unprojectPtWithIntrinsic(intrinsic, Vector3f(j*z, i*z, z), ptcloud_ptr[idx]);
			ptcloud_ptr[idx].w = getPf(rgb_ptr[idx], histogram->posterior, histogram->noBins);
		}
		else ptcloud_ptr[idx] = Vector4f(0, 0, 0, -1);
	}
}

void LibISR::Engine::ISRLowlevelEngine_CPU::preparePointCloudForRGBDTrackerAllInOne(ISRFloat4Image *ptcloud_out, ISRShortImage *raw_depth_in, ISRUChar4Image *rgb_in, Objects::ISRCalib* calib, Objects::ISRHistogram *histogram, const Vector4i& boundingbox)
{
	int w = raw_depth_in->noDims.width;
	int h = raw_depth_in->noDims.height;

	short* depth_ptr = raw_depth_in->GetData(false);
	Vector4f* ptcloud_ptr = ptcloud_out->GetData(false);
	Vector4u* rgb_ptr = rgb_in->GetData(false);
	Vector4f intrinsic = calib->intrinsics_d.getParam();
	Matrix3f H = calib->homo_depth_to_color.H;
	Vector3f T = calib->homo_depth_to_color.T;

	for (int i = 0; i < h; i++) for (int j = 0; j < w; j++)
	{
		int idx = i * w + j;
		if (j<boundingbox.x || j>=boundingbox.z || i<boundingbox.y || i>=boundingbox.w)
		{
			ptcloud_ptr[idx] = Vector4f(0, 0, 0, -1);
		}
		else
		{ 
			ushort rawdepth = (ushort)depth_ptr[idx];
			float z = (rawdepth == 65535) ? 0 : ((float)rawdepth) / 1000.0f;
			preparePtCouldDataAllInOne(ptcloud_ptr[idx], Vector3f(j*z, i*z, z), rgb_ptr, raw_depth_in->noDims, intrinsic, H, T, histogram->posterior, histogram->noBins);
		}
	}


}

