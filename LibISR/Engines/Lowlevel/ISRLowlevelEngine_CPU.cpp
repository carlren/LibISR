#include "ISRLowlevelEngine_CPU.h"
#include "ISRLowlevelEngine_DA.h"

#include "../../../LibISRUtils/IOUtil.h"

using namespace LibISR;
using namespace LibISR::Engine;
using namespace LibISR::Objects;

//////////////////////////////////////////////////////////////////////////
// Below are the functions that are currently used
//////////////////////////////////////////////////////////////////////////

void LibISR::Engine::ISRLowlevelEngine_CPU::subsampleImageRGBDImage(ISRFloat4Image *outimg, ISRFloat4Image *inimg)
{
	Vector2i oldDims = inimg->noDims;
	Vector2i newDims; newDims.x = inimg->noDims.x / 2; newDims.y = inimg->noDims.y / 2;

	outimg->ChangeDims(newDims);

	const Vector4f *imageData_in = inimg->GetData(false);
	Vector4f *imageData_out = outimg->GetData(false);

	for (int y = 0; y < newDims.y; y++) for (int x = 0; x < newDims.x; x++)
		filterSubsampleWithHoles(imageData_out, x, y, newDims, imageData_in, oldDims);
}

void LibISR::Engine::ISRLowlevelEngine_CPU::prepareAlignedRGBDData(ISRFloat4Image *outimg, ISRShortImage *raw_depth_in, ISRUChar4Image *rgb_in, Objects::ISRExHomography *home)
{
	int w = raw_depth_in->noDims.width;
	int h = raw_depth_in->noDims.height;

	short* depth_ptr = raw_depth_in->GetData(false);
	Vector4u* rgb_in_ptr = rgb_in->GetData(false);
	Vector4f* rgbd_out_ptr = outimg->GetData(false);

	bool alreadyAligned = home->T == Vector3f(0, 0, 0);

	for (int i = 0; i < h; i++) for (int j = 0; j < w; j++)
	{
		int idx = i * w + j;
		ushort rawdepth = depth_ptr[idx];
		float z = rawdepth == 65535 ? 0 : ((float)rawdepth) / 1000.0f;

		if (alreadyAligned)
		{
			rgbd_out_ptr[idx].x = rgb_in_ptr[idx].r;
			rgbd_out_ptr[idx].y = rgb_in_ptr[idx].g;
			rgbd_out_ptr[idx].z = rgb_in_ptr[idx].b;
			rgbd_out_ptr[idx].w = z;
		}
		else
		{
			mapRGBDtoRGB(rgbd_out_ptr[idx], Vector3f(j*z, i*z, z), rgb_in_ptr, raw_depth_in->noDims, home->H, home->T);
			rgbd_out_ptr[idx].w = z;
		}

	}

}

void LibISR::Engine::ISRLowlevelEngine_CPU::preparePointCloudFromAlignedRGBDImage(ISRFloat4Image *ptcloud_out, ISRFloat4Image *inimg, Objects::ISRHistogram *histogram, const Vector4f &intrinsic, const Vector4i &boundingbox)
{
	if (inimg->noDims != ptcloud_out->noDims) ptcloud_out->ChangeDims(inimg->noDims);
	
	int w = inimg->noDims.width;
	int h = inimg->noDims.height;

	int noBins = histogram->noBins;

	Vector4f *inimg_ptr = inimg->GetData(false);
	Vector4f* ptcloud_ptr = ptcloud_out->GetData(false);
	
	for (int i = 0; i < h; i++) for (int j = 0; j < w; j++)
	{
		int idx = i * w + j;
		if (j < boundingbox.x || j >= boundingbox.z || i < boundingbox.y || i >= boundingbox.w)
		{
			ptcloud_ptr[idx] = Vector4f(0, 0, 0, -1);
		}
		else
		{
			float z = inimg_ptr[idx].w;
			unprojectPtWithIntrinsic(intrinsic, Vector3f(j*z, i*z, z), ptcloud_ptr[idx]);
			ptcloud_ptr[idx].w = getPf(inimg_ptr[idx], histogram->posterior, noBins);
		}
	}
}

void LibISR::Engine::ISRLowlevelEngine_CPU::computepfImageFromHistogram(ISRUChar4Image *rgb_in, Objects::ISRHistogram *histogram)
{
	int w = rgb_in->noDims.width;
	int h = rgb_in->noDims.height;
	int noBins = histogram->noBins;
	float pf = 0;

	Vector4u *inimg_ptr = rgb_in->GetData(false);

	for (int i = 0; i < h; i++) for (int j = 0; j < w; j++)
	{
		int idx = i * w + j;
		pf = getPf(inimg_ptr[idx], histogram->posterior, noBins);
		if (pf > 0.5f)
		{
			inimg_ptr[idx].r = 255;
			inimg_ptr[idx].g = 0;
			inimg_ptr[idx].b = 0;
		}
		else if (pf == 0.5f)
		{
			inimg_ptr[idx].r = 0;
			inimg_ptr[idx].g = 0;
			inimg_ptr[idx].b = 255;
		}

	}
}

