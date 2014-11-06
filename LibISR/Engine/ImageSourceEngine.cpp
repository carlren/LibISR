#include "ImageSourceEngine.h"

#include "../Utils/IOUtil.h"

#include <stdio.h>

using namespace LibISR::Engine;


ImageSourceEngine::ImageSourceEngine(const char *calibFilename)
{
	 readRGBDCalib(calibFilename, calib);
}


ImageFileReader::ImageFileReader(const char *calibFilename, const char *rgbImageMask, const char *depthImageMask)
	: ImageSourceEngine(calibFilename)
{
	strncpy(this->rgbImageMask, rgbImageMask, BUF_SIZE);
	strncpy(this->depthImageMask, depthImageMask, BUF_SIZE);

	currentFrameNo = 0;
	cachedFrameNo = -1;

	cached_rgb = NULL;
	cached_depth = NULL;
}

ImageFileReader::~ImageFileReader()
{
	delete cached_rgb;
	delete cached_depth;
}

void ImageFileReader::loadIntoCache(void)
{
	if ((cached_rgb != NULL) || (cached_depth != NULL)) return;

	cached_rgb = new ISRUChar4Image();
	cached_depth = new ISRShortImage();

	char str[2048];
	sprintf(str, rgbImageMask, currentFrameNo);
	if (!ReadImageFromFile(cached_rgb, str)) {
		delete cached_rgb; cached_rgb = NULL;
		printf("error reading file '%s'\n", str);
	}
	sprintf(str, depthImageMask, currentFrameNo);
	if (!ReadImageFromFile(cached_depth, str)) {
		delete cached_depth; cached_depth = NULL;
		printf("error reading file '%s'\n", str);
	}
}

bool ImageFileReader::hasMoreImages(void)
{
	loadIntoCache();
	return ((cached_rgb != NULL) && (cached_depth != NULL));
}

void ImageFileReader::getImages(ISRView *out)
{
	bool bUsedCache = false;
	if (cached_rgb != NULL) {
		out->rgb->SetFrom(cached_rgb);
		delete cached_rgb;
		cached_rgb = NULL;
		bUsedCache = true;
	}
	if (cached_depth != NULL) {
		out->rawDepth->SetFrom(cached_depth);
		delete cached_depth;
		cached_depth = NULL;
		bUsedCache = true;
	}

	if (!bUsedCache) {
		char str[2048];
		sprintf(str, rgbImageMask, currentFrameNo);
		if (!ReadImageFromFile(out->rgb, str)) {
			printf("error reading file '%s'\n", str);
		}

		sprintf(str, depthImageMask, currentFrameNo);
		if (!ReadImageFromFile(out->rawDepth, str)) {
			printf("error reading file '%s'\n", str);
		}
	}

	if (calib.disparityCalib.params.y == 0) out->inputDepthType = ISRView::ISR_SHORT_DEPTH;
	else out->inputDepthType = ISRView::ISR_DISPARITY_DEPTH;

	if (calib.homo_depth_to_color.T.x == 0) out->inputImageType = ISRView::ISR_RGBD_HOMOGRAPHY;
	else out->inputImageType = ISRView::ISR_RGBD_EXTRINSIC;

	++currentFrameNo;
}

Vector2i ImageFileReader::getDepthImageSize(void)
{
	loadIntoCache();
	return cached_depth->noDims;
}

Vector2i ImageFileReader::getRGBImageSize(void)
{
	loadIntoCache();
	return cached_rgb->noDims;
}


//void loadColorAndDepthFrame(ISRUChar4Image* inputColor, ISRUShortImage* inputDepth, InputImageType imgType)
//{
//	bool visualizeDepth = true;
//
//	this->colorImage->SetFrom(inputColor);
//	this->rawDepthImage->SetFrom(inputDepth);
//
//	ISRUChar4Image* tmpAlignImg = new ISRUChar4Image(inputDepth->noDims, false);
//	ISRFloatImage* tmpDepthImg = new ISRFloatImage(inputDepth->noDims, false);
//
//	USHORT *depthp = inputDepth->GetData(false);
//	Vector4u *rgbp = inputColor->GetData(false);
//
//	float* tmpdepthp = tmpDepthImg->GetData(false);
//	Vector4u *tmprgbp = tmpAlignImg->GetData(false);
//
//	if (imgType == ISR_RGBD_HOMOGRAPHY)
//		for (int j = 0; j < inputDepth->noDims.x; j++) for (int i = 0; i < inputDepth->noDims.y; i++)
//		{
//		int idx = j*inputDepth->noDims.x + i;
//		USHORT rawDepth = depthp[idx];
//
//		float realDepth = rawDepth == 65535 ? 0 : ((float)rawDepth) / 1000.0f;
//		tmpdepthp[idx] = realDepth;
//
//
//
//		if (visualizeDepth)
//		{
//			int intensity = (int)(realDepth * 1000) % 256;
//			displayDepthImage->data[idx].x = intensity;
//			displayDepthImage->data[idx].y = intensity;
//			displayDepthImage->data[idx].z = intensity;
//		}
//
//		// align color and depth
//		float* rH = extrinsics->rH;
//		float* rT = extrinsics->rT;
//
//		if (realDepth != 0)
//		{
//			float x = (float)i * realDepth;
//			float y = (float)j * realDepth;
//
//			float fIX = rH[0] * x + rH[1] * y + rH[2] * realDepth + rT[0];
//			float fIY = rH[3] * x + rH[4] * y + rH[5] * realDepth + rT[1];
//			float fIZ = rH[6] * x + rH[7] * y + rH[8] * realDepth + rT[2];
//
//			int iX = (int)(fIX / fIZ);
//			int iY = (int)(fIY / fIZ);
//
//			int imgIdx = iY*this->width + iX;
//
//			if (iX >= 0 && iX < this->width && iY >= 0 && iY < this->height)
//			{
//				this->alignedColorImage->data[idx].x = this->colorImage->data[imgIdx].x;
//				this->alignedColorImage->data[idx].y = this->colorImage->data[imgIdx].y;
//				this->alignedColorImage->data[idx].z = this->colorImage->data[imgIdx].z;
//			}
//
//		}
//		else
//		{
//			this->alignedColorImage->data[idx].x = 0;
//			this->alignedColorImage->data[idx].y = 255;
//			this->alignedColorImage->data[idx].z = 0;
//		}
//
//		}
//}