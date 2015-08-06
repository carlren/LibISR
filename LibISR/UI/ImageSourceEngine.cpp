#include "ImageSourceEngine.h"

#include "../Utils/IOUtil.h"

#include <stdio.h>

using namespace LibISRUtils;
using namespace LibISR::Objects;

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

	cached_rgb = new UChar4Image(true,false);
	cached_depth = new ShortImage(true,false);

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
		out->rgb->SetFrom(cached_rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
		delete cached_rgb;
		cached_rgb = NULL;
		bUsedCache = true;
	}
	if (cached_depth != NULL) {
		out->rawDepth->SetFrom(cached_depth, ORUtils::MemoryBlock<short>::CPU_TO_CPU);
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
