// Copyright 2014-2015 Isis Innovation Limited and the authors of LibISR

#pragma once

#include "../LibISR.h"

namespace LibISRUtils
{
	class ImageSourceEngine
	{
	public:
		LibISR::Objects::ISRCalib calib;

		ImageSourceEngine(const char *calibFilename);
		virtual ~ImageSourceEngine() {}

		virtual bool hasMoreImages(void) = 0;
		virtual void getImages(LibISR::Objects::ISRView *out) = 0;
		virtual Vector2i getDepthImageSize(void) = 0;
		virtual Vector2i getRGBImageSize(void) = 0;
	};

	class ImageFileReader : public ImageSourceEngine
	{
	private:
		static const int BUF_SIZE = 2048;
		char rgbImageMask[BUF_SIZE];
		char depthImageMask[BUF_SIZE];

		UChar4Image *cached_rgb;
		ShortImage *cached_depth;

		void loadIntoCache();
		int cachedFrameNo;

	public:
		int currentFrameNo;

		ImageFileReader(const char *calibFilename, const char *rgbImageMask, const char *depthImageMask);
		~ImageFileReader();

		bool hasMoreImages(void);
		void getImages(LibISR::Objects::ISRView *out);
		Vector2i getDepthImageSize(void);
		Vector2i getRGBImageSize(void);

	};
}

