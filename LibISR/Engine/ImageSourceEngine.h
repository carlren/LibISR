#pragma once

#include "../CoreISR/LibISR.h"

namespace LibISR
{
	namespace Engine
	{
		class ImageSourceEngine
		{
		public:
			ISRCalib calib;

			ImageSourceEngine(const char *calibFilename);
			virtual ~ImageSourceEngine() {}

			virtual bool hasMoreImages(void) = 0;
			virtual void getImages(ISRView *out) = 0;
			virtual Vector2i getDepthImageSize(void) = 0;
			virtual Vector2i getRGBImageSize(void) = 0;
		};

		class ImageFileReader : public ImageSourceEngine
		{
		private:
			static const int BUF_SIZE = 2048;
			char rgbImageMask[BUF_SIZE];
			char depthImageMask[BUF_SIZE];

			ISRUChar4Image *cached_rgb;
			ISRShortImage *cached_depth;
			
			void loadIntoCache();
		public:
			int currentFrameNo;

			ImageFileReader(const char *calibFilename, const char *rgbImageMask, const char *depthImageMask);
			~ImageFileReader();

			bool hasMoreImages(void);
			void getImages(ISRView *out);
			Vector2i getDepthImageSize(void);
			Vector2i getRGBImageSize(void);
		};
	}
}

