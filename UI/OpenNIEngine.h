#pragma once

#include "ImageSourceEngine.h"

#pragma comment(lib, "OpenNI2")

namespace LibISRUtils
{
	class OpenNIEngine : public ImageSourceEngine
	{
	private:
		class PrivateData;
		PrivateData *data;
		Vector2i imageSize_rgb, imageSize_d;
		bool colorAvailable, depthAvailable;
	public:
		OpenNIEngine(const char *calibFilename, const char *deviceURI = NULL, const bool useInternalCalibration = false,
			Vector2i imageSize_rgb = Vector2i(640, 480), Vector2i imageSize_d = Vector2i(640, 480));

		~OpenNIEngine();

		bool hasMoreImages(void);
		void getImages(LibISR::Objects::ISRView *out);
		Vector2i getDepthImageSize(void);
		Vector2i getRGBImageSize(void);
	};
}

