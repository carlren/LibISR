#ifndef __ISR_KINECT_CAP__
#define __ISR_KINECT_CAP__


#include <Windows.h>

#include "../Dependency/Kinect/inc/NuiApi.h"
#include "../Dependency/Kinect/inc/NuiImageCamera.h"
#include "../Dependency/Kinect/inc/NuiSensor.h"

#include "LibISRDefine.h"
#include "ISRImage.h"

namespace LibISR
{
	namespace Object
	{
		class ISRKinectCapture
		{
		private:
			HANDLE rgbStream;
			HANDLE depthStream;
			INuiSensor* sensor;

		public:
			
			int width;
			int height;

			ISRUChar4Image *colorFrame;
			ISRUShortImage *depthFrame;
			
			ISRKinectCapture()
			{
				this->width = 640;
				this->height = 480;

				colorFrame = new ISRUChar4Image(this->width,this->height);
				depthFrame = new ISRUShortImage(this->width,this->height);

				initKinect();
			}


			bool initKinect() 
			{
				// Get a working kinect sensor
				int numSensors;
				if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1) return false;
				if (NuiCreateSensorByIndex(0, &this->sensor) < 0) return false;

				// Initialize sensor
				this->sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);

				this->sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, // Depth camera or rgb camera?
					NUI_IMAGE_RESOLUTION_640x480,    // Image resolution
					0,		// Image stream flags, e.g. near mode
					2,		// Number of frames to buffer
					NULL,   // Event handle
					&rgbStream);

				this->sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, // Depth camera or rgb camera?
					NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
					NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE,         // Image stream flags, e.g. near mode // NUI_IMAGE_FRAME_FLAG_NONE for no flag
					2,        // Number of frames to buffer
					NULL,     // Event handle
					&depthStream);

				return this->sensor;
			}

			void getKinectRGBData(BYTE* dest) 
			{
				NUI_IMAGE_FRAME imageFrame;
				NUI_LOCKED_RECT LockedRect;
				if (sensor->NuiImageStreamGetNextFrame(rgbStream, 0, &imageFrame) < 0) return;
				INuiFrameTexture* texture = imageFrame.pFrameTexture;
				texture->LockRect(0, &LockedRect, NULL, 0);
				if (LockedRect.Pitch != 0)
				{
					const BYTE* curr = (const BYTE*) LockedRect.pBits;
					int size = LockedRect.size;
					memcpy(dest,curr,size);
				}
				texture->UnlockRect(0);
				sensor->NuiImageStreamReleaseFrame(rgbStream, &imageFrame);
			}

			void getKinectDepthData(USHORT* dest) 
			{
				NUI_IMAGE_FRAME imageFrame;
				NUI_LOCKED_RECT LockedRect;
				if (sensor->NuiImageStreamGetNextFrame(depthStream, 0, &imageFrame) < 0) return;
				INuiFrameTexture* texture = imageFrame.pFrameTexture;
				texture->LockRect(0, &LockedRect, NULL, 0);
				if (LockedRect.Pitch != 0) 
				{
					const USHORT* curr = (const USHORT*) LockedRect.pBits;
					const USHORT* dataEnd = curr + (width*height);

					while (curr < dataEnd) 
					{
						// Get depth in millimeters
						USHORT depth = NuiDepthPixelToDepth(*curr++);
						*dest++ = depth;
					}
				}
				texture->UnlockRect(0);
				sensor->NuiImageStreamReleaseFrame(depthStream, &imageFrame);
			}

			void pullColorFrame()
			{
				getKinectRGBData((BYTE*)colorFrame->data);
			}

			void pullDepthFrame()
			{
				getKinectDepthData((USHORT*)depthFrame->data);
			}
		};
	}
}

#endif