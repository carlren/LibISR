#pragma once

#include "../CoreISR/CoreISR.h"
#include "../Utils/NVTimer.h"

#include "ImageSourceEngine.h"


namespace LibISR
{
	namespace Engine
	{
		class UIEngine
		{
			static UIEngine* instance;

			enum MainLoopAction
			{
				PROCESS_PAUSED, PROCESS_FRAME, PROCESS_VIDEO, EXIT, SAVE_TO_DISK
			}mainLoopAction;

			ImageSourceEngine *imageSource;
			
			StopWatchInterface *timer;

			ISRCoreEngine *mainEngine;

		private: // For UI layout
			static const int NUM_WIN = 3;
			Vector4f winReg[NUM_WIN]; // (x1, y1, x2, y2)
			Vector2i winSize;
			uint textureId[NUM_WIN];
			ISRUChar4Image *outImage[NUM_WIN];
			
			int mouseState;
			Vector2i mouseLastClick;

			int currentFrameNo; bool isRecording;
		public:
			static UIEngine* Instance(void) {
				if (instance == NULL) instance = new UIEngine();
				return instance;
			}

			static void glutDisplayFunction();
			static void glutIdleFunction();
			static void glutKeyUpFunction(unsigned char key, int x, int y);

			const Vector2i & getWindowSize(void) const
			{
				return winSize;
			}

			float processedTime;
			int processedFrameNo;
			char *outFolder;
			bool needsRefresh;

			void Initialise(int & argc, char** argv, ImageSourceEngine *imageSource, ISRCoreEngine* mainEngine ,const char *outFolder);
			void Shutdown();

			void Run();
			void ProcessFrame();

			void GetScreenshot(ISRUChar4Image *dest) const;
			void SaveScreenshot(const char *filename) const;
		};
	}
}

