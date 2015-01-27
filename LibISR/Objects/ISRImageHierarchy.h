#pragma once

#include <stdio.h>
#include <stdlib.h>

#include "../Utils/LibISRDefine.h"

#include "../Objects/ISRView.h"


namespace LibISR
{
	namespace Objects
	{
		class ISRImageHierarchy
		{
		public:
			int noLevels;
			
			typedef struct 
			{ 
				ISRFloat4Image* rgbd;
				Vector4f intrinsic;
				Vector4i boundingbox;
			} ImageLevel;

			ImageLevel *levels;

			ISRImageHierarchy(Vector2i imgsize, int noimagelevel, bool usegpu)
			{
				noLevels = noimagelevel;
				
				levels = new ImageLevel[noLevels];

				Vector2i currentsize = imgsize;

				for (int i = 0; i < noLevels;i++)
				{
					levels[i].rgbd = new ISRFloat4Image(currentsize, usegpu);
					currentsize /= 2;
				}
			}

			void UpdateHostFromDevice()
			{
				for (int i = 0; i < noLevels; i++) this->levels[i].rgbd->UpdateHostFromDevice();
			}

			void UpdateDeviceFromHost()
			{
				for (int i = 0; i < noLevels; i++) this->levels[i].rgbd->UpdateDeviceFromHost();
			}

			~ISRImageHierarchy(void)
			{
				for (int i = 0; i < noLevels; i++) { delete levels[i].rgbd; }
				delete[] levels;
			}

			ISRImageHierarchy(const ISRImageHierarchy&);
			ISRImageHierarchy& operator=(const ISRImageHierarchy&);

		};
	}
}