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
			ISRFloat4Image **levels;

			ISRImageHierarchy(Vector2i imgsize, int noimagelevel, bool usegpu)
			{
				noLevels = noimagelevel;
				levels = new ISRFloat4Image *[noLevels];

				Vector2i currentsize = imgsize;

				for (int i = 0; i < noLevels;i++)
				{
					levels[i] = new ISRFloat4Image(currentsize, usegpu);
					currentsize /= 2;
				}
			}

			void UpdateHostFromDevice()
			{
				for (int i = 0; i < noLevels; i++) this->levels[i]->UpdateHostFromDevice();
			}

			void UpdateDeviceFromHost()
			{
				for (int i = 0; i < noLevels; i++) this->levels[i]->UpdateDeviceFromHost();
			}

			~ISRImageHierarchy(void)
			{
				for (int i = 0; i < noLevels; i++) delete levels[i];
				delete[] levels;
			}

			ISRImageHierarchy(const ISRImageHierarchy&);
			ISRImageHierarchy& operator=(const ISRImageHierarchy&);

		};
	}
}