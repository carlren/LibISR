#ifndef __ISR_IMAGE__
#define __ISR_IMAGE__

#include <stdlib.h>
#include <string.h>

namespace LibISR
{
	namespace Objects
	{
		template <typename T>
		class ISRImage
		{
		public:
			int isAllocated;
			int width, height;

			T* data;

			ISRImage(int width, int height) 
			{
				this->width = width; this->height = height;
				data = (T*)malloc(sizeof(T) * width * height);

				this->Clear();

				isAllocated = true;
			}

			void Clear(unsigned char defaultValue = 0) { memset(data, defaultValue, width * height * sizeof(T));  }
			void Clear(int width, int height, unsigned char val) { this->width = width; this->height = height; memset(data, val, width * height * sizeof(T)); }

			void Free()
			{
				free(data);
				this->isAllocated = false;
			}

			~ISRImage() { if (isAllocated) this->Free(); }
		};
	}
}

#endif