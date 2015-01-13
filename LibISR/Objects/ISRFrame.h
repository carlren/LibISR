#pragma once

#include <stdio.h>
#include <stdlib.h>

#include "../Utils/LibISRDefine.h"

#include "../Objects/ISRHistogram.h"
#include "../Objects/ISRView.h"

namespace LibISR
{
	namespace Objects
	{
		/** \brief
		Represents intermediat data for a RGB-D frame, 
		including occlusion map, per-pixel forground map
		*/
		class ISRFrame
		{
		public:

			int height;
			int width;

			Vector2i d_size;

			ISRView* view;

			ISRHistogram *histogram;

			ISRUCharImage *occMap;
			ISRFloatImage *pfImage;
			ISRFloatImage *idxImage;

			int samplingRate;

			ISRFrame(const ISRCalib &calib, Vector2i rgb_size, Vector2i d_size, bool  useGPU = false)
			{
				this->d_size = d_size;
				this->occMap = new ISRUCharImage(d_size, useGPU);
				this->occMap->Clear(1);

				this->pfImage = new ISRFloatImage(d_size,useGPU);
				this->idxImage = new ISRFloatImage(d_size, useGPU);
				this->view = new ISRView(calib, rgb_size, d_size, useGPU);
				
				this->samplingRate = 1;
			}


			~ISRFrame()
			{
				delete this->occMap;
				delete this->pfImage;
				delete this->idxImage;
				delete this->view;

			}

			//void loadColorAndDepthFrame(ISRUChar4Image* inputColor, ISRUShortImage* inputDepth, bool _fromKinect)
			//{
			//	bool visualizeDepth = true;

			//	memcpy(this->colorImage->data, inputColor->data, this->width*this->height*sizeof(Vector4u));
			//	memcpy(this->rawDepthImage->data, inputDepth->data, this->width*this->height*sizeof(USHORT));


			//	for (int j = 0; j < this->height; j++) for(int i = 0; i < this->width; i++)
			//	{
			//		int idx = j*this->width + i;
			//		
			//		USHORT rawDepth = this->rawDepthImage->data[idx];

			//		float realDepth = rawDepth==65535 ? 0 : ((float) rawDepth) / 1000.0f;
			//		this->depthImage->data[idx] = realDepth;

			//		if (visualizeDepth)
			//		{
			//			int intensity = (int)(realDepth*1000)%256;
			//			displayDepthImage->data[idx].x = intensity;
			//			displayDepthImage->data[idx].y = intensity;
			//			displayDepthImage->data[idx].z = intensity;
			//		}

			//		// align color and depth
			//		float* rH = extrinsics->rH;
			//		float* rT = extrinsics->rT;

			//		if (realDepth!=0)
			//		{
			//			float x = (float)i * realDepth;
			//			float y = (float)j * realDepth;

			//			float fIX = rH[0] * x + rH[1] * y + rH[2] * realDepth + rT[0];
			//			float fIY = rH[3] * x + rH[4] * y + rH[5] * realDepth + rT[1];
			//			float fIZ = rH[6] * x + rH[7] * y + rH[8] * realDepth + rT[2];

			//			int iX = (int)(fIX / fIZ);
			//			int iY = (int)(fIY / fIZ);

			//			int imgIdx = iY*this->width + iX;

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

			//	}

			//}


		};
	}
}

