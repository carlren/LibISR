#pragma once

#include <stdio.h>
#include <stdlib.h>

#include "..//Utils//LibISRDefine.h"
#include "..//Utils//MathUtils.h"

#include "..//Objects//ISRHistogram.h"
#include "..//Objects//ISRView.h"

using namespace CoreISR::Objects;


namespace CoreISR
{
	namespace Objects
	{

		class ISRFrame
		{
		public:

			int height;
			int width;

			ISRView* view;

			ISRUChar4Image *displayDepthImage;
			ISRUChar4Image *renderingImage;
			ISRUCharImage *occMap;

			ISRFloatImage *pfImage;
			ISRFloatImage *idxImage;

			ISRHistogram* histogram;
			
			int samplingRate;


			ISRFrame(int w, int h)
			{
				this->width = w;
				this->height = h;
				this->samplingRate = 1;

				this->colorImage = new ISRUChar4Image(w,h);
				this->rawDepthImage = new ISRUShortImage(w,h);
				this->displayDepthImage = new ISRUChar4Image(w,h);
				this->renderingImage = new ISRUChar4Image(w,h);
				this->depthImage = new ISRFloatImage(w,h);
				this->alignedColorImage = new ISRUChar4Image(w,h);
				this->occMap = new ISRUCharImage(w, h);
				this->occMap->Clear(1);
				
				this->pfImage = new ISRFloatImage(w, h);
				this->idxImage = new ISRFloatImage(w, h);

				this->histogram = new ISRHistogram(HISTOGRAM_BIN);


				float A[9] = {592.82911f, 0.0f, 321.18620f,
									 0.0f, 596.14642f, 236.39559f,
									 0.0f, 0.0f, 1.0f};
				float rH[9] = { 1.44222043401605f, 0.0224968194843458f, 60.8747851772631f,
					- 0.0207371618019255f, 1.46227330363573f, 48.2429634968192f,
					- 3.09022577581801e-05f, 5.63097673183537e-05f, 1.63774981727871f };
				float rT[3] = { 11.6739429499538f, -8.81340702835841f, -0.0311516255635917f};
	
				this->intrinsics = new ISRIntrinsics();
				this->extrinsics = new ISRExtrinsics();

				
				this->extrinsics->SetFrom(rH,rT);

			}


			void loadColorAndDepthFrame(ISRUChar4Image* inputColor, ISRUShortImage* inputDepth, bool _fromKinect)
			{
				bool visualizeDepth = true;

				memcpy(this->colorImage->data, inputColor->data, this->width*this->height*sizeof(Vector4u));
				memcpy(this->rawDepthImage->data, inputDepth->data, this->width*this->height*sizeof(USHORT));


				for (int j = 0; j < this->height; j++) for(int i = 0; i < this->width; i++)
				{
					int idx = j*this->width + i;
					
					USHORT rawDepth = this->rawDepthImage->data[idx];

					float realDepth = rawDepth==65535 ? 0 : ((float) rawDepth) / 1000.0f;
					this->depthImage->data[idx] = realDepth;

					if (visualizeDepth)
					{
						int intensity = (int)(realDepth*1000)%256;
						displayDepthImage->data[idx].x = intensity;
						displayDepthImage->data[idx].y = intensity;
						displayDepthImage->data[idx].z = intensity;
					}

					// align color and depth
					float* rH = extrinsics->rH;
					float* rT = extrinsics->rT;

					if (realDepth!=0)
					{
						float x = (float)i * realDepth;
						float y = (float)j * realDepth;

						float fIX = rH[0] * x + rH[1] * y + rH[2] * realDepth + rT[0];
						float fIY = rH[3] * x + rH[4] * y + rH[5] * realDepth + rT[1];
						float fIZ = rH[6] * x + rH[7] * y + rH[8] * realDepth + rT[2];

						int iX = (int)(fIX / fIZ);
						int iY = (int)(fIY / fIZ);

						int imgIdx = iY*this->width + iX;

						if (iX >= 0 && iX < this->width && iY >= 0 && iY < this->height)
						{
							this->alignedColorImage->data[idx].x = this->colorImage->data[imgIdx].x;
							this->alignedColorImage->data[idx].y = this->colorImage->data[imgIdx].y;
							this->alignedColorImage->data[idx].z = this->colorImage->data[imgIdx].z;
						}
						
					}
					else
					{
						this->alignedColorImage->data[idx].x = 0;
						this->alignedColorImage->data[idx].y = 255;
						this->alignedColorImage->data[idx].z = 0;
					}

				}

			}

			~ISRFrame()
			{
				delete this->colorImage;
				delete this->rawDepthImage;
				delete this->displayDepthImage;
				delete this->renderingImage;
				delete this->depthImage;
				delete this->alignedColorImage;
				delete this->occMap;
				delete this->pfImage;
				delete this->idxImage;
			}
		};
	}
}

