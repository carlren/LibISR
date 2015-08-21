// Copyright 2014-2015 Isis Innovation Limited and the authors of LibISR

#pragma once
#include "../LibISRDefine.h"
#include <fstream>
#include <string.h>

namespace LibISR
{
	namespace Objects
	{
		/**
		\brief
			color histogram as appearance model
			
			refactored: Jan/13/2015
		*/

		class ISRHistogram
		{
		public:
			
			Vector2f *data_notnormalised;
			Vector2f *data_normalised;

			float* posterior;
			float* posterior_device;

			bool initialised;
			int noBins,dim;
			
			bool useGPU;

			ISRHistogram(int nBins, bool usegpu=false)
			{ 
				useGPU = usegpu;
				noBins = nBins;
				dim = noBins*noBins*noBins;

				data_normalised = new Vector2f[dim];
				data_notnormalised = new Vector2f[dim];
				posterior = new float[dim];

				if (usegpu) ORcudaSafeCall(cudaMalloc((void**)&posterior_device, dim*sizeof(float)));

				this->clear();
			}

			float* getPosteriorHistogram(bool fromgpu = false)
			{ 
				if (fromgpu)
				{
					ORcudaSafeCall(cudaMemcpy(posterior_device, posterior, dim*sizeof(float), cudaMemcpyHostToDevice));
					return posterior_device;

				} else 
					return posterior;
			}

			void updateHistogram(ISRHistogram *newHist, float rf, float rb)
			{
				for (int i = 0; i < this->dim; i++)
				{
					this->data_normalised[i].x = this->data_normalised[i].x * (1 - rf) + newHist->data_normalised[i].x * rf;
					this->data_normalised[i].y = this->data_normalised[i].y * (1 - rb) + newHist->data_normalised[i].y * rb;
					this->posterior[i] = this->data_normalised[i].x / (this->data_normalised[i].x + this->data_normalised[i].y);
				}

			}

			template <class T>
			void buildHistogram(ORUtils::Image<T> *color, UChar4Image *mask)
			{
				int idx_mask;
				int ru, gu, bu;
				int pidx;

				float sumHistogramForeground = 0; 
				float sumHistogramBackground = 0;

				T *pixels = color->GetData(MEMORYDEVICE_CPU);
				Vector4u *maskdata = mask->GetData(MEMORYDEVICE_CPU);

				int height = mask->noDims.x;
				int width = mask->noDims.y;

				for (int j = 0; j < height; j++) for (int i = 0; i < width; i++)
				{
					int idx = i + j * width;

					ru = pixels[idx].r / noBins; 
					gu = pixels[idx].g / noBins; 
					bu = pixels[idx].b / noBins;

					pidx = ru*noBins*noBins + gu * noBins + bu; 

					int maskvalue = (maskdata[idx].x + maskdata[idx].y + maskdata[idx].z) / 3;

					switch (maskdata[idx].x)
					{
					case WHITE: case 254: 
						data_notnormalised[pidx].x++; sumHistogramForeground++; break; // white is forground
					case BLACK: case 1: 
						break; // black is far background
					default:
						data_notnormalised[pidx].y++; sumHistogramBackground++; break; // other colors are all immediat backgroud
					}

				}

				sumHistogramForeground = (sumHistogramForeground != 0) ? 1.0f / sumHistogramForeground : 0;
				sumHistogramBackground = (sumHistogramBackground != 0) ? 1.0f / sumHistogramBackground : 0;

				for (int i=0; i<this->dim; i++)
				{
					this->data_normalised[i].x = this->data_notnormalised[i].x * sumHistogramForeground + 0.0001f;
					this->data_normalised[i].y = this->data_notnormalised[i].y * sumHistogramBackground + 0.0001f;

					this->posterior[i] = this->data_normalised[i].x / (this->data_normalised[i].x + this->data_normalised[i].y);
					
				}

				this->initialised = true;

				if (useGPU) ORcudaSafeCall(cudaMemcpy(posterior_device, posterior, dim*sizeof(float), cudaMemcpyHostToDevice));
			}

			void buildHistogramFromLabeledRGBD(Float4Image *inimg)
			{
				int idx_mask;
				int ru, gu, bu;
				int pidx;

				float sumHistogramForeground = 0;
				float sumHistogramBackground = 0;

				Vector4f *pixels = inimg->GetData(MEMORYDEVICE_CPU);

				int height = inimg->noDims.y;
				int width = inimg->noDims.x;

				for (int j = 0; j < height; j++) for (int i = 0; i < width; i++)
				{
					int idx = i + j * width;

					if (pixels[idx].w>=HIST_USELESS_PIXEL) continue;
					
					ru = pixels[idx].r / noBins;
					gu = pixels[idx].g / noBins;
					bu = pixels[idx].b / noBins;

					pidx = ru*noBins*noBins + gu * noBins + bu;

					switch ((int)(pixels[idx].w))
					{
					case HIST_FG_PIXEL:
						data_notnormalised[pidx].x++; sumHistogramForeground++; break; // white is forground
					case HIST_BG_PIXEL:
						data_notnormalised[pidx].y++; sumHistogramBackground++; break; // other colors are all immediat backgroud
					default: break;
					}
				}

				sumHistogramForeground = (sumHistogramForeground != 0) ? 1.0f / sumHistogramForeground : 0;
				sumHistogramBackground = (sumHistogramBackground != 0) ? 1.0f / sumHistogramBackground : 0;

				for (int i = 0; i < this->dim; i++)
				{
					this->data_normalised[i].x = this->data_notnormalised[i].x * sumHistogramForeground + 0.0001f;
					this->data_normalised[i].y = this->data_notnormalised[i].y * sumHistogramBackground + 0.0001f;

					this->posterior[i] = this->data_normalised[i].x / (this->data_normalised[i].x + this->data_normalised[i].y);
				}

				this->initialised = true;
			}


			void buildHistogramFromLabeledRGBD(Float4Image *inimg, const Vector4i& bb)
			{
				int idx_mask;
				int ru, gu, bu;
				int pidx;

				float sumHistogramForeground = 0;
				float sumHistogramBackground = 0;

				Vector4f *pixels = inimg->GetData(MEMORYDEVICE_CPU);

				int height = inimg->noDims.y;
				int width = inimg->noDims.x;

				for (int j = bb.y; j < bb.w; j++) for (int i = bb.x; i < bb.z; i++)
				{
					int idx = i + j * width;

					if (pixels[idx].w >= HIST_USELESS_PIXEL) continue;

					ru = pixels[idx].r / noBins;
					gu = pixels[idx].g / noBins;
					bu = pixels[idx].b / noBins;

					pidx = ru*noBins*noBins + gu * noBins + bu;

					switch ((int)(pixels[idx].w))
					{
					case HIST_FG_PIXEL:
						data_notnormalised[pidx].x++; sumHistogramForeground++; break; // white is forground
					case HIST_BG_PIXEL:
						data_notnormalised[pidx].y++; sumHistogramBackground++; break; // other colors are all immediat backgroud
					default: break;
					}
				}

				sumHistogramForeground = (sumHistogramForeground != 0) ? 1.0f / sumHistogramForeground : 0;
				sumHistogramBackground = (sumHistogramBackground != 0) ? 1.0f / sumHistogramBackground : 0;

				for (int i = 0; i < this->dim; i++)
				{
					this->data_normalised[i].x = this->data_notnormalised[i].x * sumHistogramForeground + 0.0001f;
					this->data_normalised[i].y = this->data_notnormalised[i].y * sumHistogramBackground + 0.0001f;

					this->posterior[i] = this->data_normalised[i].x / (this->data_normalised[i].x + this->data_normalised[i].y);
				}

				this->initialised = true;
			}


			void updateHistogramFromLabeledRGBD(Float4Image *inimg, float rf, float rb)
			{
				ISRHistogram* tmphist = new ISRHistogram(this->noBins);
				buildHistogramFromLabeledRGBD(inimg);
				updateHistogram(tmphist, rf, rb);
			}

			void updateHistogramFromLabeledRGBD(Float4Image *inimg, float rf, float rb, const Vector4i& bb)
			{
				ISRHistogram* tmphist = new ISRHistogram(this->noBins);
				buildHistogramFromLabeledRGBD(inimg,bb);
				updateHistogram(tmphist, rf, rb);
			}

			void loadPosteriorFromFile(const char* fileName)
			{
				std::ifstream infile;
				infile.open(fileName, std::ios::in);
				for (int i = 0; i < dim;i++) infile >> posterior[i];
				infile.close();	
			}

			void clearNormalised() 
			{
				for (int i = 0; i < dim;i++)
				{
					data_normalised[i].x = 0;
					data_normalised[i].y = 0;
				}				
			}

			void clearNotNormalised() 
			{
				for (int i = 0; i < dim; i++)
				{
					data_notnormalised[i].x = 0;
					data_notnormalised[i].y = 0;
				}
			}

			void clearPosterior() 
			{ 
				for (int i = 0; i < dim; i++) posterior[i] = 0; 
				if (useGPU) ORcudaSafeCall(cudaMemcpy(posterior_device, posterior, dim*sizeof(float), cudaMemcpyHostToDevice));
			}
			
			void clear()
			{
				clearNormalised();
				clearNotNormalised();
				clearPosterior();
				this->initialised = false;
			}		

			~ISRHistogram(void) 
			{
				delete data_normalised;
				delete data_notnormalised;
				delete posterior;
				if (useGPU) ORcudaSafeCall(cudaFree(posterior_device));
			}
		};
	}
}

