#pragma once

#include <string.h>

#include "../Utils/LibISRDefine.h"
#include <fstream>

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

			bool initialised;
			int noBins,dim;

			ISRHistogram(int nBins)
			{ 
				noBins = nBins;
				dim = noBins*noBins*noBins;

				data_normalised = new Vector2f[dim];
				data_notnormalised = new Vector2f[dim];
				posterior = new float[dim];

				this->clear();
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

			void buildHistogram(ISRUChar4Image *color, ISRUCharImage *mask)
			{
				int idx_mask;
				int ru, gu, bu;
				int pidx;

				float sumHistogramForeground = 0; 
				float sumHistogramBackground = 0;

				Vector4u *pixels = color->GetData(false);
				unsigned char* maskdata = mask->GetData(false);

				int height = mask->noDims.x;
				int width = mask->noDims.y;

				for (int j = 0; j < height; j++) for (int i = 0; i < width; i++)
				{
					int idx = i + j * width;

					ru = pixels[idx].r / noBins; 
					gu = pixels[idx].g / noBins; 
					bu = pixels[idx].b / noBins;
					pidx = ru*noBins*noBins + gu * noBins + bu; 

					switch (maskdata[idx])
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


			void clearPosterior() { for (int i = 0; i < dim; i++) posterior[i] = 0; }
			
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
			}
		};
	}
}

