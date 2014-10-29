#pragma once

#include <string.h>

#include "..//Utils//LibISRDefine.h"
#include "..//Utils//MathUtils.h"
#include "..//Objects//ISRImage.h"


namespace CoreISR
{
	namespace Objects
	{
		class ISRHistogram
		{
		public:
			
			Vector2f *data_notnormalised;
			Vector2f *data_normalised;

			float* posterior;

			bool initialised;
			int noBins,dim;

			ISRHistogram(int noBins)
			{ 
				this->noBins = noBins;
				this->dim = noBins*noBins*noBins;

				this->data_notnormalised = (Vector2f*)malloc(sizeof(Vector2f) * this->dim);
				this->data_normalised = (Vector2f*)malloc(sizeof(Vector2f) * this->dim);
				this->posterior=(float*)malloc(sizeof(float) * this->dim);

				this->Clear();
			}

			void UpdateHistogram(ISRHistogram *newHist, float rf, float rb)
			{
				for (int i = 0; i < this->dim; i++)
				{
					this->data_normalised[i].x = this->data_normalised[i].x * (1 - rf) + newHist->data_normalised[i].x * rf;
					this->data_normalised[i].y = this->data_normalised[i].y * (1 - rb) + newHist->data_normalised[i].y * rb;
					this->posterior[i] = this->data_normalised[i].x / (this->data_normalised[i].x + this->data_normalised[i].y);
				}

			}


			void BuildHistogram(ISRUChar4Image *color, ISRUCharImage *mask)
			{
				int idx_mask;
				int ru, gu, bu;
				int pidx;

				float sumHistogramForeground = 0; 
				float sumHistogramBackground = 0;

				Vector4u *pixels = (Vector4u*) color->data;

				for (int j = 0; j < mask->height; j++) for (int i = 0; i < mask->width; i++)
				{
					int idx = i + j * mask->width;

					ru = pixels[idx].r / noBins; 
					gu = pixels[idx].g / noBins; 
					bu = pixels[idx].b / noBins;
					pidx = ru*noBins*noBins + gu * noBins + bu; 

					switch (mask->data[idx])
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
			

			void ClearNormalised() { memset(this->data_normalised, 0, sizeof(Vector2f) * this->dim);  }
			void ClearNotNormalised() { memset(this->data_notnormalised, 0, sizeof(Vector2f) * this->dim); }
			void ClearPosterior() {memset(this->posterior,0,sizeof(float) * this->dim);}
			
			void Clear()
			{
				ClearNormalised();
				ClearNotNormalised();
				ClearPosterior();
				this->initialised = false;
			}		

			~ISRHistogram(void) 
			{
				free(data_normalised);
				free(data_notnormalised);
				free(posterior);
			}
		};
	}
}

