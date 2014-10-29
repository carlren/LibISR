#pragma once

#include <stdio.h>

#include "..//Utils//LibISRDefine.h"
#include "..//Utils//MathUtils.h"
#include "..//Objects//ISRPose.h"
#include "..//Objects//ISRPoints.h"

using namespace CoreISR::Objects;

namespace CoreISR
{
	namespace Objects
	{
		class ISRShape
		{
		private:
			float heavisideScale;

		public:
			ISRPose* pose;
			int objectId;
			
			float *model_shape; // shape
			float *model_dt; // SDF
			float *delta_dt;
			float *heaviside_dt;
			float *d_delta_dt;
			float *d_heaviside_dt;

			Vector3f *d_dt;

			Vector3d volSize;
			int volSizeP3; 
			
			bool hasPose;
			bool hasShape;
			bool hasPts;
			bool hasJacobian;

			float *pfList;
			ISRPoints *unprojectedPts;
			ISRPoints *tmpUnprojectPts;
			
			ISRPoints *jacobianT;
			ISRPoints *jacobianR;

			ISRShape(Vector3d volSize,float heavisideScale, int objectId)
			{

				this->objectId = objectId;
				this->volSize=volSize;
				this->volSizeP3 = volSize.x*volSize.y*volSize.z;
				this->heavisideScale = heavisideScale;

				this->hasPose = false;
				this->hasShape = false;
				this->hasPts = false;
				this->hasJacobian = false;

				this->model_shape = (float*)malloc(sizeof(float) * volSizeP3);
				this->model_dt = (float*)malloc(sizeof(float) * volSizeP3);
				this->delta_dt = (float*)malloc(sizeof(float)* volSizeP3);
				this->heaviside_dt = (float*)malloc(sizeof(float)* volSizeP3);
				this->d_delta_dt = (float*)malloc(sizeof(float)* volSizeP3);
				this->d_heaviside_dt = (float*)malloc(sizeof(float)* volSizeP3);
				this->d_dt = (Vector3f*)malloc(sizeof(Vector3f)* volSizeP3);

				this->unprojectedPts = new ISRPoints(MAX_IMG_PTS);
				this->tmpUnprojectPts = new ISRPoints(MAX_IMG_PTS);
				this->jacobianR = new ISRPoints(MAX_IMG_PTS);
				this->jacobianT = new ISRPoints(MAX_IMG_PTS);

				pose =  new ISRPose();
			}

			void LoadShapeFromFile(char* fileName)
			{
				FILE* f;
				
				float *volume = (float*)malloc(sizeof(float) * this->volSizeP3);
				f = fopen(fileName, "rb");
				fread(volume, sizeof(float) * this->volSizeP3, 1, f); 
				fclose(f);

				float exp_dt, deto, dbase;

				for (int i = 0; i<this->volSizeP3; i++)
				{

					model_dt[i] = volume[i];
					model_shape[i] = (float)(volume[i] < 0);

					exp_dt = expf( - model_dt[i] / heavisideScale);
					deto = exp_dt + 1.0f;
					dbase = exp_dt / (deto * deto);

					heaviside_dt[i] = 1.0f / deto;
					d_heaviside_dt[i] = dbase / heavisideScale;
					delta_dt[i] = 4.0f * dbase;
					d_delta_dt[i] = 4.0f * expf(-model_dt[i]) / (deto * deto* deto) - 2 * dbase;
				}

				for (int z = 1; z < volSize.z - 1; z++) for (int y = 1; y < volSize.y - 1; y++) for (int x = 1; x < volSize.x - 1; x++) 
				{
					Vector3f dphi;

					dphi.x = 0.5f * (VOLVAL(this->model_dt, x + 1, y, z) - VOLVAL(this->model_dt, x - 1, y, z));
					dphi.y = 0.5f * (VOLVAL(this->model_dt, x, y + 1, z) - VOLVAL(this->model_dt, x, y - 1, z));
					dphi.z = 0.5f * (VOLVAL(this->model_dt, x, y, z + 1) - VOLVAL(this->model_dt, x, y, z - 1));

					this->d_dt[((z)* DT_VOL_SIZE + (y)) * DT_VOL_SIZE + (x)] = dphi;
				}

				this->hasShape = true;

			}

			float GetVolValue(Vector3f pt, float* vol)
			{
				int x = pt.x * VOL_SCALE + volSize.x / 2 - 1;
				int y = pt.y * VOL_SCALE + volSize.y / 2 - 1;
				int z = pt.z * VOL_SCALE + volSize.z / 2 - 1;

				if (x > 0 && x < volSize.x-1 && y > 0 && y < volSize.y-1 && z > 0 && z < volSize.z-1)
					return VOLVAL(vol, x, y, z);
				return 121.0f; // max dt for 200*200*200 vol
			}

			Vector3f GetVolGradient(Vector3f pt, Vector3f* vol)
			{
				Vector3f emptyRet; emptyRet.x = 0; emptyRet.y = 0; emptyRet.z = 0;
				int x = pt.x * VOL_SCALE + volSize.x / 2 - 1;
				int y = pt.y * VOL_SCALE + volSize.y / 2 - 1;
				int z = pt.z * VOL_SCALE + volSize.z / 2 - 1;

				if (x > 0 && x < volSize.x-1 && y > 0 && y < volSize.y-1 && z > 0 && z < volSize.z-1)
					return VOLVAL(vol, x, y, z);
				return emptyRet;
			}

			~ISRShape()
			{ 
				free(this->model_dt);
				free(this->delta_dt);
				free(this->heaviside_dt);
				free(this->d_heaviside_dt);
				free(this->d_delta_dt);
				free(this->d_dt);

				delete this->unprojectedPts;
				delete this->tmpUnprojectPts;
				delete this->jacobianR;
				delete this->jacobianT;
			}
		};
	}
}
