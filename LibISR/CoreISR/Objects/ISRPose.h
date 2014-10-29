#ifndef __ISR_POSE__
#define __ISR_POSE__

#include "..//Utils//LibISRDefine.h"
#include "..//Utils//MathUtils.h"

namespace LibISR
{
	namespace Objects
	{
		class ISRPose
		{
		public:


			float scale;
			Vector3f *r; // angle
			Vector3f *mrp; // mrp parameter
			Matrix3f *R; // c->o rotation matrix
			Vector3f *t; // c->o transformation
			
			Matrix4f *H;// this is c->o
			Matrix4f *invH;// this is o->c

		
			ISRPose(void) 
			{ 
				r = (Vector3f*)malloc(sizeof(Vector3f));
				mrp = (Vector3f*)malloc(sizeof(Vector3f));
				R = (Matrix3f*)malloc(sizeof(Matrix3f));
                t = (Vector3f*)malloc(sizeof(Vector3f)); 
				H = (Matrix4f*)malloc(sizeof(Matrix4f)); 
				invH = (Matrix4f*)malloc(sizeof(Matrix4f)); 
			} 

			~ISRPose(void) 
			{ 
				free(r);
				free(mrp);
				free(R);
				free(t);
				free(H); 
				free(invH);
			}


			void SetFrom(ISRPose *pose)
			{
				SetVectorFromVector_3(this->r, pose->r);
				SetVectorFromVector_3(this->mrp, pose->mrp);
				SetMatrixFromMatrix_3(this->R, pose->R);
				SetVectorFromVector_3(this->t, pose->t);
				
				SetMatrixFromMatrix_4(this->H, pose->H);
				SetMatrixFromMatrix_4(this->invH, pose->invH);

				this->scale = pose->scale;
			}

			// T,R, the rotation is already in MRP
			void SetFromStep(float* step)
			{
				for (int i = 0; i < 3; i++) this->t->v[i] = step[i];
				for (int i = 0; i < 3; i++) this->mrp->v[i] = step[i+3];
				GetRotationMatrixFromMRP(this->R, this->mrp);
				UpdateHfromCurrentRT();

			}

			void UpdateFromStep(float* step)
			{
				Matrix3f tmpRR;
				GetRotationMatrixFromMRP(&tmpRR, (Vector3f*)&step[3]);

				UpdateRT(this->R, this->t, tmpRR.m, step);
				UpdateHfromCurrentRT();
			}

			void UpdateHfromCurrentRT()
			{
				int idx = 0;
				for (int i = 0; i<3; i++){
					this->H->m[i * 4 + 3] = this->t->v[i];
					this->H->m[12 + i] = 0;
					for (int j = 0; j<3; j++, idx++){
						this->H->m[i * 4 + j] = this->R->m[idx];
					}
				}
				this->H->m33 = 1;
				MatrixInvert_4(this->invH, this->H);
			}
		};
	}
}

#endif