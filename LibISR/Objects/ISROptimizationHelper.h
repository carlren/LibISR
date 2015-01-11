#pragma once

#include "../Utils/LibISRDefine.h"
#include "../Utils/ISRMath.h"

#include "../Objects/ISRShape.h"
#include "../Objects/ISRShapeUnion.h"


namespace CoreISR
{
	namespace Objects
	{
		class ISROptimizationHelper
		{
		
		private:
			float *tmpHessian;
			float *LHessian;
		
		    void CHsolveDecomposed(const float *A, int size, const float *b, float *x, int num = 1, int ldb = -1)
			{
				int i, k;
				float sum;
				// Solve L y = b, storing y in x.
				for (int eq = 0; eq<num; ++eq) {
					for (i = 0; i<size; i++) {
						for (sum = b[i + eq*ldb], k = i - 1; k >= 0; k--) sum -= A[i*size + k] * x[k + eq*ldb];
						x[i + eq*ldb] = sum / A[i*size + i];
					}
				}
				// Solve LT x = y
				for (int eq = 0; eq<num; ++eq) {
					for (i = size - 1; i >= 0; i--) {
						for (sum = x[i + eq*ldb], k = i + 1; k<size; k++) sum -= A[k*size + i] * x[k + eq*ldb];
						x[i + eq*ldb] = sum / A[i*size + i];
					}
				}
			}

		   int CHdecompose(const float *input, int size, float *output)
			{
				int i, j, k;
				float sum;
				bool divide;
				int ret = 0;
				for (i = 0; i<size; i++) {
					divide = true;
					for (j = 0; j<i; j++) output[j*size + i] = 0.0;
					for (; j<size; j++) {
						sum = input[i*size + j];
						for (k = i - 1; k >= 0; k--) sum -= output[i*size + k] * output[j*size + k];
						if (i == j) {
							/* The following applies if A, with rounding errors, is not positive definite.*/
							if (almostZero(sum)) {
								output[i*size + i] = 0.0;
								divide = false;
							}
							else if (sum<0.0) {
								output[i*size + i] = 0.0;
								divide = false;
								printf("choldc failed: sqrt(%f)\n", sum);
								ret = -1;
							}
							else {
								output[i*size + i] = sqrt(sum);
							}
						}
						else {
							if (!divide) output[j*size + i] = 0.0;
							else output[j*size + i] = sum / output[i*size + i];
						}
					}
				}
				return ret;
			}

		public:
			float *Hessian;
			float *step;
			float *finalStep;
			int nObj;
			int nDof;
			int nHessianSize;
			

			ISROptimizationHelper(int nObjCount)
			{
				this->nObj = nObjCount;
				this->nDof = nObjCount * 6;
				this->nHessianSize = this->nDof*this->nDof;

				this->step = (float*)malloc(this->nDof*sizeof(float));
				this->finalStep = (float*)malloc(this->nDof*sizeof(float));
				this->Hessian = (float*)malloc(this->nHessianSize*sizeof(float));
				this->tmpHessian = (float*)malloc(this->nHessianSize*sizeof(float));
				this->LHessian = (float*)malloc(this->nHessianSize*sizeof(float));
			}

			void Clear()
			{
				memset(this->step, 0, this->nDof*sizeof(float));
				memset(this->finalStep, 0, this->nDof*sizeof(float));
				memset(this->Hessian, 0,this->nHessianSize*sizeof(float));
				memset(this->tmpHessian,0,this->nHessianSize*sizeof(float));
				memset(this->LHessian,0,this->nHessianSize*sizeof(float));
			}
			

			~ISROptimizationHelper() 
			{
				free(this->finalStep);
				free(this->step);
				free(this->Hessian);
				free(this->tmpHessian);
				free(this->LHessian);
			}

			bool SolveLM(float lambda)
			{
				memcpy(this->tmpHessian, this->Hessian, this->nHessianSize * sizeof(float));
				for (int i = 0; i < this->nHessianSize; i += (this->nDof + 1)) this->tmpHessian[i] += this->tmpHessian[i] * lambda + 1;
				CHdecompose(this->tmpHessian, this->nDof, this->LHessian);
				CHsolveDecomposed(this->LHessian, this->nDof, this->step, this->finalStep);
				for (int i = 0; i < this->nDof; i++) this->finalStep[i] = -this->finalStep[i];
				return true;
			}
		};
	}
}

