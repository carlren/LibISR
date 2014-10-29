#ifndef __ISR_OPTIMIZATION__
#define __ISR_OPTIMIZATION__

#include <stdlib.h>

#include "LibISRDefine.h"
#include "MathUtils.h"
#include "ISRShape.h"
#include "ISRShapeUnion.h"

#include "IOUtil.h"

namespace LibISR
{
	namespace Objects
	{
		class ISROptimizationHelper
		{
		
		private:
			float *tmpHessian;
			float *LHessian;
		

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
				UnimindCHdecompose(this->tmpHessian, this->nDof, this->LHessian);
				UnimindCHsolveDecomposed(this->LHessian, this->nDof, this->step, this->finalStep);
				for (int i = 0; i < this->nDof; i++) this->finalStep[i] = -this->finalStep[i];
				return true;
			}
		};
	}
}

#endif