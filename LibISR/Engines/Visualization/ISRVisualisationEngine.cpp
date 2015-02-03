#include "ISRVisualisationEngine.h"

using namespace LibISR::Engine;
using namespace LibISR::Objects;

void LibISR::Engine::ISRVisualisationEngine::updateMinmaxmImage(ISRFloat2Image* minmaximg, const Matrix4f& H, const Matrix3f& K, const Vector2i& imgsize)
{
	Vector3f crns[8], ipts[8];
	Vector4i bb(imgsize.x, imgsize.y, 0, 0);
	float maxz = 0.0f, minz = 99999.9f;

	for (int i = -1, idx = 0; i <= 1; i += 2)
		for (int j = -1; j <= 1; j += 2)
			for (int k = -1; k <= 1; k += 2, idx++)
				crns[idx] = Vector3f(i*0.1f, j*0.1f, k*0.1f);

	for (int idx = 0; idx < 8; idx++)
	{
		ipts[idx] = K*(H*crns[idx]);		
		ipts[idx].x /= ipts[idx].z;		
		ipts[idx].y /= ipts[idx].z;

		bb.x = ipts[idx].x < bb.x ? ipts[idx].x : bb.x;
		bb.y = ipts[idx].y < bb.y ? ipts[idx].y : bb.y;
		bb.z = ipts[idx].x > bb.z ? ipts[idx].x : bb.z;
		bb.w = ipts[idx].y > bb.w ? ipts[idx].y : bb.w;

		maxz = ipts[idx].z > maxz ? ipts[idx].z : maxz;
		minz = ipts[idx].z < minz ? ipts[idx].z : minz;
	}

	bb.x = bb.x < 0 ? 0 : bb.x;
	bb.y = bb.y < 0 ? 0 : bb.y;
	bb.z = bb.z > imgsize.x ? imgsize.x : bb.z;
	bb.w = bb.w < imgsize.y ? imgsize.y : bb.w;
	
	
	Vector2f* minmax_ptr = minmaximg->GetData(false);

	for (int i = 0; i < imgsize.y; i++)  for (int j = 0; j < imgsize.x;j++)
	{
		if (j>=bb.x && j<bb.z && i>=bb.y && i<bb.w)
		{
			minmax_ptr[i*imgsize.x + j].x = minz;
			minmax_ptr[i*imgsize.x + j].y = maxz;
		}
		else
		{
			minmax_ptr[i*imgsize.x + j].x = -1;
			minmax_ptr[i*imgsize.x + j].y = -1;
		}
	}
}

