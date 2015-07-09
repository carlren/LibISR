#include "ISRLowlevelEngine.h"


Vector4i LibISR::Engine::ISRLowlevelEngine::findBoundingBoxFromCurrentState(const Objects::ISRTrackingState* state, const Matrix3f& K, const Vector2i& imgsize)
{
	Vector3f crns[8];
	Vector3f *ipts = new Vector3f[state->numPoses() * 8];
	Vector4i bb(imgsize.x,imgsize.y,0,0);

	for (int i = -1, idx = 0; i <= 1; i += 2)
		for (int j = -1; j <= 1; j += 2)
			for (int k = -1; k <= 1; k += 2, idx++) 
				crns[idx] = Vector3f(i*0.1f, j*0.1f, k*0.1f);

	for (int i = 0, idx=0; i < state->numPoses(); i++) for (int j = 0; j < 8;j++, idx++)
	{
		Matrix4f H = state->getPose(i)->getH();
		ipts[idx] = K*(H*crns[j]);		ipts[idx].x /= ipts[idx].z;		ipts[idx].y /= ipts[idx].z;
		
		bb.x = ipts[idx].x < bb.x ? ipts[idx].x : bb.x;
		bb.y = ipts[idx].y < bb.y ? ipts[idx].y : bb.y;
		bb.z = ipts[idx].x > bb.z ? ipts[idx].x : bb.z;
		bb.w = ipts[idx].y > bb.w ? ipts[idx].y : bb.w;
	}

	bb.x = bb.x < 0 ? 0 : bb.x; 
	bb.y = bb.y < 0 ? 0 : bb.y;
	bb.z = bb.z > imgsize.x ? imgsize.x : bb.z;
	bb.w = bb.w > imgsize.y ? imgsize.y : bb.w;

	return bb;
}
