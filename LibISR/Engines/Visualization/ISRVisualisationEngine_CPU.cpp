#include "ISRVisualisationEngine_CPU.h"
#include "ISRVisualisationEngine_DA.h"

#include "../../../LibISRUtils/IOUtil.h"

using namespace LibISR::Engine;
using namespace LibISR::Objects;


void LibISR::Engine::ISRVisualisationEngine_CPU::renderObject(Objects::ISRVisualisationState* rendering, const Matrix4f& invH, const Objects::ISRShape_ptr shape, const Vector4f& intrinsic)
{
	const float *voxelData = shape->getSDFVoxel();
	Vector4u *outimage = rendering->outputImage->GetData(false);
	Vector2f *minmaximg = rendering->minmaxImage->GetData(false);

	Vector2i imgSize = rendering->outputImage->noDims;
	float mu = 0.5;
	Vector3f lightSource = -Vector3f(invH.getColumn(2));

	Vector4f invIntrinsic; 
	invIntrinsic.x = 1 / intrinsic.x; invIntrinsic.y = 1 / intrinsic.y;
	invIntrinsic.z = -intrinsic.z*invIntrinsic.x; invIntrinsic.w = -intrinsic.w*invIntrinsic.y;

	float one_on_top_of_maxVoxelRange = 1 / sqrtf(DT_VOL_SIZE*DT_VOL_SIZE + DT_VOL_SIZE*DT_VOL_SIZE + DT_VOL_SIZE*DT_VOL_SIZE);

#pragma omp parallel 
	{
#pragma omp for
		for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		{
			raycastAndRender(outimage, x, y, imgSize, voxelData, invH, invIntrinsic, minmaximg, lightSource, one_on_top_of_maxVoxelRange);
		}
	}
}

void LibISR::Engine::ISRVisualisationEngine_CPU::renderDepth(ISRUShortImage* renderedDepth, Objects::ISRVisualisationState* rendering, const Matrix4f& invH, const Objects::ISRShape_ptr shape, const Vector4f& intrinsic)
{
	const float *voxelData = shape->getSDFVoxel();
	ushort *outimage = renderedDepth->GetData(false);
	Vector2f *minmaximg = rendering->minmaxImage->GetData(false);

	Vector2i imgSize = rendering->outputImage->noDims;
	float mu = 0.5;
	Vector3f lightSource = -Vector3f(invH.getColumn(2));

	Vector4f invIntrinsic;
	invIntrinsic.x = 1 / intrinsic.x; invIntrinsic.y = 1 / intrinsic.y;
	invIntrinsic.z = -intrinsic.z*invIntrinsic.x; invIntrinsic.w = -intrinsic.w*invIntrinsic.y;

	float one_on_top_of_maxVoxelRange = 1 / sqrtf(DT_VOL_SIZE*DT_VOL_SIZE + DT_VOL_SIZE*DT_VOL_SIZE + DT_VOL_SIZE*DT_VOL_SIZE);

#pragma omp parallel 
	{
#pragma omp for
		for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		{
			raycastAndRenderDepth(outimage, x, y, imgSize, voxelData, invH, invIntrinsic, minmaximg, one_on_top_of_maxVoxelRange);
		}
	}
}

void LibISR::Engine::ISRVisualisationEngine_CPU::renderDepthNormalAndObject(ISRUShortImage* renderedDepth, ISRUChar4Image* renderNormal, Objects::ISRVisualisationState* rendering, const Matrix4f& invH, const Objects::ISRShape_ptr shape, const Vector4f& intrinsic)
{
	const float *voxelData = shape->getSDFVoxel();
	ushort *outimageD = renderedDepth->GetData(false);
	Vector4u* outimageGray = rendering->outputImage->GetData(false);
	Vector4u* outimageNormal = renderNormal->GetData(false);


	Vector2f *minmaximg = rendering->minmaxImage->GetData(false);

	Vector2i imgSize = rendering->outputImage->noDims;
	float mu = 0.5;
	Vector3f lightSource = -Vector3f(invH.getColumn(2));

	Vector4f invIntrinsic;
	invIntrinsic.x = 1 / intrinsic.x; invIntrinsic.y = 1 / intrinsic.y;
	invIntrinsic.z = -intrinsic.z*invIntrinsic.x; invIntrinsic.w = -intrinsic.w*invIntrinsic.y;

	float one_on_top_of_maxVoxelRange = 1 / sqrtf(DT_VOL_SIZE*DT_VOL_SIZE + DT_VOL_SIZE*DT_VOL_SIZE + DT_VOL_SIZE*DT_VOL_SIZE);

#pragma omp parallel 
	{
#pragma omp for
		for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		{
			raycastAndRenderWithDepthAndSurfaceNormal(outimageD, outimageGray, outimageNormal, x, y, imgSize, voxelData, invH, invIntrinsic, minmaximg, lightSource, one_on_top_of_maxVoxelRange);
		}
	}
}

