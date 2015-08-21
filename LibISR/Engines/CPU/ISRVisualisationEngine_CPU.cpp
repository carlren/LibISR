// Copyright 2014-2015 Isis Innovation Limited and the authors of LibISR

#include "ISRVisualisationEngine_CPU.h"
#include "../shared/ISRVisualisationEngine_shared.h"

#include "../../Utils/IOUtil.h"

using namespace LibISR::Engine;
using namespace LibISR::Objects;


void LibISR::Engine::ISRVisualisationEngine_CPU::renderObject(Objects::ISRVisualisationState* rendering, const Matrix4f& invH, const Objects::ISRShape_ptr shape, const Vector4f& intrinsic)
{
	const float *voxelData = shape->getSDFVoxel();
	Vector4u *outimage = rendering->outputImage->GetData(MEMORYDEVICE_CPU);
	Vector2f *minmaximg = rendering->minmaxImage->GetData(MEMORYDEVICE_CPU);

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

void LibISR::Engine::ISRVisualisationEngine_CPU::renderDepth(UShortImage* renderedDepth, Objects::ISRVisualisationState* rendering, const Matrix4f& invH, const Objects::ISRShape_ptr shape, const Vector4f& intrinsic)
{
	const float *voxelData = shape->getSDFVoxel();
	ushort *outimage = renderedDepth->GetData(MEMORYDEVICE_CPU);
	Vector2f *minmaximg = rendering->minmaxImage->GetData(MEMORYDEVICE_CPU);

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

void LibISR::Engine::ISRVisualisationEngine_CPU::renderDepthNormalAndObject(UShortImage* renderedDepth, UChar4Image* renderNormal, Objects::ISRVisualisationState* rendering, const Matrix4f& invH, const Objects::ISRShape_ptr shape, const Vector4f& intrinsic)
{
	const float *voxelData = shape->getSDFVoxel();
	ushort *outimageD = renderedDepth->GetData(MEMORYDEVICE_CPU);
	Vector4u* outimageGray = rendering->outputImage->GetData(MEMORYDEVICE_CPU);
	Vector4u* outimageNormal = renderNormal->GetData(MEMORYDEVICE_CPU);


	Vector2f *minmaximg = rendering->minmaxImage->GetData(MEMORYDEVICE_CPU);

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

void LibISR::Engine::ISRVisualisationEngine_CPU::renderAsSDF(FloatImage* SDFImage, Float4Image* ptCloud, Objects::ISRVisualisationState* rendering, const Matrix4f& invH, const Objects::ISRShape_ptr shape, const Vector4f& intrinsic)
{

	const float *voxelData = shape->getSDFVoxel();
	float *outSDF_ptr = SDFImage->GetData(MEMORYDEVICE_CPU);
	Vector4f *outPtCloud_ptr = ptCloud->GetData(MEMORYDEVICE_CPU);
	Vector4u* outimageGray = rendering->outputImage->GetData(MEMORYDEVICE_CPU);
	Vector2f *minmaximg = rendering->minmaxImage->GetData(MEMORYDEVICE_CPU);

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
			raycaseAsSDF(outSDF_ptr, outPtCloud_ptr, outimageGray, x, y, imgSize, voxelData, invH, invIntrinsic, minmaximg, lightSource, one_on_top_of_maxVoxelRange);
		}
	}

}

