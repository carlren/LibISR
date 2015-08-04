// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "UIEngine.h"

#include <string.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#ifdef FREEGLUT
#include <GL/freeglut.h>
#endif

#include "../LibISRUtils/IOUtil.h"
#include <iostream>

using namespace LibISRUtils;
using namespace LibISR::Engine;

UIEngine* UIEngine::instance;

static void safe_glutBitmapString(void *font, const char *str)
{
	size_t len = strlen(str);
	for (size_t x = 0; x < len; ++x) {
		glutBitmapCharacter(font, str[x]);
	}
}

void UIEngine::glutDisplayFunction()
{
	UIEngine *uiEngine = UIEngine::Instance();

	// get updated images from processing thread
	//uiEngine->mainEngine->GetImage(uiEngine->outImage[0], uiEngine->outImageType[0]);

	//for (int w = 1; w < NUM_WIN; w++) uiEngine->mainEngine->GetImage(uiEngine->outImage[w], uiEngine->outImageType[w]);

	// do the actual drawing
	glClear(GL_COLOR_BUFFER_BIT);
	glColor3f(1.0f, 1.0f, 1.0f);
	glEnable(GL_TEXTURE_2D);

	UChar4Image** showImgs = uiEngine->outImage;
	Vector4f *winReg = uiEngine->winReg;
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	{
		glLoadIdentity();
		glOrtho(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		{
			glEnable(GL_TEXTURE_2D);
			for (int w = 0; w < NUM_WIN; w++)	{// Draw each sub window
				glBindTexture(GL_TEXTURE_2D, uiEngine->textureId[w]);
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, showImgs[w]->noDims.x, showImgs[w]->noDims.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, showImgs[w]->GetData(MEMORYDEVICE_CPU));
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
				glBegin(GL_QUADS); {
					glTexCoord2f(0, 1); glVertex2f(winReg[w][0], winReg[w][1]); // glVertex2f(0, 0);
					glTexCoord2f(1, 1); glVertex2f(winReg[w][2], winReg[w][1]); // glVertex2f(1, 0);
					glTexCoord2f(1, 0); glVertex2f(winReg[w][2], winReg[w][3]); // glVertex2f(1, 1);
					glTexCoord2f(0, 0); glVertex2f(winReg[w][0], winReg[w][3]); // glVertex2f(0, 1);
				}
				glEnd();
			}
			glDisable(GL_TEXTURE_2D);
		}
		glPopMatrix();
	}
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();


	//if (uiEngine->startDrawingBoundingBox)
	//{
	//	glColor3f(0.0, 1.0, 0.0);
	//	glBegin(GL_QUADS);
	//	glVertex2f((float)uiEngine->mouseBB.x / (float)showImgs[0]->noDims.x, (float)uiEngine->mouseBB.y / (float)showImgs[0]->noDims.y);
	//	glVertex2f((float)uiEngine->mouseBB.z / (float)showImgs[0]->noDims.x, (float)uiEngine->mouseBB.y / (float)showImgs[0]->noDims.y);
	//	glVertex2f((float)uiEngine->mouseBB.z / (float)showImgs[0]->noDims.x, (float)uiEngine->mouseBB.w / (float)showImgs[0]->noDims.y);
	//	glVertex2f((float)uiEngine->mouseBB.x / (float)showImgs[0]->noDims.x, (float)uiEngine->mouseBB.w / (float)showImgs[0]->noDims.y);
	//	glEnd();
	//}

	glColor3f(1.0f, 0.0f, 0.0f); 
	glRasterPos2f(0.75f, -0.967f);
	char str[200]; sprintf(str, "%04.2f fps",1000* uiEngine->currentFrameNo / uiEngine->processedTime );
	safe_glutBitmapString(GLUT_BITMAP_HELVETICA_18, (const char*)str);

	glColor3f(0.0f, 1.0f, 0.0f);
	glRasterPos2f(0.0f, -0.967f);
	sprintf(str, "P(Obj): %04.2f", uiEngine->energy);
	safe_glutBitmapString(GLUT_BITMAP_HELVETICA_18, (const char*)str);

	glColor3f(1.0f, 1.0f, 0.0f);
	glRasterPos2f(-0.95f, -0.95f);
	sprintf(str, "n - next frame \t b - all frames \t e - exit");
	safe_glutBitmapString(GLUT_BITMAP_HELVETICA_12, (const char*)str);

	glutSwapBuffers();
	uiEngine->needsRefresh = false;
}

void inline updateHistogramFromRendering(UChar4Image* rendering, UChar4Image* rgb, LibISR::Objects::ISRHistogram* hist)
{
	Vector4u* imgptr = rendering->GetData(MEMORYDEVICE_CPU);
	Vector4u bpix((uchar)0);
	for (int i = 0; i < rendering->dataSize;i++)
		if (imgptr[i] != bpix) imgptr[i] = Vector4u(255,255,255,255);
		else imgptr[i] = Vector4u(100, 100, 100, 100);
	
	hist->buildHistogram(rgb, rendering);

}

void UIEngine::glutIdleFunction()
{
	UIEngine *uiEngine = UIEngine::Instance();

	switch (uiEngine->mainLoopAction)
	{
	case REINIT_HIST:
		updateHistogramFromRendering(uiEngine->mainEngine->getRenderingState()->outputImage, uiEngine->mainEngine->getView()->rgb, uiEngine->mainEngine->frame->histogram);
		uiEngine->mainEngine->needStarTracker = true;
		uiEngine->mainLoopAction = PROCESS_VIDEO; uiEngine->processedFrameNo++;
		uiEngine->needsRefresh = true;
		break;
	case PROCESS_FRAME:
		uiEngine->ProcessFrame(); uiEngine->processedFrameNo++;
		uiEngine->mainLoopAction = PROCESS_PAUSED;
		uiEngine->needsRefresh = true;
		break;
	case PROCESS_VIDEO:
		uiEngine->ProcessFrame(); uiEngine->processedFrameNo++;
		uiEngine->needsRefresh = true;
		break;
	case EXIT:
#ifdef FREEGLUT
		glutLeaveMainLoop();
#else
		exit(0);
#endif
		break;
	case PROCESS_PAUSED:
	default:
		break;
	}

	if (uiEngine->needsRefresh) {
		glutPostRedisplay();
	}
}

void UIEngine::glutKeyUpFunction(unsigned char key, int x, int y)
{
	UIEngine *uiEngine = UIEngine::Instance();

	switch (key)
	{
	case 'r':
		printf("re-initialize histogram ...\n");
		uiEngine->mainLoopAction = UIEngine::REINIT_HIST;
		break;
	case 'n':
		printf("processing one frame ...\n");
		uiEngine->mainLoopAction = UIEngine::PROCESS_FRAME;
		break;
	case 'b':
		printf("processing input source ...\n");
		uiEngine->mainLoopAction = UIEngine::PROCESS_VIDEO;
		break;
	case 's':
		if (uiEngine->isRecording)
		{
			printf("stopped recoding disk ...\n");
			uiEngine->isRecording = false;
		}
		else
		{
			printf("started recoding disk ...\n");
			uiEngine->currentFrameNo = 0;
			uiEngine->isRecording = true;
		}
		break;
	case 'e':
		printf("exiting ...\n");
		uiEngine->mainLoopAction = UIEngine::EXIT;
		break;
	case  'f':
		glutFullScreenToggle();
		break;
	default:
		break;
	}
}

//void UIEngine::glutMouseButtonFunction(int button, int state, int x, int y)
//{
//	UIEngine *uiEngine = UIEngine::Instance();
//
//	if (state == GLUT_DOWN)
//	{
//		if (button == GLUT_LEFT_BUTTON)
//		{
//			uiEngine->startDrawingBoundingBox = true;
//			uiEngine->mouseBB.x = x;
//			uiEngine->mouseBB.y = y;
//		}
//	}
//	else if (state == GLUT_UP)
//	{
//		if (button == GLUT_LEFT_BUTTON && uiEngine->startDrawingBoundingBox)
//		{
//			uiEngine->startDrawingBoundingBox = false;
//			uiEngine->finsihedDrawingBoundingBox = true;
//			uiEngine->mouseBB.z = x;
//			uiEngine->mouseBB.w = y;
//		}
//	}
//}

//void UIEngine::glutMouseMoveFunction(int x, int y)
//{
//	UIEngine *uiEngine = UIEngine::Instance();
//	if (uiEngine->startDrawingBoundingBox = true)
//	{
//		uiEngine->mouseBB.z = x;
//		uiEngine->mouseBB.w = y;
//	}
//}

void UIEngine::Initialise(int & argc, char** argv, ImageSourceEngine *imageSource, ISRCoreEngine *mainEngine, const char *outFolder)
{
	this->mainEngine = mainEngine;
	this->imageSource = imageSource;
	{
		size_t len = strlen(outFolder);
		this->outFolder = new char[len + 1];
		strcpy(this->outFolder, outFolder);
	}

	int textHeight = 30; // Height of text area
	winSize.x = (int)(1.5f * (float)(imageSource->getDepthImageSize().x));
	winSize.y = imageSource->getDepthImageSize().y + textHeight;
	float h1 = textHeight / (float)winSize.y, h2 = (1.f + h1) / 2;
	winReg[0] = Vector4f(0.0f, h1, 0.665f, 1.0f);   // Main render
	winReg[1] = Vector4f(0.665f, h2, 1.0f, 1.0f);   // Side sub window 0
	winReg[2] = Vector4f(0.665f, h1, 1.0f, h2);     // Side sub window 2

	this->isRecording = false;
	this->currentFrameNo = 0;

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(winSize.x, winSize.y);
	glutCreateWindow("LibISR");
	glGenTextures(NUM_WIN, textureId);

	glutDisplayFunc(UIEngine::glutDisplayFunction);
	glutKeyboardUpFunc(UIEngine::glutKeyUpFunction);
	glutIdleFunc(UIEngine::glutIdleFunction);

	//glutMouseFunc(UIEngine::glutMouseButtonFunction);
	//glutMotionFunc(UIEngine::glutMouseMoveFunction);

#ifdef FREEGLUT
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, 1);
#endif

	for (int w = 0; w < NUM_WIN; w++)
		outImage[w] = new UChar4Image(imageSource->getDepthImageSize(),true, false);

	mainLoopAction = PROCESS_PAUSED;
	mouseState = 0;
	needsRefresh = false;
	processedFrameNo = 0;
	processedTime = 0.0f;

	sdkCreateTimer(&timer);

	printf("initialised.\n");
}

void UIEngine::SaveScreenshot(const char *filename) const
{
	UChar4Image screenshot(getWindowSize(),true,false);
	GetScreenshot(&screenshot);
	SaveImageToFile(&screenshot, filename, true);
}

void UIEngine::GetScreenshot(UChar4Image *dest) const
{
	glReadPixels(0, 0, dest->noDims.x, dest->noDims.y, GL_RGBA, GL_UNSIGNED_BYTE, dest->GetData(MEMORYDEVICE_CPU));
}

static inline float interpolate(float val, float y0, float x0, float y1, float x1) {
	return (val - x0)*(y1 - y0) / (x1 - x0) + y0;
}

static inline float base(float val) {
	if (val <= -0.75f) return 0.0f;
	else if (val <= -0.25f) return interpolate(val, 0.0f, -0.75f, 1.0f, -0.25f);
	else if (val <= 0.25f) return 1.0f;
	else if (val <= 0.75f) return interpolate(val, 1.0f, 0.25f, 0.0f, 0.75f);
	else return 0.0;
}

static inline void  DepthToUchar4(UChar4Image *dst, ShortImage *src)
{
	Vector4u *dest = dst->GetData(MEMORYDEVICE_CPU);
	short *source = src->GetData(MEMORYDEVICE_CPU);
	int dataSize = dst->dataSize;

	memset(dst->GetData(MEMORYDEVICE_CPU), 0, dataSize * 4);

	Vector4u *destUC4;
	float lims[2], scale;

	destUC4 = (Vector4u*)dest;
	lims[0] = 100000.0f; lims[1] = -100000.0f;

	for (int idx = 0; idx < dataSize; idx++)
	{
		
		float sourceVal = source[idx];
		if (sourceVal>65530) sourceVal = 0.0f;
		if (sourceVal > 0.0f) { lims[0] = MIN(lims[0], sourceVal); lims[1] = MAX(lims[1], sourceVal); }
	}

	scale = ((lims[1] - lims[0]) != 0) ? 1.0f / (lims[1] - lims[0]) : 1.0f / lims[1];

	if (lims[0] == lims[1]) return;

	for (int idx = 0; idx < dataSize; idx++)
	{
		float sourceVal = source[idx];

		if (sourceVal > 0.0f)
		{
			sourceVal = (sourceVal - lims[0]) * scale;

			destUC4[idx].r = (uchar)(base(sourceVal - 0.5f) * 255.0f);
			destUC4[idx].g = (uchar)(base(sourceVal) * 255.0f);
			destUC4[idx].b = (uchar)(base(sourceVal + 0.5f) * 255.0f);
			destUC4[idx].a = 255;
		}
	}
}

static inline void  DepthToUchar4_overlay(UChar4Image *dst, ShortImage *src, UChar4Image *rgb)
{
	Vector4u *srgb = rgb->GetData(MEMORYDEVICE_CPU);
	Vector4u *dest = dst->GetData(MEMORYDEVICE_CPU);
	short *source = src->GetData(MEMORYDEVICE_CPU);
	int dataSize = dst->dataSize;

	memset(dst->GetData(MEMORYDEVICE_CPU), 0, dataSize * 4);

	Vector4u *destUC4;
	float lims[2], scale;

	destUC4 = (Vector4u*)dest;
	lims[0] = 100000.0f; lims[1] = -100000.0f;

	for (int idx = 0; idx < dataSize; idx++)
	{

		float sourceVal = source[idx];
		if (sourceVal>65530) sourceVal = 0.0f;
		if (sourceVal > 0.0f) { lims[0] = MIN(lims[0], sourceVal); lims[1] = MAX(lims[1], sourceVal); }
	}

	scale = ((lims[1] - lims[0]) != 0) ? 1.0f / (lims[1] - lims[0]) : 1.0f / lims[1];

	if (lims[0] == lims[1]) return;

	for (int idx = 0; idx < dataSize; idx++)
	{
		float sourceVal = source[idx];

		if (sourceVal > 0.0f)
		{
			sourceVal = (sourceVal - lims[0]) * scale;

			destUC4[idx].r = (uchar)(base(sourceVal - 0.5f) * 127.5f + srgb[idx].r*0.5f);
			destUC4[idx].g = (uchar)(base(sourceVal) * 127.5f + srgb[idx].g*0.5f);
			destUC4[idx].b = (uchar)(base(sourceVal + 0.5f) * 127.5f + srgb[idx].b*0.5f);
			destUC4[idx].a = 255;
		}
	}
}

void UIEngine::ProcessFrame()
{
	if (!imageSource->hasMoreImages()) return;
	imageSource->getImages(mainEngine->getView());

	if (isRecording)
	{
		char str[250];

		//sprintf(str, "%s/%04d.pgm", outFolder, currentFrameNo);
		//SaveImageToFile(mainEngine->getView()->rawDepth, str);

		sprintf(str, "%s/c_%04d.ppm", outFolder, currentFrameNo);
		SaveImageToFile(mainEngine->getView()->rgb, str);
	}

	//DepthToUchar4(outImage[1], mainEngine->getView()->rawDepth);
	DepthToUchar4_overlay(outImage[1], mainEngine->getView()->rawDepth,mainEngine->getView()->rgb);

	//actual processing on the mailEngine

	sdkResetTimer(&timer); sdkStartTimer(&timer);
	mainEngine->processFrame();
	sdkStopTimer(&timer); processedTime += sdkGetTimerValue(&timer);

	this->energy = mainEngine->getEnergy();


	outImage[0]->SetFrom(mainEngine->getView()->alignedRgb, ORUtils::MemoryBlock< Vector4u >::CPU_TO_CPU);
	outImage[2]->SetFrom(mainEngine->getView()->rgb, ORUtils::MemoryBlock< Vector4u >::CPU_TO_CPU);

	if (isRecording)
	{
		char str[250];

		//sprintf(str, "%s/%04d.pgm", outFolder, currentFrameNo);
		//SaveImageToFile(mainEngine->getView()->rawDepth, str);

		sprintf(str, "%s/r_%04d.ppm", outFolder, currentFrameNo);
		SaveImageToFile(outImage[0], str);

		sprintf(str, "%s/o_%04d.ppm", outFolder, currentFrameNo);
		SaveImageToFile(outImage[1], str);
	}

	currentFrameNo++;
}

void UIEngine::Run() { glutMainLoop(); }
void UIEngine::Shutdown()
{
	sdkDeleteTimer(&timer);

	for (int w = 0; w < NUM_WIN; w++)
		delete outImage[w];
	delete[] outFolder;
	delete instance;
}

