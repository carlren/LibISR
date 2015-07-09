#include "PWPTracker.h"
#include <iostream>

#ifndef _DELETE
	#define _DELETE(x) {if(x) {delete(x); x=NULL;}}
	#define _DELETEARRAY(x) {if(x) {delete [] (x); x=NULL;}}
#endif

#ifndef min
#define min(a,b) (a<b?a:b)
#endif

#ifndef max
#define max(a,b) (a>b?a:b)
#endif

using namespace std;
using namespace ORUtils;
using namespace pwp;
//---------------------------------------------------------------------------------

PWPTracker::PWPTracker() 
{
	// learning settings
	m_sSS.fAppearanceLearningRate	=	0.5;
	m_sSS.fShapeLearningRate		=	0.3;

	// openGL drawing setting
	m_sDS.bDiscretizeImage			=	false;
	m_sDS.bClassifyWholeImage		=	false;
	m_sDS.bDrawLevelSet				=	true;
	m_sDS.eOverlayType				=	OVERLAY_NONE;
	m_sDS.bDrawNarrowBand			=	true;
	m_sDS.n2DOverlaySize			=	150;
	m_sDS.n3DOverlaySize			=	150;
	m_sDS.bOverlay2D				=	true;
	m_sDS.bOverlay3D				=	true;
	m_sDS.bAnimateLevelSet			=	true;
	m_sDS.fRange					=	1000.0f;
	m_sDS.fTheta					=	DegToRad(-90.0);
	m_sDS.fPhi						=	DegToRad(0.0);
	
	// pwp advance settings
	m_sAS.fPriorBackground			=	0.6;
	m_sAS.bUseSegmentation			=	true;
	m_sAS.fEpsilon					=	0.3; // smoothness for heaviside
	m_sAS.fMu						=	0.2; // evolution step size, can't be too big
	m_sAS.fPsi						=	1.0;
	m_sAS.nShapeIterations			=	1;
	m_sAS.nTrackIterations			=	50;
	m_sAS.fDamping					=	1.0;
	m_sAS.fStepSize					=	0.25;
	m_sAS.bUseDriftCorrection		=	true;
	m_sAS.fNarrowBand				=	3.0;
	m_sAS.eModelType				=	MODEL_TSR;
	m_sAS.nBitsY					=	5;
	m_sAS.nBitsU					=	5;
	m_sAS.nBitsV					=	5;
	m_sAS.fSigma					=	1.0;
	m_sAS.eOpinionPoolType			=	OP_LINEAR;//	OP_LOGARITHMIC;
	//m_sAS.eOpinionPoolType			= 	OP_LOGARITHMIC;

	// others
	m_fLSAnimationTime				=	0.0;
	m_pcBackHist					=	NULL;
	m_pcDiscretizedImage			=	NULL;
	m_pasImageInfo					=	NULL;
	m_bHasTarget					=	false;
}

//---------------------------------------------------------------------------------

PWPTracker::~PWPTracker()
{
	Clear();
}

//---------------------------------------------------------------------------------

bool PWPTracker::_InsideImage(int nW, int nH, int nX, int nY)
{
	return nX>=0 && nX<nW && nY>=0 && nY<nH;
}

//---------------------------------------------------------------------------------

Matrix3f PWPTracker::_BuildWarpMatrix(Pose vp)
{
	Matrix3f mW; 

	if(m_sAS.eModelType == MODEL_T)
	{
		mW(0, 0) = 1;								mW(0, 1) = 0;								mW(0, 2) = vp[0];
		mW(1, 0) = 0;								mW(1, 1) = 1;								mW(1, 2) = vp[1];
		mW(2, 0) = 0;								mW(2, 1) = 0;								mW(2, 2) = 1;
		return mW;
	}
	if(m_sAS.eModelType == MODEL_TS)
	{
		mW(0, 0) = 1 + vp[2];						mW(0, 1) = 0;								mW(0, 2) = vp[0];
		mW(1, 0) = 0;								mW(1, 1) = 1 + vp[2];						mW(1, 2) = vp[1];
		mW(2, 0) = 0;								mW(2, 1) = 0;								mW(2, 2) = 1;
		return mW;
	}
	if(m_sAS.eModelType == MODEL_TSR)
	{
		mW(0, 0) = (1 + vp[2])*cos(vp[3]);			mW(0, 1) = -(1 + vp[2])*sin(vp[3]);			mW(0, 2) = vp[0];
		mW(1, 0) = (1 + vp[2])*sin(vp[3]);			mW(1, 1) = (1 + vp[2])*cos(vp[3]);			mW(1, 2) = vp[1];
		mW(2, 0) = 0;								mW(2, 1) = 0;								mW(2, 2) = 1;
		return mW;
	}
	if(m_sAS.eModelType == MODEL_AFFINE)
	{
		mW(0, 0) = (1 + vp[2]);						mW(0, 1) = vp[4];							mW(0, 2) = vp[0];
		mW(1, 0) = vp[3];							mW(1, 1) = (1 + vp[5]);						mW(1, 2) = vp[1];
		mW(2, 0) = 0;								mW(2, 1) = 0;								mW(2, 2) = 1;
		return mW;
	}
	if(m_sAS.eModelType == MODEL_HOMOGRAPHY)
	{
		mW(0, 0) = (1 + vp[2]);						mW(0, 1) = vp[4];							mW(0, 2) = vp[0];
		mW(1, 0) = vp[3];							mW(1, 1) = (1 + vp[5]);						mW(1, 2) = vp[1];
		mW(2, 0) = vp[6];							mW(2, 1) = vp[7];							mW(2, 2) = 1;
		return mW;
	}
	return mW;
}

//---------------------------------------------------------------------------------

Matrix3f PWPTracker::_BuildInverseWarpMatrix(Pose vp)
{
	Matrix3f mW;

	if(m_sAS.eModelType == MODEL_T)
	{
		mW(0,0) = 1.0;								mW(0,1) = 0.0;								mW(0,2) = -vp[0];
		mW(1,0) = 0.0;								mW(1,1) = 1.0;								mW(1,2) = -vp[1];
		mW(2,0) = 0.0;								mW(2,1) = 0.0;								mW(2,2) = 1.0;
		return mW;
	}
	if(m_sAS.eModelType == MODEL_TS)
	{
		double fDenom = (1+vp[2]);
		mW(0,0) = 1.0/fDenom;						mW(0,1) = 0.0;								mW(0,2) = -vp[0]/fDenom;
		mW(1,0) = 0.0;								mW(1,1) = 1.0/fDenom;						mW(1,2) = -vp[1]/fDenom;
		mW(2,0) = 0.0;								mW(2,1) = 0.0;								mW(2,2) = 1.0;
		return mW;
	}
	if(m_sAS.eModelType == MODEL_TSR)
	{
		double fDenom = (1+vp[2]);
		mW(0,0) = cos(vp[3])/fDenom;				mW(0,1) = sin(vp[3])/fDenom;				mW(0,2) = -(vp[0]*cos(vp[3])+vp[1]*sin(vp[3]))/fDenom;
		mW(1,0) = -sin(vp[3])/fDenom;				mW(1,1) = cos(vp[3])/fDenom;				mW(1,2) = (vp[0]*sin(vp[3])-vp[1]*cos(vp[3]))/fDenom;
		mW(2,0) = 0.0;								mW(2,1) = 0.0;								mW(2,2) = 1.0;
		return mW;
	}
	if(m_sAS.eModelType == MODEL_AFFINE)
	{
		double fDenom = (1+vp[5]+vp[2]+vp[2]*vp[5]-vp[3]*vp[4]);
		mW(0,0) = (1+vp[5])/fDenom;					mW(0,1) = -vp[4]/fDenom;					mW(0,2) = (-vp[0]+vp[1]*vp[4]-vp[0]*vp[5])/fDenom;
		mW(1,0) = -vp[3]/fDenom;					mW(1,1) = (1.0+vp[2])/fDenom;				mW(1,2) = (-vp[1]-vp[1]*vp[2]+vp[0]*vp[3])/fDenom;
		mW(2,0) = 0.0;								mW(2,1) = 0.0;								mW(2,2) = 1.0;
		return mW;
	}

	return mW;
}

//---------------------------------------------------------------------------------

void PWPTracker::Clear()		
{
	if(m_bHasTarget)
	{
		_DELETE(m_pcBackHist);
		_DELETEARRAY(m_pasImageInfo);
		_DELETE(m_sTarget.pcForeHist);
		_DELETEARRAY(m_sTarget.pasLevelSet);
		_DELETE(m_sTarget.pmTemplate);
		m_sTarget.vFGCorners.clear();
		m_sTarget.vBGCorners.clear();

		m_bHasTarget = false;
	}
}

//---------------------------------------------------------------------------------

void PWPTracker::_DiscretizeImage(UChar4Image* cImage)
{
	if (!m_pcDiscretizedImage)
		m_pcDiscretizedImage = new UChar4Image(cImage->noDims, MEMORYDEVICE_CPU);
	m_pcDiscretizedImage->ChangeDims(cImage->noDims);

	Vector4u* dImg_ptr = m_pcDiscretizedImage->GetData(MEMORYDEVICE_CPU);
	Vector4u* inImg_ptr = cImage->GetData(MEMORYDEVICE_CPU);

	for (int i = 0; i < cImage->dataSize;i++)
	{
		dImg_ptr[i].r = unsigned char(inImg_ptr[i].r >> (8 - m_sAS.nBitsY)) << (8 - m_sAS.nBitsY);
		dImg_ptr[i].g = unsigned char(inImg_ptr[i].g >> (8 - m_sAS.nBitsY)) << (8 - m_sAS.nBitsY);
		dImg_ptr[i].b = unsigned char(inImg_ptr[i].b >> (8 - m_sAS.nBitsY)) << (8 - m_sAS.nBitsY);
	}
}

//---------------------------------------------------------------------------------

void PWPTracker::_ClassifyImage(UChar4Image* cImage)
{
	m_mClassifiedImage->ChangeDims(cImage->noDims);
	m_mClassifiedImage->Clear(PIXEL_BACKGROUND);

	int* classifiImg_ptr = m_mClassifiedImage->GetData(MEMORYDEVICE_CPU);

	if(!m_bHasTarget)
		return;

	// Image info
	const int imagewidth = m_nImageWidth;
	const int imageheight = m_nImageHeight;
	ImageInfo* const pII = m_pasImageInfo;

	for (int r = 0; r < imageheight; r++)
		for (int c = 0; c<imagewidth; c++)
			if (pII[r*imagewidth + c].LHF > pII[r*imagewidth + c].LHB)
				classifiImg_ptr[r*imagewidth + c] = 0;
}

//---------------------------------------------------------------------------------

Matrix3f PWPTracker::_ComputeDriftCorrection()
{
	Matrix3f mW;
	
	double fMinX = 999999;
	double fMaxX = -999999;
	double fMinY = 999999;
	double fMaxY = -999999;

	// Level set
	const int rowlength = m_sTarget.nBGWidth;
	LevelSetInfo* pLS = m_sTarget.pasLevelSet;

	for(int nr=0,cr=-m_sTarget.nBGHalfHeight; cr<=m_sTarget.nBGHalfHeight; nr++, cr++)
	{
		for(int nc=0,cc=-m_sTarget.nBGHalfWidth; cc<=m_sTarget.nBGHalfWidth; nc++, cc++)
		{
			if(pLS[nr*rowlength+nc].U<=0.0f) 
				continue;
		
			fMinX = min(double(cc),fMinX);
			fMaxX = max(double(cc),fMaxX);
			fMinY = min(double(cr),fMinY);
			fMaxY = max(double(cr),fMaxY);
		}
	}

	double fRB = m_sTarget.nFGHalfWidth-fMaxX;
	double fLB = fMinX+m_sTarget.nFGHalfWidth;
	double fBB = m_sTarget.nFGHalfHeight-fMaxY;
	double fTB = fMinY+m_sTarget.nFGHalfHeight;

	// Drift correction
	double fDTX = max(min((fLB-fRB),0.4),-0.4);
	double fDTY = max(min((fTB-fBB),0.4),-0.4);
	double fDSX = min(fRB,fLB);
	double fDSY = min(fTB,fBB);
	double fDScaleControl = max(min(0.005f*(BORDERDRIFT-min(fDSX,fDSY)),0.1f),-0.1f);
	double fDScaleControlX = max(min(0.02f*(BORDERDRIFT-fDSX),0.2f),-0.2f);
	double fDScaleControlY = max(min(0.02f*(BORDERDRIFT-fDSY),0.2f),-0.2f);
	
	if(m_sAS.eModelType == MODEL_T)
	{
		mW(0,0) = 1.0;							mW(0,1) = 0.0;							mW(0,2) = fDTX;
		mW(1,0) = 0.0;							mW(1,1) = 1.0;							mW(1,2) = fDTY;
		mW(2,0) = 0.0;							mW(2,1) = 0.0;							mW(2,2) = 1.0;
	}
	else if(m_sAS.eModelType <= MODEL_TSR)
	{
		double fDThetaControl = max(min(-m_sTarget.vPose[3],0.001f),-0.001f);
		mW(0,0) = (1.0+fDScaleControl)*cos(fDThetaControl);		mW(0,1) = -sin(fDThetaControl);							mW(0,2) = fDTX;
		mW(1,0) = sin(fDThetaControl);							mW(1,1) = (1.0+fDScaleControl)*cos(fDThetaControl);		mW(1,2) = fDTY;
		mW(2,0) = 0.0;											mW(2,1) = 0.0;											mW(2,2) = 1.0;
	}
	else
	{
		mW(0,0) = 1.0+fDScaleControlX;				mW(0,1) = 0.0;								mW(0,2) = fDTX;
		mW(1,0) = 0.0;								mW(1,1) = 1.0+fDScaleControlY;				mW(1,2) = fDTY;
		mW(2,0) = 0.0;								mW(2,1) = 0.0;								mW(2,2) = 1.0;
	}

	const double WDC_00 = mW(0,0);
	const double WDC_01 = mW(0,1);
	const double WDC_02 = mW(0,2);
	const double WDC_10 = mW(1,0);
	const double WDC_11 = mW(1,1);
	const double WDC_12 = mW(1,2);

	const int rowlength0 = m_sTarget.nBGWidth;
	LevelSetInfo* pLS0 = m_sTarget.pasLevelSet;

	const int ybegin0 = -m_sTarget.nBGHalfHeight;
	const int yend0 = m_sTarget.nBGHalfHeight;
	const int xbegin0 = -m_sTarget.nBGHalfWidth;
	const int xend0 =m_sTarget.nBGHalfWidth;

	// Copy stuff so we can use it for our bilinear interpolation
	int lssize = m_sTarget.nBGWidth*m_sTarget.nBGHeight;
	LevelSetInfo* pCLS = pLS0;
	while(lssize--)
	{
		pCLS->CopyU = pCLS->U;
		pCLS->CopyPF = pCLS->PF;
		pCLS->CopyPB = pCLS->PB;
		pCLS++;
	}

	pCLS = pLS0;
	for(int y=ybegin0; y<=yend0; y++)
	{
		for(int x=xbegin0; x<=xend0; x++)
		{
			const double xdc = WDC_00*x+WDC_01*y+WDC_02;
			const double ydc = WDC_10*x+WDC_11*y+WDC_12;
			const int LX = floor(xdc);
			const int LY = floor(ydc);

			if(LX>=xbegin0 && LX+1<=xend0 && LY>=ybegin0 && LY+1<=yend0)
			{
				const double RX = xdc-LX;
				const double RY = ydc-LY;
				const double OneMinusRX = 1.0-RX;
				const double OneMinusRY = 1.0-RY;

				LevelSetInfo* pLXLY = pLS0+(LY+yend0)*rowlength0+(LX+xend0);
				LevelSetInfo* pUXLY = pLS0+(LY+yend0)*rowlength0+(LX+xend0+1);
				LevelSetInfo* pLXUY = pLS0+(LY+yend0+1)*rowlength0+(LX+xend0);
				LevelSetInfo* pUXUY = pLS0+(LY+yend0+1)*rowlength0+(LX+xend0+1);

				pCLS->PF = OneMinusRY*(OneMinusRX*pLXLY->CopyPF+RX*pUXLY->CopyPF)+RY*(OneMinusRX*pLXUY->CopyPF+RX*pUXUY->CopyPF);
				pCLS->PB = OneMinusRY*(OneMinusRX*pLXLY->CopyPB+RX*pUXLY->CopyPB)+RY*(OneMinusRX*pLXUY->CopyPB+RX*pUXUY->CopyPB);
				pCLS->U =  OneMinusRY*(OneMinusRX*pLXLY->CopyU +RX*pUXLY->CopyU) +RY*(OneMinusRX*pLXUY->CopyU +RX*pUXUY->CopyU);
			}
			pCLS++;
		}
	}

	return mW;
}

//---------------------------------------------------------------------------------

void PWPTracker::_IterateShapeAlignment(const UChar4Image* cImage)
{
	// Pose
	const Pose vplast = m_sTarget.vPose;
	Pose& vp = m_sTarget.vPose;
	const Pose vppred = (vplast+m_sTarget.vVelocity);

	// Do a velocity prediction
	// vp = vppred;

	// Compute intial warp
	Matrix3f mW = _BuildWarpMatrix(vp);

	// Image info
	const int imagewidth = m_nImageWidth;
	const int imageheight = m_nImageHeight;
	ImageInfo* const pII = m_pasImageInfo;
		
	// Level set
	const int rowlength0 = m_sTarget.nBGWidth;
	LevelSetInfo* const pLS0 = m_sTarget.pasLevelSet;

	bool bRankDeficiency;
	int nSize;

	if(m_sAS.eModelType == MODEL_T)
		nSize = 2;
	else if(m_sAS.eModelType == MODEL_TS)
		nSize = 3;
	else if(m_sAS.eModelType == MODEL_TSR)
		nSize = 4;
	else if(m_sAS.eModelType == MODEL_AFFINE)
		nSize = 6;
	else if(m_sAS.eModelType == MODEL_HOMOGRAPHY)
		nSize = 8;
	else
	{
		cerr << "Invalid model type" << endl;
		return;
	}

	MatrixSQX<float, MAXPS> mJTJ; mJTJ.setZeros();
	MatrixSQX<float, MAXPS> mJTJPrior; mJTJPrior.setZeros();

	VectorX<float, MAXPS> vJTB;
	VectorX<float, MAXPS> vJTBPrior;

	//double avJTB[MAXPS];
	
	MODELTYPE eModelType = MODEL_T;
	const double fFiveTimesEpsilon = m_sAS.fEpsilon;

	// Pre compute stuff
	const int nWI = cImage->noDims.x;
	const int nHI = cImage->noDims.y;
	
	const int rbegin0 = 0+1;
	const int ybegin0 = -m_sTarget.nBGHalfHeight+1;
	const int yend0 = m_sTarget.nBGHalfHeight-1;
	const int cbegin0 = 0+1;
	const int xbegin0 = -m_sTarget.nBGHalfWidth+1;
	const int xend0 = m_sTarget.nBGHalfWidth-1;

	double fNb = 0.0;
	double fNf = 0.0;

	int nPixels = 0;
	{
		// Compute warps
		const double W_00 = mW(0, 0);
		const double W_01 = mW(0, 1);
		const double W_02 = mW(0, 2);
		const double W_10 = mW(1, 0);
		const double W_11 = mW(1, 1);
		const double W_12 = mW(1, 2);

		//ofstream ofs("e:/pwp/levelset.txt");
		//for (int r = rbegin0, _y = ybegin0; _y <= yend0; r++, _y++)
		//{
		//	for (int c = cbegin0, _x = xbegin0; _x <= xend0; c++, _x++)
		//	{
		//		LevelSetInfo* pCLS = pLS0 + r*rowlength0 + c;
		//		ofs << pCLS->U << "\t";
		//	}
		//	ofs << "\n";
		//}
		//ofs.close();

		for(int r=rbegin0,_y=ybegin0; _y<=yend0; r+=1, _y+=1)
		{
			for(int c=cbegin0,_x=xbegin0; _x<=xend0; c+=1, _x+=1)
			{
				const double fU = pLS0[r*rowlength0+c].U;

				if(abs(fU)>m_sAS.fNarrowBand)
					continue;

				PrecompStuff &cPC = m_asPreComp[nPixels++];
				
				cPC.nR = r;
				cPC.nC = c;
				cPC.nX = _x;
				cPC.nY = _y;
				cPC.H = _Heaviside(fU,fFiveTimesEpsilon);

				const double fH = cPC.H;

				fNb  += (1.0-fH);
				fNf  += fH;

				const double dx = pLS0[r*rowlength0+c].DX;
				const double dy = pLS0[r*rowlength0+c].DY;
				const double fDirac = _Dirac(fU,fFiveTimesEpsilon);

				// Compute jacobian
				double J0,J1,J2,J3,J4,J5,J6,J7;
				J0 = dx*fDirac;
				J1 = dy*fDirac;

				double fX = _x;
				double fY = _y;

				if(m_sAS.eModelType == MODEL_TS)
				{
					J2 = (dx*fX+dy*fY)*fDirac;
				}
				else if(m_sAS.eModelType == MODEL_TSR) // Evaluated at zero
				{
					J2 = (dx*fX+dy*fY)*fDirac;
					J3 = (-dx*fY+dy*fX)*fDirac;
				}
				else if(m_sAS.eModelType == MODEL_AFFINE)
				{
					J2 = dx*fX*fDirac;
					J3 = dy*fX*fDirac;	
					J4 = dx*fY*fDirac;	
					J5 = dy*fY*fDirac;
				}
				else if(m_sAS.eModelType == MODEL_HOMOGRAPHY)
				{
					J2 = dx*fX*fDirac;
					J3 = dy*fX*fDirac;	
					J4 = dx*fY*fDirac;	
					J5 = dy*fY*fDirac;
					J6 = fDirac*(-dx*fX*fX-dy*fX*fY);
					J7 = fDirac*(-dx*fX*fY-dy*fY*fY);
				}

				// Copy for use in main loop
				cPC.J0 = J0;
				cPC.J1 = J1;
				cPC.J2 = J2;
				cPC.J3 = J3;
				/*cPC.J4 = J4;
				cPC.J5 = J5;
				cPC.J6 = J6;
				cPC.J7 = J7;*/

				// JTJ
				mJTJ(0,0) += J0*J0;
				mJTJ(0,1) += J0*J1;
				mJTJ(1,1) += J1*J1;

				switch(m_sAS.eModelType)
				{
					case(MODEL_HOMOGRAPHY):
					{
						// JTJ
						mJTJ(1,6) += J0*J6;
						mJTJ(1,7) += J0*J7;
						mJTJ(1,6) += J1*J6;
						mJTJ(1,7) += J1*J7;
						mJTJ(2,6) += J2*J6;
						mJTJ(2,7) += J2*J7;
						mJTJ(3,6) += J3*J6;
						mJTJ(3,7) += J3*J7;
						mJTJ(4,6) += J4*J6;
						mJTJ(4,7) += J4*J7;
						mJTJ(5,6) += J5*J6;
						mJTJ(5,7) += J5*J7;
						mJTJ(6,6) += J6*J6;
						mJTJ(6,7) += J6*J7;
						mJTJ(7,7) += J7*J7;
					}
					case(MODEL_AFFINE):
					{
						// JTJ
						mJTJ(0,4) += J0*J4;
						mJTJ(0,5) += J0*J5;
						mJTJ(1,4) += J1*J4;
						mJTJ(1,5) += J1*J5;
						mJTJ(2,4) += J2*J4;
						mJTJ(2,5) += J2*J5;
						mJTJ(3,4) += J3*J4;
						mJTJ(3,5) += J3*J5;
						mJTJ(4,4) += J4*J4;
						mJTJ(4,5) += J4*J5;
						mJTJ(5,5) += J5*J5;
					}
					case(MODEL_TSR):
					{
						// JTJ
						mJTJ(0,3) += J0*J3;
						mJTJ(1,3) += J1*J3;
						mJTJ(2,3) += J2*J3;
						mJTJ(3,3) += J3*J3;
					}
					case(MODEL_TS):
					{
						// JTJ
						mJTJ(0,2) += J0*J2;
						mJTJ(1,2) += J1*J2;
						mJTJ(2,2) += J2*J2;
					}
				}
			}
		}
	}

	// Copy upper triangle into lower triangle
	for(int r=0;r<nSize;r++)
		for(int c=r+1;c<nSize;c++)
			mJTJ(c,r) = mJTJ(r,c);

	// Optimisation 
	//double fScaleProb = m_sTarget.pcForeHist->Bins();
	int nIt;

	ImageInfo IIOutside;
	IIOutside.LHB = 0.5;
	IIOutside.LHF = 0.5;

	double fN = fNb+fNf;
	double fStepSize = m_sAS.fStepSize/**fN*/;
	//bool bUsingLargeStepSize = true;

	for(nIt=0;nIt<m_sAS.nTrackIterations;nIt++)
	{
		double fForegroundProb = 0.0;
		double fBackgroundProb = 0.0;

		// Compute warps
		const double W_00 = mW(0, 0);
		const double W_01 = mW(0, 1);
		const double W_02 = mW(0, 2);
		const double W_10 = mW(1, 0);
		const double W_11 = mW(1, 1);
		const double W_12 = mW(1, 2);

		int nCurrPix = nPixels;
		PrecompStuff* pPC = m_asPreComp;

		double vJTB0 = 0.0;
		double vJTB1 = 0.0;
		double vJTB2 = 0.0;
		double vJTB3 = 0.0;

		double fCostLinPWP = 0.0;
		double fCostLogPWP = 0.0;
		double fCostLogLike = 0.0;
		double fCostLogLikeCremers = 0.0;

		while(nCurrPix--)
		{
			const int &x = pPC->nX;
			const int &y = pPC->nY;
			const int &r = pPC->nR;
			const int &c = pPC->nC;

			const double cf = W_00*x+W_01*y+W_02;
			const double rf = W_10*x+W_11*y+W_12;

			const int ci = floor(cf);
			const int ri = floor(rf);

			// Copy likelihoods from image data or outside pixel (uniform)
			ImageInfo* pIILCLR;
			ImageInfo* pIIUCLR;
			ImageInfo* pIILCUR;
			ImageInfo* pIIUCUR;
			double LHF,LHB;
			if(ci<0 || ci>=nWI-1 || ri<0 || ri>=nHI-1)
			{
				LHF = 0.5;
				LHB = 0.5;
			}
			else
			{
				const double RC = cf-ci;
				const double RR = rf-ri;
				const double OneMinusRC = 1.0-RC;
				const double OneMinusRR = 1.0-RR;

				pIILCLR = &pII[ri*imagewidth+ci];
				pIIUCLR = &pII[ri*imagewidth+ci+1];
				pIILCUR = &pII[(ri+1)*imagewidth+ci];
				pIIUCUR = &pII[(ri+1)*imagewidth+(ci+1)];

				LHF = OneMinusRR*(OneMinusRC*pIILCLR->LHF+RC*pIIUCLR->LHF)+RR*(OneMinusRC*pIILCUR->LHF+RC*pIIUCUR->LHF);
				LHB = OneMinusRR*(OneMinusRC*pIILCLR->LHB+RC*pIIUCLR->LHB)+RR*(OneMinusRC*pIILCUR->LHB+RC*pIIUCUR->LHB);
			}

			// Compute heaviside step functions
			const double fH = pPC->H;
			
			double fB;
			if(m_sAS.eOpinionPoolType==OP_LOGARITHMIC)
				fB = (LHF-LHB)/(LHF*fH+LHB*(1.0-fH));
			else
				fB = (LHF-LHB)/((fNf*LHF+fNb*LHB)/fN);

			//fForegroundProb += (LHF*fH+LHB*(1.0-fH))/(fNf*LHF+fNb*LHB);
			//fBackgroundProb += LHB*(1.0-m1*fH[1])*(1.0-m2*fH[2]))/(LHF+;

			//JTB
			/*avJTB[0]*/vJTB0 += fB*pPC->J0;
			/*avJTB[1]*/vJTB1 += fB*pPC->J1;
			/*avJTB[2]*/vJTB2 += fB*pPC->J2;
			/*avJTB[3]*/vJTB3 += fB*pPC->J3;
			/*avJTB[4] += fB*J4;
			avJTB[5] += fB*J5;
			avJTB[6] += fB*J6;
			avJTB[7] += fB*J7;*/

			pPC++;
		}

		m_sTarget.fForegroundProb = fCostLinPWP;
		m_sTarget.fBackgroundProb = fBackgroundProb;
//			m_sTarget.afCost[nIt] = m_sTarget.fForegroundProb+m_sTarget.fBackgroundProb;

		mJTJPrior.setZeros();
		vJTBPrior.Clear(0.0);

		
		// Prior
		if(eModelType==MODEL_TS) 
		{
			mJTJPrior(2,2) += m_sAS.fDamping;
			vJTBPrior[2] += -m_sAS.fDamping*(vppred[2]-vp[2]);
		}
		if(eModelType==MODEL_TSR)
		{
			mJTJPrior(0,0) = m_sAS.fDamping;
			mJTJPrior(1,1) = m_sAS.fDamping;
			mJTJPrior(2,2) = 100.0*m_sAS.fDamping;
			mJTJPrior(3,3) = 100.0*m_sAS.fDamping;
			vJTBPrior[0] = -m_sAS.fDamping*(vppred[0]-vp[0]);
			vJTBPrior[1] = -m_sAS.fDamping*(vppred[1]-vp[1]);
			vJTBPrior[2] = -100.0*m_sAS.fDamping*(vppred[2]-vp[2]);
			vJTBPrior[3] = -100.0*m_sAS.fDamping*(vppred[3]-vp[3]);
		}
		/*if(eModelType==MODEL_AFFINE)
		{
			mJTJPrior[2][2] += m_sAS.fDamping*fN;
			mJTJPrior[3][3] += m_sAS.fDamping*fN;
			vJTBPrior[2] += -m_sAS.fDamping*fN*(vplast[2]-vp[2]);
			vJTBPrior[3] += -m_sAS.fDamping*fN*(vplast[3]-vp[3]);
		}
		if(eModelType==MODEL_HOMOGRAPHY)
		{
			amJTJ[2][2] += 100.0;
			amJTJ[3][3] += 100.0;
			amJTJ[4][4] += 100.0;
			amJTJ[5][5] += 100.0; 
			amJTJ[6][6] += m_sAS.fDamping*fN;
			amJTJ[7][7] += m_sAS.fDamping*fN; 
			avJTB[6] += m_sAS.fDamping*fN*vp[6];
			avJTB[7] += m_sAS.fDamping*fN*vp[7];
		}*/

		//mJTJPrior.Fill(0.0);
		//vJTBPrior.Fill(0.0);

		Pose vDP;

		/*for(int r=0;r<nSize;r++)
			vJTB[r] = avJTB[r];*/
		vJTB[0] = vJTB0;
		vJTB[1] = vJTB1;
		vJTB[2] = vJTB2;
		vJTB[3] = vJTB3;

		MatrixSQX<float, MAXPS> combineJTJ = mJTJ + mJTJPrior;
		Matrix4f finalJTJ;
		for (int r = 0; r < 4; r++) for (int c = 0; c < 4;c++)
		{
			finalJTJ(r, c) = combineJTJ(r, c);
		}

		Cholesky cChol(finalJTJ.m, 4);
		cChol.Backsub(vDP.v, (vJTB + vJTBPrior).v);
		bRankDeficiency = false;

		if(!bRankDeficiency)
		{
			if(eModelType!=MODEL_HOMOGRAPHY)
			{
				MODELTYPE eRealModelType = m_sAS.eModelType;
				m_sAS.eModelType = eModelType;
				mW = _BuildInverseWarpMatrix(vDP)*mW; // this one need to check whether it's this way
				m_sAS.eModelType = eRealModelType;

				// Update pose parameters
				vp[0] = mW(0,2);
				vp[1] = mW(1,2);

				if(eModelType == MODEL_TS)
				{
					vp[2] = mW(0,0)-1;
					vp[2] = max(vp[2],-0.9f);
				}
				else if(eModelType == MODEL_TSR)
				{
					vp[3] = atan2(mW(1,0),mW(0,0));
					vp[2] =	sqrt((mW(0,0)*mW(1,1))-(mW(0,1)*mW(1,0)))-1;
					vp[2] = max(vp[2],-0.9f);
				}
				else if(eModelType == MODEL_AFFINE)
				{
					vp[2] = mW(0,0)-1;
					vp[3] = mW(1,0);
					vp[4] = mW(0,1);
					vp[5] = mW(1,1)-1;
					vp[2] = max(vp[2],-0.9f);
				}
			}
			else
			{
				// Inverse
				Pose vpi;
				double fDet = 1.0+vDP[5]-vDP[1]*vDP[7]+vDP[2]+vDP[2]*vDP[5]-vDP[2]*vDP[1]*vDP[7]-vDP[3]*vDP[4]+vDP[3]*vDP[0]*vDP[7]+vDP[6]*vDP[4]*vDP[1]-vDP[6]*vDP[0]-vDP[6]*vDP[0]*vDP[5];
				double fDetAlpha = fDet*((1.0+vDP[2])*(1.0+vDP[5])-vDP[3]*vDP[4]);
				vpi[0] = -vDP[0]-vDP[5]*vDP[0]+vDP[4]*vDP[1];
				vpi[1] = -vDP[1]-vDP[2]*vDP[1]+vDP[3]*vDP[0];
				vpi[2] = 1.0+vDP[5]-vDP[1]*vDP[7]-fDetAlpha;
				vpi[3] = -vDP[3]+vDP[1]*vDP[6];
				vpi[4] = -vDP[4]+vDP[0]*vDP[7];
				vpi[5] = 1.0+vDP[2]-vDP[0]*vDP[6]-fDetAlpha;
				vpi[6] = -vDP[6]-vDP[5]*vDP[6]+vDP[3]*vDP[7];
				vpi[7] = -vDP[7]-vDP[2]*vDP[7]+vDP[4]*vDP[6];
				vpi = vpi/fDetAlpha;

				// Composition
				Pose vpc = vp;
				vp[0] = vpc[0]+vpi[0]+vpc[2]*vpi[0]+vpc[4]*vpi[1];
				vp[1] = vpc[1]+vpi[1]+vpc[3]*vpi[0]+vpc[5]*vpi[1];
				vp[2] = vpc[2]+vpi[2]+vpc[2]*vpi[2]+vpc[4]*vpi[3]+vpc[0]*vpi[6]-vpc[6]*vpi[0]-vpc[7]*vpi[1];
				vp[3] = vpc[3]+vpi[3]+vpc[3]*vpi[2]+vpc[5]*vpi[3]+vpc[1]*vpi[6];
				vp[4] = vpc[4]+vpi[4]+vpc[2]*vpi[4]+vpc[4]*vpi[5]+vpc[0]*vpi[7];
				vp[5] = vpc[5]+vpi[5]+vpc[3]*vpi[4]+vpc[5]*vpi[5]+vpc[1]*vpi[7]-vpc[6]*vpi[0]-vpc[7]*vpi[1];
				vp[6] = vpc[6]+vpi[6]+vpc[6]*vpi[2]+vpc[7]*vpi[3];
				vp[7] = vpc[7]+vpi[7]+vpc[6]*vpi[4]+vpc[7]*vpi[5];
				vp = vp/(1.0+vpc[6]*vpi[0]+vpc[7]*vpi[1]);

				mW = _BuildWarpMatrix(vp);
			}
		}

		if(eModelType==MODEL_T && m_sAS.eModelType!=MODEL_T)
		{
			if (vDP[0] * vDP[0] + vDP[1] + vDP[1]<1.0)
			{
				eModelType = m_sAS.eModelType;
//					nEpsilon = m_nEpsilon;
				m_sTarget.nIterationsTOnly = nIt+1;
			}
		}
		else
		{
			float posediff=0; for (int i = 0; i < MAXPS; i++) posediff += vDP[i] * vDP[i];
			if(posediff<0.1) break;
		}
	}

	m_sTarget.nIterations = nIt+1;

	//m_sTarget.fTargetScore = m_sTarget.afCost[m_nMaxIterations-1];

	if(bRankDeficiency /*|| !_InsideImage(m_nImageWidth,m_nImageHeight,vp[0],vp[1])*/)
	{
		Clear();
	}

	// Rather dodgy motion model alpha/beta filter (ish)
	m_sTarget.vVelocity = 0.5*m_sTarget.vVelocity+0.5*(vp-vplast);
	m_sTarget.vVelocity[2] = 0.0;
	m_sTarget.vVelocity[3] = 0.0;

}

//---------------------------------------------------------------------------------

void PWPTracker::_ComputePFPBFromImage(const UChar4Image* cImage)
{
	if(!m_bHasTarget)
		return;

	// Image info
	const int imagewidth = m_nImageWidth;
	const int imageheight = m_nImageHeight;
	ImageInfo* pII = m_pasImageInfo;

	Histogram* const pcBH = m_pcBackHist;
	Histogram* const pcFH = m_sTarget.pcForeHist;

	const int nWI = cImage->noDims.x;
	const int nHI = cImage->noDims.y;

	const Vector4u *pIm = cImage->GetData(MEMORYDEVICE_CPU);

	const double fUniform = 1.0/double(pcBH->Bins());

	const double fPriorBackground = m_sAS.fPriorBackground;
	const double fPriorForeground = (1.0-m_sAS.fPriorBackground);

	for(int p=0; p<nHI*nWI; p++)
	{
		const unsigned char r = pIm[p].r;
		const unsigned char g = pIm[p].g;
		const unsigned char b = pIm[p].b;
		pII->LHB = fPriorBackground*(pcBH->GetBinVal(r,g,b)+fUniform);
		pII->LHF = fPriorForeground*(pcFH->GetBinVal(r,g,b)+fUniform);
		pII++;
	}
}

//---------------------------------------------------------------------------------

void PWPTracker::_ComputeCostImageFromHistogramsAndEdgeMap(const UChar4Image* cImage, Matrix3f* pmWDC /*= NULL*/)
{
	Histogram* const pcFH = m_sTarget.pcForeHist;
	Histogram* const pcBH = m_pcBackHist;
	const double fUniform = 1.0/m_pcBackHist->Bins();

	// Image info
	const int imagewidth = m_nImageWidth;
	const int imageheight = m_nImageHeight;
	ImageInfo* const pII = m_pasImageInfo;
	
	// Level set
	const int rowlength0 = m_sTarget.nBGWidth;
	LevelSetInfo* pLS0 = m_sTarget.pasLevelSet;

	Pose& vp = m_sTarget.vPose;

	// Build warp matrix
	Matrix3f mW = _BuildWarpMatrix(vp);
	
	if(pmWDC)
		mW = mW*(*pmWDC);

	// Compute warps
	const double W_00 = mW(0,0);
	const double W_01 = mW(0,1);
	const double W_02 = mW(0,2);
	const double W_10 = mW(1,0);
	const double W_11 = mW(1,1);
	const double W_12 = mW(1,2);

	const double WDC_00 = pmWDC?(*pmWDC)(0,0):1.0;
	const double WDC_01 = pmWDC?(*pmWDC)(0,1):0.0;
	const double WDC_02 = pmWDC?(*pmWDC)(0,2):0.0;
	const double WDC_10 = pmWDC?(*pmWDC)(1,0):0.0;
	const double WDC_11 = pmWDC?(*pmWDC)(1,1):1.0;
	const double WDC_12 = pmWDC?(*pmWDC)(1,2):0.0;

	const int nWI = cImage->noDims.x;
	const int nHI = cImage->noDims.y;

	const int ybegin0 = -m_sTarget.nBGHalfHeight;
	const int yend0 = m_sTarget.nBGHalfHeight;
	const int xbegin0 = -m_sTarget.nBGHalfWidth;
	const int xend0 =m_sTarget.nBGHalfWidth;

	const int MFGHalfHeight = -m_sTarget.nFGHalfHeight;
	const int FGHalfHeight = m_sTarget.nFGHalfHeight;
	const int MFGHalfWidth = -m_sTarget.nFGHalfWidth;
	const int FGHalfWidth = m_sTarget.nFGHalfWidth;

	for(int r=0,y=ybegin0; y<=yend0; r++, y++)
	{
		for(int c=0,x=xbegin0; x<=xend0; c++, x++)
		{
			LevelSetInfo* pCLS = pLS0+r*rowlength0+c;
					
			if(y<ybegin0+BGBORDERSIZE || y>yend0-BGBORDERSIZE || x<xbegin0+BGBORDERSIZE || x>xend0-BGBORDERSIZE)
			{

				pCLS->PF = 0.0;
				pCLS->PB = 1.0;
				continue;
			}

			const int ci = int(W_00*x+W_01*y+W_02+0.5);
			const int ri = int(W_10*x+W_11*y+W_12+0.5);

			if(_InsideImage(nWI,nHI,ci,ri))
			{		
				pCLS->PF = pII[ri*imagewidth+ci].LHF;
				pCLS->PB = pII[ri*imagewidth+ci].LHB;
			}
			else
			{
				pCLS->PF = 0.0;
				pCLS->PB = fUniform;
			}
		}
	}
}

//---------------------------------------------------------------------------------

void PWPTracker::_InitialiseLevelSet()
{
	// Level set
	const int rowlength = m_sTarget.nBGWidth;
	LevelSetInfo* pLS = m_sTarget.pasLevelSet;

	// Initialise level set
	double fMaxU = -1e30;
	double fMinU = 1e30;

	double aa = double(m_sTarget.nFGHalfWidth)*double(m_sTarget.nFGHalfWidth);
	double bb = double(m_sTarget.nFGHalfHeight)*double(m_sTarget.nFGHalfHeight);
	double fScale = 0.5*min(double(m_sTarget.nFGHalfWidth),double(m_sTarget.nFGHalfHeight));

	for(int r=0,cr=-m_sTarget.nBGHalfHeight; cr<=m_sTarget.nBGHalfHeight; r++, cr++)
	{
		for(int c=0,cc=-m_sTarget.nBGHalfWidth; cc<=m_sTarget.nBGHalfWidth; c++, cc++)
		{
			LevelSetInfo* pCLS = pLS+r*rowlength+c;

			double fRad = sqrt(double(cr*cr)/bb+double(cc*cc)/aa)-1;
			const double fU = -fRad*fScale;

			//const double fU = (pCLS->PF-pCLS->PB);
			pCLS->U = fU;

			if(fU>fMaxU)
				fMaxU = fU;
			if(fU<fMinU)
				fMinU = fU;
		}
	}

	m_sTarget.fMaxU = fMaxU;
	m_sTarget.fMinU = fMinU;

	LevelSetInfo* pLS0 = m_sTarget.pasLevelSet;
	const int rowlength0 = m_sTarget.nBGWidth;
	const int rbegin0 = 1;
	const int ybegin0 = -m_sTarget.nBGHalfHeight+1;
	const int yend0 = m_sTarget.nBGHalfHeight-1;
	const int cbegin0 = 1;
	const int xbegin0 = -m_sTarget.nBGHalfWidth+1;
	const int xend0 = m_sTarget.nBGHalfWidth-1;

	// Compute Derivatives of Level Set			
	for(int r=rbegin0,_y=ybegin0; _y<=yend0; r++, _y++)
	{
		for(int c=cbegin0,_x=xbegin0; _x<=xend0; c++, _x++)
		{
			LevelSetInfo* pCLS = pLS0 + r*rowlength0 + c;

			// Derivative
			double fUDX = (pLS0[r*rowlength0+(c+1)].U-pLS0[r*rowlength0+(c-1)].U)*0.5;
			double fUDY = (pLS0[(r+1)*rowlength0+c].U-pLS0[(r-1)*rowlength0+c].U)*0.5;

			// Magnitude of derivative
			double fNormDU = 1.0/(sqrt(fUDX*fUDX+fUDY*fUDY)+1e-30);
			
			// Normalised derivative
			double fNUDX = fUDX*fNormDU;
			double fNUDY = fUDY*fNormDU;

			// Copy into structure
			pCLS->DX = fUDX;
			pCLS->DY = fUDY;
			pCLS->NDX = fNUDX;
			pCLS->NDY = fNUDY;
		}
	}
}

//---------------------------------------------------------------------------------

void PWPTracker::_EnforceNeumannBoundaryConditions()
{
	const int nW = m_sTarget.nBGWidth;
	const int nH = m_sTarget.nBGHeight;
	// Level set
	const int rowlength = m_sTarget.nBGWidth;
	LevelSetInfo* pLS = m_sTarget.pasLevelSet;
	
	for(int r=0; r<nH; r++)
	{
		pLS[r*rowlength].U = pLS[r*rowlength+1].U;
		pLS[r*rowlength+nW-1].U = pLS[r*rowlength+nW-2].U;
	}

	for(int c=0; c<nW; c++)
	{
		pLS[c].U = pLS[rowlength+c].U;
		pLS[(nH-1)*rowlength+c].U = pLS[(nH-2)*rowlength+c].U;
	}
}

//---------------------------------------------------------------------------------

void PWPTracker::_IterateLevelSet(int nIterations, double fTimestep)
{
	double fMu;

	if (m_sAS.fMu*fTimestep>0.24)
		fMu = 0.24 / fTimestep;
	else
		fMu = m_sAS.fMu;

	Matrix3f mW = _BuildWarpMatrix(m_sTarget.vPose);

	LevelSetInfo* pLS0 = m_sTarget.pasLevelSet;
	const int rowlength0 = m_sTarget.nBGWidth;

	const int rbegin0 = 1;
	const int ybegin0 = -m_sTarget.nBGHalfHeight + 1;
	const int yend0 = m_sTarget.nBGHalfHeight - 1;
	const int cbegin0 = 1;
	const int xbegin0 = -m_sTarget.nBGHalfWidth + 1;
	const int xend0 = m_sTarget.nBGHalfWidth - 1;


	// Iterate PDEs
	for (int nIt = 0; nIt < nIterations; nIt++)
	{
		_EnforceNeumannBoundaryConditions();

		// Prior for different model types
		double fNf = 0.0;
		double fNb = 0.0;

		// Compute Derivatives of Level Set			
		for (int r = rbegin0, _y = ybegin0; _y <= yend0; r++, _y++)
		{
			for (int c = cbegin0, _x = xbegin0; _x <= xend0; c++, _x++)
			{
				LevelSetInfo* pCLS = pLS0 + r*rowlength0 + c;

				// Heaviside step function (blurred)
				pCLS->H = _Heaviside(pCLS->U, m_sAS.fEpsilon);

				// Nf and Nb used to give prior prob. of foreground or background
				fNf += pCLS->H;
				fNb += 1.0 - pCLS->H;

				// Derivative
				double fUDX = (pLS0[r*rowlength0 + (c + 1)].U - pLS0[r*rowlength0 + (c - 1)].U)*0.5;
				double fUDY = (pLS0[(r + 1)*rowlength0 + c].U - pLS0[(r - 1)*rowlength0 + c].U)*0.5;

				// Magnitude of derivative
				double fNormDU = 1.0 / (sqrt(fUDX*fUDX + fUDY*fUDY) + 1e-30);

				// Normalised derivative
				double fNUDX = fUDX*fNormDU;
				double fNUDY = fUDY*fNormDU;

				// Copy into structure
				pCLS->DX = fUDX;
				pCLS->DY = fUDY;
				pCLS->NDX = fNUDX;
				pCLS->NDY = fNUDY;
			}
		}

		const double fN = fNf + fNb;

		// Evolve Level Set
		for (int r = rbegin0+1, _y = ybegin0+1; _y <= yend0-1; r++, _y++)
		{
			for (int c = cbegin0+1, _x = xbegin0+1; _x <= xend0-1; c++, _x++)
			{
				LevelSetInfo* pCLS = pLS0 + r*rowlength0 + c;

				// Dirac
				double fDiracU = _Dirac(pCLS->U, m_sAS.fEpsilon);

				// Compute curvature
				const double fDNX = (pLS0[r*rowlength0 + (c + 1)].NDX - pLS0[r*rowlength0 + (c - 1)].NDX) / 2.0f;
				const double fDNY = (pLS0[(r + 1)*rowlength0 + c].NDY - pLS0[(r - 1)*rowlength0 + c].NDY) / 2.0f;
				const double fK = fDNX + fDNY;

				const double fDel2U = -(8.0f*pLS0[r*rowlength0 + c].U - (pLS0[(r - 1)*rowlength0 + c].U +
					pLS0[(r + 1)*rowlength0 + c].U +
					pLS0[r*rowlength0 + (c - 1)].U +
					pLS0[r*rowlength0 + (c + 1)].U +
					pLS0[(r - 1)*rowlength0 + (c + 1)].U +
					pLS0[(r - 1)*rowlength0 + (c - 1)].U +
					pLS0[(r + 1)*rowlength0 + (c + 1)].U +
					pLS0[(r + 1)*rowlength0 + (c - 1)].U));

				const double fJacNumer = (pCLS->PF - pCLS->PB);
				const double fJacDenom = (pCLS->PF*pCLS->H + pCLS->PB*(1.0f - pCLS->H));

				double fWeightedDataTerm;

				if (m_sAS.eOpinionPoolType == OP_LOGARITHMIC)
					fWeightedDataTerm = m_sAS.fPsi*fDiracU*fJacNumer / fJacDenom;
				else
					fWeightedDataTerm = m_sAS.fPsi*fDiracU*fJacNumer / ((pCLS->PF*fNf + pCLS->PB*fNb) / fN);

				const double fPenalizingTerm = fMu*(1.0*fDel2U - fK);

				pCLS->U += fTimestep*(fWeightedDataTerm + fPenalizingTerm);
			}
		}
	}
	// Compute useful things for drawing level set
	double fMaxU = -1e30;
	double fMinU = 1e30;
	double fNormU = 0.0f;
	const int lssize = m_sTarget.nBGWidth*m_sTarget.nBGHeight;
	for(int p=0;p<lssize; p++)
	{
		double fU = (pLS0+p)->U;
		if(fU>fMaxU)
			fMaxU = fU;
		if(fU<fMinU)
			fMinU = fU;
		if(fU>0.0f)
			fNormU += fU;
	}	
	m_sTarget.fMaxU = fMaxU;
	m_sTarget.fMinU = fMinU;
	m_sTarget.fNormU = fNormU;
}

//---------------------------------------------------------------------------------

void PWPTracker::_ComputeHistogramsUsingSegmentation(const UChar4Image* cImage)
{
	Histogram* const pcBH = m_pcBackHist;
	bool bBlendBackground = m_sSS.fAppearanceLearningRate<1.0;

	if(bBlendBackground)
		pcBH->BeginAddingNewSamples();
	else
		pcBH->Clear();

	Histogram* const pcFH = m_sTarget.pcForeHist;
	bool bBlendForeground = m_sSS.fAppearanceLearningRate<1.0;
	
	if(bBlendForeground)
		pcFH->BeginAddingNewSamples();
	else
		pcFH->Clear();

	// Level set
	const int rowlength0 = m_sTarget.nBGWidth;
	LevelSetInfo* pLS0 = m_sTarget.pasLevelSet;

	// Build warp matrix
	const Matrix3f mW = _BuildWarpMatrix(m_sTarget.vPose);

	// Compute warps
	const double W_00 = mW(0, 0);
	const double W_01 = mW(0, 1);
	const double W_02 = mW(0, 2);
	const double W_10 = mW(1, 0);
	const double W_11 = mW(1, 1);
	const double W_12 = mW(1, 2);

	const int nWI = cImage->noDims.x;
	const int nHI = cImage->noDims.y;

	const int ybegin0 = -m_sTarget.nBGHalfHeight;
	const int yend0 = m_sTarget.nBGHalfHeight;
	const int xbegin0 = -m_sTarget.nBGHalfWidth;
	const int xend0 =m_sTarget.nBGHalfWidth;

	const Vector4u *imgptr = cImage->GetData(MEMORYDEVICE_CPU);

	for(int r=0, y=ybegin0; y<=yend0; r++,y++)
	{
		for(int c=0,x=xbegin0; x<=xend0; c++,x++)
		{
			const int ci = int(W_00*x+W_01*y+W_02+0.5);
			const int ri = int(W_10*x+W_11*y+W_12+0.5);

			const bool bInsideImage = _InsideImage(nWI,nHI,ci,ri);
			
			if(!bInsideImage)
				continue;

			const Vector4u &cPix = imgptr[ri*nWI + ci];

			double fH = _Heaviside(pLS0[r*rowlength0+c].U,m_sAS.fEpsilon);

			if(bBlendForeground)
				pcFH->AddNewSample(cPix.r,cPix.g,cPix.b,fH);
			else
				pcFH->AddSample(cPix.r,cPix.g,cPix.b,fH);

			if(bBlendBackground)
				pcBH->AddNewSample(cPix.r,cPix.g,cPix.b,1.0-fH);
			else
				pcBH->AddSample(cPix.r,cPix.g,cPix.b,1.0-fH);
		}
	}
	if(bBlendForeground)
		pcFH->CommitNewSamples(m_sSS.fAppearanceLearningRate);
	else
		pcFH->Normalise();

	if(bBlendBackground)
		pcBH->CommitNewSamples(m_sSS.fAppearanceLearningRate);
	else
		pcBH->Normalise();
}

//---------------------------------------------------------------------------------

void PWPTracker::AddTarget(UChar4Image *cImage, PWPBoundingBox cBB)
{
	int nW = cImage->noDims.x;
	int nH = cImage->noDims.y;

	// Check we are inside the image
	bool bInsideImage = _InsideImage(nW, nH, cBB.TLX-BGBORDERSIZE, cBB.BRY+BGBORDERSIZE);
	bInsideImage &= _InsideImage(nW, nH, cBB.TLX-BGBORDERSIZE, cBB.TLY-BGBORDERSIZE);
	bInsideImage &= _InsideImage(nW, nH, cBB.BRX+BGBORDERSIZE, cBB.TLY-BGBORDERSIZE);
	bInsideImage &= _InsideImage(nW, nH, cBB.BRX+BGBORDERSIZE, cBB.BRY+BGBORDERSIZE);

	if(!bInsideImage)
		return;

	// Initialise sizes that get used in loops
	double fWidth = cBB.BRX-cBB.TLX+1;
	double fHeight = cBB.BRY-cBB.TLY+1;
	
	double fScale = 1.0f;
	if(fWidth<fHeight)
		fScale = fWidth/double(PATCHSIZE);
	else
		fScale = fHeight/double(PATCHSIZE);

	m_sTarget.fOrigScale = fScale;

	fWidth /= fScale;
	fHeight /= fScale;

	m_sTarget.nFGHalfWidth = ceil(fWidth/2.0f);
	m_sTarget.nFGHalfHeight = ceil(fHeight/2.0f);
	m_sTarget.nBGHalfWidth = m_sTarget.nFGHalfWidth+BGBORDERSIZE;
	m_sTarget.nBGHalfHeight = m_sTarget.nFGHalfHeight+BGBORDERSIZE;
	m_sTarget.nBGWidth = 2*m_sTarget.nBGHalfWidth+1;
	m_sTarget.nBGHeight = 2*m_sTarget.nBGHalfHeight+1;

	// Add corners
	Point cPoint; cPoint[2] = 1.0;
	cPoint[0] = -m_sTarget.nFGHalfWidth-10;		cPoint[1] = -m_sTarget.nFGHalfHeight-10;	m_sTarget.vFGCorners.push_back(cPoint);
	cPoint[0] = m_sTarget.nFGHalfWidth+10;		cPoint[1] = -m_sTarget.nFGHalfHeight-10;	m_sTarget.vFGCorners.push_back(cPoint);
	cPoint[0] = m_sTarget.nFGHalfWidth+10;		cPoint[1] = m_sTarget.nFGHalfHeight+10;		m_sTarget.vFGCorners.push_back(cPoint);
	cPoint[0] = -m_sTarget.nFGHalfWidth-10;		cPoint[1] = m_sTarget.nFGHalfHeight+10;		m_sTarget.vFGCorners.push_back(cPoint);
	
	cPoint[0] = -m_sTarget.nBGHalfWidth;		cPoint[1] = -m_sTarget.nBGHalfHeight;		m_sTarget.vBGCorners.push_back(cPoint);
	cPoint[0] = m_sTarget.nBGHalfWidth;			cPoint[1] = -m_sTarget.nBGHalfHeight;		m_sTarget.vBGCorners.push_back(cPoint);
	cPoint[0] = m_sTarget.nBGHalfWidth;			cPoint[1] = m_sTarget.nBGHalfHeight;		m_sTarget.vBGCorners.push_back(cPoint);
	cPoint[0] = -m_sTarget.nBGHalfWidth;		cPoint[1] = m_sTarget.nBGHalfHeight;		m_sTarget.vBGCorners.push_back(cPoint);

	// Used for 3D view
	m_sDS.fRange = 1.5*max(m_sTarget.nBGWidth,m_sTarget.nBGHeight);

	// Create histogram distributions and stuff shared between targets
	if(!m_bHasTarget)
	{
		m_pcBackHist = new Histogram(m_sAS.nBitsY,m_sAS.nBitsU,m_sAS.nBitsV,m_sAS.fSigma);
		m_pasImageInfo = new ImageInfo[nH*nW];
		for(int p=0;p<nH*nW;p++)
			m_pasImageInfo[p].LHF=0.0;
		m_nImageWidth = nW;
		m_nImageHeight = nH;
	}

	m_sTarget.pcForeHist = new Histogram(m_sAS.nBitsY,m_sAS.nBitsU,m_sAS.nBitsV,m_sAS.fSigma);

	m_sTarget.pasLevelSet = new LevelSetInfo[m_sTarget.nBGHeight*m_sTarget.nBGWidth];
	m_sTarget.pmTemplate = new FloatImage(Vector2i(m_sTarget.nBGWidth, m_sTarget.nBGHeight), MEMORYDEVICE_CPU);
	m_sTarget.fTargetScore = m_sTarget.fLastTargetScore = 0.0f;
	m_sTarget.nIterations = 0;
	m_sTarget.bLost = false;
	m_sTarget.fForegroundProb = 1.0f;
	m_sTarget.fBackgroundProb = 1.0f;
	m_sTarget.ID = 0;
	m_sTarget.vVelocity.Clear(0.0);
		
	// Initialise pose vector
	Pose& vp = m_sTarget.vPose;

	double fTX = (cBB.BRX+cBB.TLX)/2.0f;
	double fTY = (cBB.TLY+cBB.BRY)/2.0f;
	double fTheta = 0.0f;
	m_sTarget.fAspectRatio = 1.0f;

	// Initialise pose parameters
	vp.Clear(0.0);

	vp[0] = fTX;
	vp[1] = fTY;

	if(m_sAS.eModelType == MODEL_TS)
	{
		vp[2] = fScale-1.0f;
	}
	else if(m_sAS.eModelType == MODEL_TSR)
	{
		vp[2] =	fScale-1.0f;
		vp[3] = fTheta;
	}
	else if(m_sAS.eModelType == MODEL_AFFINE)
	{
		// Using Baker Matthews group parameterisation for affine model
		vp[2] = fScale*cos(fTheta)-1.0;
		vp[3] = fScale*sin(fTheta);
		vp[4] = -fScale*sin(fTheta);
		vp[5] = fScale*cos(fTheta)-1.0;
	}
	else if(m_sAS.eModelType == MODEL_HOMOGRAPHY)
	{
		// Using Baker Matthews group parameterisation for homography model
		vp[2] = fScale*cos(fTheta)-1.0;
		vp[3] = fScale*sin(fTheta);
		vp[4] = -fScale*sin(fTheta);
		vp[5] = fScale*cos(fTheta)-1.0;
	}

	// Build warp matrix
	Matrix3f mW = _BuildWarpMatrix(vp);
	
	Histogram* const pcFH = m_sTarget.pcForeHist;
	Histogram* const pcBH = m_pcBackHist;
	pcFH->Clear();
	pcBH->Clear();

	Vector4u* img_ptr = cImage->GetData(MEMORYDEVICE_CPU);

	UChar4Image *tmpimg = new UChar4Image(cImage->noDims, MEMORYDEVICE_CPU);
	Vector4u* tmpimg_tpr = tmpimg->GetData(MEMORYDEVICE_CPU);
	tmpimg->Clear();

	for(int cr=-m_sTarget.nBGHalfHeight; cr<=m_sTarget.nBGHalfHeight; cr++)
	{
		for(int cc=-m_sTarget.nBGHalfWidth; cc<=m_sTarget.nBGHalfWidth; cc++)
		{
			const double fC = mW(0,0)*cc+mW(0,1)*cr+mW(0,2);
			const double fR = mW(1,0)*cc+mW(1,1)*cr+mW(1,2);
			const int nC = int(fC+0.5);
			const int nR = int(fR+0.5);

			if(!_InsideImage(nW,nH,nC,nR))
				continue;

			Vector4u &cPix = img_ptr[nR*nW + nC];

			if(cr>-m_sTarget.nFGHalfHeight && cr<m_sTarget.nFGHalfHeight && cc>-m_sTarget.nFGHalfWidth && cc<m_sTarget.nFGHalfWidth) // Inside box
			{
				pcFH->AddSample(cPix.r,cPix.g,cPix.b,1.0);
			}
			else
			{
				pcBH->AddSample(cPix.r,cPix.g,cPix.b,1.0);
			}
		}
	}

	pcFH->Normalise();
	pcBH->Normalise();

	// Ignore the prior for a new target
	float fOldAppearanceLRate = m_sSS.fAppearanceLearningRate;
	m_sSS.fAppearanceLearningRate = 0.4;

	// Increment the number of targets
	m_bHasTarget = true;
	_InitialiseLevelSet();

	if(!m_sAS.bUseSegmentation)
	{
		_ComputeHistogramsUsingSegmentation(cImage);
		_ComputePFPBFromImage(cImage);
		_ComputeCostImageFromHistogramsAndEdgeMap(cImage);
		m_sSS.fAppearanceLearningRate = fOldAppearanceLRate;
		return;
	}
	for(int i=0;i<2;i++)
	{
		_ComputeHistogramsUsingSegmentation(cImage);
		_ComputePFPBFromImage(cImage);
		_ComputeCostImageFromHistogramsAndEdgeMap(cImage);
		_IterateLevelSet(25,2.0);
	}

	_ComputeHistogramsUsingSegmentation(cImage);
	_ComputePFPBFromImage(cImage);
	_ComputeCostImageFromHistogramsAndEdgeMap(cImage);

	m_sSS.fAppearanceLearningRate = fOldAppearanceLRate;
}

//---------------------------------------------------------------------------------

void PWPTracker::Process(UChar4Image* cImage)
{
	if(m_sDS.bAnimateLevelSet)
	{
		m_sDS.fTheta = DegToRad(-90.0+50.0*sin(m_fLSAnimationTime));
		m_sDS.fPhi = DegToRad(90.0*cos(1.0*m_fLSAnimationTime));
		m_fLSAnimationTime += 6.28/15.0/30.0;
	}
	
	if(m_sDS.bDiscretizeImage)
	{
		_DiscretizeImage(cImage);
	}
	else
	{
		_DELETE(m_pcDiscretizedImage);
	}

	if(!m_bHasTarget)
		return;

	_ComputePFPBFromImage(cImage);
	_IterateShapeAlignment(cImage);
	
	if(!m_bHasTarget)
		return;

	if(m_sAS.bUseSegmentation)
	{
		if(m_sAS.bUseDriftCorrection)
		{
			Matrix3f mWDC = _ComputeDriftCorrection();
			_ComputeCostImageFromHistogramsAndEdgeMap(cImage, &mWDC);
		}
		else
		{
			_ComputeCostImageFromHistogramsAndEdgeMap(cImage);
		}


		_IterateLevelSet(m_sAS.nShapeIterations,m_sSS.fShapeLearningRate);
		_ComputeHistogramsUsingSegmentation(cImage);
	}

	if(m_sDS.bClassifyWholeImage)
	{
		_ClassifyImage(cImage);
	}
}

//---------------------------------------------------------------------------------

bool PWPTracker::GetSnapshotOfObject(UChar4Image* cImage, UChar4Image* cSnapshot)
{
	if(!m_bHasTarget)
		return false;

	const int nWI = cImage->noDims.x;
	const int nHI = cImage->noDims.y;

	// Build warp matrix
	const Matrix3f mW = _BuildWarpMatrix(m_sTarget.vPose);

	cSnapshot->ChangeDims(Vector2i(m_sTarget.nBGWidth, m_sTarget.nBGHeight));

	const double W_00 = mW(0, 0);
	const double W_01 = mW(0, 1);
	const double W_02 = mW(0, 2);
	const double W_10 = mW(1, 0);
	const double W_11 = mW(1, 1);
	const double W_12 = mW(1, 2);
	
	const int ybegin0 = -m_sTarget.nBGHalfHeight;
	const int yend0 = m_sTarget.nBGHalfHeight;
	const int xbegin0 = -m_sTarget.nBGHalfWidth;
	const int xend0 = m_sTarget.nBGHalfWidth;

	Vector4u *ssptr = cSnapshot->GetData(MEMORYDEVICE_CPU);
	Vector4u *imgptr = cImage->GetData(MEMORYDEVICE_CPU);

	for(int r=0, y=ybegin0; y<=yend0; r++,y++)
	{
		for(int c=0,x=xbegin0; x<=xend0; c++,x++)
		{
			const double ci = W_00*x+W_01*y+W_02;
			const double ri = W_10*x+W_11*y+W_12;

			// Find colour by bi-linear interpolation
			const int x0=(int)ci;
			const int y0=(int)ri;

			if(x0 >=  nWI - 1 || x0 <= 0 || y0 >=  nHI - 1 || y0 <= 0)
			{
				ssptr[r*cSnapshot->noDims.x + c] = Vector4u(0,0,0,0);
				continue;
			}

			const double xfrac=ci-x0;
			const double yfrac=ri-y0;

			double mm_frac = (1-xfrac) * (1-yfrac);
			double mp_frac = (1-xfrac) * (yfrac);
			double pm_frac = (xfrac)   * (1-yfrac);
			double pp_frac = (xfrac)   * (yfrac);
  
			const Vector4u& mm = imgptr[y0 * cImage->noDims.x + x0];
			const Vector4u& pm = imgptr[y0 * cImage->noDims.x + (x0+1)];
			const Vector4u& mp = imgptr[(y0+1) * cImage->noDims.x + x0];
			const Vector4u& pp = imgptr[(y0+1) * cImage->noDims.x + (x0+1)];
  
			ssptr[r*cSnapshot->noDims.x + c].r = (mm_frac*mm.r + mp_frac*mp.r + pm_frac*pm.r + pp_frac*pp.r);
			ssptr[r*cSnapshot->noDims.x + c].g = (mm_frac*mm.g + mp_frac*mp.g + pm_frac*pm.g + pp_frac*pp.g);
			ssptr[r*cSnapshot->noDims.x + c].b = (mm_frac*mm.b + mp_frac*mp.b + pm_frac*pm.b + pp_frac*pp.b);
		}
	}

	return true;
}

//---------------------------------------------------------------------------------

bool PWPTracker::GetPose(Pose &vPose)
{
	if(!m_bHasTarget)
		return false;

	vPose =  m_sTarget.vPose;

	vPose[0] -= m_nImageWidth/2;
	vPose[1] -= m_nImageHeight/2;
	vPose[2] = (1.0+vPose[2]);
	vPose[0] *= -1.0;

	return true;
}

//---------------------------------------------------------------------------------

void PWPTracker::Draw(GLuint nTexID /*= -1*/, int nTW /*= 0*/, int nTH /*= 0*/)
{
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_BLEND);	
	glDisable(GL_LINE_SMOOTH);
	glDisable(GL_POLYGON_SMOOTH);
	glDisable(GL_DEPTH_TEST);

	if(m_pcDiscretizedImage)
	{
		Vector4u *dcimg_ptr = m_pcDiscretizedImage->GetData(MEMORYDEVICE_CPU);

		glBegin(GL_QUADS);
		for(unsigned int r=0; r<m_pcDiscretizedImage->noDims.y; r++)
		{
			for(unsigned int c=0; c<m_pcDiscretizedImage->noDims.x; c++)
			{
				Vector4u &dc_pix = dcimg_ptr[r*m_pcDiscretizedImage->noDims.x + c];
				glColor4f(dc_pix.r / 255.0f, dc_pix.g / 255.0f, dc_pix.b / 255.0f, 1.0f);

				glVertex2f(c-0.5f,r-0.5f);	
				glVertex2f(c-0.5f,r+0.5f);	
				glVertex2f(c+0.5f,r+0.5f);	
				glVertex2f(c+0.5f,r-0.5f);	
			}
		}
		glEnd();
	}

	if(m_sDS.bClassifyWholeImage && m_bHasTarget)
	{
		glBegin(GL_QUADS);
		ImageInfo* pII = m_pasImageInfo;
		for(unsigned int r=0; r<m_nImageHeight; r++)
		{
			for(unsigned int c=0; c<m_nImageWidth; c++)
			{	
				if(pII->LHF>pII->LHB)
					glColor4f(1.0,1.0,1.0,1.0);
				else
					glColor4f(0.0,0.0,0.0,1.0);

				glVertex2f(c-0.5,r-0.5);
				glVertex2f(c+0.5,r-0.5);
				glVertex2f(c+0.5,r+0.5);
				glVertex2f(c-0.5,r+0.5);
				pII++; 
			}
		}
		glEnd();
	}

	if(!m_bHasTarget)
		return;

	// Image info
	const int imagewidth = m_nImageWidth;
	const int imageheight = m_nImageHeight;
	ImageInfo* pII = m_pasImageInfo;

	// Level set
	const int rowlength = m_sTarget.nBGWidth;
	LevelSetInfo* pLS = m_sTarget.pasLevelSet;
	const double fMaxU = m_sTarget.fMaxU;
	const double fMinU = m_sTarget.fMinU;

	Pose& vp = m_sTarget.vPose;

	glPushMatrix();
	//------------------------------------------------------------------------------------
		
		Matrix3f mW = _BuildWarpMatrix(vp);

		float aW[16];

		aW[0] = mW(0, 0);
		aW[1] = mW(1, 0);
		aW[2] = 0.0;
		aW[3] = mW(2, 0);

		aW[4] = mW(0, 1);
		aW[5] = mW(1, 1);
		aW[6] = 0.0;
		aW[7] = mW(2, 1);

		aW[8] = 0.0;
		aW[9] = 0.0;
		aW[10] = 1.0;
		aW[11] = 0.0;

		aW[12] = mW(0, 2);
		aW[13] = mW(1, 2);
		aW[14] = 0.0;
		aW[15] = mW(2, 2);

		glMultMatrixf(aW);

		if(!m_sDS.bClassifyWholeImage)
		{
			switch(m_sDS.eOverlayType)
			{
				case(OVERLAY_LEVELSET):
				{
					glBegin(GL_QUADS);
					for(int r=0,cr=-m_sTarget.nBGHalfHeight; cr<=m_sTarget.nBGHalfHeight; r++, cr++)
					{
						for(int c=0,cc=-m_sTarget.nBGHalfWidth; cc<=m_sTarget.nBGHalfWidth; c++, cc++)
						{
							const double fU = pLS[r*rowlength+c].U;
							const double fVal = (fU-fMinU)/(fMaxU-fMinU);
							glColor4f(1.0-fVal,fVal,0.0,1.0);
							glVertex2f(cc-0.5,cr-0.5);
							glVertex2f(cc+0.5,cr-0.5);
							glVertex2f(cc+0.5,cr+0.5);
							glVertex2f(cc-0.5,cr+0.5);
						}
					}
					glEnd();
					break;
				}
				case(OVERLAY_LIKELIHOOD):
				{
					double fMaxPF = -1e300;
					for(int r=0,cr=-m_sTarget.nBGHalfHeight; cr<=m_sTarget.nBGHalfHeight; r++, cr++)
						for(int c=0,cc=-m_sTarget.nBGHalfWidth; cc<=m_sTarget.nBGHalfWidth; c++, cc++)
							fMaxPF = max(fMaxPF,pLS[r*rowlength+c].PF);

					glBegin(GL_QUADS);
					for(int r=0,cr=-m_sTarget.nBGHalfHeight; cr<=m_sTarget.nBGHalfHeight; r++, cr++)
					{
						for(int c=0,cc=-m_sTarget.nBGHalfWidth; cc<=m_sTarget.nBGHalfWidth; c++, cc++)
						{
							const double fVal = pLS[r*rowlength+c].PF/fMaxPF;
							glColor4f(1.0-fVal,fVal,0.0,1.0);
							glVertex2f(cc-0.5,cr-0.5);
							glVertex2f(cc+0.5,cr-0.5);
							glVertex2f(cc+0.5,cr+0.5);
							glVertex2f(cc-0.5,cr+0.5);
						}
					}
					glEnd();
					break;
				}
				case(OVERLAY_POSTERIOR):
				{
					glBegin(GL_QUADS);
					for(int r=0,cr=-m_sTarget.nBGHalfHeight; cr<=m_sTarget.nBGHalfHeight; r++, cr++)
					{
						for(int c=0,cc=-m_sTarget.nBGHalfWidth; cc<=m_sTarget.nBGHalfWidth; c++, cc++)
						{
							const double fVal = pLS[r*rowlength+c].PF/(pLS[r*rowlength+c].PF+pLS[r*rowlength+c].PB);
							glColor4f(1.0-fVal,fVal,0.0,1.0);
							glVertex2f(cc-0.5,cr-0.5);
							glVertex2f(cc+0.5,cr-0.5);
							glVertex2f(cc+0.5,cr+0.5);
							glVertex2f(cc-0.5,cr+0.5);
						}
					}
					glEnd();
					break;
				}
			}
		}

		if(m_sDS.bDrawNarrowBand)
		{
			int nPixelsDraw = 0;
			glEnable(GL_BLEND);		
			glBegin(GL_QUADS);
			for(int r=1,cr=-m_sTarget.nBGHalfHeight+1; cr<=m_sTarget.nBGHalfHeight-1; r++, cr++)
			{
				for(int c=1,cc=-m_sTarget.nBGHalfWidth+1; cc<=m_sTarget.nBGHalfWidth-1; c++, cc++)
				{
					if(abs(pLS[r*rowlength+c].U)>m_sAS.fNarrowBand) continue;
					float fBlendVal = _Dirac(pLS[r*rowlength+c].U,m_sAS.fEpsilon)*m_sAS.fEpsilon*PI;
					glColor4f(1.0,1.0,1.0,fBlendVal*fBlendVal);
					glVertex2f(cc-0.5,cr-0.5);
					glVertex2f(cc+0.5,cr-0.5);
					glVertex2f(cc+0.5,cr+0.5);
					glVertex2f(cc-0.5,cr+0.5);
					nPixelsDraw++;
				}
			}
			glEnd();
			glDisable(GL_BLEND);		
		}

		float fBlendVal = max(0.1,m_sTarget.fForegroundProb);
		
		glEnable(GL_BLEND);	
		glEnable(GL_LINE_SMOOTH);
		/*glLineWidth(4.0);
		glColor4f(0.0,0.0,0.0,1.0);
		
		glBegin(GL_LINE_LOOP);
			glVertex2f(-m_sTarget.nFGHalfWidth, -m_sTarget.nFGHalfHeight);
			glVertex2f(m_sTarget.nFGHalfWidth, -m_sTarget.nFGHalfHeight);
			glVertex2f(m_sTarget.nFGHalfWidth, m_sTarget.nFGHalfHeight);
			glVertex2f(-m_sTarget.nFGHalfWidth, m_sTarget.nFGHalfHeight);
		glEnd();*/

		glColor4f(0.0,0.0,0.0,1.0);
		glLineWidth(2.0);
		glBegin(GL_LINE_LOOP);
			glVertex2f(-m_sTarget.nBGHalfWidth, -m_sTarget.nBGHalfHeight);	
			glVertex2f(m_sTarget.nBGHalfWidth, -m_sTarget.nBGHalfHeight);	
			glVertex2f(m_sTarget.nBGHalfWidth, m_sTarget.nBGHalfHeight);	
			glVertex2f(-m_sTarget.nBGHalfWidth, m_sTarget.nBGHalfHeight);	
		glEnd();
	//------------------------------------------------------------------------------------
	glPopMatrix();

	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);

	/*VNL::Vector<double> vPos(2); vPos[0] = m_nImageWidth/2; vPos[1] = m_nImageHeight/2;
	if(m_sTarget.mCov.Rows()>=2)
		CCB::DrawCovariance2D(vPos,m_sTarget.mCov.Extract(2,2),20.0,0.0,0.0,1.0,1.0);
	if(m_sTarget.mCov.Rows()>=4)
		CCB::DrawCovariance2D(vPos,m_sTarget.mCov.Extract(2,2,2,2),100.0,1.0,0.0,0.0,1.0);*/

	//// Draw foreground background probabilities
	//double fMaxBarHeight = 105.0f;
	//double fBarWidth = /*10.0f;*/15.0f;
	//double fSpacing = 2.0f;
	//double fCurrentX = m_nImageWidth-fSpacing;

	//glEnable(GL_BLEND);	
	//glEnable(GL_LINE_SMOOTH);

	//double fNormForeProb = m_sTarget.fForegroundProb;
	//glColor4f(1.0-fNormForeProb,fNormForeProb,0.0,1.0);
	//double fBarHeight = fNormForeProb*fMaxBarHeight;
	//glBegin(GL_QUADS);
	//	glVertex2f(fCurrentX, 0);	
	//	glVertex2f(fCurrentX, fBarHeight);	
	//	glVertex2f(fCurrentX-fBarWidth, fBarHeight);	
	//	glVertex2f(fCurrentX-fBarWidth, 0);	
	//glEnd();

	//glColor4f(0.0,0.0,0.0,1.0);
	//glBegin(GL_LINE_LOOP);
	//	glVertex2f(fCurrentX, 0);	
	//	glVertex2f(fCurrentX, fMaxBarHeight);	
	//	glVertex2f(fCurrentX-fBarWidth, fMaxBarHeight);	
	//	glVertex2f(fCurrentX-fBarWidth, 0);	
	//glEnd();

	//glColor4f(0.0,0.0,0.0,0.4);
	//glPushMatrix();
	//	glTranslatef(fCurrentX-2.1,3,0);
	//	glRotatef(90,0,0,1);
	//	CCB::GLUTText(0,0,0.06,"Fore Prob");
	//glPopMatrix();

	//// Move to next bar
	//fCurrentX-=fBarWidth+fSpacing;

	///*double fNormBackProb = m_sTarget.fBackgroundProb;
	//glColor4f(1-fNormBackProb,fNormBackProb,0.0,1.0);
	//fBarHeight = fNormBackProb*fMaxBarHeight;
	//glBegin(GL_QUADS);
	//	glVertex2f(fCurrentX, 0);	
	//	glVertex2f(fCurrentX, fBarHeight);	
	//	glVertex2f(fCurrentX-fBarWidth, fBarHeight);	
	//	glVertex2f(fCurrentX-fBarWidth, 0);	
	//glEnd();

	//glColor4f(0.0,0.0,0.0,1.0);
	//glBegin(GL_LINE_LOOP);
	//	glVertex2f(fCurrentX, 0);	
	//	glVertex2f(fCurrentX, fMaxBarHeight);	
	//	glVertex2f(fCurrentX-fBarWidth, fMaxBarHeight);	
	//	glVertex2f(fCurrentX-fBarWidth, 0);	
	//glEnd();

	//glColor4f(0.0,0.0,0.0,0.4);
	//glPushMatrix();
	//	glTranslatef(fCurrentX-2.1,3,0);
	//	glRotatef(90,0,0,1);
	//	CCB::GLUTText(0,0,0.06,"Back Prob");
	//glPopMatrix();

	//// Move to next bar
	//fCurrentX-=fBarWidth+fSpacing;*/

	//glColor4f(0.0,1.0,0.0,1.0);
	//fBarHeight = double(m_sTarget.nIterations)/double(m_nMaxIterations+1)*fMaxBarHeight;
	//glBegin(GL_QUADS);
	//	glVertex2f(fCurrentX, 0);	
	//	glVertex2f(fCurrentX, fBarHeight);	
	//	glVertex2f(fCurrentX-fBarWidth, fBarHeight);	
	//	glVertex2f(fCurrentX-fBarWidth, 0);	
	//glEnd();

	//glColor4f(0.0,0.5,0.0,1.0);
	//fBarHeight = double(m_sTarget.nIterationsTOnly)/double(m_nMaxIterations+1)*fMaxBarHeight;
	//glBegin(GL_QUADS);
	//	glVertex2f(fCurrentX, 0);	
	//	glVertex2f(fCurrentX, fBarHeight);	
	//	glVertex2f(fCurrentX-fBarWidth, fBarHeight);	
	//	glVertex2f(fCurrentX-fBarWidth, 0);	
	//glEnd();

	//glColor4f(0.0,0.0,0.0,1.0);
	//glBegin(GL_LINE_LOOP);
	//	glVertex2f(fCurrentX, 0);	
	//	glVertex2f(fCurrentX, fMaxBarHeight);	
	//	glVertex2f(fCurrentX-fBarWidth, fMaxBarHeight);	
	//	glVertex2f(fCurrentX-fBarWidth, 0);	
	//glEnd();

	//glColor4f(0.0,0.0,0.0,0.4);
	//glPushMatrix();
	//	/*glTranslatef(fCurrentX-2.1,1,0);
	//	glRotatef(90,0,0,1);
	//	CCB::GLUTText(0,0,0.06,"Iterations");*/

	//	glTranslatef(fCurrentX-2.1*1.5,1*1.5,0);
	//	glRotatef(90,0,0,1);
	//	CCB::GLUTText(0,0,0.06*1.5,"Iterations");
	//glPopMatrix();

	//glDisable(GL_POLYGON_SMOOTH);

	if(m_sDS.bOverlay2D && nTexID != -1)
	{
		Matrix3f mW = _BuildWarpMatrix(m_sTarget.vPose);

		Point cProjectedCorner1 = mW*m_sTarget.vFGCorners[0]; cProjectedCorner1=cProjectedCorner1/cProjectedCorner1[2];
		Point cProjectedCorner2 = mW*m_sTarget.vFGCorners[3]; cProjectedCorner2=cProjectedCorner2/cProjectedCorner2[2];
		Point cProjectedCorner3 = mW*m_sTarget.vFGCorners[2]; cProjectedCorner3=cProjectedCorner3/cProjectedCorner3[2];
		Point cProjectedCorner4 = mW*m_sTarget.vFGCorners[1]; cProjectedCorner4=cProjectedCorner4/cProjectedCorner4[2];

		const double fTW = nTW;
		const double fTH = nTH;

		cProjectedCorner1[0] /= fTW;
		cProjectedCorner2[0] /= fTW;
		cProjectedCorner3[0] /= fTW;
		cProjectedCorner4[0] /= fTW;
		cProjectedCorner1[1] /= fTH;
		cProjectedCorner2[1] /= fTH;
		cProjectedCorner3[1] /= fTH;
		cProjectedCorner4[1] /= fTH;

		double OVERLAYSIZE = m_sDS.n2DOverlaySize;

		if(m_sTarget.nBGHalfWidth>m_sTarget.nBGHalfHeight)
		{
			int nHeight = int(OVERLAYSIZE/double(m_sTarget.nBGHalfWidth)*double(m_sTarget.nBGHalfHeight)+0.5);
			glViewport(10,viewport[3]-10-nHeight,OVERLAYSIZE,nHeight);
		}
		else
			glViewport(10,viewport[3]-(OVERLAYSIZE+10.0),int(OVERLAYSIZE/double(m_sTarget.nBGHalfHeight)*double(m_sTarget.nBGHalfWidth)+0.5),OVERLAYSIZE);

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity(); 
		glOrtho(-m_sTarget.nBGHalfWidth,m_sTarget.nBGHalfWidth,-m_sTarget.nBGHalfHeight,m_sTarget.nBGHalfHeight,-200,200); 
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		glDisable(GL_BLEND);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, nTexID);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,GL_LINEAR);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		
		glColor4f(0.0,0.0,1.0,1.0);
		
			glBegin(GL_QUADS);
				glTexCoord2f(cProjectedCorner1[0],cProjectedCorner1[1]);glVertex2f(-m_sTarget.nBGHalfWidth, -m_sTarget.nBGHalfHeight);
				glTexCoord2f(cProjectedCorner2[0],cProjectedCorner2[1]);glVertex2f(-m_sTarget.nBGHalfWidth, m_sTarget.nBGHalfHeight);
				glTexCoord2f(cProjectedCorner3[0],cProjectedCorner3[1]);glVertex2f(m_sTarget.nBGHalfWidth, m_sTarget.nBGHalfHeight);
				glTexCoord2f(cProjectedCorner4[0],cProjectedCorner4[1]);glVertex2f(m_sTarget.nBGHalfWidth, -m_sTarget.nBGHalfHeight);
			glEnd();

		glDisable(GL_TEXTURE_2D);

		glLineWidth(4.0);
		glColor4f(0.0,0.0,1.0,1.0);
		
		glEnable(GL_BLEND);
		glBegin(GL_LINE_LOOP);
			glVertex2f(-m_sTarget.nBGHalfWidth, -m_sTarget.nBGHalfHeight);
			glVertex2f(m_sTarget.nBGHalfWidth, -m_sTarget.nBGHalfHeight);
			glVertex2f(m_sTarget.nBGHalfWidth, m_sTarget.nBGHalfHeight);
			glVertex2f(-m_sTarget.nBGHalfWidth, m_sTarget.nBGHalfHeight);
		glEnd();

		glColor4f(1.0,1.0,1.0,1.0);
		glBegin(GL_LINES);
			glVertex2f(-10,0);
			glVertex2f(10,0);
			glVertex2f(0,-10);
			glVertex2f(0,10);
		glEnd();

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();

		glViewport(viewport[0],viewport[1],viewport[2],viewport[3]);
	}

	glDisable(GL_LINE_SMOOTH);
	glDisable(GL_BLEND);

	if(m_sDS.bOverlay3D)
	{
		glViewport(viewport[2]-250,viewport[3]-180,300,200);
		Draw3D(nTexID, nTW, nTH);
		glViewport(viewport[0],viewport[1],viewport[2],viewport[3]);
	}
}

//---------------------------------------------------------------------------------

void PWPTracker::Draw3D(GLuint nTexID /*= -1*/, int nTW /*= 0*/, int nTH /*= 0*/)
{
	if(!m_sDS.bDrawLevelSet)
		return;

	if(!m_bHasTarget)
		return;

	// Level set
	const int rowlength = m_sTarget.nBGWidth;
	LevelSetInfo* pLS = m_sTarget.pasLevelSet;
	const double fMaxU = m_sTarget.fMaxU;
	const double fMinU = m_sTarget.fMinU;

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glColor4f(1.0,1.0,0.0,1.0);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity(); 

	const int SEGMENTATIONSIZE = max(m_sTarget.nBGWidth,m_sTarget.nBGHeight);

	gluPerspective(70.0,1.25,10,10*SEGMENTATIONSIZE);

	double fRSinPhi = m_sDS.fRange*sin(m_sDS.fPhi);
	double fRCosPhi = m_sDS.fRange*cos(m_sDS.fPhi);
	gluLookAt(fRCosPhi*cos(m_sDS.fTheta),fRCosPhi*sin(m_sDS.fTheta),fRSinPhi,0,0,0,0,0,fRCosPhi/abs(fRCosPhi));

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);	
	glEnable(GL_LINE_SMOOTH);
	glDisable(GL_POLYGON_SMOOTH);
	glDepthFunc(GL_LESS);
	glEnable(GL_DEPTH_TEST);
	

		Matrix3f mW = _BuildWarpMatrix(m_sTarget.vPose);

		Point cProjectedCorner1 = mW*m_sTarget.vBGCorners[0]; cProjectedCorner1=cProjectedCorner1/cProjectedCorner1[2];
		Point cProjectedCorner2 = mW*m_sTarget.vBGCorners[3]; cProjectedCorner2=cProjectedCorner2/cProjectedCorner2[2];
		Point cProjectedCorner3 = mW*m_sTarget.vBGCorners[2]; cProjectedCorner3=cProjectedCorner3/cProjectedCorner3[2];
		Point cProjectedCorner4 = mW*m_sTarget.vBGCorners[1]; cProjectedCorner4=cProjectedCorner4/cProjectedCorner4[2];

		const double fTW = nTW;
		const double fTH = nTH;

		cProjectedCorner1[0] /= fTW;
		cProjectedCorner2[0] /= fTW;
		cProjectedCorner3[0] /= fTW;
		cProjectedCorner4[0] /= fTW;
		cProjectedCorner1[1] /= fTH;
		cProjectedCorner2[1] /= fTH;
		cProjectedCorner3[1] /= fTH;
		cProjectedCorner4[1] /= fTH;

		glDisable(GL_BLEND);
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, nTexID);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,GL_LINEAR);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		
		glColor4f(0.0,0.0,1.0,1.0);
		
			glBegin(GL_QUADS);
				glTexCoord2f(cProjectedCorner1[0],cProjectedCorner1[1]);glVertex2f(-m_sTarget.nBGHalfWidth, -m_sTarget.nBGHalfHeight);
				glTexCoord2f(cProjectedCorner2[0],cProjectedCorner2[1]);glVertex2f(-m_sTarget.nBGHalfWidth, m_sTarget.nBGHalfHeight);
				glTexCoord2f(cProjectedCorner3[0],cProjectedCorner3[1]);glVertex2f(m_sTarget.nBGHalfWidth, m_sTarget.nBGHalfHeight);
				glTexCoord2f(cProjectedCorner4[0],cProjectedCorner4[1]);glVertex2f(m_sTarget.nBGHalfWidth, -m_sTarget.nBGHalfHeight);
			glEnd();

		glDisable(GL_TEXTURE_2D);


	glBegin(GL_QUADS);
	const double fNorm = 1.0/(fMaxU-fMinU);
	for(int r=BGBORDERSIZE,cr=-m_sTarget.nFGHalfHeight; cr<=m_sTarget.nFGHalfHeight; r++, cr++)
	{
		for(int c=BGBORDERSIZE,cc=-m_sTarget.nFGHalfWidth; cc<=m_sTarget.nFGHalfWidth; c++, cc++)
		{
			const double fU = pLS[r*rowlength+c].U;
			const double fVal = (fU-fMinU)*fNorm;
			glColor4f(fVal,1.0-fVal,0.0,1.0);
			glVertex3f(cc-0.5,cr-0.5,SEGMENTATIONSIZE*((pLS[(r-1)*rowlength+(c-1)].U+pLS[r*rowlength+(c-1)].U+pLS[(r-1)*rowlength+c].U+fU)/4.0f)*fNorm);
			glVertex3f(cc+0.5,cr-0.5,SEGMENTATIONSIZE*((pLS[(r-1)*rowlength+(c+1)].U+pLS[r*rowlength+(c+1)].U+pLS[(r-1)*rowlength+c].U+fU)/4.0f)*fNorm);
			glVertex3f(cc+0.5,cr+0.5,SEGMENTATIONSIZE*((pLS[(r+1)*rowlength+(c+1)].U+pLS[r*rowlength+(c+1)].U+pLS[(r+1)*rowlength+c].U+fU)/4.0f)*fNorm);
			glVertex3f(cc-0.5,cr+0.5,SEGMENTATIONSIZE*((pLS[(r+1)*rowlength+(c-1)].U+pLS[r*rowlength+(c-1)].U+pLS[(r+1)*rowlength+c].U+fU)/4.0f)*fNorm);
		}
	}
	glEnd();

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LINE_SMOOTH);
	glDisable(GL_BLEND);

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
}

bool PWPTracker::DrawTargeLSOverlay(UChar4Image* cImage)
{
	if (!m_bHasTarget)
		return false;

	const int nWI = cImage->noDims.x;
	const int nHI = cImage->noDims.y;

	// Build warp matrix
	const Matrix3f mW = _BuildWarpMatrix(m_sTarget.vPose);

	const double W_00 = mW(0, 0);
	const double W_01 = mW(0, 1);
	const double W_02 = mW(0, 2);
	const double W_10 = mW(1, 0);
	const double W_11 = mW(1, 1);
	const double W_12 = mW(1, 2);

	const int ybegin0 = -m_sTarget.nBGHalfHeight;
	const int yend0 = m_sTarget.nBGHalfHeight;
	const int xbegin0 = -m_sTarget.nBGHalfWidth;
	const int xend0 = m_sTarget.nBGHalfWidth;

	Vector4u *imgptr = cImage->GetData(MEMORYDEVICE_CPU);
	LevelSetInfo* pLS0 = m_sTarget.pasLevelSet;
	const int rowlength0 = m_sTarget.nBGWidth;

	for (int r = 0, y = ybegin0; y <= yend0; r++, y++)
	{
		for (int c = 0, x = xbegin0; x <= xend0; c++, x++)
		{
			const double fU = pLS0[r*rowlength0 + c].U;
			if (abs(fU) > 2)
				continue;

			const double ci = W_00*x + W_01*y + W_02;
			const double ri = W_10*x + W_11*y + W_12;

			// Find colour by bi-linear interpolation
			const int x0 = (int)ci;
			const int y0 = (int)ri;

			if (x0 >= nWI - 2 || x0 <= 1 || y0 >= nHI - 2 || y0 <= 1)continue;
			imgptr[y0 * cImage->noDims.x + x0] = Vector4u(0, 0, 255, 0);
			imgptr[(y0 + 1) * cImage->noDims.x + x0] = Vector4u(0, 0, 255, 0);
			imgptr[(y0 - 1)* cImage->noDims.x + x0] = Vector4u(0, 0, 255, 0);
			imgptr[y0 * cImage->noDims.x + (x0 + 1)] = Vector4u(0, 0, 255, 0);
			imgptr[y0 * cImage->noDims.x + (x0 - 1)] = Vector4u(0, 0, 255, 0);
		}
	}


	return true;
}

void PWPTracker::GetBoundingBox(Vector2f* ptlist)
{
	const int ybegin0 = -m_sTarget.nBGHalfHeight;
	const int yend0 = m_sTarget.nBGHalfHeight;
	const int xbegin0 = -m_sTarget.nBGHalfWidth;
	const int xend0 = m_sTarget.nBGHalfWidth;

	const Matrix3f mW = _BuildWarpMatrix(m_sTarget.vPose);

	const double W_00 = mW(0, 0);
	const double W_01 = mW(0, 1);
	const double W_02 = mW(0, 2);
	const double W_10 = mW(1, 0);
	const double W_11 = mW(1, 1);
	const double W_12 = mW(1, 2);

	float tlc = W_00*xbegin0 + W_01*ybegin0 + W_02;
	float tlr = W_10*xbegin0 + W_11*ybegin0 + W_12;
	float brc = W_00*xend0 + W_01*yend0 + W_02;
	float brr = W_10*xend0 + W_11*yend0 + W_12;

	float trc = W_00*xend0 + W_01*ybegin0 + W_02;
	float trr = W_10*xend0 + W_11*ybegin0 + W_12;
	float blc = W_00*xbegin0 + W_01*yend0 + W_02;
	float blr = W_10*xbegin0 + W_11*yend0 + W_12;

	ptlist[0] = Vector2f(tlc, tlr);
	ptlist[1] = Vector2f(trc, trr);
	ptlist[2] = Vector2f(brc, brr);
	ptlist[3] = Vector2f(blc, blr);
}

//---------------------------------------------------------------------------------