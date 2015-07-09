#pragma once

#include <math.h>
#include <algorithm>

using namespace std;

namespace pwp
{
	
	class Histogram
	{
		public:
			Histogram(const int nBitsA,const int nBitsB,const int nBitsC, const double fSigma):	
																									m_nBinsA(int(pow(2.0,double(nBitsA)))),
																									m_nBinsB(int(pow(2.0,double(nBitsB)))),
																									m_nBinsC(int(pow(2.0,double(nBitsC)))),
																									m_nShiftA(8-nBitsA),
																									m_nShiftB(8-nBitsB),
																									m_nShiftC(8-nBitsC),
																									m_fSigma(fSigma)
			{
				double fSum = 0.0;
				m_nSigmaLimit = (int)ceil(2.0f*m_fSigma);
				double fSig = m_fSigma+1e-30;
	
				for(int i=0;i<=m_nSigmaLimit;i++)
				{
					for(int j=0;j<=m_nSigmaLimit;j++)
					{
						for(int k=0;k<=m_nSigmaLimit;k++)
						{
							m_afWeights[i][j][k] = exp(-double(i*i+j*j+k*k)/fSig/fSig/2.0);
							fSum += m_afWeights[i][j][k];
						}
					}
				}
	
				for(int i=0;i<=m_nSigmaLimit;i++)
					for(int j=0;j<=m_nSigmaLimit;j++)
						for(int k=0;k<=m_nSigmaLimit;k++)
							m_afWeights[i][j][k] /= fSum;
	
				Clear();
			}
	
			~Histogram() {}
	
			inline double		ComputeBhattacharyyaCoeff(Histogram& cP);
			inline double		ComputeKLDiv(Histogram& cP);
	
			inline void			AddSample(int nA, int nB, int nC, double fWeight);
			inline void			Normalise();
			inline void			SetUniform();
			inline double		GetBinVal(int nA, int nB, int nC);
			inline double		MaxValue();
			inline double		MinValue();
			inline void			ScaleBy(double fScale);
			inline void         Clear();
			inline int			Bins()		{return m_nBinsA*m_nBinsB*m_nBinsC;}
			inline int			NonZeroBins();
			inline double		BinVolume()	{return 1.0f/double(m_nBinsA*m_nBinsB*m_nBinsC);}
			inline double		CheckNorm();
	
			inline void			BeginAddingNewSamples();
			inline void			AddNewSample(int nA, int nB, int nC, double fWeight);
			inline void			CommitNewSamples(double fPrior);
	
		protected:
			static const int MAX_BINS = 32;
			
			const int m_nBinsA, m_nBinsB, m_nBinsC, m_nShiftA, m_nShiftB, m_nShiftC;
			const double m_fSigma;
	
			int	m_nSigmaLimit;
	
			double				m_afWeights[16][16][16];
	
			double				m_afHistogram[MAX_BINS][MAX_BINS][MAX_BINS];
			double				m_fHistNorm;
			double				m_afNewHistogram[MAX_BINS][MAX_BINS][MAX_BINS];
			double				m_fNewHistNorm;
			double				m_fNewHistProbWeightNorm;
	};
	
	//----------------------------------------------------------------------
	
	void Histogram::AddSample(int nA, int nB, int nC, double fWeight)
	{
		//m_afHistogram[nA >> m_nShiftA][nB >> m_nShiftB][nC >> m_nShiftC] += fWeight;
		//m_fHistNorm += fWeight;
	
		nA = nA>>m_nShiftA;
		nB = nB>>m_nShiftB;
		nC = nC>>m_nShiftC;
	
		for(int i=max(0,nA-m_nSigmaLimit); i<=min(m_nBinsA-1,nA+m_nSigmaLimit); i++)
		{
			for(int j=max(0,nB-m_nSigmaLimit); j<=min(m_nBinsB-1,nB+m_nSigmaLimit); j++)
			{
				for(int k=max(0,nC-m_nSigmaLimit); k<=min(m_nBinsC-1,nC+m_nSigmaLimit); k++)
				{
					double fW = m_afWeights[abs(i-nA)][abs(j-nB)][abs(k-nC)]*fWeight;
					m_afHistogram[i][j][k] += fW;
					m_fHistNorm += fW;
				}
			}
		}
	
	}
	
	//----------------------------------------------------------------------
	
	void Histogram::Normalise()
	{
		if(m_fHistNorm > 0.0f)
		{
			for(int i=0; i<m_nBinsA; i++)
				for(int j=0; j<m_nBinsB; j++)
					for(int k=0; k<m_nBinsC; k++)
						if(m_afHistogram[i][j][k])
							m_afHistogram[i][j][k] /= m_fHistNorm;
	
			m_fHistNorm = 1.0;
		}
	}
	
	//----------------------------------------------------------------------
	
	double Histogram::CheckNorm()
	{
		double fNorm = 0.0f;
		for(int i=0; i<m_nBinsA; i++)
			for(int j=0; j<m_nBinsB; j++)
				for(int k=0; k<m_nBinsC; k++)
					fNorm+=m_afHistogram[i][j][k];
		return fNorm;
	}
	
	//----------------------------------------------------------------------
	
	double Histogram::MaxValue()
	{
		double fMaxVal = -1e30f;
	
		for(int i=0; i<m_nBinsA; i++)
		{
			for(int j=0; j<m_nBinsB; j++)
			{
				for(int k=0; k<m_nBinsC; k++)
				{
					fMaxVal = max(fMaxVal,m_afHistogram[i][j][k]);
				}
			}
		}
	
		return fMaxVal;
	}
	
	
	//----------------------------------------------------------------------
	
	double Histogram::MinValue()
	{
		double fMinVal = 1e30f;
	
		for(int i=0; i<m_nBinsA; i++)
		{
			for(int j=0; j<m_nBinsB; j++)
			{
				for(int k=0; k<m_nBinsC; k++)
				{
					if(m_afHistogram[i][j][k]!=0.0f)
						fMinVal = min(fMinVal,m_afHistogram[i][j][k]);
				}
			}
		}
	
		return fMinVal;
	}
	
	//----------------------------------------------------------------------
	
	void Histogram::ScaleBy(double fScale)
	{
		for(int i=0; i<m_nBinsA; i++)
			for(int j=0; j<m_nBinsB; j++)
				for(int k=0; k<m_nBinsC; k++)
					m_afHistogram[i][j][k] *= fScale;
	}
	
	//----------------------------------------------------------------------
	
	void Histogram::SetUniform()
	{
		for(int i=0; i<m_nBinsA; i++)
			for(int j=0; j<m_nBinsB; j++)
				for(int k=0; k<m_nBinsC; k++)
					m_afHistogram[i][j][k] = 1.0;
		
		m_fHistNorm = double(m_nBinsA*m_nBinsB*m_nBinsC);
	}
	
	//----------------------------------------------------------------------
	
	double Histogram::GetBinVal(int nA, int nB, int nC)
	{
		return m_afHistogram[nA>>m_nShiftA][nB>>m_nShiftB][nC>>m_nShiftC]/*/m_fHistNorm*/;
	}
	
	//----------------------------------------------------------------------
	
	void Histogram::Clear()
	{
		m_fHistNorm=0.0f;
	
		for(int i=0; i<m_nBinsA; i++)
			for(int j=0; j<m_nBinsB; j++)
				for(int k=0; k<m_nBinsC; k++)
					m_afHistogram[i][j][k] = 0.0;
	}
	
	//----------------------------------------------------------------------
	
	double Histogram::ComputeBhattacharyyaCoeff(Histogram& cP)
	{
		double fBhatt = 0.0f;
		for(int i=0; i<m_nBinsA; i++)
			for(int j=0; j<m_nBinsB; j++)
				for(int k=0; k<m_nBinsC; k++)
					fBhatt += sqrt(m_afHistogram[i][j][k]*cP.m_afHistogram[i][j][k]);
		return fBhatt;
	}
	
	//----------------------------------------------------------------------
	
	double Histogram::ComputeKLDiv(Histogram& cP)
	{
		double fKLDiv = 0.0f;
		for(int i=0; i<m_nBinsA; i++)
			for(int j=0; j<m_nBinsB; j++)
				for(int k=0; k<m_nBinsC; k++)
					fKLDiv += m_afHistogram[i][j][k]*log((m_afHistogram[i][j][k]+1e-200)/(cP.m_afHistogram[i][j][k]+1e-200));
		return fKLDiv;
	}
	
	//----------------------------------------------------------------------
	
	void Histogram::BeginAddingNewSamples()
	{
		m_fNewHistNorm=0.0;
		m_fNewHistProbWeightNorm=0.0;
		
		for(int i=0; i<m_nBinsA; i++)
			for(int j=0; j<m_nBinsB; j++)
				for(int k=0; k<m_nBinsC; k++)
					m_afNewHistogram[i][j][k] = 0.0;
	}
	
	//----------------------------------------------------------------------
	
	void Histogram::AddNewSample(int nA, int nB, int nC, double fWeight)
	{
		nA = nA>>m_nShiftA;
		nB = nB>>m_nShiftB;
		nC = nC>>m_nShiftC;
	
		double fW = fWeight;
		m_afNewHistogram[nA][nB][nC] += fW;
		m_fNewHistProbWeightNorm += fW;
		m_fNewHistNorm += 1.0;
	}
	
	//----------------------------------------------------------------------
	
	void Histogram::CommitNewSamples(double fPrior)
	{
		//fPrior *= m_fNewHistProbWeightNorm/m_fNewHistNorm;
	
		// Normalise
		if(m_fNewHistProbWeightNorm > 0.0f)
		{
			for(int i=0; i<m_nBinsA; i++)
				for(int j=0; j<m_nBinsB; j++)
					for(int k=0; k<m_nBinsC; k++)
						if(m_afNewHistogram[i][j][k])
							m_afNewHistogram[i][j][k] /= m_fNewHistProbWeightNorm;
	
			m_fNewHistProbWeightNorm = 1.0;
		}
	
		// Marginalise over two distributions
		for(int i=0; i<m_nBinsA; i++)
			for(int j=0; j<m_nBinsB; j++)
				for(int k=0; k<m_nBinsC; k++)
						m_afHistogram[i][j][k] = m_afNewHistogram[i][j][k]*fPrior+m_afHistogram[i][j][k]*(1.0-fPrior);
	}
	
	//----------------------------------------------------------------------
	
	int Histogram::NonZeroBins()
	{
		int nRetVal = 0;
		for(int i=0; i<m_nBinsA; i++)
			for(int j=0; j<m_nBinsB; j++)
				for(int k=0; k<m_nBinsC; k++)
					if(m_afNewHistogram[i][j][k])
						nRetVal++;
	}
	
	//----------------------------------------------------------------------
	//----------------------------------------------------------------------
	//----------------------------------------------------------------------
	//----------------------------------------------------------------------
	
	
	class GreyScaleHistogram
	{
		public:
			GreyScaleHistogram(const int nBitsA,const double fSigma):	
																									m_nBinsA(int(pow(2.0,double(nBitsA)))),
																									m_nShiftA(8-nBitsA),
																									m_fSigma(fSigma)
			{
				double fSum = 0.0;
				m_nSigmaLimit = (int)ceil(2.0f*m_fSigma);
				double fSig = m_fSigma+1e-30;
	
				for(int i=0;i<=m_nSigmaLimit;i++)
				{
					m_afWeights[i] = exp(-double(i*i)/fSig/fSig/2.0);
					fSum += m_afWeights[i];
				}
	
				for(int i=0;i<=m_nSigmaLimit;i++)
							m_afWeights[i] /= fSum;
	
				Clear();
			}
	
			~GreyScaleHistogram() {}
	
			inline double		ComputeBhattacharyyaCoeff(GreyScaleHistogram& cP);
			inline double		ComputeKLDiv(GreyScaleHistogram& cP);
	
			inline void			AddSample(int nA, double fWeight);
			inline void			Normalise();
			inline void			SetUniform();
			inline double		GetBinVal(int nA);
			inline double		MaxValue();
			inline double		MinValue();
			inline void			ScaleBy(double fScale);
			inline void         Clear();
			inline int			Bins()		{return m_nBinsA;}
			inline int			NonZeroBins();
			inline double		BinVolume()	{return 1.0f/double(m_nBinsA);}
			inline double		CheckNorm();
	
			inline void			BeginAddingNewSamples();
			inline void			AddNewSample(int nA, double fWeight);
			inline void			CommitNewSamples(double fPrior);
	
		protected:
			static const int MAX_BINS = 256;
			
			const int m_nBinsA, m_nShiftA;
			const double m_fSigma;
	
			int	m_nSigmaLimit;
	
			double				m_afWeights[16];
	
			double				m_afGreyScaleHistogram[MAX_BINS];
			double				m_fHistNorm;
			double				m_afNewGreyScaleHistogram[MAX_BINS];
			double				m_fNewHistNorm;
			double				m_fNewHistProbWeightNorm;
	};
	
	//----------------------------------------------------------------------
	
	void GreyScaleHistogram::AddSample(int nA, double fWeight)
	{
		nA = nA>>m_nShiftA;
	
		for(int i=max(0,nA-m_nSigmaLimit); i<=min(m_nBinsA-1,nA+m_nSigmaLimit); i++)
		{
			double fW = m_afWeights[abs(i-nA)]*fWeight;
			m_afGreyScaleHistogram[i] += fW;
			m_fHistNorm += fW;
		}
	}
	
	//----------------------------------------------------------------------
	
	void GreyScaleHistogram::Normalise()
	{
		if(m_fHistNorm > 0.0f)
		{
			for(int i=0; i<m_nBinsA; i++)
				if(m_afGreyScaleHistogram[i])
					m_afGreyScaleHistogram[i] /= m_fHistNorm;
			m_fHistNorm = 1.0;
		}
	}
	
	//----------------------------------------------------------------------
	
	double GreyScaleHistogram::CheckNorm()
	{
		double fNorm = 0.0f;
		for(int i=0; i<m_nBinsA; i++)
			fNorm+=m_afGreyScaleHistogram[i];
		return fNorm;
	}
	
	//----------------------------------------------------------------------
	
	double GreyScaleHistogram::MaxValue()
	{
		double fMaxVal = -1e30f;
	
		for(int i=0; i<m_nBinsA; i++)
		{
			fMaxVal = max(fMaxVal,m_afGreyScaleHistogram[i]);
		}
	
		return fMaxVal;
	}
	
	
	//----------------------------------------------------------------------
	
	double GreyScaleHistogram::MinValue()
	{
		double fMinVal = 1e30f;
	
		for(int i=0; i<m_nBinsA; i++)
		{
			if(m_afGreyScaleHistogram[i]!=0.0f)
				fMinVal = min(fMinVal,m_afGreyScaleHistogram[i]);
		}
	
		return fMinVal;
	}
	
	//----------------------------------------------------------------------
	
	void GreyScaleHistogram::ScaleBy(double fScale)
	{
		for(int i=0; i<m_nBinsA; i++)
			m_afGreyScaleHistogram[i] *= fScale;
	}
	
	//----------------------------------------------------------------------
	
	void GreyScaleHistogram::SetUniform()
	{
		for(int i=0; i<m_nBinsA; i++)
			m_afGreyScaleHistogram[i] = 1.0;
		
		m_fHistNorm = double(m_nBinsA);
	}
	
	//----------------------------------------------------------------------
	
	double GreyScaleHistogram::GetBinVal(int nA)
	{
		return m_afGreyScaleHistogram[nA>>m_nShiftA]/*/m_fHistNorm*/;
	}
	
	//----------------------------------------------------------------------
	
	void GreyScaleHistogram::Clear()
	{
		m_fHistNorm=0.0f;
	
		for(int i=0; i<m_nBinsA; i++)
			m_afGreyScaleHistogram[i] = 0.0;
	}
	
	//----------------------------------------------------------------------
	
	double GreyScaleHistogram::ComputeBhattacharyyaCoeff(GreyScaleHistogram& cP)
	{
		double fBhatt = 0.0f;
		for(int i=0; i<m_nBinsA; i++)
			fBhatt += sqrt(m_afGreyScaleHistogram[i]*cP.m_afGreyScaleHistogram[i]);
		return fBhatt;
	}
	
	//----------------------------------------------------------------------
	
	double GreyScaleHistogram::ComputeKLDiv(GreyScaleHistogram& cP)
	{
		double fKLDiv = 0.0f;
		for(int i=0; i<m_nBinsA; i++)
			fKLDiv += m_afGreyScaleHistogram[i]*log((m_afGreyScaleHistogram[i]+1e-200)/(cP.m_afGreyScaleHistogram[i]+1e-200));
		return fKLDiv;
	}
	
	//----------------------------------------------------------------------
	
	void GreyScaleHistogram::BeginAddingNewSamples()
	{
		m_fNewHistNorm=0.0;
		m_fNewHistProbWeightNorm=0.0;
		
		for(int i=0; i<m_nBinsA; i++)
			m_afNewGreyScaleHistogram[i] = 0.0;
	}
	
	//----------------------------------------------------------------------
	
	void GreyScaleHistogram::AddNewSample(int nA , double fWeight)
	{
		nA = nA>>m_nShiftA;
	
		double fW = fWeight;
		m_afNewGreyScaleHistogram[nA] += fW;
		m_fNewHistProbWeightNorm += fW;
		m_fNewHistNorm += 1.0;
	}
	
	//----------------------------------------------------------------------
	
	void GreyScaleHistogram::CommitNewSamples(double fPrior)
	{
		//fPrior *= m_fNewHistProbWeightNorm/m_fNewHistNorm;
	
		// Normalise
		if(m_fNewHistProbWeightNorm > 0.0f)
		{
			for(int i=0; i<m_nBinsA; i++)
				if(m_afNewGreyScaleHistogram[i])
					m_afNewGreyScaleHistogram[i] /= m_fNewHistProbWeightNorm;
	
			m_fNewHistProbWeightNorm = 1.0;
		}
	
		// Marginalise over two distributions
		for(int i=0; i<m_nBinsA; i++)
			m_afGreyScaleHistogram[i] = m_afNewGreyScaleHistogram[i]*fPrior+m_afGreyScaleHistogram[i]*(1.0-fPrior);
	}
	
	//----------------------------------------------------------------------
	
	int GreyScaleHistogram::NonZeroBins()
	{
		int nRetVal = 0;
		for(int i=0; i<m_nBinsA; i++)
			if(m_afNewGreyScaleHistogram[i])
				nRetVal++;
	}
}
