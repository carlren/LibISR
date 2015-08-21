// Copyright 2014-2015 Isis Innovation Limited and the authors of LibISR

#pragma once


namespace LibISR
{
	namespace Objects
	{
		class ISRLibSettings
		{
		public:
			bool useGPU;

			int noTrackingObj;

			int noHistogramDim;

			bool singleAappearanceModel;

			ISRLibSettings(void){}
			~ISRLibSettings(void){}
		};
	}
}
