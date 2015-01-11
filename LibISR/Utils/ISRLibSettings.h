#pragma once


namespace CoreISR
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
			~ISRLibSettings(void) { }
		};
	}
}
