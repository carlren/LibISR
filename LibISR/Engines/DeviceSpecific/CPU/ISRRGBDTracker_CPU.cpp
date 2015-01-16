
#include "ISRRGBDTracker_CPU.h"
#include "../../DeviceAgnostic/ISRRGBDTracker_DA.h"

using namespace LibISR::Engine;
using namespace LibISR::Objects;

LibISR::Engine::ISRRGBDTracker_CPU::ISRRGBDTracker_CPU(int nObjs) :ISRRGBDTracker(nObjs,false){}
LibISR::Engine::ISRRGBDTracker_CPU::~ISRRGBDTracker_CPU(){}

void LibISR::Engine::ISRRGBDTracker_CPU::evaluateEnergy(float *energy, Objects::ISRTrackingState * trackerState)
{

}

void LibISR::Engine::ISRRGBDTracker_CPU::computeJacobianAndHessian(float *gradient, float *hessian, Objects::ISRTrackingState * trackerState)
{

}


