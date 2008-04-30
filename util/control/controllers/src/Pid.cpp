#include "Pid.h"
#include<iostream>  
#include "timer.h"

using namespace std;


Pid::Pid(double P,double I, double D, double I1, double I2 ):
  pGain(P),iGain(I),dGain(D),iMax(I1),iMin(I2)
{
  pError          = 0.0;
  dError          = 0.0;
  iError          = 0.0;
  currentCommand  = 0.0;
  currentTime     = 0.0;
     lastTime     = 0.0;
  timeInitiated   = false;
}


Pid::~Pid( )
{
}


void Pid::InitPid( double P,double I, double D, double I1, double I2 )
{
  //cout << "Retuned=" << P<<" "<<I<<" "<<D << endl;
  pGain           = P;
  iGain           = I;
  dGain           = D;
  iMax            = I1;
  iMin            = I2;
  pError          = 0.0;
  dError          = 0.0;
  iError          = 0.0;
  currentCommand  = 0.0;
  currentTime     = 0.0;
     lastTime     = 0.0;
  timeInitiated   = false;
}


double Pid::UpdatePid( double pError, double dt )
{
  double pTerm, dTerm, iTerm, dT;
  // Get current time
  currentTime = Timer::GetTimeSeconds();
  // For the very first time step, initialize lastTime to currentTime. First dT will be 0.
  if (!timeInitiated)
  {
    timeInitiated = true;
    lastTime      = currentTime;
  }
  dT = currentTime - lastTime;
  
  if (dt !=0)
  {
    dT=dt;
  }

  // calculate proportional contribution to command
  pTerm = pGain * pError;

  // CALCULATE THE INTEGRAL ERROR ACCUMULATION WITH APPROPRIATE LIMITING
  iError = iError + dT * pError;
  // limit iError
  if (iError > iMax)
  {
    iError = iMax;
  }
  else if (iError < iMin)
  {
    iError = iMin;
  }
  // calculate integral contribution to command
  iTerm = iGain * iError;
  // CALCULATE DERIVATIVE ERROR
  if (dT != 0)
  {
      dError   = pError / dT;
  }
  // calculate derivative contribution to command
  dTerm = dGain * dError;

  // create current command value
  currentCommand = pTerm + iTerm + dTerm;
  // save time value
  lastTime = currentTime;
  return currentCommand;
}
