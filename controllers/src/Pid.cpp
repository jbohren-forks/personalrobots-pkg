#include "Pid.h"
#include<iostream>  

using namespace std;


Pid::Pid(double P,double I, double D, double I1, double I2 ):
  pGain(P),iGain(I),dGain(D),iMax(I1),iMin(I2)
{
  pError          = 0.0;
  dError          = 0.0;
  iError          = 0.0;
  currentCommand  = 0.0;
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
}


double Pid::UpdatePid( double pError, double dt )
{
  double pTerm, dTerm, iTerm;

  if (dt == 0)
  {
    throw "dividebyzero";
  }

  // calculate proportional contribution to command
  pTerm = pGain * pError;

  // CALCULATE THE INTEGRAL ERROR ACCUMULATION WITH APPROPRIATE LIMITING
  iError = iError + dt * pError;
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
  if (dt != 0)
  {
      dError   = pError / dt;
  }
  // calculate derivative contribution to command
  dTerm = dGain * dError;

  // create current command value
  currentCommand = pTerm + iTerm + dTerm;

  return currentCommand;
}
