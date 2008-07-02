#include <genericControllers/Pid.h>
#include<iostream>  

using namespace std;

Pid::Pid(double P,double I, double D, double I1, double I2 ) :
  pGain(P),iGain(I),dGain(D),iMax(I1),iMin(I2)
{
  pErrorLast  = 0.0;
  dError      = 0.0;
  iError      = 0.0;
  currentCmd  = 0.0;
}


Pid::~Pid( )
{
}


void Pid::InitPid( double P,double I, double D, double I1, double I2 )
{
  //cout << "Retuned=" << P<<" "<<I<<" "<<D << endl;
  pGain       = P;
  iGain       = I;
  dGain       = D;
  iMax        = I1;
  iMin        = I2;
  pErrorLast  = 0.0;
  dError      = 0.0;
  iError      = 0.0;
  currentCmd  = 0.0;
}


double Pid::UpdatePid( double pError, double dt )
{
  double pTerm, dTerm, iTerm;

  if (dt == 0)
  {
    throw "dividebyzero";
  }
  else
  {
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
        // dError should be d (pError) / dt
        dError     = (pError - pErrorLast) / dt;
        // save pError for next time
        pErrorLast = pError;
    }
    // calculate derivative contribution to command
    dTerm = dGain * dError;

    // create current command value
    currentCmd = pTerm + iTerm - dTerm;

  }

  return currentCmd;
}

void Pid::SetCurrentCmd(double cmd)
{
  currentCmd = cmd;
}

double Pid::GetCurrentCmd()
{
  return currentCmd;
}


