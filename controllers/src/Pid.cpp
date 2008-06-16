#include <controllers/Pid.h>
#include<iostream>  

using namespace std;

double smooth(double v0, double v, double dt)
{
  double w1 = 10.0;
  double w2 =  1.0;
  return (  w1 * v0 + w2 * dt * v ) / ( w1 + w2 * dt );
}

Pid::Pid(double P,double I, double D, double I1, double I2, double cmdMax, double cmdMin ):
  pGain(P),iGain(I),dGain(D),iMax(I1),iMin(I2),cmdMax(cmdMax),cmdMin(cmdMin)
{
  pErrorLast  = 0.0;
  dError      = 0.0;
  iError      = 0.0;
  currentCmd  = 0.0;
  lastTimeInitialized = false;
  lastTime    = 0.0;
}


Pid::~Pid( )
{
}


void Pid::InitPid( double P,double I, double D, double I1, double I2, double cmdMax, double cmdMin )
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
  cmdMax      =  1e16;
  cmdMin      = -1e16;
  lastTimeInitialized = false;
  lastTime    = 0.0;
}


double Pid::UpdatePid( double pError, double currentTime )
{
  double pTerm, dTerm, iTerm;
  double dt;

  // initialize lastTime if not set already
  if (!lastTimeInitialized)
  {
    lastTimeInitialized = true;
    lastTime = currentTime;
  }

  dt = currentTime - lastTime;
  
  if (dt == 0)
  {
    // no update except for lastTime
    //throw "dividebyzero";
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
        dError     = smooth(dError,(pError - pErrorLast) / dt, dt);
        // save pError for next time
        pErrorLast = pError;
    }
    // calculate derivative contribution to command
    dTerm = dGain * dError;

    // create current command value
    currentCmd = pTerm + iTerm + dTerm;

    // command limits
    currentCmd = (currentCmd > cmdMax) ? cmdMax :
                ((currentCmd < cmdMin) ? cmdMin : currentCmd);
  }

  // save timestamp
  lastTime = currentTime;

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


