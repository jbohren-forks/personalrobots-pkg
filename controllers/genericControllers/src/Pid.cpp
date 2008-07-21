/////////////////////////////////////////////////////////////////////////////////////
//Software License Agreement (BSD License)
//
//Copyright (c) 2008, Melonee Wise
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of the Willow Garage nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.
/////////////////////////////////////////////////////////////////////////////////////

#include <genericControllers/Pid.h>
#include<iostream>  

using namespace std;
using namespace CONTROLLER;

Pid::Pid(double P,double I, double D, double I1, double I2 ) :
  pGain(P),iGain(I),dGain(D),iMax(I1),iMin(I2)
{
  pErrorLast  = 0.0;
  pError      = 0.0;
  dError      = 0.0;
  iError      = 0.0;
  motorCmd    = 0.0;
}


Pid::~Pid( )
{
}


void Pid::InitPid( double P,double I, double D, double I1, double I2 )
{
  pGain       = P;
  iGain       = I;
  dGain       = D;
  iMax        = I1;
  iMin        = I2;
  pErrorLast  = 0.0;
  pError      = 0.0;
  dError      = 0.0;
  iError      = 0.0;
  motorCmd    = 0.0;
}


double Pid::UpdatePid( double error, double dt )
{
  double pTerm, dTerm, iTerm;
  pError = error;

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

    //    printf("Pid.cpp:: %f, %f, %f, %f, %f\n",iTerm,iGain,iError,iMax,iMin);

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

    // create current command value TODO: rename as motor command?
    motorCmd = -pTerm -iTerm -dTerm;
  }
  return motorCmd;
}

void Pid::SetCurrentCmd(double cmd)
{
  motorCmd = cmd;
}

double Pid::GetCurrentCmd()
{
  return motorCmd;
}

void Pid::GetCurrentPIDErrors(double *pe, double *ie, double *de)
{
  *pe = this->pError;
  *ie = this->iError;
  *de = this->dError;
}

