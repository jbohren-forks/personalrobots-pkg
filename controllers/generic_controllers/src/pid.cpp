/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <generic_controllers/pid.h>

using namespace controller;

Pid::Pid(double P, double I, double D, double I1, double I2) :
  p_gain_(P), i_gain_(I), d_gain_(D), i_max_(I1), i_min_(I2)
{
  p_error_last_ = 0.0;
  p_error_ = 0.0;
  d_error_ = 0.0;
  i_error_ = 0.0;
  motor_cmd_ = 0.0;
}

Pid::~Pid()
{
}

void Pid::initPid(double P, double I, double D, double I1, double I2)
{
  p_gain_ = P;
  i_gain_ = I;
  d_gain_ = D;
  i_max_ = I1;
  i_min_ = I2;
  p_error_last_ = 0.0;
  p_error_ = 0.0;
  d_error_ = 0.0;
  i_error_ = 0.0;
  motor_cmd_ = 0.0;
}

double Pid::updatePid(double error, double dt)
{
  double p_term, d_term, i_term;
  p_error_ = error;

  if (dt == 0)
  {
    throw "dividebyzero";
  }
  else
  {
    // calculate proportional contribution to command
    p_term = p_gain_ * p_error_;

    // calculate the integral error accumulation with appropriate limiting
    i_error_ = i_error_ + dt * p_error_;
    // limit i_error_
    if (i_error_ > i_max_)
    {
      i_error_ = i_max_;
    }
    else if (i_error_ < i_min_)
    {
      i_error_ = i_min_;
    }
    // calculate integral contribution to command
    i_term = i_gain_ * i_error_;

    //    printf("Pid.cpp:: %f, %f, %f, %f, %f\n",i_term,i_gain_,i_error_,i_max_,i_min_);

    // CALCULATE DERIVATIVE ERROR
    if (dt != 0)
    {
      // d_error_ should be d (p_error_) / dt
      d_error_ = (p_error_ - p_error_last_) / dt;
      // save p_error_ for next time
      p_error_last_ = p_error_;
    }
    // calculate derivative contribution to command
    d_term = d_gain_ * d_error_;

    // create current command value TODO: rename as motor command?
    motor_cmd_ = -p_term - i_term - d_term;
  }
  return motor_cmd_;
}

void Pid::setCurrentCmd(double cmd)
{
  motor_cmd_ = cmd;
}

double Pid::getCurrentCmd()
{
  return motor_cmd_;
}

void Pid::getCurrentPIDErrors(double *pe, double *ie, double *de)
{
  *pe = p_error_;
  *ie = i_error_;
  *de = d_error_;
}

