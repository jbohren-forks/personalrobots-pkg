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
#pragma once

/***************************************************/
/*! \class CONTROLLER::Pid
 \brief A basic pid class.

 This class implements a generic structure that
 can be used to create a wide range of pid
 controllers. It can function independently or
 be subclassed to provide more specific controls
 based on a particular control loop.

 In particular, this class implements the standard
 pid equation:

 command  = Pterm + Iterm + Dterm

 where: <br>
 <UL TYPE="none">
 <LI>  Pterm  = pGain * pError
 <LI>  Iterm  = iGain * iError
 <LI>  Dterm  = dGain * dError
 <LI>  iError = iError + pError * dT
 <LI>  dError = dError = dError + (pError - lastpError) / dT
 </UL>

 given:<br>
 <UL TYPE="none">
 <LI>  pError = pTarget - pState.
 </UL>

 If the fixedTime input of UpdatePid is set to alpha,
 dT = alpha. Otherwise the time step is computed when
 the function is called.

 */
/***************************************************/
namespace controller
{

class Pid
{
public:

  /*!
   * \brief Constructor, zeros out Pid values when created and
   * initialize Pid-gains and integral term limits:[iMax:iMin]-[I1:I2].
   *
   * \param P  The proportional gain.
   * \param I  The integral gain.
   * \param D  The derivative gain.
   * \param I1 The integral upper limit.
   * \param I2 The integral lower limit.
   */
  Pid(double P = 0.8, double I = 0.5, double D = 0.0, double I1 = 1.0, double I2 = -1.0);

  /*!
   * \brief Destructor of Pid class.
   */
  ~Pid();

  /*!
   * \brief Update the Pid loop with nonuniform time step size.
   *
   * \param pState  This is the current measured state or position of the object
   * being controlled.
   * \param pTarget This is the set point the controller is trying to reach.
   * \param fixedTime Set to a value for fixed time step of that value
   */
  double updatePid(double p_error, double dt);

  /*!
   * \brief Initialize PID-gains and integral term limits:[iMax:iMin]-[I1:I2]
   *
   * \param P  The proportional gain.
   * \param I  The integral gain.
   * \param D  The derivative gain.
   * \param I1 The integral upper limit.
   * \param I2 The integral lower limit.
   */
  void initPid(double P, double I, double D, double I1, double I2);

  /*!
   * \brief Set current command for this PID controller
   */
  void setCurrentCmd(double cmd);

  /*!
   * \brief Return current command for this PID controller
   */
  double getCurrentCmd();

  /*!
   * \brief Return PID error terms for the controller.
   */
  void getCurrentPIDErrors(double *pe, double *ie, double *de);

private:
  double p_error_last_; /**< _Save position state for derivative state calculation. */
  double p_error_; /**< position state. */
  double d_error_; /**< _Derivative state. */
  double i_error_; /**< _Integrator state. */
  double p_gain_; /**< _Proportional gain. */
  double i_gain_; /**< _Integral gain. */
  double d_gain_; /**< _Derivative gain. */
  double i_max_; /**< _Maximum allowable integrator state. */
  double i_min_; /**< _Minimum allowable integrator state. */
  double motor_cmd_; /**< _Command to send to motor. */
};

}
