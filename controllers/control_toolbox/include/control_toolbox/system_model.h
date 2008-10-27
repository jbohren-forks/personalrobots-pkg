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
#ifndef SYSTEM_MODEL_H
#define SYSTEM_MODEL_H

#include <Eigen/Core>

/** @class DynamicsModel
  * @author Timothy Hunter <tjhunter@willowgarage.com>
  * @brief Provides a framework to provide generic fully observable models to controllers or other systems
  * This class provides af framework for expressing model of the form:
  *   x_dot = f(x) + g(x,u)
  * With x fully observable of size (StateSize by 1) and u of size (InputSize by 1)
  * It is expected to be able to provide a linearization around a point x0, i.e. 
  * if x = x0 + e (with e small) then the dynamics of the system can be written in first approximation:
  * e_dot = A e + B (u - u0)
  * with u0 solution of the equilibrium equation f(x0) + g(x0,u0) = 0
  *      A = grad(f)(x0)+grad_x(g)(x0,u0)
  *      B = grad_u(g)(x0,u0)
  * Hence the linearization is expected to return the matrices A,B and u0
  */
template<typename Scalar, int StateSize, int InputSize>
class DynamicsModel
{
public:

  typedef Eigen::Matrix<Scalar, StateSize, 1> StateVector;
  typedef Eigen::Matrix<Scalar, InputSize, 1> InputVector;
  typedef Eigen::Matrix<Scalar, StateSize, StateSize> StateMatrix;
  typedef Eigen::Matrix<Scalar, StateSize, InputSize> InputMatrix;
  
  /** @brief returns the size of the state vector
    * @return the size of the state vector, or -1 if it is not available
    */
  virtual int states() = 0;

  /** @brief returns the size of the commands vector
    * @return the size of the input vector, or -1 if it is not available
    */
  virtual int inputs() = 0;

  /** @brief returns a linearization y_dot = A y + B (u - u0) around the state x
    * @returns true if the linearization could be computed
    */
  virtual bool getLinearization(const StateVector & x, StateMatrix & A, InputMatrix & B, InputVector & u0) = 0;
  
  /** @brief Returns an estimation of forward dynamics of the system
    */
  virtual bool forward(const StateVector & x, const InputVector & u, const Scalar dt, StateVector  & next) = 0;
  
  /** @brief convenience function if the model can be updated by observations from the actual dynamics
    */
  // FIXME: this function is currently called in realtime loop => BAD
  virtual void observation(const StateVector & x_next, const StateVector & x, const InputVector & u, Scalar dt){}
};

#endif