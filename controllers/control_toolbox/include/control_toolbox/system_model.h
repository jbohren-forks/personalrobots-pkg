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

// Forward declarations
// Do not forget to include it afterwards if you want to use the functions using these parameters
// If you do not want to implement any interface with ROS and PR2, you do not need
// any further inclusions.
namespace mechanism
{
  class RobotState;
}

namespace robot_msgs
{
  class JointCmd;
};

class TiXmlElement;

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
  * @note The state of any system expressed in this interface can consist of a
  * combination of positions, velocities, accelerations and torques of joint
  * angles when it makes sense. (This is due to the fact that the model must
  * provide an mapping from the robot statespace to the model statespace and
  * from the joint command statespace to the model statespace).
  */
template<typename Scalar, int StateSize, int InputSize>
class DynamicsModel
{
public:

  typedef Eigen::Matrix<Scalar, StateSize, 1> StateVector;
  typedef Eigen::Matrix<Scalar, InputSize, 1> InputVector;
  typedef Eigen::Matrix<Scalar, StateSize, StateSize> StateMatrix;
  typedef Eigen::Matrix<Scalar, StateSize, InputSize> InputMatrix;
  
  // Virtual destructor
  virtual ~DynamicsModel(){}
  
  /** @brief returns the size of the state vector
    * The user should assume the number of states is initialized after initXml()
    * has been called.
    * @return the size of the state vector, or -1 if it is not available
    */
  virtual int states() const = 0;

  /** @brief returns the size of the commands vector
    * The user should assume the number of inputs is initialized after initXml()
    * has been called.
    * @return the size of the input vector, or -1 if it is not available
    */
  virtual int inputs() const = 0;

  /** @brief returns a linearization y_dot = A y + B (u - u0) around the state x
    * @returns true if the linearization could be computed
    */
  virtual bool getLinearization(const StateVector & x, StateMatrix & A, InputMatrix & B, InputVector & u0) = 0;
  
  /** @brief Returns an estimation of forward dynamics of the system
    * Computes the state x(dt) = integral_{0}{dt}{f(x)+g(x,u)dt}
    * @param x the current state. Must be such that x.rows()==states(),
    *     x.cols()==1
    * @param u the current input (command). Must be such that
    *     u.rows()==inputs(), u.cols() == 1
    * @param dt the forward time, reasonably small. ('Reasonably' depending on
    *     the model)
    * @param next the resulting new state
    * @returns true if the forward state could be computed
    */
  virtual bool forward(const StateVector & x, const InputVector & u, const Scalar dt, StateVector  & next) = 0;

  // All the functions above are interface functions with controllers and the
  // mechanism model used in the robot.

  /** @brief Initialization function compatible with the API used by the mechanism model.
    * After calling this function, all the other methods are expected to return
    *     a consistent result.
    * @return false if the model could not be initialized, true otherwise.
    */
  virtual bool initXml(mechanism::RobotState *robot, TiXmlElement *config){return false;}

  
  /** @brief Given a the state of the robot, fills a state vector
    * This function is responsible for interfacing the model with some controller. 
    * It converts a RobotState to a vector state compatible with the
    * representation of the system provided by the model.
    * @note state must be such that state.rows() == states(). The model is not
    * responsible for resizing the state vector and is expected to return false
    * if the provided state if of the wrong size.
    * @param rstate the current state of the robot
    * @param state the corresponding state for the model
    * @returns true if the conversion could be done, false otherwise
    */
  virtual bool toState(const mechanism::RobotState * rstate, StateVector &
state) const {return false;}
  
  /** @brief converts a command message to a target state
    * This methods allow the programmer of a controller to convert a ROS
    * message to a state representation.
    * @returns true if it could be converted, false otherwise
    * @param cmd the target
    * @param state the corresponding state
    * @note state must be such that state.rows() == states(). The model is not
    * responsible for resizing the state vector and is expected to return false
    * if the provided state if of the wrong size.
    */
  virtual bool toState(const robot_msgs::JointCmd * cmd, StateVector & state)
const {return false;}

  /** @brief sets the commanded effort of the robot from an input vector
    * @returns true if the commanded effort could be set for all the mapped joints, false otherwise
    * @param robot_state the robot to set efforts on
    * @param effort the commanded effort vector
    */
  virtual bool setEffort(mechanism::RobotState *robot_state, const InputVector &effort)const{return false;}
};

#endif