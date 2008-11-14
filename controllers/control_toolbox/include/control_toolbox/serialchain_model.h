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
#ifndef LINEARSYSTEM_MODEL_H
#define LINEARSYSTEM_MODEL_H

#include <Eigen/Core>
#include "system_model.h"
#include <map>

#include <robot_kinematics/robot_kinematics.h>

/** @class SerialChainModel
  * @brief Provides a model for a chain of rigid body
  * Given a chain of n links, this class provides a model of a system of 2n states (angles and angular velocities) and n inputs (torques at each node)
  * The state of the system is: angle_1 angle_2  ... vel_1 vel_2 ... 
  * Example of a XML configuration snippet:
  * <model type="SerialChainModel">
  *   <kinematics>kin_chain_name</kinematics>
  *   <robot_description>pr2</robot_description>
  *   <joints>
  *     <joint name="joint1"/>
  *     <joint name="joint2"/>
  *   </joints>
  * </model>
  */
class SerialChainModel : public DynamicsModel<double,Eigen::Dynamic,Eigen::Dynamic>
{
public:
  typedef DynamicsModel<double,Eigen::Dynamic,Eigen::Dynamic> Base;
  typedef Base::StateMatrix StateMatrix;
  typedef Base::InputMatrix InputMatrix;
  typedef Base::StateVector StateVector;
  typedef Base::InputVector InputVector;
  
  SerialChainModel() : kdl_torque_(NULL),states_(-1),inputs_(-1){}
  
  virtual ~SerialChainModel(){}
  
  int states() const { return states_; }
  
  int inputs() const { return inputs_; }
  
  /** @brief This function contains all the parameters required to initialize the model
    */
  bool init(const std::string & robot_description, const std::string & chain_name);
  
  bool init(const robot_kinematics::RobotKinematics & r_kin, robot_kinematics::SerialChain *chain);
  
  bool getLinearization(const StateVector & x, StateMatrix & A, InputMatrix & B, InputVector & u0);
  
  bool forward(const StateVector & x, const InputVector & u, double dt, StateVector & next);

  /** These functions depend on ROS-specific informations
    */
  virtual bool initXml(mechanism::RobotState *robot, TiXmlElement *config){return false;}
  
  virtual bool toState(const mechanism::RobotState * rstate, StateVector &state)const {return false;}
  
  virtual bool toState(const robot_msgs::JointCmd * cmd, StateVector & state) const {return false;}


private:
  robot_kinematics::RobotKinematics robot_kin_;

  robot_kinematics::SerialChain *chain_;
  
  // An array that stores temporary results from inverse dynamics
  KDL::Vector *kdl_torque_;
  
  int states_, inputs_;
  
};

#endif