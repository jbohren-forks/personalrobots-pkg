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

/*
 * The robot model tracks the state of the robot.
 *
 * State path:
 *               +---------------+
 * Actuators --> | Transmissions | --> Joints 
 *               +---------------+
 *
 * Author: Stuart Glaser
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <map>
#include <string>
#include <kdl/tree.hpp>
#include "pr2_mechanism_model/joint.h"
#include "pr2_mechanism_model/transmission.h"
#include "pr2_hardware_interface/hardware_interface.h"

class TiXmlElement;

namespace pr2_mechanism
{

/** \brief This class provides the controllers with an interface to the robot model. 
 *  
 * Most controllers that need the robot model should use the 'robot_model_', which is
 * a kinematic/dynamic model of the robot, represented by a KDL Tree structure. 
 * 
 * Some specialized controllers (such as the calibration controllers) can get access
 * to actuators, transmissions and special joint parameters.
 */
class Robot
{
public:
  /// Constructor
  Robot(HardwareInterface *hw): hw_(hw) {}

  /// Destructor
  ~Robot()
  {
    deleteElements(&transmissions_);
    deleteElements(&joints_);
  }

  /// Initialize the robot model form xml
  bool initXml(TiXmlElement *root);

  /// The kinematic/dynamic model of the robot, represented by a KDL tree
  KDL::Tree robot_model_;

  /// The list of actuators
  std::vector<Actuator*> actuators_;
  /// The list of transmissions
  std::vector<Transmission*> transmissions_;
  /// The list of joints
  std::vector<Joint* > joints_;

  /// get the joint index based on the joint name. Returns -1 on failure
  int getJointIndex(const std::string &name) const;
  /// get the actuator index based on the actuator name. Returns -1 on failure
  int getActuatorIndex(const std::string &name) const;
  /// get the transmission index based on the transmission name. Returns -1 on failure
  int getTransmissionIndex(const std::string &name) const;

  /// get a joint pointer based on the joint name. Returns NULL on failure
  Joint* getJoint(const std::string &name) const;
  /// get an actuator pointer based on the actuator name. Returns NULL on failure
  Actuator* getActuator(const std::string &name) const;
  /// get a transmission pointer based on the transmission name. Returns NULL on failure
  Transmission* getTransmission(const std::string &name) const;

private:
  HardwareInterface *hw_;
};



/** \brief This class provides the controllers with an interface to the robot state
 *  
 * Most controllers that need the robot state should use the joint states, to get
 * access to the joint position/velocity/effort, and to command the effort a joint
 * should apply. Controllers can get access to the hard realtime clock through getTime()
 * 
 * Some specialized controllers (such as the calibration controllers) can get access
 * to actuator states, and transmission states.
 */
class RobotState
{
public:
  RobotState(Robot *model, HardwareInterface *hw);

  Robot *model_;

  std::vector<JointState> joint_states_;
  JointState *getJointState(const std::string &name);
  const JointState *getJointState(const std::string &name) const;

  ros::Time getTime() {return hw_->current_time_;};

 /**
  * Each transmission refers to the actuators and joints it connects by name.
  * Since name lookup is slow, for each transmission in the robot model we
  * cache pointers to the actuators and joints that it connects.
  **/
  std::vector<std::vector<Actuator*> > transmissions_in_;
  std::vector<std::vector<JointState*> > transmissions_out_;

  void propagateState();
  void propagateEffort();
  void enforceSafety();
  void zeroCommands();

  void propagateStateBackwards();
  void propagateEffortBackwards();

private:
  HardwareInterface *hw_; 
};

}

#endif
