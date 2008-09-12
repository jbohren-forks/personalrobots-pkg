/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef TEST_ACTUATORS_H
#define TEST_ACTUATORS_H

#include <vector>
#include <gazebo/Controller.hh>
#include <gazebo/Entity.hh>
#include <gazebo/Model.hh>

#include <generic_controllers/joint_position_controller.h>
#include <generic_controllers/joint_velocity_controller.h>

#include <pr2_controllers/arm_position_controller.h>
#include <pr2_controllers/base_controller.h>
#include <pr2_controllers/laser_scanner_controller.h>
#include "hardware_interface/hardware_interface.h"
#include <mechanism_control/mechanism_control.h>

#include <rosTF/rosTF.h>
#include <ros/node.h>

// roscpp - used for broadcasting time over ros
#include <rostools/Time.h>
// ros messages
#include <std_msgs/RobotBase2DOdom.h>
#include <std_msgs/BaseVel.h>
#include <std_msgs/PR2Arm.h>
#include <pr2_msgs/EndEffectorState.h>

// Ioan's parser
#include <urdf/URDF.h>

// Advait and Gil's cartesian services
//#include <gazebo_plugin/MoveCartesian.h>
//#include <gazebo_plugin/GripperCmd.h>

namespace gazebo
{
class HingeJoint;
class PositionIface;
class XMLConfigNode;

class TestActuators : public gazebo::Controller
{
public:
  TestActuators(Entity *parent);
  virtual ~TestActuators();

  //void LoadFrameTransformOffsets();
  //void PublishFrameTransforms();
  int AdvertiseSubscribeMessages();

protected:
  // Inherited from gazebo::Controller
  virtual void LoadChild(XMLConfigNode *node);
  virtual void InitChild();
  virtual void UpdateChild();
  virtual void FiniChild();

  // Callbacks for subscribed messages
  void CmdBaseVelReceived();
  void CmdLeftarmconfigReceived();
  void CmdRightarmconfigReceived();
  void CmdLeftarmcartesianReceived();
  void CmdRightarmcartesianReceived();
#if 0
  bool reset_IK_guess(gazebo_plugin::VoidVoid::request &req, gazebo_plugin::VoidVoid::response &res);
  bool SetRightArmCartesian(gazebo_plugin::MoveCartesian::request &req, gazebo_plugin::MoveCartesian::response &res);
  bool OperateRightGripper(gazebo_plugin::GripperCmd::request &req, gazebo_plugin::GripperCmd::response &res);
  // arm joint data
  std_msgs::PR2Arm leftarm;
  std_msgs::PR2Arm rightarm;

  // end effector cmds
  pr2_msgs::EndEffectorState cmd_leftarmcartesian;
  pr2_msgs::EndEffectorState cmd_rightarmcartesian;

  // need newRightArmPos, UpdateRightArm(), etc...

#endif

private:

  std_msgs::BaseVel velMsg;
   // arm joint data
  std_msgs::PR2Arm leftarmMsg;
  std_msgs::PR2Arm rightarmMsg;
  // end effector cmds
  pr2_msgs::EndEffectorState leftarmcartesianMsg;
  pr2_msgs::EndEffectorState rightarmcartesianMsg;

  rostools::Time timeMsg;

  std_msgs::Point3DFloat32 objectPosMsg;

 

  Model *parent_model_;

  std::vector<robot_desc::URDF::Link*> pr2Links;

  //---------------------------------------------------------------------
  //  for mechanism control
  //---------------------------------------------------------------------
  MechanismControl mc_;
  MechanismControlNode mcn_;

  mechanism::RobotState *fake_state_;


  void LoadMC(XMLConfigNode *node);
  void UpdateMC();
  void PublishROS();
  void UpdateMCJoints();
  void UpdateGazeboJoints();

  // for the mechanism control code loading xml
  //std::string interface; // xml filename for the hardware interface
  //std::string xml_file; // xml filename for the robot
  //TiXmlElement *root;

  HardwareInterface hw_;

  //---------------------------------------------------------------------
  //                                                                   --
  // BELOW IS JOHN'S VERSION OF MECHANISM CONTROL, WAITING ... FOR     --
  // THE ACTUAL ONE TO BE FUNCTIONAL.                                  --
  //                                                                   --
  //---------------------------------------------------------------------
  // Ioan's ultimate parser
  robot_desc::URDF pr2Description;

  // for storing pr2 xml
  //mechanism::Robot* mech_robot_;

  // for storing reverse transmission results
  //mechanism::Robot* reverse_mech_robot_;

  std_msgs::PR2Arm larm,rarm;

  // for storing controller xml
  struct Gazebo_joint_
  {
    std::string* name_;
    std::vector<gazebo::Joint*> gaz_joints_;
    mechanism::JointState* fake_joint_state_;
    double saturationTorque;
    double explicitDampingCoefficient;
  };
  std::vector<Gazebo_joint_*> gazebo_joints_;
  double currentTime;
  double lastTime;

  // topic name
  //std::string topicName;

  // A mutex to lock access to fields that are used in message callbacks
  private: ros::thread::mutex lock;

  // pointer to ros node
  ros::node *rosnode_;

};

}

#endif

