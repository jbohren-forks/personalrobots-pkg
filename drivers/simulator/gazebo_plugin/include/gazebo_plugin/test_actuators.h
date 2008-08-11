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
#include "hardware_interface/hardware_interface.h"
#include <mechanism_control/mechanism_control.h>

#include <rosTF/rosTF.h>
#include <ros/node.h>

// Ioan's parser
#include <urdf/URDF.h>

namespace gazebo
{
class HingeJoint;
class PositionIface;
class XMLConfigNode;

class GazeboActuators : public gazebo::Controller
{
public:
  GazeboActuators(Entity *parent);
  virtual ~GazeboActuators();

protected:
  // Inherited from gazebo::Controller
  virtual void LoadChild(XMLConfigNode *node);
  virtual void InitChild();
  virtual void UpdateChild();
  virtual void FiniChild();

private:

  Model *parent_model_;

  //---------------------------------------------------------------------
  //  for mechanism control
  //---------------------------------------------------------------------
  //MechanismControl mc_;
  //MechanismControlNode mcn_; // multiple nodes per process

  // pointer to ros node
  ros::node *rosnode_;

  void LoadMC(XMLConfigNode *node);
  void UpdateMC();

  // for the mechanism control code loading xml
  //std::string interface; // xml filename for the hardware interface
  //std::string xml_file; // xml filename for the robot
  //TiXmlElement *root;

  // JMH: as far as I know, this parsing in mech-control is not functional yet,
  // so I am going to use either:
  //   1. Ioan's parser 
  //   2. My own parser to fill out mappings
  // 
  // steps are defined below
  //   1. fill in MechanismControl->Robot->Joint                                    (defines robot)
  //std::vector<mechanism::Joint*> mech_joints_;
  //std::vector<mechanism::Transmission*> transmissions_;
  //std::vector<std::string> actuator_names_;
  //std::vector<gazebo::Joint*> gazebo_joints_;
  HardwareInterface *hw_;

  //   2. fill in HardwareInterface
  //           actuators_ is a vector
  //           current_time_ is a double
  //   3. fill in MechanismControl->Robot->Transmission->Actuators->ActuatorState   (define transmissions)
  //      fill in MechanismControl->Robot->Transmission->Actuators->ActuatorCommand
  //   4. fill in MechanismControl->JointController                                 (define controllers)
  //      fill in MechanismControl->JointController->*Joints                        (define joints controlled)
  //
  //---------------------------------------------------------------------
  //  for gazebo hardware
  //---------------------------------------------------------------------
  // Each joint in joints_ corresponds to the joint with the same
  // index in mech_joints_.  The mech_joints_ vector exists so that
  // each transmission has a mechanism::Joint to write to, because it
  // would be best if the transmissions did not depend on Gazebo
  // objects.
  // define a map to keep track of joints mapping from mech to gaz
  // struct robot_to_gazebo_joints_
  // {
  //   mechanism::Joint* mecj_;
  //   gazebo::Joint* gazj_;
  // };
  // std::map<std::string,robot_to_gazebo_joints_*> joint_map_;

  //---------------------------------------------------------------------
  //                                                                   --
  // BELOW IS JOHN'S VERSION OF MECHANISM CONTROL, WAITING ... FOR     --
  // THE ACTUAL ONE TO BE FUNCTIONAL.                                  --
  //                                                                   --
  //---------------------------------------------------------------------
  // Ioan's ultimate parser
  robot_desc::URDF pr2Description;

  // for storing pr2 xml
  mechanism::Robot* mech_robot_;

  // for storing reverse transmission results
  mechanism::Robot* reverse_mech_robot_;

  // for storing controller xml
  struct Robot_controller_
  {
    std::string name;
    std::string type;
    std::string joint_name;
    std::string joint_type;
    mechanism::Joint* mech_joint_;
    mechanism::Joint* reverse_mech_joint_;

    std::string control_mode; // obsolete? use to pick controller for now
    double p_gain,i_gain,d_gain,windup, init_time;
    controller::JointPositionController pcontroller; // our fancy controller
    controller::JointVelocityController vcontroller; // our fancy controller

    double saturation_torque;
    double explicitDampingCoefficient;
    std::string gazebo_joint_type;
    std::vector<gazebo::Joint*> gazebo_joints_;

  };
  std::vector<Robot_controller_> robot_controllers_;

  // for storing transmission xml
  struct Robot_transmission_
  {
      std::string name;
      std::string joint_name;
      std::string actuator_name;
      mechanism::SimpleTransmission simple_transmission;
      gazebo::Joint* gazebo_joints_;
  };
  std::vector<Robot_transmission_> robot_transmissions_;
  std::vector<Robot_transmission_> reverse_robot_transmissions_;

  //std::vector<mechanism::SimpleTransmission> forward_simple_transmission;
  //std::vector<mechanism::SimpleTransmission> reverse_simple_transmission;

  // for storing actuator xml
  struct Robot_actuator_
  {
      std::string name;
      std::string motorboardID;
      double maxCurrent;
      std::string motor;
      std::string ip;
      double port;
      double reduction;
      Vector3 polymap;

      // use our fancy Actuator class
      Actuator actuator;

      // link to joint?
      //gazebo::Joint* gazebo_joints_;
  };
  std::map<std::string,Robot_actuator_> robot_actuators_;





  // topic name
  std::string topicName;

  // frame transform name, should match link name
  // FIXME: extract link name directly?
  std::string frameName;

  // A mutex to lock access to fields that are used in message callbacks
  private: ros::thread::mutex lock;

  // transform server
  private: rosTFClient *tfc;


};

}

#endif

