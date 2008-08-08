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

#include <gazebo_plugin/gazebo_actuators.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <unistd.h>
#include <stl_utils/stl_utils.h>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Model.hh>
#include <gazebo/HingeJoint.hh>
#include <gazebo/SliderJoint.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

namespace gazebo {

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_actuators", GazeboActuators);

GazeboActuators::GazeboActuators(Entity *parent)
  : Controller(parent), hw_(0), mc_(&hw_)
{
  this->parent_model_ = dynamic_cast<Model*>(this->parent);

  if (!this->parent_model_)
    gzthrow("GazeboActuators controller requires a Model as its parent");

  rosnode_ = ros::g_node;
  int argc = 0;
  char** argv = NULL;
  if (rosnode_ == NULL)
  {
    // start a ros node if none exist
    ros::init(argc,argv);
    rosnode_ = new ros::node("ros_gazebo",ros::node::DONT_HANDLE_SIGINT);
    printf("-------------------- starting node in GazeboActuators\n");
  }
}

GazeboActuators::~GazeboActuators()
{
  deleteElements(&joints_);
}

void GazeboActuators::LoadChild(XMLConfigNode *node)
{
  XMLConfigNode *xit;  // XML iterator

  // Reads the joints out of Gazebo's config.
  std::map<std::string,int> joint_lookup;
  for (xit = node->GetChild("joint"); xit; xit = xit->GetNext("joint"))
  {
    std::string joint_name = xit->GetString("name", "", 1);
    gazebo::Joint *joint = parent_model_->GetJoint(joint_name);
    assert(joint != NULL);
    joints_.push_back(joint);
    mech_joints_.push_back(new mechanism::Joint);

    joint_lookup.insert(std::pair<std::string,int>(joint_name, joints_.size()-1));
  }

  // Reads the transmission information from the config.
  for (xit = node->GetChild("transmission"); xit; xit = xit->GetNext("transmission"))
  {
    if (0 == strcmp("SimpleTransmission", xit->GetString("type", "", 0).c_str()))
    {
      // Looks up the joint by name
      XMLConfigNode *joint_node = xit->GetChild("joint");
      assert(joint_node != NULL);
      std::map<std::string,int>::iterator jit = joint_lookup.find(joint_node->GetString("name", "", 1));
      if (jit == joint_lookup.end())
      {
        // TODO: report: Could not find the joint named xxxx
        continue;
      }

      // Creates the actuator
      XMLConfigNode *actuator_node = xit->GetChild("actuator");
      assert(actuator_node != NULL);
      Actuator *actuator = new Actuator;
      hw_.actuators_.push_back(actuator);
      actuator_names_.push_back(actuator_node->GetString("name", "", 1));

      // Creates the transmission
      transmissions_.push_back(
        new mechanism::SimpleTransmission(mech_joints_[jit->second], actuator,
                                          xit->GetDouble("mechanicalReduction", 0.0, 1),
                                          xit->GetDouble("motorTorqueConstant", 0.0, 1),
                                          xit->GetDouble("pulsesPerRevolution", 0.0, 1)));

    }
    else
    {
      // TODO: report: Unknown transmission type
      continue;
    }
  }
}

void GazeboActuators::InitChild()
{
  // Registers the actuators with MechanismControl
  assert(actuator_names_.size() == hw_.actuators_.size());
  for (unsigned int i = 0; i < actuator_names_.size(); ++i)
    mc_.registerActuator(actuator_names_[i], i);


  // TODO: mc.init();

  hw_.current_time_ = Simulator::Instance()->GetSimTime();
}

void GazeboActuators::UpdateChild()
{
  assert(joints_.size() == mech_joints_.size());

  //--------------------------------------------------
  //  Pushes out simulation state
  //--------------------------------------------------

  // Copies the state from the gazebo joints into the mechanism joints.
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    assert(joints_[i]->GetType() == Joint::HINGE);
    HingeJoint *hj = (HingeJoint*)joints_[i];
    mech_joints_[i]->position_ = hj->GetAngle();
    mech_joints_[i]->velocity_ = hj->GetAngleRate();
    mech_joints_[i]->applied_effort_ = mech_joints_[i]->commanded_effort_;
  }

  // Reverses the transmissions to propagate the joint position into the actuators.
  for (unsigned int i = 0; i < transmissions_.size(); ++i)
    transmissions_[i]->propagatePositionBackwards();

  //--------------------------------------------------
  //  Runs Mechanism Control
  //--------------------------------------------------
  hw_.current_time_ = Simulator::Instance()->GetSimTime();
  mc_.update();

  //--------------------------------------------------
  //  Takes in actuation commands
  //--------------------------------------------------

  // Reverses the transmissions to propagate the actuator commands into the joints.
  for (unsigned int i = 0; i < transmissions_.size(); ++i)
    transmissions_[i]->propagateEffortBackwards();

  // Copies the commands from the mechanism joints into the gazebo joints.
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    assert(joints_[i]->GetType() == Joint::HINGE);
    HingeJoint *hj = (HingeJoint*)joints_[i];
    hj->SetTorque(mech_joints_[i]->commanded_effort_);
  }
}

void GazeboActuators::FiniChild()
{

}

} // namespace gazebo
