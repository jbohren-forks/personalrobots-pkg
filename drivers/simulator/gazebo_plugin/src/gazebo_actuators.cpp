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
#include <set>
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
#include <urdf/parser.h>

namespace gazebo {

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_actuators", GazeboActuators);

GazeboActuators::GazeboActuators(Entity *parent)
  : Controller(parent), hw_(0), mc_(&hw_), mcn_(&mc_), fake_model_("fake")
{
  fake_model_.hw_ = &hw_;
  this->parent_model_ = dynamic_cast<Model*>(this->parent);

  if (!this->parent_model_)
    gzthrow("GazeboActuators controller requires a Model as its parent");
}

GazeboActuators::~GazeboActuators()
{
  deleteElements(&joints_);
}

void GazeboActuators::LoadChild(XMLConfigNode *node)
{
  XMLConfigNode *robot = node->GetChild("robot");
  if (!robot)
  {
    fprintf(stderr, "Error loading gazebo_actuators config: no robot element\n");
    return;
  }

  // TODO: should look for the file relative to the path of the world file.
  std::string filename = robot->GetString("filename", "", 1);
  printf("Loading %s\n", filename.c_str());

  TiXmlDocument doc(filename);
  doc.LoadFile();
  assert(doc.RootElement());
  urdf::normalizeXml(doc.RootElement());

  // Pulls out the list of actuators used in the robot configuration.
  struct GetActuators : public TiXmlVisitor
  {
    std::set<std::string> actuators;
    virtual bool VisitEnter(const TiXmlElement &elt, const TiXmlAttribute *)
    {
      if (elt.ValueStr() == std::string("actuator") && elt.Attribute("name"))
        actuators.insert(elt.Attribute("name"));
      return true;
    }
  } get_actuators;
  doc.RootElement()->Accept(&get_actuators);

  // Places the found actuators into the hardware interface.
  std::set<std::string>::iterator it;
  for (it = get_actuators.actuators.begin(); it != get_actuators.actuators.end(); ++it)
  {
    hw_.actuators_.push_back(new Actuator(*it));
  }

  // Initializes the fake model (for running the transmissions backwards).
  fake_model_.initXml(doc.RootElement());

  // The gazebo joints and mechanism joints should match up.
  for (unsigned int i = 0; i < fake_model_.joints_.size(); ++i)
  {
    std::string joint_name = fake_model_.joints_[i]->name_;
    gazebo::Joint *joint = parent_model_->GetJoint(joint_name);
    if (joint)
      joints_.push_back(joint);
    else
    {
      fprintf(stderr, "Gazebo does not know about a joint named \"%s\"\n", joint_name.c_str());
      joints_.push_back(NULL);
    }
  }

  hw_.current_time_ = Simulator::Instance()->GetSimTime();
  mcn_.initXml(doc.RootElement());
}

void GazeboActuators::InitChild()
{
  hw_.current_time_ = Simulator::Instance()->GetSimTime();
}

void GazeboActuators::UpdateChild()
{
  assert(joints_.size() == fake_model_.joints_.size());

  //--------------------------------------------------
  //  Pushes out simulation state
  //--------------------------------------------------

  // Copies the state from the gazebo joints into the mechanism joints.
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    if (!joints_[i])
      continue;

    fake_model_.joints_[i]->applied_effort_ = fake_model_.joints_[i]->commanded_effort_;

    switch(joints_[i]->GetType())
    {
    case Joint::HINGE: {
      HingeJoint *hj = (HingeJoint*)joints_[i];
      fake_model_.joints_[i]->position_ = hj->GetAngle();
      fake_model_.joints_[i]->velocity_ = hj->GetAngleRate();
      break;
    }
    case Joint::SLIDER: {
      SliderJoint *sj = (SliderJoint*)joints_[i];
      fake_model_.joints_[i]->position_ = sj->GetPosition();
      fake_model_.joints_[i]->velocity_ = sj->GetPositionRate();
      break;
    }
    default:
      abort();
    }
  }

  // Reverses the transmissions to propagate the joint position into the actuators.
  for (unsigned int i = 0; i < fake_model_.transmissions_.size(); ++i)
    fake_model_.transmissions_[i]->propagatePositionBackwards();

  //--------------------------------------------------
  //  Runs Mechanism Control
  //--------------------------------------------------
  hw_.current_time_ = Simulator::Instance()->GetSimTime();
  mcn_.update();

  //--------------------------------------------------
  //  Takes in actuation commands
  //--------------------------------------------------

  // Reverses the transmissions to propagate the actuator commands into the joints.
  for (unsigned int i = 0; i < fake_model_.transmissions_.size(); ++i)
    fake_model_.transmissions_[i]->propagateEffortBackwards();

  // Copies the commands from the mechanism joints into the gazebo joints.
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    if (!joints_[i])
      continue;

    double effort = fake_model_.joints_[i]->commanded_effort_;
    switch (joints_[i]->GetType())
    {
    case Joint::HINGE:
      ((HingeJoint*)joints_[i])->SetTorque(effort);
      break;
    case Joint::SLIDER:
      ((SliderJoint*)joints_[i])->SetSliderForce(effort);
      break;
    default:
      abort();
    }
  }
}

void GazeboActuators::FiniChild()
{

}

} // namespace gazebo
