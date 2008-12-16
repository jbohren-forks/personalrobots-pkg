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

#include <gazebo_plugin/gazebo_mechanism_control.h>
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
#include <map>

namespace gazebo {

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_mechanism_control", GazeboMechanismControl);

GazeboMechanismControl::GazeboMechanismControl(Entity *parent)
  : Controller(parent), hw_(0), mc_(&hw_), mcn_(&mc_), fake_state_(NULL)
{
  this->parent_model_ = dynamic_cast<Model*>(this->parent);

  if (!this->parent_model_)
    gzthrow("GazeboMechanismControl controller requires a Model as its parent");

    rosnode_ = ros::g_node; // comes from where?
    int argc = 0;
    char** argv = NULL;
    if (rosnode_ == NULL)
    {
      // this only works for a single camera.
      ros::init(argc,argv);
      rosnode_ = new ros::node("ros_gazebo",ros::node::DONT_HANDLE_SIGINT);
      printf("-------------------- starting node in Gazebo Mechanism Control \n");
    }

  if (getenv("CHECK_SPEEDUP"))
  {
    wall_start = Simulator::Instance()->GetWallTime();
    sim_start  = Simulator::Instance()->GetSimTime();
  }

}

GazeboMechanismControl::~GazeboMechanismControl()
{
}

void GazeboMechanismControl::LoadChild(XMLConfigNode *node)
{
  // read pr2.xml (pr2_gazebo_mechanism_control.xml)
  // setup actuators, then setup mechanism control node
  ReadPr2Xml(node);

  // Initializes the fake state (for running the transmissions backwards).
  fake_state_ = new mechanism::RobotState(&mc_.model_, &hw_);

  // The gazebo joints and mechanism joints should match up.
  for (unsigned int i = 0; i < mc_.model_.joints_.size(); ++i)
  {
    std::string joint_name = mc_.model_.joints_[i]->name_;

    // fill in gazebo joints pointer
    gazebo::Joint *joint = parent_model_->GetJoint(joint_name);
    if (joint)
    {
      joints_.push_back(joint);
    }
    else
    {
      fprintf(stderr, "WARNING (gazebo_mechanism_control): A joint named \"%s\" is not part of Mechanism Controlled joints.\n", joint_name.c_str());
      joints_.push_back(NULL);
    }

  }

  hw_.current_time_ = Simulator::Instance()->GetSimTime();
}

void GazeboMechanismControl::InitChild()
{
  hw_.current_time_ = Simulator::Instance()->GetSimTime();
}

void GazeboMechanismControl::UpdateChild()
{

  if (getenv("CHECK_SPEEDUP"))
  {
    double wall_elapsed = Simulator::Instance()->GetWallTime() - wall_start;
    double sim_elapsed  = Simulator::Instance()->GetSimTime()  - sim_start;
    std::cout << " real time: " <<  wall_elapsed
              << "  sim time: " <<  sim_elapsed
              << "  speed up: " <<  sim_elapsed / wall_elapsed
              << std::endl;
  }
  assert(joints_.size() == fake_state_->joint_states_.size());

  //--------------------------------------------------
  //  Pushes out simulation state
  //--------------------------------------------------

  // Copies the state from the gazebo joints into the mechanism joints.
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    if (!joints_[i])
      continue;

    fake_state_->joint_states_[i].applied_effort_ = fake_state_->joint_states_[i].commanded_effort_;

    switch(joints_[i]->GetType())
    {
    case Joint::HINGE: {
      HingeJoint *hj = (HingeJoint*)joints_[i];
      fake_state_->joint_states_[i].position_ = hj->GetAngle();
      fake_state_->joint_states_[i].velocity_ = hj->GetAngleRate();
      break;
    }
    case Joint::SLIDER: {
      SliderJoint *sj = (SliderJoint*)joints_[i];
      fake_state_->joint_states_[i].position_ = sj->GetPosition();
      fake_state_->joint_states_[i].velocity_ = sj->GetPositionRate();
      break;
    }
    default:
      abort();
    }
  }

  // Reverses the transmissions to propagate the joint position into the actuators.
  fake_state_->propagateStateBackwards();

  //--------------------------------------------------
  //  Runs Mechanism Control
  //--------------------------------------------------
  hw_.current_time_ = Simulator::Instance()->GetSimTime();
  try
  {
    mcn_.update();
  }
  catch (const char* c)
  {
    if (strcmp(c,"dividebyzero")==0)
      std::cout << "WARNING:pid controller reports divide by zero error" << std::endl;
    else
      std::cout << "unknown const char* exception: " << c << std::endl;
  }

  //--------------------------------------------------
  //  Takes in actuation commands
  //--------------------------------------------------

  // Reverses the transmissions to propagate the actuator commands into the joints.
  fake_state_->propagateEffortBackwards();

  // Copies the commands from the mechanism joints into the gazebo joints.
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    if (!joints_[i])
      continue;

    double damping_force;
    double effort = fake_state_->joint_states_[i].commanded_effort_;
    switch (joints_[i]->GetType())
    {
    case Joint::HINGE:
      //std::cout << " hinge joint name " << mc_.model_.joints_[i]->name_ << std::endl;
      //std::cout << " check damping coef " << mc_.model_.joints_[i]->joint_damping_coefficient_ << std::endl;
      damping_force = mc_.model_.joints_[i]->joint_damping_coefficient_ * ((HingeJoint*)joints_[i])->GetAngleRate();
      ((HingeJoint*)joints_[i])->SetTorque(effort - damping_force);
      break;
    case Joint::SLIDER:
      //std::cout << " slider joint name " << mc_.model_.joints_[i]->name_ << std::endl;
      //std::cout << " check damping coef " << mc_.model_.joints_[i]->joint_damping_coefficient_ << std::endl;
      damping_force = mc_.model_.joints_[i]->joint_damping_coefficient_ * ((SliderJoint*)joints_[i])->GetPositionRate();
      ((SliderJoint*)joints_[i])->SetSliderForce(effort - damping_force);
      break;
    default:
      abort();
    }
  }
}

void GazeboMechanismControl::FiniChild()
{
  std::cout << "--------------- calling FiniChild in GazeboMechanismControl --------------------" << std::endl;

  hw_.~HardwareInterface();
  mc_.~MechanismControl();
  mcn_.~MechanismControlNode();

  deleteElements(&joints_);
  delete fake_state_;
}

void GazeboMechanismControl::ReadPr2Xml(XMLConfigNode *node)
{

  std::string tmp_param_string;
  this->rosnode_->get_param("robotdesc/pr2",tmp_param_string);


  // wait for robotdesc/pr2 on param server
  while(tmp_param_string.empty())
  {
    std::cout << "WARNING: gazebo mechanism control plugin is waiting for robotdesc/pr2 in param server.  run merge/roslaunch send.xml or similar." << std::endl;
    this->rosnode_->get_param("robotdesc/pr2",tmp_param_string);
    usleep(100000);
  }

  std::cout << "gazebo mechanism control got pr2.xml from param server, parsing it..." << std::endl;
  //std::cout << tmp_param_string << std::endl;

  // initialize TiXmlDocument doc with a string
  TiXmlDocument doc;
  if (!doc.Parse(tmp_param_string.c_str()))
  {
    fprintf(stderr, "Error: Could not load the gazebo mechanism_control plugin's configuration file: %s\n",
            tmp_param_string.c_str());
    abort();
  }
  urdf::normalizeXml(doc.RootElement());
  //std::cout << *(doc.RootElement()) << std::endl;

  // Pulls out the list of actuators used in the robot configuration.
  struct GetActuators : public TiXmlVisitor
  {
    std::set<std::string> actuators;
    virtual bool VisitEnter(const TiXmlElement &elt, const TiXmlAttribute *)
    {
      if (elt.ValueStr() == std::string("actuator") && elt.Attribute("name"))
        actuators.insert(elt.Attribute("name"));
      else if (elt.ValueStr() == std::string("rightActuator") && elt.Attribute("name"))
        actuators.insert(elt.Attribute("name"));
      else if (elt.ValueStr() == std::string("leftActuator") && elt.Attribute("name"))
        actuators.insert(elt.Attribute("name"));
      return true;
    }
  } get_actuators;
  doc.RootElement()->Accept(&get_actuators);

  // Places the found actuators into the hardware interface.
  std::set<std::string>::iterator it;
  for (it = get_actuators.actuators.begin(); it != get_actuators.actuators.end(); ++it)
  {
    //std::cout << " adding actuator " << (*it) << std::endl;
    hw_.actuators_.push_back(new Actuator(*it));
  }

  // Setup mechanism control node
  mcn_.initXml(doc.RootElement());

  for (unsigned int i = 0; i < mc_.state_->joint_states_.size(); ++i)
    mc_.state_->joint_states_[i].calibrated_ = true;
}

} // namespace gazebo
