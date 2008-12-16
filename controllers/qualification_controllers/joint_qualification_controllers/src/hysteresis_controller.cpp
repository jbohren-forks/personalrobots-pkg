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
#include <joint_qualification_controllers/hysteresis_controller.h>

using namespace std;
using namespace controller;

ROS_REGISTER_CONTROLLER(HysteresisController)

HysteresisController::HysteresisController():
  joint_(NULL), robot_(NULL), node(ros::node::instance())
{
  test_data_.test_name ="hysteresis";
  test_data_.time.resize(80000);
  test_data_.cmd.resize(80000);
  test_data_.effort.resize(80000);
  test_data_.position.resize(80000);
  test_data_.velocity.resize(80000);
  test_data_.arg_name.resize(4);
  test_data_.arg_name[0]="min_expected_effort";
  test_data_.arg_name[1]="max_expected_effort";
  test_data_.arg_name[2]="min_pos";
  test_data_.arg_name[3]="max_pos";
  test_data_.arg_value.resize(4);
  state = STOPPED;
  starting_count = 0;
  velocity_=0;
  initial_time_=0;
  max_effort_=0;
  complete = false;
  start =true;
  loop_count_=0;
  count_=0;
  
}

HysteresisController::~HysteresisController()
{
}

void HysteresisController::init( double velocity, double max_effort, double max_expected_effort, double min_expected_effort, double min_pos, double max_pos, double time, std::string name ,mechanism::RobotState *robot)
{
  assert(robot);
  robot_ = robot;
  joint_ = robot->getJointState(name);
  if(name=="r_gripper_joint" || name=="l_gripper_joint")
  {
    joint_->calibrated_ = true;

  }
  
  test_data_.arg_value[0]=min_expected_effort;
  test_data_.arg_value[1]=max_expected_effort;
  test_data_.arg_value[2]=min_pos;
  test_data_.arg_value[3]=max_pos;
  
  node->advertise<robot_msgs::TestData>( "/test_data", 0);
  
  velocity_=velocity;
  max_effort_=max_effort;
  initial_time_=time;
  initial_position_=joint_->position_;

}

bool HysteresisController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  
  TiXmlElement *j = config->FirstChildElement("joint");
  if (!j)
  {
    fprintf(stderr, "HysteresisController was not given a joint\n");
    return false;
  }

  const char *joint_name = j->Attribute("name");
  joint_ = joint_name ? robot->getJointState(joint_name) : NULL;
  if (!joint_)
  {
    fprintf(stderr, "HysteresisController could not find joint named \"%s\"\n", joint_name);
    return false;
  }
  ///\todo check velocity controller comes up
  velocity_controller_ = new JointVelocityController();
  velocity_controller_->initXml(robot, config);

  TiXmlElement *cd = j->FirstChildElement("controller_defaults");
  if (cd)
  {
    double velocity = atof(cd->Attribute("velocity"));
    double max_effort = atof(cd->Attribute("max_effort"));
    double max_expected_effort = atof(cd->Attribute("max_expected_effort"));
    double min_expected_effort = atof(cd->Attribute("min_expected_effort"));
    double min_pos = atof(cd->Attribute("min_pos"));
    double max_pos = atof(cd->Attribute("max_pos")); 
    init(velocity, max_effort, max_expected_effort, min_expected_effort, min_pos, max_pos, robot->hw_->current_time_,j->Attribute("name"), robot);
  }
  else
  {
    fprintf(stderr, "HysteresisController's config did not specify the default control parameters.\n");
    return false;
  }
  return true;
}

void HysteresisController::update()
{

  // wait until the joint is calibrated if it has limits
  if(!joint_->calibrated_ && joint_->joint_->type_!=mechanism::JOINT_CONTINUOUS)
  {
    return;
  }

  double time = robot_->hw_->current_time_;
  velocity_controller_->update();
  
  if (state == STOPPED || state == STARTING || state == MOVING)
  {
    if(state!=DONE && count_<80000 && loop_count_>1)
    {
      double cmd;
      velocity_controller_->getCommand(cmd);
      
      test_data_.time[count_]=time;
      test_data_.cmd[count_]=cmd;
      test_data_.effort[count_]=joint_->applied_effort_;
      test_data_.position[count_]=joint_->position_;
      test_data_.velocity[count_]=joint_->velocity_;
      count_++;
    }
  }

  switch (state)
  {
  case STOPPED:
    velocity_controller_->setCommand(velocity_);
    velocity_ *= -1.0;
    ++loop_count_;
    starting_count = 0;
    state = STARTING;
    break;
  case STARTING:
    
    ++starting_count;
    if (starting_count > 100)
      state = MOVING;
    break;
  case MOVING:
    if (fabs(joint_->velocity_) < 1 && fabs(joint_->commanded_effort_) > max_effort_ && joint_->joint_->type_!=mechanism::JOINT_CONTINUOUS)
    {
      velocity_controller_->setCommand(0.0);
      if (loop_count_ < 3)
        state = STOPPED;
      else
        state = ANALYZING;
    }
    else if(fabs(joint_->position_-initial_position_)>6.28 && joint_->joint_->type_==mechanism::JOINT_CONTINUOUS) 
    {
   
      velocity_controller_->setCommand(0.0);
      initial_position_=joint_->position_;
      if (loop_count_ < 3)
        state = STOPPED;
      else
        state = ANALYZING;
    }
    break;
  case ANALYZING:
    velocity_controller_->setCommand(0.0);
    analysis();
    state = DONE;
    break;
  case DONE:
    velocity_controller_->setCommand(0.0);
    break;
  }

}

void HysteresisController::analysis()
{
  diagnostic_message_.set_status_size(1);
  robot_msgs::DiagnosticStatus *status = &diagnostic_message_.status[0];

  status->name = "HysteresisTest";
  count_=count_-1;
  //test done
  assert(count_>0);
  status->level = 0;
  status->message = "OK: Done.";
  test_data_.time.resize(count_);
  test_data_.cmd.resize(count_);
  test_data_.effort.resize(count_);
  test_data_.position.resize(count_);
  test_data_.velocity.resize(count_);
  
  if ((node = ros::node::instance()) != NULL)
  { 
    node->publish("/test_data", test_data_);
    node->publish("/diagnostics", diagnostic_message_);
  }
  return; 
}

ROS_REGISTER_CONTROLLER(HysteresisControllerNode)
HysteresisControllerNode::HysteresisControllerNode()
{
  c_ = new HysteresisController();
}

HysteresisControllerNode::~HysteresisControllerNode()
{
  delete c_;
}

void HysteresisControllerNode::update()
{
  c_->update();
}

bool HysteresisControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  if (!c_->initXml(robot, config))
    return false;
  return true;
}

