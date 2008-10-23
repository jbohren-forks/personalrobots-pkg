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
#include <gripper_qualification_controllers/gripper_test1.h>

using namespace std;
using namespace controller;

ROS_REGISTER_CONTROLLER(GripperTest1)

GripperTest1::GripperTest1():
  publisher_("/diagnostics", 1),data_publisher_("/gripper_test_data", 5)
{
  robot_ = NULL;
  joint_ = NULL;
  test_effort_.set_vals_size(80000);
  test_velocity_.set_vals_size(80000);
  test_cmd_.set_vals_size(80000);
  test_position_.set_vals_size(80000);
  test_time_.set_vals_size(80000);
  test_effort_.name="effort";
  test_velocity_.name="velocity";
  test_cmd_.name="cmd";
  test_position_.name="position";
  test_time_.name="time";

  velocity_=0;
  initial_time_=0;
  max_effort_=0;
  complete = false;
  start =true;
  loop_count_=0;
  count_=1;
}

GripperTest1::~GripperTest1()
{
}

void GripperTest1::init( double velocity, double max_effort, double time, std::string name ,mechanism::RobotState *robot)
{
  robot_ = robot;
  joint_ = robot->getJointState(name);
  actuator_ = robot->model_->getActuator("gripper_left_motor");
  
  //printf("velocity: %f\n",velocity);
  velocity_=velocity;
  max_effort_=max_effort;
  initial_time_=time;

}

bool GripperTest1::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  
  assert(robot);
  robot_ = robot;

  TiXmlElement *j = config->FirstChildElement("joint");
  if (!j)
  {
    fprintf(stderr, "GripperTest1 was not given a joint\n");
    return false;
  }

  const char *joint_name = j->Attribute("name");
  joint_ = joint_name ? robot->getJointState(joint_name) : NULL;
  if (!joint_)
  {
    fprintf(stderr, "GripperTest1 could not find joint named \"%s\"\n", joint_name);
    return false;
  }
 

  velocity_controller_ = new JointVelocityController();
  velocity_controller_->initXml(robot, config);
   
  TiXmlElement *cd = j->FirstChildElement("controller_defaults");
  if (cd)
  { 
    double velocity = atof(cd->Attribute("velocity"));
    double max_effort = atof(cd->Attribute("max_effort"));
    init(velocity, max_effort, robot->hw_->current_time_,j->Attribute("name"), robot);
  }
  else
  {
    fprintf(stderr, "GripperTest1's config did not specify the default control parameters.\n");
  }
  return true;
}




void GripperTest1::update()
{
  double time = robot_->hw_->current_time_;
  
  velocity_controller_->update();
  
  
  //fprintf(stderr,"joint_cmd: %f, effort: %f, velocity: %f\n",velocity_controller_->getCommand(),actuator_->state_.last_measured_effort_,joint_->velocity_);
  
  enum { STOPPED, STARTING, MOVING, ANALYZING, DONE};
  static int state = STOPPED;
  static int starting_count = 0;
  if (state == STOPPED || state == STARTING || state == MOVING)
  {
    test_effort_.vals[count_] = actuator_->state_.last_measured_effort_;   
    test_velocity_.vals[count_] =joint_->velocity_;
    test_position_.vals[count_] =joint_->position_;
    test_time_.vals[count_] = time;
    test_cmd_.vals[count_]= velocity_controller_->getCommand();
    count_++;
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
    if (fabs(joint_->velocity_) < 1 && fabs(actuator_->state_.last_measured_effort_) > max_effort_)
    {
      velocity_controller_->setCommand(0.0);
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

void GripperTest1::analysis()
{
  diagnostic_message_.set_status_size(1);
  
  robot_msgs::DiagnosticStatus *status = &diagnostic_message_.status[0];

  status->name = "GripperTest";
  
  //test passed
  status->level = 0;
  status->message = "OK: Passed.";
  
  ros::node* node;
  
  if ((node = ros::node::instance()) != NULL)
  {
    node->publish("/gripper_test_data", test_effort_);
    node->publish("/gripper_test_data", test_velocity_);
    node->publish("/gripper_test_data", test_position_);
    node->publish("/gripper_test_data", test_time_);
    node->publish("/gripper_test_data", test_cmd_);
    node->publish("/diagnostics", diagnostic_message_);
  }

  //publisher_.publish(diagnostic_message_);
  return;
}

ROS_REGISTER_CONTROLLER(GripperTest1Node)
GripperTest1Node::GripperTest1Node()
{
  c_ = new GripperTest1();
}

GripperTest1Node::~GripperTest1Node()
{
  delete c_;
}

void GripperTest1Node::update()
{
  c_->update();
}


bool GripperTest1Node::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  std::string topic = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic == "")
  {
    fprintf(stderr, "No topic given to GripperTest1Node\n");
    return false;
  }
  // Advertise topics
  if (!c_->initXml(robot, config))
    return false;
  return true;
}

