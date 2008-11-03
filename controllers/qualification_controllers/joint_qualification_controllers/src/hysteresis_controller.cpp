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
  joint_(NULL), robot_(NULL), publisher_("/diagnostics", 1),data_publisher_("/hysteresis_data", 5)
{
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
  state = STOPPED;
  starting_count = 0;
  velocity_=0;
  initial_time_=0;
  max_effort_=0;
  complete = false;
  start =true;
  loop_count_=0;
  count_=1;
}

HysteresisController::~HysteresisController()
{
}

void HysteresisController::init( double velocity, double max_effort, double time, std::string name ,mechanism::RobotState *robot)
{
  robot_ = robot;
  joint_ = robot->getJointState(name);

  //printf("velocity: %f\n",velocity);
  velocity_=velocity;
  max_effort_=max_effort;
  initial_time_=time;

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
    fprintf(stderr, "HysteresisController's config did not specify the default control parameters.\n");
  }
  return true;
}

void HysteresisController::update()
{
  double time = robot_->hw_->current_time_;
  velocity_controller_->update();

  static int state = STOPPED;
  static int starting_count = 0;
  if (state == STOPPED || state == STARTING || state == MOVING || count_<80000)
  { 
    test_effort_.vals[count_] = joint_->applied_effort_;
    test_velocity_.vals[count_] =joint_->velocity_;
    test_position_.vals[count_] =joint_->position_;
    test_time_.vals[count_] = time;
    double temp;
    velocity_controller_->getCommand(temp);
    test_cmd_.vals[count_] = temp;
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
    if (fabs(joint_->velocity_) < 1 && fabs(joint_->commanded_effort_) > max_effort_)
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

void HysteresisController::analysis()
{
  diagnostic_message_.set_status_size(1);

  robot_msgs::DiagnosticStatus *status = &diagnostic_message_.status[0];

  status->name = "HysteresisTest";

  //test passed
  status->level = 0;
  status->message = "OK: Passed.";

  ros::node* node;

  if ((node = ros::node::instance()) != NULL)
  {
    node->publish("/hysteresis_data", test_effort_);
    node->publish("/hysteresis_data", test_velocity_);
    node->publish("/hysteresis_data", test_position_);
    node->publish("/hysteresis_data", test_time_);
    node->publish("/hysteresis_data", test_cmd_);
    //node->publish("/diagnostics", diagnostic_message_);
  }

  //publisher_.publish(diagnostic_message_);
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
  std::string topic = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic == "")
  {
    fprintf(stderr, "No topic given to HysteresisControllerNode\n");
    return false;
  }
  // Advertise topics
  if (!c_->initXml(robot, config))
    return false;
  return true;
}

