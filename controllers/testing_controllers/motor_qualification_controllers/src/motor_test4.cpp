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
#include <motor_qualification_controllers/motor_test4.h>

using namespace std;
using namespace controller;

ROS_REGISTER_CONTROLLER(MotorTest4)

MotorTest4::MotorTest4():
  publisher_("/diagnostics", 1), test_effort_(8000),test_velocity_(8000)
{
  robot_ = NULL;
  actuator_ = NULL;
  joint_ = NULL;
  
  count_=1;
  duration_=6;
  torque_=0;
  speed_torque_constant_=100;
  initial_time_=0;
  complete = false;
  
  
}

MotorTest4::~MotorTest4()
{
}

void MotorTest4::init(double speed_torque_constant, double torque, std::string fixture_name, double time, std::string name ,mechanism::RobotState *robot)
{
  robot_ = robot;
  joint_ = robot->getJointState(name);
  actuator_ = robot->model_->getActuator("test_motor");
  fixture_joint_ =robot->getJointState(fixture_name);

  torque_=torque;
  speed_torque_constant_=speed_torque_constant;
  initial_time_=time;

}

bool MotorTest4::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  robot_ = robot;

  TiXmlElement *j = config->FirstChildElement("joint");
  if (!j)
  {
    fprintf(stderr, "MotorTest4 was not given a joint\n");
    return false;
  }

  const char *joint_name = j->Attribute("name");
  joint_ = joint_name ? robot->getJointState(joint_name) : NULL;
  if (!joint_)
  {
    fprintf(stderr, "MotorTest4 could not find joint named \"%s\"\n", joint_name);
    return false;
  }
  
  TiXmlElement *cd = j->FirstChildElement("controller_defaults");
  if (cd)
  {
    double speed_torque_constant = atof(cd->Attribute("speed_torque_constant"));
    double torque = atof(cd->Attribute("torque"));
    std::string fixture_name = cd->Attribute("fixture_name");
    init(speed_torque_constant, torque, fixture_name, robot->hw_->current_time_,j->Attribute("name"), robot);
  }
  else
  {
    fprintf(stderr, "MotorTest4's config did not specify the default control parameters.\n");
  }
  return true;
}


double MotorTest4::getTime()
{
  return robot_->hw_->current_time_;
}

void MotorTest4::update()
{

  double time = robot_->hw_->current_time_;

  
  if((time-initial_time_)<duration_)
  {
    joint_->commanded_effort_ = torque_;
  }
  else if((time-initial_time_)<2*duration_)
  {
    joint_->commanded_effort_ = torque_;
    test_effort_(count_) = actuator_->state_.last_measured_effort_;   
    test_velocity_(count_) =joint_->velocity_;

    count_++;
    
  }
  else if (!complete)
  {
    joint_->commanded_effort_ = 0.0; 
    analysis();
    complete = true;
  }
  else 
    return;
}

void MotorTest4::analysis()
{
  diagnostic_message_.set_status_size(1);
  robot_msgs::DiagnosticStatus *status = diagnostic_message_.status;
  status->set_values_size(2);
  status->name = "MotorTest";
 
  double rms_effort  = test_effort_.NormFrobenius()/sqrt(count_);
  double rms_velocity = test_velocity_.NormFrobenius()/sqrt(count_);
  double speed_torque_const_meas = rms_velocity/rms_effort; 
  printf("rms_velocity: %lf, rms_effort: %lf \n", rms_velocity,rms_effort);
   
  if (speed_torque_const_meas < speed_torque_constant_-10 || speed_torque_const_meas > speed_torque_constant_+10)
  {
    //the motor isn't moving
    status->level = 2;
    status->message = "ERROR: The motor is not correctly labeled. The speed constant is not correct.";
    status->values[0].label = "measured speed constant";
    status->values[0].value = speed_torque_const_meas;
    status->values[1].label = "expected speed constant";
    status->values[1].value = speed_torque_constant_;
  }
  else
  {
    //test passed
    status->level = 0;
    status->message = "OK: Passed.";
    status->values[0].label = "measured speed constant";
    status->values[0].value = speed_torque_const_meas;
    status->values[1].label = "expected speed constant";
    status->values[1].value = speed_torque_constant_;
   }
   publisher_.publish(diagnostic_message_);
  return;
}

ROS_REGISTER_CONTROLLER(MotorTest4Node)
MotorTest4Node::MotorTest4Node()
{
  c_ = new MotorTest4();
}

MotorTest4Node::~MotorTest4Node()
{
  delete c_;
}

void MotorTest4Node::update()
{
  c_->update();
}


bool MotorTest4Node::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  std::string topic = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic == "")
  {
    fprintf(stderr, "No topic given to MotorTest4Node\n");
    return false;
  }
  // Advertise topics
  if (!c_->initXml(robot, config))
    return false;
  return true;
}

