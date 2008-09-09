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
#include <motor_qualification_controllers/motor_test1.h>

using namespace std;
using namespace controller;

ROS_REGISTER_CONTROLLER(MotorTest1)

MotorTest1::MotorTest1():
  publisher_("/diagnostics", 1)
{
  robot_ = NULL;
  joint_ = NULL;
  
  duration_=0;
  initial_time_=0;
  fixture_joint_start_pos_= 0;
  test_joint_start_pos_= 0;
  fixture_joint_end_pos_= 0;
  test_joint_end_pos_= 0;
  complete = false;
  
}

MotorTest1::~MotorTest1()
{
}

void MotorTest1::init(double duration, std::string fixture_name, double time, std::string name ,mechanism::RobotState *robot)
{
  robot_ = robot;
  joint_ = robot->getJointState(name);
  fixture_joint_ =robot->getJointState(fixture_name);

  duration_=duration;
  initial_time_=time;
  fixture_joint_start_pos_=fixture_joint_->position_;
  test_joint_start_pos_=joint_->position_;
}

bool MotorTest1::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  robot_ = robot;

  TiXmlElement *j = config->FirstChildElement("joint");
  if (!j)
  {
    fprintf(stderr, "MotorTest1 was not given a joint\n");
    return false;
  }

  const char *joint_name = j->Attribute("name");
  joint_ = joint_name ? robot->getJointState(joint_name) : NULL;
  if (!joint_)
  {
    fprintf(stderr, "MotorTest1 could not find joint named \"%s\"\n", joint_name);
    return false;
  }
  
  TiXmlElement *cd = j->FirstChildElement("controller_defaults");
  if (cd)
  {
    double duration = atof(cd->Attribute("duration"));
    std::string fixture_name = cd->Attribute("fixture_name");
    init(duration, fixture_name, robot->hw_->current_time_,j->Attribute("name"), robot);
  }
  else
  {
    fprintf(stderr, "MotorTest1's config did not specify the default control parameters.\n");
  }
  return true;
}


double MotorTest1::getTime()
{
  return robot_->hw_->current_time_;
}

void MotorTest1::update()
{
  double time = robot_->hw_->current_time_;
  
  if((time-initial_time_)<duration_)
  {
    joint_->commanded_effort_ = 0.01;
  }
  else if((time-initial_time_)<2*duration_)
  {
    joint_->commanded_effort_ = 0.0;
  }
  else if (!complete)
  {
    fixture_joint_end_pos_=fixture_joint_->position_;
    test_joint_end_pos_=joint_->position_;
    analysis();
    complete = true;
  }
  else 
    return;
}

void MotorTest1::analysis()
{
  double f_delta = fixture_joint_end_pos_-fixture_joint_start_pos_;
  double t_delta = test_joint_end_pos_-test_joint_start_pos_;
  diagnostic_message_.set_status_size(1);
  robot_msgs::DiagnosticStatus *status = diagnostic_message_.status;
  status->name = "MotorTest1";
  if (f_delta==0 && t_delta==0)
  {
    //the motor isn't moving
    status->level = 2;
    status->message = "ERROR: The motor is not moving.";
  }
  else if (f_delta>0 && t_delta==0)
  {
    //the motor doesn't have an encoder
    status->level = 2;
    status->message = "ERROR: The motor encoder is not attached or not powered.";
    
  }
  else if(f_delta+1<t_delta || f_delta-1>t_delta)
  {
    //the encoder is slipping
    status->level = 2;
    status->message = "ERROR: The motor encoder is slipping.";
  }
  else if(f_delta>0 && t_delta<0)
  {
    //the encoder is reversed 
    status->level = 2;
    status->message = "ERROR: The motor encoder is reversed.";
  }
  else if(f_delta<0 && t_delta<0) 
  {
    //motor reversed
    status->level = 2;
    status->message = "ERROR: The motor wiring is reversed.";
  }   
  else
  {
    //test passed
    status->level = 0;
    status->message = "OK: The motor passed test.";
   }
   publisher_.publish(diagnostic_message_);
  return;
}

ROS_REGISTER_CONTROLLER(MotorTest1Node)
MotorTest1Node::MotorTest1Node()
{
  c_ = new MotorTest1();
}

MotorTest1Node::~MotorTest1Node()
{
  delete c_;
}

void MotorTest1Node::update()
{
  c_->update();
}


bool MotorTest1Node::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  ros::node *node = ros::node::instance();

  std::string topic = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic == "")
  {
    fprintf(stderr, "No topic given to MotorTest1Node\n");
    return false;
  }
  // Advertise topics
  if (!c_->initXml(robot, config))
    return false;
  return true;
}

