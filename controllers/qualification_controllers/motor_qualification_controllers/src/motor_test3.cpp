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
#include <motor_qualification_controllers/motor_test3.h>

using namespace std;
using namespace controller;

ROS_REGISTER_CONTROLLER(MotorTest3)

MotorTest3::MotorTest3():
  publisher_("/diagnostics", 1)
{
  robot_ = NULL;

  joint_ = NULL;


  duration_=20;

  amplitude_=0;
  initial_time_=0;
  complete = false;

}

MotorTest3::~MotorTest3()
{
}

void MotorTest3::init( double amplitude, std::string fixture_name, double time, std::string name ,mechanism::RobotState *robot)
{
  robot_ = robot;
  joint_ = robot->getJointState(name);
  fixture_joint_ =robot->getJointState(fixture_name);
  duration_=20;
  sweep_controller_ = new SineSweepController();
  sweep_controller_->init(1.0, 40, duration_, amplitude,robot->hw_->current_time_ ,name,robot);

  amplitude_=amplitude;
  initial_time_=time;

}

bool MotorTest3::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  robot_ = robot;

  TiXmlElement *j = config->FirstChildElement("joint");
  if (!j)
  {
    fprintf(stderr, "MotorTest3 was not given a joint\n");
    return false;
  }

  const char *joint_name = j->Attribute("name");
  joint_ = joint_name ? robot->getJointState(joint_name) : NULL;
  if (!joint_)
  {
    fprintf(stderr, "MotorTest3 could not find joint named \"%s\"\n", joint_name);
    return false;
  }

  TiXmlElement *cd = j->FirstChildElement("controller_defaults");
  if (cd)
  {
    double amplitude = atof(cd->Attribute("amplitude"));
    std::string fixture_name = cd->Attribute("fixture_name");
    init(amplitude, fixture_name, robot->hw_->current_time_,j->Attribute("name"), robot);
  }
  else
  {
    fprintf(stderr, "MotorTest3's config did not specify the default control parameters.\n");
  }
  return true;
}



void MotorTest3::update()
{
  double time = robot_->hw_->current_time_;


  if((time-initial_time_)<duration_)
  {
    sweep_controller_->update();
  }
  else if (!complete)
  {

    analysis();
    complete = true;
  }
  else
    return;
}

void MotorTest3::analysis()
{
  publisher_.lock();  // Screw realtime
  publisher_.msg_.set_status_size(1);
  robot_msgs::DiagnosticStatus *status = &publisher_.msg_.status[0];

  status->name = "MotorTest";

  //test passed
  status->level = 0;
  status->message = "OK: Passed.";

  publisher_.unlockAndPublish();
  return;
}

ROS_REGISTER_CONTROLLER(MotorTest3Node)
MotorTest3Node::MotorTest3Node()
{
  c_ = new MotorTest3();
}

MotorTest3Node::~MotorTest3Node()
{
  delete c_;
}

void MotorTest3Node::update()
{
  c_->update();
}


bool MotorTest3Node::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  std::string topic = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic == "")
  {
    fprintf(stderr, "No topic given to MotorTest3Node\n");
    return false;
  }
  // Advertise topics
  if (!c_->initXml(robot, config))
    return false;
  return true;
}

