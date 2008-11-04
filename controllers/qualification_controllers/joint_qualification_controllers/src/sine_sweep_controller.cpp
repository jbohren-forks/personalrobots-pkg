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
#include <joint_qualification_controllers/sine_sweep_controller.h>
#include <math.h>

using namespace std;
using namespace control_toolbox;

namespace controller {

ROS_REGISTER_CONTROLLER(SineSweepController)

SineSweepController::SineSweepController():
joint_state_(NULL), robot_(NULL), publisher_("/diagnostics", 1), data_publisher_("/sinesweep_data", 5)
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
  sweep_=NULL;
  duration_ =0.0;
  initial_time_=0;
  count_=1;
  done_=0;
}

SineSweepController::~SineSweepController()
{
}

void SineSweepController::init(double start_freq, double end_freq, double duration, double amplitude, double time,std::string name,mechanism::RobotState *robot)
{
  robot_ = robot;
  joint_state_ = robot->getJointState(name);
  sweep_ = new SineSweep;
  sweep_->init(start_freq, end_freq, duration, amplitude);
  
  duration_ = duration;     //in seconds
  initial_time_=time;       //in seconds

}

bool SineSweepController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{

  TiXmlElement *jnt = config->FirstChildElement("joint");
  if (jnt)
  {
    double start_freq = atof(jnt->FirstChildElement("controller_defaults")->Attribute("start_freq"));
    double end_freq = atof(jnt->FirstChildElement("controller_defaults")->Attribute("end_freq"));
    double amplitude = atof(jnt->FirstChildElement("controller_defaults")->Attribute("amplitude"));
    double duration = atof(jnt->FirstChildElement("controller_defaults")->Attribute("duration"));
    init(start_freq,  end_freq, duration, amplitude,robot->hw_->current_time_,jnt->Attribute("name"), robot);
  }
  return true;
}

void SineSweepController::update()
{
  double time = robot_->hw_->current_time_;
  if (count_<80000)
  { 
    test_effort_.vals[count_] = joint_state_->applied_effort_;
    test_velocity_.vals[count_] =joint_state_->velocity_;
    test_position_.vals[count_] =joint_state_->position_;
    test_time_.vals[count_] = time;
    test_cmd_.vals[count_] = joint_state_->commanded_effort_;
    count_++;
  }
  
  if((time-initial_time_)<=duration_)
  {
    joint_state_->commanded_effort_ = sweep_->update(time-initial_time_);
  }
  else if(!done_)
  {
    analysis();
    done_=1;
  }
  else
  {
    joint_state_->commanded_effort_=0;
  }
}

void SineSweepController::analysis()
{
  diagnostic_message_.set_status_size(1);

  robot_msgs::DiagnosticStatus *status = &diagnostic_message_.status[0];

  status->name = "SineSweepTest";

  //test passed
  status->level = 0;
  status->message = "OK: Done.";

  ros::node* node;

  if ((node = ros::node::instance()) != NULL)
  {
    node->publish("/sinesweep_data", test_effort_);
    node->publish("/sinesweep_data", test_velocity_);
    node->publish("/sinesweep_data", test_position_);
    node->publish("/sinesweep_data", test_time_);
    node->publish("/sinesweep_data", test_cmd_);
    //node->publish("/diagnostics", diagnostic_message_);
  }

  //publisher_.publish(diagnostic_message_);
  return;
}


ROS_REGISTER_CONTROLLER(SineSweepControllerNode)
SineSweepControllerNode::SineSweepControllerNode()
{
  c_ = new SineSweepController();
}

SineSweepControllerNode::~SineSweepControllerNode()
{
  delete c_;
}

void SineSweepControllerNode::update()
{
  c_->update();
}


bool SineSweepControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  string prefix = config->Attribute("name");

  c_->initXml(robot, config);

  return true;
}

}
