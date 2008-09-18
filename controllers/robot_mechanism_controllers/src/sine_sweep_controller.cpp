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
#include <robot_mechanism_controllers/sine_sweep_controller.h>
#include <math.h>

using namespace std;
namespace controller {

ROS_REGISTER_CONTROLLER(SineSweepController)

SineSweepController::SineSweepController()
{
  robot_ = NULL;
  joint_state_ = NULL;

  start_freq_=0;
  end_freq_=0;
  amplitude_=0;
  duration_=0;
  initial_time_=0;
}

SineSweepController::~SineSweepController()
{
}

void SineSweepController::init(double start_freq, double end_freq, double duration, double amplitude, double time,std::string name,mechanism::RobotState *robot)
{
  robot_ = robot;
  joint_state_ = robot->getJointState(name);

  start_freq_=start_freq;   //in Hz
  end_freq_=end_freq;       //in Hz
  amplitude_=amplitude;     //in Newtons
  duration_=duration;       //in seconds
  initial_time_=time;       //in seconds

  start_angular_freq_ =2*M_PI*start_freq_;
  end_angular_freq_ =2*M_PI*end_freq_;
  K_factor_ = (start_angular_freq_*duration_)/log(end_angular_freq_/start_angular_freq_);
  L_factor_ = (duration_)/log(end_angular_freq_/start_angular_freq_);
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

// Return the current position command
double SineSweepController::getCommand()
{
  return joint_state_->commanded_effort_;
}

// Return the measured joint position
double SineSweepController::getMeasuredEffort()
{
  return joint_state_->applied_effort_;
}


double SineSweepController::getTime()
{
  return robot_->hw_->current_time_;
}

void SineSweepController::update()
{
  double time = robot_->hw_->current_time_;

  if((time-initial_time_)<duration_)
  {
    joint_state_->commanded_effort_ = amplitude_*sin(K_factor_*(exp((time-initial_time_)/(L_factor_))-1));
  }
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
