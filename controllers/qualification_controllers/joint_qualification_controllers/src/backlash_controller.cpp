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
#include <joint_qualification_controllers/backlash_controller.h>
#include <math.h>

using namespace std;
using namespace control_toolbox;

namespace controller {

ROS_REGISTER_CONTROLLER(BacklashController)

BacklashController::BacklashController():
joint_state_(NULL), robot_(NULL), node(ros::Node::instance())
{
  test_data_.test_name ="backlash";
  test_data_.time.resize(80000);
  test_data_.cmd.resize(80000);
  test_data_.effort.resize(80000);
  test_data_.position.resize(80000);
  test_data_.velocity.resize(80000);
  test_data_.arg_name.resize(1);
  test_data_.arg_name[0]="error_tolerance";
  test_data_.arg_value.resize(1);
  duration_ =0.0;
  initial_time_=0;
  last_time_=0;
  count_=0;
  done_=0;
}

BacklashController::~BacklashController()
{
}

void BacklashController::init(double freq, double duration, double amplitude, double error_tolerance, double time, std::string name,mechanism::RobotState *robot)
{
  assert(robot);
  robot_ = robot;
  joint_state_ = robot->getJointState(name);
  freq_=freq;
  amplitude_=amplitude;
  node->advertise<robot_msgs::TestData>( "/test_data", 0 );
  test_data_.arg_value[0]=error_tolerance;
  duration_ = duration;     //in seconds
  initial_time_=time;       //in seconds

}

bool BacklashController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{

  TiXmlElement *jnt = config->FirstChildElement("joint");
  if (jnt)
  {
    double freq = atof(jnt->FirstChildElement("controller_defaults")->Attribute("freq"));
    double amplitude = atof(jnt->FirstChildElement("controller_defaults")->Attribute("amplitude"));
    double duration = atof(jnt->FirstChildElement("controller_defaults")->Attribute("duration"));
    double error_tolerance = atof(jnt->FirstChildElement("controller_defaults")->Attribute("error_tolerance")); 
    init(freq, duration, amplitude, error_tolerance, robot->hw_->current_time_,jnt->Attribute("name"), robot);
  }
  return true;
}

void BacklashController::update()
{
  double time = robot_->hw_->current_time_;
  // wait until the joint is calibrated if it has limits
  if(!joint_state_->calibrated_ && joint_state_->joint_->type_!=mechanism::JOINT_CONTINUOUS)
  {
    initial_time_=time;
    last_time_=time;
    return;
  }
  
  if((time-initial_time_)<=duration_)
  {
    //double A=amplitude_*(time-initial_time_)/duration_;
    //joint_state_->commanded_effort_ = A*sin((time-initial_time_)*2*M_PI*freq_);

    double A=amplitude_; //Eric's hack
    joint_state_->commanded_effort_ = A * ((sin(time-initial_time_*2*M_PI*freq_))>0 ? 1 : -1);

    if (count_<80000 && !done_)
    { 
    test_data_.time[count_]=time;
    test_data_.cmd[count_]=joint_state_->commanded_effort_;
    test_data_.effort[count_]=joint_state_->applied_effort_;
    test_data_.position[count_]=joint_state_->position_;
    test_data_.velocity[count_]=joint_state_->velocity_;
    count_++;
    }
  }
  else if(!done_)
  {
    joint_state_->commanded_effort_=0;
    analysis();
    done_=1;
  }
  else
  {
    joint_state_->commanded_effort_=0;
  }
}

void BacklashController::analysis()
{
  diagnostic_message_.set_status_size(1);
  robot_msgs::DiagnosticStatus *status = &diagnostic_message_.status[0];

  status->name = "BacklashTest";
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
  
  if ((node = ros::Node::instance()) != NULL)
  {
    node->publish("/test_data", test_data_);
    node->publish("/diagnostics", diagnostic_message_);
  }
  return;
}


ROS_REGISTER_CONTROLLER(BacklashControllerNode)
BacklashControllerNode::BacklashControllerNode()
{
  c_ = new BacklashController();
}

BacklashControllerNode::~BacklashControllerNode()
{
  delete c_;
}

void BacklashControllerNode::update()
{
  c_->update();
}


bool BacklashControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  if (!c_->initXml(robot, config))
    return false;

  return true;
}

}
