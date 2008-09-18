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
#include <algorithm>

#include <pr2_mechanism_controllers/laser_scanner_controller.h>
#include <math_utils/angles.h>
#include <math_utils/velocity.h>
using namespace std;
using namespace controller;
#define MIN_MOVING_SPEED 0.3
ROS_REGISTER_CONTROLLER(LaserScannerController)

LaserScannerController::LaserScannerController()
{
  robot_ = NULL;
  joint_ = NULL;

  command_ = 0;
  last_time_ = 0;
  current_mode_ = VELOCITY; //Start out in velocity mode


  passed_center_ = true;
  upper_turnaround_offset_ = 0.0;
  lower_turnaround_offset_ = 0.0;
  automatic_turnaround_ = false;
  upper_deceleration_buffer_ = 0.0;
  lower_deceleration_buffer_ = 0.0;
}

LaserScannerController::~LaserScannerController()
{

}

void LaserScannerController::init(double p_gain, double i_gain, double d_gain, double windup, double time, std::string name, mechanism::RobotState *robot)
{
  robot_ = robot;
  joint_ = robot->getJointState(name);

  abort(); //joint_position_controller_.init( p_gain,  i_gain,  d_gain,  windup, time, name, robot);
  command_= 0;
  last_time_= time;
}

bool LaserScannerController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);


  robot_ = robot;
  last_time_ = robot->hw_->current_time_;

  TiXmlElement *vel = config->FirstChildElement("velocity");

  //Perform checks at highest level to give the most informative error message possible
  TiXmlElement *j = vel->FirstChildElement("joint");
  if (!j)
  {
    fprintf(stderr, "LaserScannerController was not given a joint\n");
    return false;
  }

  const char *joint_name = j->Attribute("name");
  int index = joint_name ? robot->model_->getJointIndex(joint_name) : -1;

  if (index < 0)
  {
    fprintf(stderr, "JointPositionController could not find joint named \"%s\"\n", joint_name);
    return false;
  }
  joint_ = &robot->joint_states_[index];

 TiXmlElement *p = j->FirstChildElement("pid");
  if (!p)
    fprintf(stderr, "LaserScannerController's config did not specify the default pid parameters.\n");

  joint_velocity_controller_.initXml(robot,vel); //Pass down XML snippet to encapsulated joint_velocity_controller_

  TiXmlElement *pos = config->FirstChildElement("position");

  joint_position_controller_.initXml(robot,pos); //Pass down XML snippet to encapsulated joint_position_controller_

  last_position_ = joint_->position_;
  return true;
 }

// Set the joint velocity command
void LaserScannerController::setCommand(double command)
{
  command_ = command;
  if(current_mode_ == VELOCITY && automatic_turnaround_)
  {
    if(joint_->position_> upper_turnaround_location_)
    {
      if(command_ > 0) command_ = -command_; //Turn command around to avoid crash
      passed_center_ = false; //Allow passing through
    }
    else if (joint_->position_ < lower_turnaround_location_)
    {
      if(command_<0) command_ = -command_;
      passed_center_ = false;
    }
  }
}

// Return the current position command
double LaserScannerController::getCommand()
{
  return command_;
}

// Return the measured joint position
double LaserScannerController::getMeasuredPosition()
{
  return joint_->position_;
}

double LaserScannerController::getTime()
{
  return robot_->hw_->current_time_;
}

void LaserScannerController::update()
{
  double time = robot_->hw_->current_time_;
  double position = joint_->position_;
  double truncated_command, ratio;

  switch(current_mode_)
  {
    case VELOCITY:

     joint_velocity_controller_.setCommand(command_);

    if(automatic_turnaround_) //Automatically turn around
    {
       //Track if we passed center point
      if((position>=0 && last_position_<=0) || (position<=0 && last_position_>=0))  passed_center_ = true;

      if(passed_center_) //Check whether we're moving towards the edge
      {

        if(position >= upper_deceleration_zone_)
        {
          //Scale command for deceleration
          ratio = (1-(position-upper_deceleration_zone_)/(upper_turnaround_location_-upper_deceleration_zone_));
          truncated_command = command_* ratio ;
          truncated_command = max(MIN_MOVING_SPEED,truncated_command);
          joint_velocity_controller_.setCommand(truncated_command);
        }
        else if (position<=lower_deceleration_zone_)
        {
          ratio =  (1-(lower_deceleration_zone_ - position)/(lower_deceleration_zone_-lower_turnaround_location_));
          truncated_command = command_* ratio;
          truncated_command = min(-MIN_MOVING_SPEED,truncated_command);
          joint_velocity_controller_.setCommand(truncated_command);
        }

        if( position>= upper_turnaround_location_ ||
          position<=lower_turnaround_location_ )
        {
          command_ = -command_; //Reverse direction
          joint_velocity_controller_.setCommand(0);
          passed_center_ = false; //Make sure we don't get stuck on edges of workspace
        }
      }
      else
      {

         if(position >= upper_deceleration_zone_)
        {
          ratio = (1-(position-upper_deceleration_zone_)/(upper_turnaround_location_-upper_deceleration_zone_));
          truncated_command = command_* ratio;
          truncated_command = min(-MIN_MOVING_SPEED,truncated_command);
          joint_velocity_controller_.setCommand(truncated_command);
        }
        else if (position<=lower_deceleration_zone_)
        {
          ratio =  (1-(lower_deceleration_zone_ - position)/(lower_deceleration_zone_-lower_turnaround_location_));
          truncated_command = command_* ratio;
          truncated_command = max(MIN_MOVING_SPEED,truncated_command);
          joint_velocity_controller_.setCommand(truncated_command);
        }

      }
    }

    joint_velocity_controller_.update(); //Update lower controller
      break;
    case POSITION:
      joint_position_controller_.setCommand(command_);
      joint_position_controller_.update();
      break;
    default:
      break;
  }

    last_time_ = time; //Keep track of last time for update
    last_position_ = position;

}

void LaserScannerController::setJointEffort(double effort)
{
  joint_->commanded_effort_ = effort;
}

void LaserScannerController::setTurnaroundPoints(void)
{
  upper_turnaround_location_ = joint_->joint_->joint_limit_max_ - upper_turnaround_offset_;
  upper_deceleration_zone_ = upper_turnaround_location_-upper_deceleration_buffer_;

  lower_turnaround_location_ = joint_->joint_->joint_limit_min_ + lower_turnaround_offset_;
  lower_deceleration_zone_ = lower_turnaround_location_ + lower_deceleration_buffer_;
}

ROS_REGISTER_CONTROLLER(LaserScannerControllerNode)
LaserScannerControllerNode::LaserScannerControllerNode()
{
  c_ = new LaserScannerController();
}


LaserScannerControllerNode::~LaserScannerControllerNode()
{
  delete c_;
}

void LaserScannerControllerNode::update()
{
  c_->update();
}

// Return the measured joint position
double LaserScannerControllerNode::getMeasuredPosition()
{
  return c_->getMeasuredPosition();
}

bool LaserScannerControllerNode::setCommand(
  robot_mechanism_controllers::SetCommand::request &req,
  robot_mechanism_controllers::SetCommand::response &resp)
{

  c_->current_mode_ = LaserScannerController::VELOCITY;
  c_->setCommand(req.command);
  resp.command = c_->getCommand();

   return true;
}

bool LaserScannerControllerNode::getCommand(
  robot_mechanism_controllers::GetCommand::request &req,
  robot_mechanism_controllers::GetCommand::response &resp)
{
  resp.command = c_->getCommand();

  return true;
}

bool LaserScannerControllerNode::setPosition(
  robot_mechanism_controllers::SetPosition::request &req,
  robot_mechanism_controllers::SetPosition::response &resp)
{

  c_->current_mode_ = LaserScannerController::POSITION;
  c_->setCommand(req.position);
  resp.command = c_->getCommand();

   return true;
}

bool LaserScannerControllerNode::getPosition(
  robot_mechanism_controllers::GetPosition::request &req,
  robot_mechanism_controllers::GetPosition::response &resp)
{
  c_->current_mode_ =  LaserScannerController::POSITION;
  resp.command = c_->getCommand();

  return true;
}


bool LaserScannerControllerNode::setProfile(
  robot_mechanism_controllers::SetProfile::request &req,
  robot_mechanism_controllers::SetProfile::response &resp)
{
  setProfileCall(req.UpperTurnaround,req.LowerTurnaround, req.upperDecelBuffer, req.lowerDecelBuffer);
  resp.time = c_->getTime();
  return true;
}

bool LaserScannerControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  ros::node *node = ros::node::instance();
  string prefix = config->Attribute("name");

  if (!c_->initXml(robot, config))
    return false;
  node->advertise_service(prefix + "/set_command", &LaserScannerControllerNode::setCommand, this);
  node->advertise_service(prefix + "/get_command", &LaserScannerControllerNode::getCommand, this);
  node->advertise_service(prefix + "/set_profile", &LaserScannerControllerNode::setProfile, this);
  node->advertise_service(prefix + "/set_position", &LaserScannerControllerNode::setPosition, this);
  node->advertise_service(prefix + "/get_position", &LaserScannerControllerNode::getPosition, this);
  return true;
}

bool LaserScannerControllerNode::setProfileCall(double upper_turn_around, double lower_turn_around, double upper_decel_buffer, double lower_decel_buffer)
{
  c_->upper_turnaround_offset_ = upper_turn_around;
  c_->lower_turnaround_offset_ = lower_turn_around;
  c_->lower_deceleration_buffer_ = upper_decel_buffer;
  c_->upper_deceleration_buffer_ = lower_decel_buffer;
  c_->setTurnaroundPoints();

  if(upper_turn_around == 0 && lower_turn_around ==0) c_->automatic_turnaround_ = false;
  else c_->automatic_turnaround_ = true;
  c_->current_mode_ = LaserScannerController::VELOCITY;
  c_->setCommand(0.0);

  return true;
}

bool LaserScannerControllerNode::getActual(
  robot_mechanism_controllers::GetActual::request &req,
  robot_mechanism_controllers::GetActual::response &resp)
{
  resp.command = c_->getMeasuredPosition();
  resp.time = c_->getTime();
  return true;
}

void LaserScannerControllerNode::setCommand(double command)
{
  c_->setCommand(command);
}

// Return the current position command
double LaserScannerControllerNode::getCommand()
{
  return c_->getCommand();
}



