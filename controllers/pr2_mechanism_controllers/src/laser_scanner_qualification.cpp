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

/*********************************************************************
  Qualification test for Hokuyo tilting stage

 Drives the hokuyo through three types of motion
  1. Timed effort exerted in positive direction
  2. Sine sweep
  3. Velocity controlled move to endstops
 *********************************************************************/

#include <algorithm>

#include <pr2_mechanism_controllers/laser_scanner_qualification.h>

using namespace std;
using namespace controller;
//#define DEBUG 1
ROS_REGISTER_CONTROLLER(LaserScannerQualification)

LaserScannerQualification::LaserScannerQualification()
{
  robot_ = NULL;
  joint_ = NULL;

  command_ = 0;
  last_time_ = 0;

  current_mode_ = START;
  effort_test_status_ = INCOMPLETE;
  sinesweep_test_status_ = INCOMPLETE;
  endstop_test_status_ = INCOMPLETE;
  found_positive_endstop_ = false;
}

LaserScannerQualification::~LaserScannerQualification()
{

}

void LaserScannerQualification::init(double p_gain, double i_gain, double d_gain, double windup, double time, std::string name, mechanism::Robot *robot)
{
  robot_ = robot;
  joint_ = robot->getJoint(name);

  joint_velocity_controller_.init( p_gain,  i_gain,  d_gain,  windup, time, name, robot);
  command_= 0;
  last_time_= time;
}

bool LaserScannerQualification::initXml(mechanism::Robot *robot, TiXmlElement *config)
{
  assert(robot);
  robot_ = robot;
  last_time_ = robot->hw_->current_time_;

  //Perform checks at highest level to give the most informative error message possible
  TiXmlElement *j = config->FirstChildElement("joint");
  if (!j)
  {
    fprintf(stderr, "LaserScannerQualification was not given a joint\n");
    return false;
  }

  effort_test_percent_  = atof(j->FirstChildElement("effort_test")->Attribute("effort_percent"));
  effort_test_length_  = atof(j->FirstChildElement("effort_test")->Attribute("effort_length"));

  sinesweep_start_freq_ = atof(j->FirstChildElement("sinesweep_test")->Attribute("start_freq"));
  sinesweep_end_freq_ = atof(j->FirstChildElement("sinesweep_test")->Attribute("end_freq"));
  sinesweep_amplitude_ = atof(j->FirstChildElement("sinesweep_test")->Attribute("amplitude"));
  sinesweep_duration_ = atof(j->FirstChildElement("sinesweep_test")->Attribute("duration"));

  endstop_velocity_  = atof(j->FirstChildElement("endstop_test")->Attribute("velocity"));
  endstop_stopped_length_ = atof(j->FirstChildElement("endstop_test")->Attribute("velocity_stopped_length"));
  endstop_stopped_velocity_ = atof(j->FirstChildElement("endstop_test")->Attribute("stopped_velocity"));

  double start_angular_freq =2*M_PI*sinesweep_start_freq_;
  double end_angular_freq =2*M_PI*sinesweep_end_freq_;
  sinesweep_K_factor_ = (start_angular_freq*sinesweep_duration_)/log(end_angular_freq/start_angular_freq);
  sinesweep_L_factor_ = (sinesweep_duration_)/log(end_angular_freq/start_angular_freq);


  const char *joint_name = j->Attribute("name");
  joint_ = joint_name ? robot->getJoint(joint_name) : NULL;
  if (!joint_)
  {
    fprintf(stderr, "LaserScannerQualification could not find joint named \"%s\"\n", joint_name);
    return false;
  }
 TiXmlElement *p = j->FirstChildElement("pid");
  if (!p)
    fprintf(stderr, "LaserScannerQualification's config did not specify the default pid parameters.\n");
  joint_velocity_controller_.initXml(robot,config); //Pass down XML snippet to encapsulated joint_velocity_controller_

  return true;


 }

// Return the measured joint position
double LaserScannerQualification::getMeasuredPosition()
{
  return joint_->position_;
}

double LaserScannerQualification::getTime()
{
  return robot_->hw_->current_time_;
}

void LaserScannerQualification::update()
{
  double time = robot_->hw_->current_time_;

  switch(current_mode_)
  {
    case START:
      //Transition to effort test;
      effort_test_start_ = time; //Mark current time and apply effort
      current_mode_ = EFFORTTEST;
      joint_->commanded_effort_ = effort_test_percent_*joint_->effort_limit_;
      break;
    case EFFORTTEST:
      joint_->commanded_effort_ = effort_test_percent_*joint_->effort_limit_;
      if(time-effort_test_start_ >effort_test_length_) //Check if test is finished
      {
        effort_test_status_ = COMPLETE;
        sinesweep_test_status_ = INPROGRESS;
        current_mode_ = SINESWEEPTEST;
        sinesweep_test_start_ = time;
        #ifdef DEBUG
          std::cout<<" DEBUG Finished effort test\n";
        #endif
      }
      break;
    case SINESWEEPTEST:
       if((time-sinesweep_test_start_)<sinesweep_duration_)
      {
        double command =  sinesweep_amplitude_*sin(sinesweep_K_factor_*(exp((time-sinesweep_test_start_)/(sinesweep_L_factor_))-1));
        joint_->commanded_effort_= command;
      }
      else
      {
        #ifdef DEBUG
          std::cout<<" DEBUG Finished sinesweep test\n";
        #endif
        sinesweep_test_status_ = COMPLETE;
        endstop_test_status_ = INPROGRESS;
        current_mode_ = ENDSTOPTEST;
      }
      break;
    case ENDSTOPTEST:
        if(!found_positive_endstop_) //We just started moving
        {
          joint_velocity_controller_.setCommand(endstop_velocity_);
          joint_velocity_controller_.update();

          //If we're close to stopped, mark current time
          if(joint_->velocity_>endstop_stopped_velocity_) endstop_stopped_time_ = time;
          //We've stopped long enough to register the endstop
          if((time-endstop_stopped_time_)>endstop_stopped_length_)
          {
            #ifdef DEBUG
             std::cout<<" DEBUG :TURNING AROUND\n";
            #endif
            found_positive_endstop_ = true; //Move the opposite direction
            endstop_stopped_time_ = time; //Reset timer
          }
         }

        else //We've already hit the positive endstop
        {
          //Move in negative direction
          joint_velocity_controller_.setCommand(-endstop_velocity_);
          joint_velocity_controller_.update();

          #ifdef DEBUG
            std::cout<<" DEBUG Commanded effort negative:"<<joint_->commanded_effort_<<"\n";
          #endif

          //If we're close to stopped, mark current time
          if(fabs(joint_->velocity_)>endstop_stopped_velocity_) endstop_stopped_time_ = time;
          //We've stopped long enough to register negative endstop
          if((time-endstop_stopped_time_)>endstop_stopped_length_)
          {
              current_mode_ = DONE;
              endstop_test_status_ = COMPLETE;
          #ifdef DEBUG
              std::cout<<" DEBUG Effort Test:";
              printStatus(effort_test_status_);
              std::cout<<" DEBUG Sinesweep Test:";
              printStatus(sinesweep_test_status_);
              std::cout<<" DEBUG Endstop Test:";
              printStatus(endstop_test_status_);
          #endif
          }
        }
        break;
    case DONE:
        joint_->commanded_effort_ = 0.0;
    default:
      break;
  }

  last_time_ = time; //Keep track of last time for update

}

void LaserScannerQualification::setJointEffort(double effort)
{
  joint_->commanded_effort_ = min(max(effort, -joint_->effort_limit_), joint_->effort_limit_);
}

void LaserScannerQualification::printStatus(LaserScannerQualification::TestStatus status)
{
  switch(status)
  {
    case INCOMPLETE:
      std::cout<<"Incomplete";
      break;
    case INPROGRESS:
      std::cout<<"In progress";
      break;
    case COMPLETE:
      std::cout<<"Complete";
      break;
    case FAILED:
      std::cout<<"Failed";
      break;
    case PASSED:
      std::cout<<"Passed";
      break;
  }

  std::cout<<"\n";
}

ROS_REGISTER_CONTROLLER(LaserScannerQualificationNode)
LaserScannerQualificationNode::LaserScannerQualificationNode()
{
  c_ = new LaserScannerQualification();
}


LaserScannerQualificationNode::~LaserScannerQualificationNode()
{
  delete c_;
}

void LaserScannerQualificationNode::update()
{
  c_->update();
}

// Return the measured joint position
double LaserScannerQualificationNode::getMeasuredPosition()
{
  return c_->getMeasuredPosition();
}

bool LaserScannerQualificationNode::setCommand(
  robot_mechanism_controllers::SetCommand::request &req,
  robot_mechanism_controllers::SetCommand::response &resp)
{
  return true;
}

bool LaserScannerQualificationNode::getCommand(
  robot_mechanism_controllers::GetCommand::request &req,
  robot_mechanism_controllers::GetCommand::response &resp)
{
  return true;
}

bool LaserScannerQualificationNode::initXml(mechanism::Robot *robot, TiXmlElement *config)
{
  ros::node *node = ros::node::instance();
  string prefix = config->Attribute("name");

  if (!c_->initXml(robot, config))
    return false;
  node->advertise_service(prefix + "/set_command", &LaserScannerQualificationNode::setCommand, this);
  node->advertise_service(prefix + "/get_command", &LaserScannerQualificationNode::getCommand, this);
  return true;
}
bool LaserScannerQualificationNode::getActual(
  robot_mechanism_controllers::GetActual::request &req,
  robot_mechanism_controllers::GetActual::response &resp)
{
  resp.command = c_->getMeasuredPosition();
  resp.time = c_->getTime();
  return true;
}



