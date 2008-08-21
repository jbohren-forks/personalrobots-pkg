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

#include <generic_controllers/joint_autotuner.h>
#include <math_utils/angles.h>
using namespace std;
using namespace controller;
#define DEBUG 1
ROS_REGISTER_CONTROLLER(JointAutotuner)

JointAutotuner::JointAutotuner()
{
  robot_ = NULL;
  joint_ = NULL;

  // Initialize PID class
  pid_controller_.initPid(0, 0, 0, 0, 0);
  command_ = 0;
  last_time_ = 0;
}

JointAutotuner::~JointAutotuner()
{
}

void JointAutotuner::init(double p_gain, double i_gain, double d_gain, double windup, double time,mechanism::Robot *robot, mechanism::Joint *joint)
{
  pid_controller_.initPid(p_gain, i_gain, d_gain, windup, -windup);
  
  robot_ = robot;
  command_= 0;
  last_time_= time;
  joint_ = joint;

  relay_height_ = RELAYFRACTION * joint_->effort_limit_;
  #ifdef DEBUG
    printf("AUTO : Relay:%f\n",relay_height_);
  #endif
  current_state_ = POSITIVE_PEAK;
}

bool JointAutotuner::initXml(mechanism::Robot *robot, TiXmlElement *config)
{
  assert(robot);
  robot_ = robot;
  last_time_ = robot->hw_->current_time_;
  cycle_start_time_ = getTime();
   current_state_ = POSITIVE_PEAK;
  TiXmlElement *j = config->FirstChildElement("joint");
  if (!j)
  {
    fprintf(stderr, "JointAutotuner was not given a joint\n");
    return false;
  }

  const char *joint_name = j->Attribute("name");
  joint_ = joint_name ? robot->getJoint(joint_name) : NULL;
  if (!joint_)
  {
    fprintf(stderr, "JointAutotuner could not find joint named \"%s\"\n", joint_name);
    return false;
  }

  relay_height_ = RELAYFRACTION * joint_->effort_limit_;
  #ifdef DEBUG
    printf("AUTO : Relay:%f\n",relay_height_);
  #endif

  return true;
}

// Set the joint position command
void JointAutotuner::setCommand(double command)
{
  command_ = command;
}

// Return the current position command
double JointAutotuner::getCommand()
{
  return joint_->commanded_effort_;
}

// Return the measured joint position
double JointAutotuner::getMeasuredState()
{
  return joint_->position_;
}

double JointAutotuner::getTime()
{
  return robot_->hw_->current_time_;
}

void JointAutotuner::update()
{
  double position = joint_->position_;
  double time = getTime();
  //Scan for transitions
  if(current_state_==POSITIVE_PEAK && position<0) //Transition negative
  {
    current_state_=NEGATIVE_PEAK;
  }
  else if (current_state_==NEGATIVE_PEAK && position>0) //Transition to next period
  {
    current_state_ = POSITIVE_PEAK;
    period_ = time-cycle_start_time_;
    amplitude_ = (positive_peak_-negative_peak_)/2; //Record amplitude
    
    #ifdef DEBUG
      printf("AUTO period: %f amplitude: %f\n", period_, amplitude_);
    #endif
    
    if( (fabs(amplitude_-last_amplitude_) < AMPLITUDETOLERANCE*last_amplitude_) &&(fabs(period_-last_period_)<PERIODTOLERANCE*last_period_))//If the two peaks match closely
    {     
        successful_cycles_++; //increment successful cycles
        if(successful_cycles_>=NUMCYCLES) 
        { 
            #ifdef DEBUG
              printf("AUTO : DONE! Period: %f Amplitude: %f\n", period_, amplitude_);
            #endif
          current_state_ = DONE; //Done testing
        }
     }
    else successful_cycles_ = 0; //Otherwise, reset if we're varying too much
   
    //Reset for next period
    positive_peak_ = 0.0;
    negative_peak_ = 0.0;
    cycle_start_time_ = time;
    last_period_ = period_;
    last_amplitude_ = amplitude_;
  }

  //Update amplitude measures
  if(current_state_ == POSITIVE_PEAK)
  {
    if(position > positive_peak_) positive_peak_ = position;
    //If looking for positive peak, set positive h
    joint_->commanded_effort_ = -relay_height_;
  }
  else if(current_state_ == NEGATIVE_PEAK)
  {    
    if(position<negative_peak_) negative_peak_ = position;
    //If looking for negative peak, set negative h
    joint_->commanded_effort_ = relay_height_;
  }

}

ROS_REGISTER_CONTROLLER(JointAutotunerNode)
JointAutotunerNode::JointAutotunerNode() 
{
  c_ = new JointAutotuner();
}

JointAutotunerNode::~JointAutotunerNode()
{
  delete c_;
}

void JointAutotunerNode::update()
{
  c_->update();
}

bool JointAutotunerNode::setCommand(
  generic_controllers::SetCommand::request &req,
  generic_controllers::SetCommand::response &resp)
{
  c_->setCommand(req.command);
  resp.command = c_->getCommand();

  return true;
}

bool JointAutotunerNode::getActual(
  generic_controllers::GetActual::request &req,
  generic_controllers::GetActual::response &resp)
{
  resp.command = c_->getMeasuredState();
  resp.time = c_->getTime();
  return true;
}

bool JointAutotunerNode::initXml(mechanism::Robot *robot, TiXmlElement *config)
{
ros::node *node = ros::node::instance();
  string prefix = config->Attribute("name");

  std::string topic = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic == "")
  {
    fprintf(stderr, "No topic given to JointAutotunerNode\n");
    return false;
  }

  if (!c_->initXml(robot, config))
    return false;
  node->advertise_service(prefix + "/set_command", &JointAutotunerNode::setCommand, this);
  node->advertise_service(prefix + "/get_actual", &JointAutotunerNode::getActual, this);
  return true;


 }

