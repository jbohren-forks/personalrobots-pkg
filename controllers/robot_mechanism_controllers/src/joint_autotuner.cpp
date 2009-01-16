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

#include <robot_mechanism_controllers/joint_autotuner.h>
#include <fstream>
using namespace std;
using namespace controller;
#define POSITION 1
ROS_REGISTER_CONTROLLER(JointAutotuner)

JointAutotuner::JointAutotuner()
{
  robot_ = NULL;
  joint_ = NULL;
  robot_state_ = NULL;
  joint_state_ = NULL;
  std::cout<<"Created autotuner\n";
  // Initialize PID class
  pid_controller_.initPid(0, 0, 0, 0, 0);
  command_ = 0;
  last_time_ = 0;
}

JointAutotuner::~JointAutotuner()
{
  std::cout<<"Destroyed autotuner\n";
}

void JointAutotuner::init(double p_gain, double i_gain, double d_gain, double windup, double time,mechanism::Robot *robot, mechanism::Joint *joint)
{
  pid_controller_.initPid(p_gain, i_gain, d_gain, windup, -windup);

  robot_ = robot;
  command_= 0;
  last_time_= time;
  joint_ = joint;

  relay_height_ = relay_effort_percent_* joint_->effort_limit_;
   current_state_ = POSITIVE_PEAK;
}

bool JointAutotuner::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  robot_ = robot->model_;
  robot_state_ = robot;
  last_time_ = robot->hw_->current_time_;
  cycle_start_time_ = getTime();
   TiXmlElement *j = config->FirstChildElement("joint");
  if (!j)
  {
    fprintf(stderr, "JointAutotuner was not given a joint\n");
    return false;
  }

  const char *joint_name = j->Attribute("name");
  joint_ = joint_name ? robot_->getJoint(joint_name) : NULL;
  if (!joint_)
  {
    fprintf(stderr, "JointAutotuner could not find joint named \"%s\"\n", joint_name);
    return false;
  }
  joint_state_ = robot->getJointState(joint_name);
  if (!joint_state_)
  {
    fprintf(stderr, "JointAutotuner could not find joint state named \"%s\"\n", joint_name);
    return false;
  }



  TiXmlElement *f = config->FirstChildElement("autotune");
  file_path_ =  f->Attribute("file_path");
  if (!file_path_)
  {
    fprintf(stderr, "JointAutotuner could not find filepath \n");
    return false;
  }

  if(!f->Attribute("mode"))
  {
    std::cout<<"No mode attribute"<<std::endl;
//     return false;
  }
  tune_velocity_ = (f->Attribute("mode") && std::string(f->Attribute("mode"))==std::string("velocity"));
  num_cycles_ = atoi(f->FirstChildElement("cycles")->GetText());
  amplitude_tolerance_ = atof(f->FirstChildElement("amplitudeTolerance")->GetText());
  period_tolerance_ = atof(f->FirstChildElement("periodTolerance")->GetText());
  relay_effort_percent_ = atof(f->FirstChildElement("relayEffortPercent")->GetText());
  relay_height_ = relay_effort_percent_ * joint_->effort_limit_;
  #ifdef DEBUG
    printf("AUTO : Relay:%f\n",relay_height_);
  #endif

  current_state_ = START;

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
  return joint_state_->commanded_effort_;
}

// Return the measured joint position
double JointAutotuner::getMeasuredState()
{
  return joint_state_->position_;
}

double JointAutotuner::getTime()
{
  return robot_state_->hw_->current_time_;
}

void JointAutotuner::update()
{
  if(tune_velocity_)
  {
    double velocity = joint_state_->velocity_;
    double time = getTime();
    if(current_state_==START)//Read crossing point
      {
        current_state_ = NEGATIVE_PEAK;
      }

    //Scan for transitions
    if(current_state_==POSITIVE_PEAK && velocity<0) //Transition negative
    {
      current_state_=NEGATIVE_PEAK;
    }
    else if (current_state_==NEGATIVE_PEAK && velocity>0) //Transition to next period
    {
      current_state_ = POSITIVE_PEAK;
      period_ = time-cycle_start_time_;
      amplitude_ = (positive_peak_-negative_peak_)/2; //Record amplitude


      if( (fabs(amplitude_-last_amplitude_) < amplitude_tolerance_*last_amplitude_) &&(fabs(period_-last_period_)<period_tolerance_*last_period_))//If the two peaks match closely
      {
          successful_cycles_++; //increment successful cycles
          if(successful_cycles_>=num_cycles_)
          {
            current_state_ = DONE; //Done testing
            joint_state_->commanded_effort_ = 0;
              writeGainValues(period_,amplitude_,relay_height_);
              #ifdef DEBUG
                printf("AUTO : DONE! Period: %f Amplitude: %f\n", period_, amplitude_);
              #endif
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
      if(velocity > positive_peak_) positive_peak_ = velocity;
      //If looking for positive peak, set positive h
      joint_state_->commanded_effort_ = -relay_height_;
    }
    else if(current_state_ == NEGATIVE_PEAK)
    {
      if(velocity<negative_peak_) negative_peak_ = velocity;
      //If looking for negative peak, set negative h
      joint_state_->commanded_effort_ = relay_height_;
    }
  }
  else
  {
      double position = joint_state_->position_;
      double time = getTime();
      if(current_state_==START)//Read crossing point
      {
        crossing_point_ = joint_state_->position_;
        current_state_ = NEGATIVE_PEAK;
      }
      //Scan for transitions
       if(current_state_==POSITIVE_PEAK && position<crossing_point_) //Transition negative
      {
       current_state_=NEGATIVE_PEAK;
      }
      else if (current_state_==NEGATIVE_PEAK && position>crossing_point_) //Transition to next period
      {
      current_state_ = POSITIVE_PEAK;
      period_ = time-cycle_start_time_;
      amplitude_ = (positive_peak_-negative_peak_)/2; //Record amplitude

      if( (fabs(amplitude_-last_amplitude_) < amplitude_tolerance_*last_amplitude_) &&(fabs(period_-last_period_)<period_tolerance_*last_period_))//If the two peaks match closely
      {
          successful_cycles_++; //increment successful cycles
          if(successful_cycles_>=num_cycles_)
          {
              joint_state_->commanded_effort_ = 0;
              writeGainValues(period_,amplitude_,relay_height_);
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
      joint_state_->commanded_effort_ = -relay_height_;
    }
    else if(current_state_ == NEGATIVE_PEAK)
    {
      if(position<negative_peak_) negative_peak_ = position;
      //If looking for negative peak, set negative h
      joint_state_->commanded_effort_ = relay_height_;
    }
  }

}

void JointAutotuner::writeGainValues(double period, double amplitude, double relay_height)
{
  double ku, pu, kc, ti, td, Kp, Ki, Kd;
  //Calculate gain values
  ku = 4*relay_height/M_PI/amplitude;
  pu = period;
  kc = ku/2.2;
  ti = pu/0.45;
  td = pu/6.3;

  Kp = kc;
  Ki = Kp/ti;
  Kd = Kp*td;

//   std::ofstream datafile;
  std::ostream & datafile = std::cout;
//   datafile.open (file_path_);
  datafile<<"Autotuning result for joint: "<<joint_->name_<<"\n";
  if(tune_velocity_) datafile<<"Velocity Mode\n";
  else datafile<<"Position Mode\n";


  datafile<<"Kp:"<<Kp<<"\n";
  datafile<<"Ki:"<<Ki<<"\n";
  datafile<<"Kd:"<<Kd<<"\n";

  datafile<<"Ultimate Period:"<<period<<"\n";
  datafile<<"Ultimate Amplitude:"<<amplitude<<"\n";
  datafile<<"Relay Height:"<<relay_height<<"\n";
//   datafile.close();

}

ROS_REGISTER_CONTROLLER(JointAutotunerNode)
JointAutotunerNode::JointAutotunerNode()
{
  std::cout<<"Created autotuner\n";
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
  robot_mechanism_controllers::SetCommand::request &req,
  robot_mechanism_controllers::SetCommand::response &resp)
{
  c_->setCommand(req.command);
  resp.command = c_->getCommand();

  return true;
}

bool JointAutotunerNode::getActual(
  robot_mechanism_controllers::GetActual::request &req,
  robot_mechanism_controllers::GetActual::response &resp)
{
  resp.command = c_->getMeasuredState();
  resp.time = c_->getTime();
  return true;
}

bool JointAutotunerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
ros::Node *node = ros::Node::instance();
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

