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

#include <pr2_controllers/laser_scanner_controller.h>
#include <math_utils/angles.h>

using namespace std;
using namespace controller;

ROS_REGISTER_CONTROLLER(LaserScannerController)

LaserScannerController::LaserScannerController()
{
  // Initialize PID class
  command_ = 0;

  use_profile_ = false;
  profile_index_ = 0;
  profile_length_ = 0;

  //Clear arrays
  profile_locations_ = NULL;
  profile_dt_ = NULL;
  current_profile_=NO_PROFILE;
}

LaserScannerController::~LaserScannerController()
{
  //Free memory in profile if needed
  if(profile_locations_!=NULL) delete[] profile_locations_;
  if(profile_dt_!=NULL) delete[] profile_dt_;
 
}

void LaserScannerController::init(double p_gain, double i_gain, double d_gain, double windup, double time, mechanism::Robot *robot, mechanism::Joint *joint)
{
  joint_position_controller_.init( p_gain,  i_gain,  d_gain,  windup, time, robot, joint);
  robot_ = robot;
  command_= 0;
  last_time_= time;
  joint_ = joint;
}

void LaserScannerController::initXml(mechanism::Robot *robot, TiXmlElement *config)
{
  

  TiXmlElement *elt = config->FirstChildElement("joint");
  if (elt) {
    // TODO: error check if xml attributes/elements are missing
    double p_gain = atof(elt->FirstChildElement("pGain")->GetText());
    double i_gain = atof(elt->FirstChildElement("iGain")->GetText());
    double d_gain = atof(elt->FirstChildElement("dGain")->GetText());
    double windup= atof(elt->FirstChildElement("windup")->GetText());
    init(p_gain, i_gain, d_gain, windup, robot->hw_->current_time_, robot, robot->getJoint(elt->Attribute("name")));
  }
}

// Set the joint position command
void LaserScannerController::setCommand(double command)
{
  command_ = command;
  use_profile_ = false; //Disable automatic profile mode
}

// Return the current position command
double LaserScannerController::getCommand()
{
  return command_;
}

// Return the measured joint position
double LaserScannerController::getActual()
{
  return joint_->position_;
}

void LaserScannerController::update()
{
  double time = robot_->hw_->current_time_;
   if(use_profile_ && current_profile_ == NO_PROFILE)//Use predefined location array
  {
  
//  if(use_profile_)
//  {
    joint_position_controller_.setCommand(profile_locations_[profile_index_]); //Issue position command

    //Check if enough time has elapsed to move to next set point
    if(time-time_of_last_point_ >= profile_dt_[profile_index_])
      {  
    #ifdef DEBUG          
    printf("DLL : %f %f\n",profile_locations_[profile_index_],time- time_of_last_point_);
    #endif
      time_of_last_point_ = time;
	   
       //Advance time index
  		 if(profile_index_ == (profile_length_-1))
        {
  			  profile_index_ = 0; //Restart profile
  			  cycle_start_time_ = time;
  			} else profile_index_++;
  			 
      }
  }
  
  else if(use_profile_ && current_profile_!=NO_PROFILE)
  {
    //Advance to next period
    if(time-cycle_start_time_>period_) cycle_start_time_ = time;    

    //Issue command based on time from start of cycle    
    if(current_profile_==SINEWAVE) setSinewave(time-cycle_start_time_);
    else if(current_profile_==SAWTOOTH) setSawtooth(time-cycle_start_time_);
  }
  
  else joint_position_controller_.setCommand(command_);

  joint_position_controller_.update(); //Update lower controller

  last_time_ = time; //Keep track of last time for update

}

void LaserScannerController::setJointEffort(double effort)
{
  joint_->commanded_effort_ = min(max(effort, -joint_->effort_limit_), joint_->effort_limit_);
}

//Set mode to use sawtooth profile
void LaserScannerController::setSawtoothProfile(double period, double amplitude, double offset)
{   
  period_ = period;
  amplitude_ = amplitude;
  offset_ = offset;
  
   //Reset profile settings
  profile_length_ = 0;
  profile_index_= 0; 
  cycle_start_time_ = robot_->hw_->current_time_;

  use_profile_ = true;
}


//Set mode to use sawtooth profile
void LaserScannerController::setSawtooth(double time_from_start)
{
  double time_from_peak = fmod(time_from_start,(period_*4));
  double command = (time_from_peak-cycle_start_time_)/period_*amplitude_;  
  if(time_from_start<period_/4.0) //Quadrant I
  {
    command = command + offset_;
  }
  else if (time_from_start>period_/4.0 && time_from_start<period_/2.0) //Quadrant II
  {
    command = amplitude_-command + offset_;
  }
  else if (time_from_start>period_/2.0 && time_from_start<period_*3/4.0) //Quadrant III
  {
    command = -command + offset_;
  }
  else if (time_from_start>period_*3/4.0 && time_from_start<period_) //Quadrant IV
  {
    command = -amplitude_ + command + offset_;
  }
  else if (time_from_start>period_) //Reset
  {  
    command = command + offset_;
  }
  
  joint_position_controller_.setCommand(profile_locations_[profile_index_]);
}


//Set mode to use Sinewave profile
void LaserScannerController::setSinewaveProfile(double period, double amplitude,double offset)
{
   //Reset profile settings
  profile_length_ = 0;
  profile_index_= 0; 
  cycle_start_time_ = robot_->hw_->current_time_;
  current_profile_ = SINEWAVE;//Indicate profile

  use_profile_ = true;
  
  period_ = period;
  amplitude_ = amplitude;
  offset_ = offset;
}

//Get sinewave based on current time
void LaserScannerController::setSinewave(double time_from_start)
{
  double command;
  
  joint_position_controller_.setCommand(sin(2*M_PI*time_from_start/period_)*amplitude_+offset_);
  
}

//Set mode to use sawtooth profile
void LaserScannerController::setSawtoothProfile(double period, double amplitude, int num_elements, double offset)
{
  int smaller_num_elements = num_elements/4; //Number of elements in a single quadrant
  int total_elements = smaller_num_elements*4; //track actual number of elements after int truncation
  double delta = amplitude/smaller_num_elements; //Scale first, then determine step size
  double dt = period/total_elements;
  double current = 0;
  double newvalue = 0;

  //Error checking
  if(total_elements<=0) return;  

  //Clear arrays
  if(profile_locations_ !=NULL) delete[] profile_locations_;
  if(profile_dt_ !=NULL) delete[] profile_dt_;

  profile_locations_ = new double[total_elements];
  profile_dt_ = new double[total_elements];

  //Construct evenly spaced elements in distance along sine wave
  for(int i = 0;i<total_elements;i++)
  {
    profile_locations_[i] = current + offset; //set current point
    profile_dt_[i] = dt; //Constant dt because of linear relationship

    newvalue = current + delta; //Calculate next value
    if(i == smaller_num_elements) //Shift from quadrant 1 to 2
    {
      delta = -delta;      
      newvalue = current + delta;
    }
    else if (i == smaller_num_elements*3)//Shift from quadrant 3-4
    {
      delta = -delta;       
      newvalue = current + delta;
    }
    current = newvalue;
  }

 //Reset profile settings
  profile_length_ = total_elements; //Keep track of profile length
  profile_index_= 0; //Start at beginning
  time_of_last_point_ = robot_->hw_->current_time_;

  current_profile_ = NO_PROFILE; //Disable dynamic profile
  use_profile_ = true;

}
 
//Set mode to use Sinewave profile
void LaserScannerController::setSinewaveProfile(double period, double amplitude, int num_elements, double offset)
{
  int smaller_num_elements = num_elements/4; //Number of elements in a single quadrant
  int total_elements = smaller_num_elements*4; //track actual number of elements after int truncation
  double delta = 1.0/smaller_num_elements;
  double current = 0;
  double newvalue = 0;
  double temp_value = 0.0;
  double last_temp_value = 0.0;

  //Error checking
  if(total_elements<=0) return; 
 
  //Clear arrays
  if(profile_locations_ !=NULL) delete[] profile_locations_;
  if(profile_dt_ !=NULL) delete[] profile_dt_;

  profile_locations_ = new double[total_elements];
  profile_dt_ = new double[total_elements];
  
  //Construct evenly spaced elements in distance along sine wave
  for(int i = 0;i<total_elements;i++)
  {
    profile_locations_[i] = current; //set current point
    newvalue = current + delta; //Calculate next value
    if(i == smaller_num_elements) //Shift from quadrant 1 to 2
    {
      delta = -delta;      
      newvalue = current + delta;
    }
    else if (i == smaller_num_elements*3)//Shift from quadrant 3-4
    {
      delta = -delta;       
      newvalue = current + delta;
    }
    
    current = newvalue;   
    current = min(max(current, -1.0), 1.0); //Make sure asin doesn't fail
  }        
  
  profile_locations_[0] = offset; //set first value

   //At time 0, we wish for our location to be at offset. Start indexing at 1, but associate dt with previous value
  for(int i = 1;i<total_elements;i++)
  { 
    temp_value = asin(profile_locations_[i]); //Calculate time 
    profile_dt_[i-1] = fabs(temp_value-last_temp_value)*period/(2*M_PI); //Calculate dt, scale by period
//    if(profile_dt_[i-1]>profile_dt_[0]*3) profile_dt_[i-1]=profile_dt_[i-2];//smoothing
    profile_locations_[i] = temp_value*amplitude + offset; //Scale goal location by amplitude
    last_temp_value = temp_value;
    #ifdef DEBUG           
    printf("*** test %u %f %f\n",i,profile_dt_[i-1],profile_locations_[i]);
    #endif
  }

  profile_dt_[total_elements-1] = profile_dt_[0]; //Make symmetric

  #ifdef DEBUG
  for(int i = 0;i<total_elements;i++)
  {
    printf("*** test %u %f %f\n",i,profile_dt_[i],profile_locations_[i]);
  }
  #endif
 //Reset profile settings
  profile_length_ = total_elements; //Keep track of profile length
  profile_index_= 0; //Start at beginning
  time_of_last_point_ = robot_->hw_->current_time_;
  cycle_start_time_ = robot_->hw_->current_time_;

  use_profile_ = true;
  current_profile_ = NO_PROFILE; //Disable dynamic profile
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

bool LaserScannerControllerNode::setCommand(
  generic_controllers::SetCommand::request &req,
  generic_controllers::SetCommand::response &resp)
{
  c_->setCommand(req.command);
  resp.command = c_->getCommand();
  
  //FIXME: Backdoor method to issue command set
  if(req.command==41)c_->setSawtoothProfile(1,0.5,100,0);
  else if(req.command==42)c_->setSawtoothProfile(2,0.5,100,0);
  else if(req.command==43)c_->setSawtoothProfile(2,0.5,100,0.5);
  else if(req.command==44)c_->setSawtoothProfile(0.5,0.5,100,0);
  else if(req.command==45)c_->setSawtoothProfile(1,0.5,0);
  else if(req.command==-41)c_->setSinewaveProfile(2,0.5,100,0.5);
  else if(req.command==-42)c_->setSinewaveProfile(2,0.5,100,0);
  else if(req.command==-43)c_->setSinewaveProfile(1,0.5,100,0);
  else if(req.command==-44)c_->setSinewaveProfile(4,0.5,100,0);
  else if (req.command==-45)c_->setSinewaveProfile(1,0.5,0);
  return true;
}

bool LaserScannerControllerNode::getCommand(
  generic_controllers::GetCommand::request &req,
  generic_controllers::GetCommand::response &resp)
{
  resp.command = c_->getCommand();

  return true;
}

void LaserScannerControllerNode::initXml(mechanism::Robot *robot, TiXmlElement *config)
{
  ros::node *node = ros::node::instance();
  string prefix = config->Attribute("name");
  
  c_->initXml(robot, config);
  node->advertise_service(prefix + "/set_command", &LaserScannerControllerNode::setCommand, this);
  node->advertise_service(prefix + "/get_command", &LaserScannerControllerNode::getCommand, this);
}

