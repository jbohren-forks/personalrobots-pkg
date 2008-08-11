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
}

LaserScannerController::~LaserScannerController()
{
  //Free memory in profile if needed
  if(profile_locations_!=NULL) delete[] profile_locations_;
  if(profile_dt_!=NULL) delete[] profile_dt_;
 
}

void LaserScannerController::init(double p_gain, double i_gain, double d_gain, double windup, double time, mechanism::Joint *joint)
{
  joint_position_controller_.init( p_gain,  i_gain,  d_gain,  windup, time, joint);

  command_= 0;
  last_time_= time;
  joint_ = joint;
}

void LaserScannerController::initXml(mechanism::Robot *robot, TiXmlElement *config)
{
  robot_ = robot;
  TiXmlElement *elt = config->FirstChildElement("joint");
  if (elt) {
    // TODO: error check if xml attributes/elements are missing
    double p_gain = atof(elt->FirstChildElement("pGain")->GetText());
    double i_gain = atof(elt->FirstChildElement("iGain")->GetText());
    double d_gain = atof(elt->FirstChildElement("dGain")->GetText());
    double windup= atof(elt->FirstChildElement("windup")->GetText());
    init(p_gain, i_gain, d_gain, windup, robot->hw_->current_time_, robot->getJoint(elt->Attribute("name")));
  }
}

// Set the joint position command
void LaserScannerController::setCommand(double command)
{
  command_ = command;
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
   if(use_profile_)
  {
    joint_position_controller_.setCommand(profile_locations_[profile_index_]); //Issue position command

    //Check if enough time has elapsed to move to next set point
    if(time-cycle_start_time_ >= profile_dt_[profile_index_])
      {  
      cycle_start_time_ = time;
		  
       //Advance time index
  		 if(profile_index_ == (profile_length_-1))
        {
  			  profile_index_ = 0; //Restart profile
  			} else profile_index_++;
      }
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
    profile_locations_[i] = current*amplitude; //set current point
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
  cycle_start_time_ = robot_->hw_->current_time_;


  use_profile_ = true;

}
 
//Set mode to use Sinewave profile
void LaserScannerController::setSinewaveProfile(double period, double amplitude, int num_elements, double offset)
{
  int smaller_num_elements = num_elements/4; //Number of elements in a single quadrant
  int total_elements = smaller_num_elements*4; //track actual number of elements after int truncation
  double delta = amplitude/smaller_num_elements;
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
  }

   //At time 0, we wish for our location to be at 0. Start indexing at 1
  for(int i = 1;i<total_elements;i++)
  { 
    temp_value = asin(profile_locations_[i]); //Calculate time
    profile_dt_[i] = fabs(temp_value-last_temp_value)*period; //Calculate dt, scale by period
    profile_locations_[i] = temp_value*amplitude; //Scale goal location by amplitude
    last_temp_value = temp_value;    
  }

 //Reset profile settings
  profile_length_ = total_elements; //Keep track of profile length
  profile_index_= 0; //Start at beginning
  cycle_start_time_ = robot_->hw_->current_time_;

  use_profile_ = true;
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

