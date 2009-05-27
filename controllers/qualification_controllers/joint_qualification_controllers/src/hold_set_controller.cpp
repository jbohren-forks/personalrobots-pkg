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

/* Author: Kevin Watts */

#include <joint_qualification_controllers/hold_set_controller.h>

#define MAX_DATA_POINTS 10000

using namespace std;
using namespace controller;


ROS_REGISTER_CONTROLLER(HoldSetController)

HoldSetController::HoldSetController():
  joint_state_(NULL), robot_(NULL)
{
  dither_ = new control_toolbox::Dither::Dither(100.0);

  //hold_set_ = new vector<double>();

  hold_set_data_.test_name = "hold_set";
  hold_set_data_.joint_name = "default";

  state_ = STARTING;
  
  current_position_ = 0;
  starting_count_ = 0;
 
  initial_time_ = 0.0;
  start_time_ = 0.0;
  dither_time_ = 0.0;
  dither_count_  = 0.0;
  timeout_ = 60.0;

}

HoldSetController::~HoldSetController()
{

}

bool HoldSetController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  
  TiXmlElement *j = config->FirstChildElement("joint");
  if (!j)
  {
    fprintf(stderr, "HoldSetController was not given a joint\n");
    return false;
  }

  const char *joint_name = j->Attribute("name");
  joint_state_ = joint_name ? robot->getJointState(joint_name) : NULL;
  if (!joint_state_)
  {
    fprintf(stderr, "HoldSetController could not find joint named \"%s\"\n", joint_name);
    return false;
  }

  hold_set_data_.joint_name = joint_name;

  // Need a PID to pass into position controller...
  position_controller_ = new JointPositionController();
  position_controller_->initXml(robot, config);

  TiXmlElement *cd = config->FirstChildElement("controller_defaults");
  if (cd)
  {
    settle_time_ = atof(cd->Attribute("settle_time"));
    dither_time_ = atof(cd->Attribute("dither_time"));
    timeout_ = atof(cd->Attribute("timeout"));
    double dither_amp = atof(cd->Attribute("dither_amp"));
    dither_->init(dither_amp);
  }
  else
  {
    fprintf(stderr, "HoldSetController was not given required controller defaults\n");
    return false;
  }

  // Need to pass in a list of points here
  TiXmlElement *hs = config->FirstChildElement("hold_pt");
  while (hs)
  {
    double position;
    hs->Attribute("position", &position);

    hold_set_.push_back(position);

    hs = hs->NextSiblingElement("hold_pt");
  }
  
  if (!hs)
  {
    fprintf(stderr, "HoldSetController's config did not give the required holding set.\n");
    return false;
  }

  return true;
}

void HoldSetController::update()
{
  // wait until the joint is calibrated if it has limits
  if(!joint_state_->calibrated_)
  {
    return;
  }
  
  double time = robot_->hw_->current_time_;
  
  if (time - initial_time_ > timeout_ && state_ != DONE) 
  {
    state_ = DONE;
  }

  position_controller_->update();

  switch (state_)
  {
  case STARTING:
    {
      position_controller_->setCommand(hold_set_[current_position_]);
      start_time_ = time;
      state_ = SETTLING;
      break;
    }
  case SETTLING:
    {
      if (time - start_time_ > settle_time_)
      {
        state_ = DITHERING;
        start_time_ = time;
        dither_count_ = 0;
        hold_set_data_.hold_data[current_position_].desired = hold_set_[current_position_];
      }
      
      break;
    }
  case DITHERING:
    {
      // Need to add dither to PID
      joint_state_->commanded_effort_ += dither_->update();
      
      // Measure
      joint_qualification_controllers::HoldPositionData *data = &hold_set_data_.hold_data[current_position_];
      
      data->position.push_back(joint_state_->position_);
      data->velocity.push_back(joint_state_->velocity_);

      data->cmd.push_back(joint_state_->commanded_effort_);
      data->effort.push_back(joint_state_->applied_effort_);
      
      if (time - start_time_ > dither_time_)
      {
        // Go onto next point
        //data->position.resize(dither_count_);
        //data->cmd.resize(dither_count_);
        //data->effort.resize(dither_count_);
        //dither_count_ = 0;
        
        state_ = PAUSING;
      }
      break;
    }
  case PAUSING:
    {
    if (current_position_++ > hold_set_.size())
      state_ = DONE;
    else
      state_ = STARTING;

    break;
    }  
  case DONE:
    break;
  }


}



ROS_REGISTER_CONTROLLER(HoldSetControllerNode)
HoldSetControllerNode::HoldSetControllerNode()
: data_sent_(false), call_service_("hold_set_data")
{
  c_ = new HoldSetController();
}

HoldSetControllerNode::~HoldSetControllerNode()
{
  delete c_;
}

void HoldSetControllerNode::update()
{
  c_->update();
  if (c_->done())
  {
    if(!data_sent_)
    {
      if (call_service_.trylock())
      {
        joint_qualification_controllers::HoldSetData::Request *out = &call_service_.srv_req_;
        out->test_name = c_->hold_set_data_.test_name;
        out->joint_name = c_->hold_set_data_.joint_name;

        out->hold_data.resize(c_->hold_set_data_.hold_data.size());

        for (uint i = 0; i < c_->hold_set_data_.hold_data.size(); i++)
        {
          joint_qualification_controllers::HoldPositionData *data = &c_->hold_set_data_.hold_data[i];

          out->hold_data[i].position = data->position;
          out->hold_data[i].velocity = data->velocity;

          out->hold_data[i].cmd = data->cmd;
          out->hold_data[i].effort = data->effort;
        }
        call_service_.unlockAndCall();
        data_sent_ = true;
      }
    }
 
  }
}

bool HoldSetControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  robot_ = robot;
  
  if (!c_->initXml(robot, config))
    return false;
    
  return true;
}

