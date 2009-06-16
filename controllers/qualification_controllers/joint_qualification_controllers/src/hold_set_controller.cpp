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

using namespace std;
using namespace controller;


ROS_REGISTER_CONTROLLER(HoldSetController)

HoldSetController::HoldSetController():
  robot_(NULL)
{
  
  hold_set_data_.test_name = "hold_set";
  
  state_ = STARTING;
  
  current_position_ = 0;
  starting_count_ = 0;
 
  initial_time_ = 0.0;
  start_time_ = 0.0;
  dither_time_ = 0.0;
  dither_count_  = 0.0;
  timeout_ = 120.0;

}

HoldSetController::~HoldSetController()
{
}

bool HoldSetController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  robot_ = robot;
 
  initial_time_ = robot_->hw_->current_time_;
  
  // Adding Joint Position Controllers
  TiXmlElement *elt = config->FirstChildElement("controller");
  if (!elt)
  {
    fprintf(stderr, "HoldSetController's config did not give the required joint position controllers.\n");
    return false;
  }

  ROS_INFO("Setting up joint position controllers!");
  while (elt)
  {
    ROS_INFO("Making JPC");
    JointPositionController * jpc = new JointPositionController();
    //std::cout<<elt->Attribute("type")<<elt->Attribute("name")<<std::endl;
    assert(static_cast<std::string>(elt->Attribute("type")) == std::string("JointPositionController"));
    //ROS_INFO("Passed assert");

    ROS_INFO("Pushing back JPC, joint states");

    // Store controller, joint state, name
    joint_position_controllers_.push_back(jpc);

    if (!jpc->initXml(robot, elt))
      return false;

    hold_set_data_.joint_names.push_back(jpc->getJointName());
    joint_states_.push_back(robot->getJointState(jpc->getJointName()));

    TiXmlElement *dith = elt->FirstChildElement("dither");
    double dither_amp = atof(dith->Attribute("dither_amp"));
    control_toolbox::Dither *dither = new control_toolbox::Dither::Dither(100.0);
    dither->init(dither_amp);
    dithers_.push_back(dither);

    hold_set_data_.dither_amps.push_back(dither_amp);

    ROS_INFO("Next controller!");
    elt = elt->NextSiblingElement("controller");
  }

  num_joints_ = joint_position_controllers_.size();
    
  hold_set_data_.joint_names.resize(num_joints_);
  hold_set_data_.dither_amps.resize(num_joints_);
  
  // Setting controller defaults
  TiXmlElement *cd = config->FirstChildElement("controller_defaults");
  if (cd)
  {
    settle_time_ = atof(cd->Attribute("settle_time"));
    dither_time_ = atof(cd->Attribute("dither_time"));
    timeout_ = atof(cd->Attribute("timeout"));
   }
  else
  {
    fprintf(stderr, "HoldSetController was not given required controller defaults\n");
    return false;
  }

  // Setting holding points
  // Need to pass in a list of points here
  // Store as vector of vectors...
  TiXmlElement *hs = config->FirstChildElement("hold_pt");
  if (!hs)
  {
    fprintf(stderr, "HoldSetController's config did not give the required holding set.\n");
    return false;
  }

  ROS_INFO("Storing Hold Points");

  while (hs)
  {
    std::vector<double> position;
    
    TiXmlElement *jnt_pt = hs->FirstChildElement("joint");
    
    while (jnt_pt)
    {
      
      double point = atof(jnt_pt->Attribute("position"));

      position.push_back(point);

      jnt_pt = jnt_pt->NextSiblingElement("joint");
    }

    if (position.size() != num_joints_)
    {
      ROS_ERROR("Incorrect number of points for joint");
      fprintf(stderr, "HoldSetController's points did not have the correct number of joints.\n");
      return false;
    }

    hold_set_.push_back(position);

    hs = hs->NextSiblingElement("hold_pt");
  }

  // Set up correct number of holding points
  hold_set_data_.hold_data.resize(hold_set_.size()); 
  
  return true;
}

void HoldSetController::update()
{
  // wait until the joints are calibrated to start
  for (unsigned int i = 0; i < num_joints_; i++)
  {
    if (!joint_states_[i]->calibrated_)
    {
      ROS_INFO("Not calibrated!");
      return;
    }
  }
    
  double time = robot_->hw_->current_time_;
  
  if (time - initial_time_ > timeout_ && state_ != DONE) 
  {
    ROS_INFO("Timeout!");
    state_ = DONE;
  }

  for (unsigned int i = 0; i < num_joints_; ++i)
  {
    joint_position_controllers_[i]->update();
  }

  switch (state_)
  {
  case STARTING:
    {
      ROS_INFO("Starting!");

      std::vector<double> current = hold_set_[current_position_];
      assert(current.size() == num_joints_);
                                                        
      hold_set_data_.hold_data[current_position_].joint_data.resize(num_joints_);

      for (unsigned int i = 0; i < num_joints_; ++i)
      {
        double cmd = current[i];

        joint_position_controllers_[i]->setCommand(cmd);
        hold_set_data_.hold_data[current_position_].joint_data[i].desired = cmd;
      }

      start_time_ = time;
      state_ = SETTLING;
      ROS_INFO("Settling");
      break;
    }
  case SETTLING:
    {
      if (time - start_time_ > settle_time_)
      {
        state_ = DITHERING;
        start_time_ = time;
        ROS_INFO("Dithering!");
      }
      
      break;
    }
  case DITHERING:
    {
      joint_qualification_controllers::HoldPositionData *data = &hold_set_data_.hold_data[current_position_];  

      
      for (unsigned int i = 0; i < num_joints_; ++i)
      {
        // Add dither
        joint_states_[i]->commanded_effort_ += dithers_[i]->update();

        // Record state
        data->joint_data[i].time.push_back(time - start_time_);
        data->joint_data[i].position.push_back(joint_states_[i]->position_);
        data->joint_data[i].velocity.push_back(joint_states_[i]->velocity_);
        data->joint_data[i].cmd.push_back(joint_states_[i]->commanded_effort_);
        data->joint_data[i].effort.push_back(joint_states_[i]->applied_effort_);
      }
      
      if (time - start_time_ > dither_time_)
      {
        state_ = PAUSING;
      }
      break;
    }
  case PAUSING:
    {
    if (++current_position_ >= hold_set_.size())
    {     
      state_ = DONE;
      ROS_INFO("Done!");
    }
    else
    {
      state_ = STARTING;
      ROS_INFO("Next point!");
    }
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
        ROS_INFO("Calling results service!");
        joint_qualification_controllers::HoldSetData::Request *out = &call_service_.srv_req_;
        out->test_name   = c_->hold_set_data_.test_name;
        out->joint_names = c_->hold_set_data_.joint_names;
        out->dither_amps = c_->hold_set_data_.dither_amps;

        out->hold_data.resize(c_->hold_set_data_.hold_data.size());

        for (uint i = 0; i < c_->hold_set_data_.hold_data.size(); ++i)
        {
          out->hold_data[i].joint_data.resize(c_->hold_set_data_.hold_data[i].joint_data.size());

          for (unsigned int j = 0; j < c_->hold_set_data_.hold_data[i].joint_data.size(); ++j)
          {
            out->hold_data[i].joint_data[j].desired  = c_->hold_set_data_.hold_data[i].joint_data[j].desired;
            out->hold_data[i].joint_data[j].time     = c_->hold_set_data_.hold_data[i].joint_data[j].time;
            out->hold_data[i].joint_data[j].position = c_->hold_set_data_.hold_data[i].joint_data[j].position;
            out->hold_data[i].joint_data[j].velocity = c_->hold_set_data_.hold_data[i].joint_data[j].velocity;
            out->hold_data[i].joint_data[j].cmd      = c_->hold_set_data_.hold_data[i].joint_data[j].cmd;
            out->hold_data[i].joint_data[j].effort   = c_->hold_set_data_.hold_data[i].joint_data[j].effort;
            
          }
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

  ROS_INFO("Initing hold set controller");
  if (!c_->initXml(robot, config))
    return false;
    
  ROS_INFO("Hold set controller initialized");
  return true;
}

