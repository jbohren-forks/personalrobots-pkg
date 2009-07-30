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
#pragma once
/***************************************************/
/*! \namespace controller
 \brief The controller namespace

 \class controller::Controller
 \brief A base level controller class.

 */
/***************************************************/

#include <mechanism_model/robot.h>
#include <ros/node_handle.h>
#include <mechanism_control/mechanism_control.h>
#include <mechanism_control/controller_handle.h>

namespace controller
{

class Controller: public ControllerHandle
{
public:
  enum {BEFORE_ME, AFTER_ME};

  Controller(){}
  virtual ~Controller(){}

  // The starting method is called by the realtime thread just before
  // the first call to update.
  virtual bool starting() { return true; }
  virtual void update(void) = 0;
  virtual bool stopping() {return true;}
  virtual bool init(mechanism::RobotState *robot, const ros::NodeHandle &n) 
  {
    ROS_ERROR("Controller %s did not implement init function", n.getNamespace().c_str());
    return false; 
  }

  template<class ControllerType> bool getController(const std::string& name, int sched, ControllerType*& c)
  {
    if (mc_ == NULL){
      ROS_ERROR("No valid pointer to Mechanism Control exists");
      return false;
    }
    if (!mc_->getControllerByName(name, c)){
      ROS_ERROR("Could not find controller %s", name.c_str());
      return false;
    }
    if (sched == BEFORE_ME) before_list_.push_back(name);
    else if (sched == AFTER_ME) after_list_.push_back(name);
    else{
      ROS_ERROR("No valid scheduling specified. Need BEFORE_ME or AFTER_ME in getController function");
      return false;
    }
    return true;
  };

private:
  Controller(const Controller &c);
  Controller& operator =(const Controller &c);

};

}
