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

#include <loki/Factory.h>
#include <mechanism_model/robot.h>
#include <ros/node_handle.h>


namespace controller
{

class ControllerHandle;
typedef Loki::SingletonHolder
<
  Loki::Factory< ControllerHandle, std::string >,
  Loki::CreateUsingNew,
  Loki::LongevityLifetime::DieAsSmallObjectParent
> ControllerHandleFactory;

  Loki::Factory< controller::ControllerHandle, std::string >& getControllerHandleFactoryInstance();
#define ROS_REGISTER_CONTROLLER(c) \
  extern Loki::Factory< controller::ControllerHandle, std::string >& getControllerHandleFactoryInstance(); \
  controller::ControllerHandle *ROS_New_##c() { return new c(); }             \
  class RosControllerHandle##c { \
  public: \
    RosControllerHandle##c() \
    { \
      controller::getControllerHandleFactoryInstance().Register(#c, ROS_New_##c); \
    } \
    ~RosControllerHandle##c() \
    { \
      controller::getControllerHandleFactoryInstance().Unregister(#c); \
    } \
  }; \
  static RosControllerHandle##c ROS_CONTROLLER_##c;

class MechanismControl;

class ControllerHandle
{
public:
  enum {CONSTRUCTED, INITIALIZED, RUNNING};

  ControllerHandle():state_(CONSTRUCTED), mc_(NULL){}
  virtual ~ControllerHandle(){}

  bool isRunning();
  void updateRequest();
  bool startRequest();
  bool stopRequest();
  bool initRequest(controller::MechanismControl* mc, mechanism::RobotState *robot, const ros::NodeHandle &n);

  int state_;
  std::vector<std::string> before_list_, after_list_;


protected:
  controller::MechanismControl* mc_;

private:
  // The starting method is called by the realtime thread just before
  // the first call to update.
  virtual bool starting() = 0;
  virtual void update(void) = 0;
  virtual bool stopping() = 0;
  virtual bool init(mechanism::RobotState *robot, const ros::NodeHandle &n) = 0;

  ControllerHandle(const ControllerHandle &c);
  ControllerHandle& operator =(const ControllerHandle &c);


};

}
