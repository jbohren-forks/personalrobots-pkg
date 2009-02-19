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

#include <tinyxml/tinyxml.h>
#include <std_srvs/Empty.h>
#include <ros/node.h>

namespace controller
{

  /*! \struct
    \brief This class holds information for a joint control parameter structure.
   */
  typedef struct
  {
      double p_gain; /** P gain */

      double i_gain; /** I gain */

      double d_gain; /** D gain */

      double windup; /** windup protection value */

      std::string joint_name; /** joint name */

      std::string control_type; /** control type */

  }JointControlParam;


class Controller;
typedef Loki::SingletonHolder
<
  Loki::Factory< Controller, std::string >,
  Loki::CreateUsingNew,
  Loki::LongevityLifetime::DieAsSmallObjectParent
> ControllerFactory;

#define ROS_REGISTER_CONTROLLER(c) \
  controller::Controller *ROS_New_##c() { return new c(); }             \
  class RosController##c { \
  public: \
    RosController##c() \
    { \
      controller::ControllerFactory::Instance().Register(#c, ROS_New_##c); \
    } \
    ~RosController##c() \
    { \
      controller::ControllerFactory::Instance().Unregister(#c); \
    } \
  }; \
  static RosController##c ROS_CONTROLLER_##c;



class Controller
{
public:
  enum {CONSTRUCTED, INITIALIZED, RUNNING};
  int state_;
  bool autostart_;
  std::string controller_name_;

  Controller()
  {
    state_ = CONSTRUCTED;
    autostart_ = true;
  }
  virtual ~Controller()
  {
    if (!autostart_){
      // what if multiple controllers have same name?
      ros::Node::instance()->unadvertiseService(controller_name_+"/start");
      ros::Node::instance()->unadvertiseService(controller_name_+"/stop");
    }
  }

  virtual bool start() {return true;};
  virtual void update(void) = 0;
  virtual bool stop() {return true;};
  virtual bool initXml(mechanism::RobotState *robot, TiXmlElement *config) = 0;


  void update_request()
  {
    if (state_ == RUNNING)
      update();
  }

  bool start_request(std_srvs::Empty::Request &req,
                     std_srvs::Empty::Request &res)
  {
    bool ret = false;

    if (state_ == INITIALIZED){
      ret = start();
      if (ret) state_ = RUNNING;
    }

    return ret;
  }


  bool stop_request(std_srvs::Empty::Request &req,
                    std_srvs::Empty::Request &res)

  {
    bool ret = false;

    if (state_ == RUNNING){
      ret = stop();
      if (ret) state_ = INITIALIZED;
    }

    return ret;
  }

  bool initXml_request(mechanism::RobotState *robot, TiXmlElement *config, std::string controller_name)
  {
    if (state_ != CONSTRUCTED)
      return false;
    else
    {
      controller_name_ = controller_name;

      // initialize
      if (!initXml(robot, config))
        return false;
      state_ = INITIALIZED;

      // autostart
      ros::Node::instance()->param(controller_name+"/autostart", autostart_, true);
      std_srvs::Empty::Request req;
      std_srvs::Empty::Request res;
      if (autostart_ && !start_request(req, res))
        return false;
      
      // what if multiple controllers have same name?
      if (!autostart_){
        ros::Node::instance()->advertiseService(controller_name+"/start", &Controller::start_request, this);
        ros::Node::instance()->advertiseService(controller_name+"/stop", &Controller::stop_request, this);
      }


      return true;
    }
  }

};

}
