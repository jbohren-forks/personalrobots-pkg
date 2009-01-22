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
/*! \class controller::HysteresisController
    \brief Hystersis Controller

    This tests the hysteresis of a joint using a
    velocity controller.

*/
/***************************************************/


#include <ros/node.h>
#include <math.h>
#include <robot_msgs/DiagnosticMessage.h>
#include <robot_srvs/TestData.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_srv_call.h>
#include <mechanism_model/controller.h>
#include <robot_mechanism_controllers/joint_velocity_controller.h>

namespace controller
{

class HysteresisController : public Controller
{

public:
  enum { STOPPED, STARTING, MOVING, ANALYZING, DONE};

  HysteresisController();
  ~HysteresisController();

  /*!
   * \brief Functional way to initialize.
   * \param velocity Target velocity for the velocity controller.
   * \param max_effort Effort to limit the controller at.
   * \param *robot The robot that is being controlled.
   */
  void init( double velocity, double max_effort, double max_expected_effort, double min_expected_effort, double min_pos, double max_pos, double time, std::string name ,mechanism::RobotState *robot);
  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);


  /*!
   * \brief Perform the test analysis
   */
  void analysis();

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */

  virtual void update();
  
  bool done() { return state_ == DONE; }
  
  ros::Node* node;
  robot_msgs::DiagnosticMessage diagnostic_message_;
  robot_srvs::TestData::request test_data_;

private:

  mechanism::JointState *joint_;                        /**< Joint we're controlling. */
  mechanism::RobotState *robot_;                        /**< Pointer to robot structure. */
  controller::JointVelocityController *velocity_controller_;    /**< The sine sweep. */
  double velocity_;                                    /**< Torque applied during the test. */
  double max_effort_;
  double initial_time_;                                 /**< Start time of the test. */
  double initial_position_;
  int count_;
  int loop_count_;
  bool complete;
  bool start;
  
  int state_;
  int starting_count;

};

/***************************************************/
/*! \class controller::HysteresisControllerNode
    \brief Hystersis Controller

    This tests the hysteresis of a joint using a
    velocity controller.

*/
/***************************************************/

class HysteresisControllerNode : public Controller
{
public:

  HysteresisControllerNode();
  ~HysteresisControllerNode();

  void update();
  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

private:
  HysteresisController *c_;
  mechanism::RobotState *robot_;
  
  bool data_sent_;
  
  double last_publish_time_;
  realtime_tools::RealtimeSrvCall<robot_srvs::TestData::request, robot_srvs::TestData::response> call_service_;
  realtime_tools::RealtimePublisher<robot_msgs::DiagnosticMessage> pub_diagnostics_;
};
}


