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
/*! \class controller::MotorTest1
    \brief Motor Test1 Controller

    This tests the motor enocder assembly to check if
    the two are assembled properly.

*/
/***************************************************/


#include <ros/node.h>
#include <robot_msgs/DiagnosticMessage.h>
#include <misc_utils/realtime_publisher.h>
#include <mechanism_model/controller.h>

namespace controller
{

class MotorTest1 : public Controller
{

public:
  /*!
   * \brief Default Constructor of the MotorTest1 class.
   *
   */
  MotorTest1();

  /*!
   * \brief Destructor of the MotorTest1 class.
   */
  ~MotorTest1();

  /*!
   * \brief Functional way to initialize.
   * \param duration The duration in seconds from start to finish.
   * \param time The current hardware time.
   * \param *robot The robot that is being controlled.
   */
  void init(double duration, double torque, std::string fixture_name, double time, std::string name ,mechanism::RobotState *robot);
  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

  /*!
   * \brief Get latest time..
   */
  double getTime();

  /*!
   * \brief Perform the test analysis
   */
  void analysis();

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */

  virtual void update();

private:

  mechanism::JointState *joint_;                    /**< Joint we're controlling. */
  mechanism::JointState *fixture_joint_;            /**< Joint we're qualifying against. */
  mechanism::RobotState *robot_;                    /**< Pointer to robot structure. */
  double duration_;                                 /**< Duration of the test. */
  double torque_;                                   /**< Torque applied during the test. */
  double initial_time_;                             /**< Start time of the test. */
  double test_joint_start_pos_;                     /**< Start pos of the test joint. */
  double fixture_joint_start_pos_;                  /**< Start pos of the fixture joint. */
  double test_joint_end_pos_;                       /**< End pos of the test joint. */
  double fixture_joint_end_pos_;                    /**< End pos of the fixture joint. */
  bool complete;
  misc_utils::RealtimePublisher<robot_msgs::DiagnosticMessage> publisher_;
};

/***************************************************/
/*! \class controller::MotorTest1Node
    \brief Motor Test1 Controller

    This tests the motor enocder assembly to check if
    the two are assembled properly.

*/
/***************************************************/

class MotorTest1Node : public Controller
{
public:
  /*!
   * \brief Default Constructor
   *
   */
  MotorTest1Node();

  /*!
   * \brief Destructor
   */
  ~MotorTest1Node();

  void update();

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

private:
  MotorTest1 *c_;
};
}


