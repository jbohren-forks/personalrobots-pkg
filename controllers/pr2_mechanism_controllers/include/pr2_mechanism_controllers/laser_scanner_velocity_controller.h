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

#include <ros/node.h>

#include <mechanism_model/controller.h>
#include <robot_mechanism_controllers/joint_position_controller.h>
#include <robot_mechanism_controllers/joint_velocity_controller.h>

// Services
#include <robot_mechanism_controllers/SetCommand.h>
#include <robot_mechanism_controllers/GetCommand.h>
#include <pr2_mechanism_controllers/SetProfile.h>
#include <robot_mechanism_controllers/SetPosition.h>
#include <robot_mechanism_controllers/GetPosition.h>
namespace controller
{

class LaserScannerVelocityController : public Controller
{

public:
  //Indicates whether we close the loop around position or velocity
  enum LaserControllerMode
  {
    VELOCITY,POSITION
  };

  /*!
   * \brief Default Constructor of the JointController class.
   *
   */
  LaserScannerVelocityController();

  /*!
   * \brief Destructor of the JointController class.
   */
  ~LaserScannerVelocityController();

  /*!
   * \brief Functional way to initialize limits and gains.
   *
   */
  void init(double p_gain, double i_gain, double d_gain, double windup, double time, std::string name, mechanism::RobotState *robot);
  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *
   * \param double pos Position command to issue
   */
  void setCommand(double command);

  /*!
   * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
   */
  double getCommand();

  /*!
   * \brief Read the torque of the motor
   */
  double getMeasuredPosition();

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */

  virtual void update();

  /*!
   * \brief Returns the time
   */
  double getTime();

  double upper_turnaround_offset_; /*!<Distance from positive endstop where turnaround occurs>*/
  double lower_turnaround_offset_;/*!<Distance from negative endstop where turnaround occurs>*/

  double upper_deceleration_buffer_;/*!<Distance from positive turnaround where deceleration occurs>*/
  double lower_deceleration_buffer_;/*!<Distance from negative turnaround where deceleration occurs>*/

  bool passed_center_; /*!<Marker that indicates that we've recently moved past the center point>*/
  double last_position_; /*!<Record last read position>*/
  bool automatic_turnaround_; /*!<Do we automatically turn around at edges of workspace?>*/

  void setTurnaroundPoints(void);

  LaserControllerMode current_mode_; /*!<Indicates the current status of the controller>*/

  private:

  double upper_deceleration_zone_; /*!<Location near upper endstop where deceleration starts>*/
  double upper_turnaround_location_;/*!<Location near upper endstop where turnaround actually occurs>*/

  double lower_deceleration_zone_;/*!<Location near lower endstop where deceleration starts>*/
  double lower_turnaround_location_;/*!<Location near lower endstop where turnaround actually occurs>*/

   /*!
   * \brief Actually issue torque set command of the joint motor.
   */
  void setJointEffort(double torque);

  mechanism::JointState* joint_; /*!< Joint we're controlling>*/
  JointPositionController joint_position_controller_; /*!< Internal PID controller for position>*/
  JointVelocityController joint_velocity_controller_;/*!< Internal PID controller for velocity>*/
  double last_time_; /*!< Last time stamp of update> */
  double command_; /*!< Last commanded position> */
  mechanism::RobotState *robot_; /*!< Pointer to robot structure>*/

 };

class LaserScannerVelocityControllerNode : public Controller
{
public:
  /*!
   * \brief Default Constructor
   *
   */
  LaserScannerVelocityControllerNode();

  /*!
   * \brief Destructor
   */
  ~LaserScannerVelocityControllerNode();

  double getMeasuredPosition();

  void update();

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

  // Services
   /*!
   * \brief Send velocity command
   */

  bool setCommand(robot_mechanism_controllers::SetCommand::request &req,
                  robot_mechanism_controllers::SetCommand::response &resp);
 /*!
   * \brief Send velocity command
   */

  bool getCommand(robot_mechanism_controllers::GetCommand::request &req,
                  robot_mechanism_controllers::GetCommand::response &resp);
 /*!
   * \brief Send velocity command
   */

  bool setPosition(robot_mechanism_controllers::SetPosition::request &req,
                   robot_mechanism_controllers::SetPosition::response &resp);
 /*!
   * \brief Send velocity command
   */

  bool getPosition(robot_mechanism_controllers::GetPosition::request &req,
                   robot_mechanism_controllers::GetPosition::response &resp);
 
  /*!
   * \brief Send velocity command
   */

  bool setProfile(pr2_mechanism_controllers::SetProfile::request &req,
                  pr2_mechanism_controllers::SetProfile::response &resp);
  /*!
   * \brief Send velocity command
   */

  bool setProfileCall(double upper_turn_around, double lower_turn_around, double upper_decel_buffer, double lower_decel_buffer);


  void setCommand(double command);
  double getCommand();

private:
  LaserScannerVelocityController *c_;
};
}


