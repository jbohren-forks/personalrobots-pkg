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
#include <robot_mechanism_controllers/joint_velocity_controller.h>

// Services
#include <robot_mechanism_controllers/SetCommand.h>
#include <robot_mechanism_controllers/GetCommand.h>
#include <robot_mechanism_controllers/SetProfile.h>
namespace controller
{

class LaserScannerQualification : public Controller
{
public:
  enum LaserQualificationMode
  {
    START, EFFORTTEST, SINESWEEPTEST, ENDSTOPTEST, DONE
  };

  enum TestStatus
  {
    INCOMPLETE, INPROGRESS,FAILED,PASSED, COMPLETE
  };

  /*!
   * \brief Default Constructor of the JointController class.
   *
   */
  LaserScannerQualification();

  /*!
   * \brief Destructor of the JointController class.
   */
  ~LaserScannerQualification();

  /*!
   * \brief Functional way to initialize limits and gains.
   *
   */
  void init(double p_gain, double i_gain, double d_gain, double windup, double time, std::string name, mechanism::Robot *robot);
  bool initXml(mechanism::Robot *robot, TiXmlElement *config);

  /*!
   * \brief Read the torque of the motor
   */
  double getMeasuredPosition();

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */

  virtual void update();

  double getTime();
private:
  /*!
   * \brief Actually issue torque set command of the joint motor.
   */
  void setJointEffort(double torque);

  mechanism::Joint* joint_; /*!< Joint we're controlling>*/
  JointVelocityController joint_velocity_controller_; /*!< Internal PID controller>*/
  double last_time_; /*!< Last time stamp of update> */
  double command_; /*!< Last commanded effort> */
  mechanism::Robot *robot_; /*!< Pointer to robot structure>*/

  TestStatus effort_test_status_;/*!< Status of effort test>*/
  TestStatus sinesweep_test_status_;/*!< Status of sinesweep test>*/
  TestStatus endstop_test_status_;/*!< Status of endstop test>*/


  double effort_test_start_; /*!< Start time of effort test>*/
  double effort_test_percent_;/*!< Percentage of max effort to apply for effort test>*/
  double effort_test_length_; /*!< Length of time to apply effort>*/


  double sinesweep_test_start_;/*!< Start time of sinesweep>*/
  double sinesweep_start_freq_;/*!< Starting frequency for sinesweep>*/
  double sinesweep_end_freq_;/*!< Ending frequency for sinesweep>*/
  double sinesweep_amplitude_;/*!< Amplitude of sine>*/
  double sinesweep_duration_;/*!< Length of sinesweep test>*/
  double sinesweep_K_factor_;/*!< K_Factor for sinesweep >*/
  double sinesweep_L_factor_;/*!< L_Factor for sinesweep >*/


  double endstop_velocity_;/*!< Speed at which to run test>*/
  bool found_positive_endstop_;/*!< Indicates positive endstop has been located>*/
  double endstop_stopped_time_;/*!< Marker to indicate when endstop was first encountered>*/;
  double endstop_stopped_length_;/*!<Amount of time stopped to register endstop >*/
  double endstop_stopped_velocity_;/*!< Threshold below which to register as stopped>*/

  LaserQualificationMode current_mode_; /*!<Indicates the current status of the controller>*/

  void printStatus(TestStatus status);/*!< Translates enums to strings for debugging>*/

};

class LaserScannerQualificationNode : public Controller
{
public:
  /*!
   * \brief Default Constructor
   *
   */
  LaserScannerQualificationNode();

  /*!
   * \brief Destructor
   */
  ~LaserScannerQualificationNode();

  double getMeasuredPosition();

  void update();

  bool initXml(mechanism::Robot *robot, TiXmlElement *config);

  // Services
  bool setCommand(robot_mechanism_controllers::SetCommand::Request &req,
                  robot_mechanism_controllers::SetCommand::Response &resp);

  bool getCommand(robot_mechanism_controllers::GetCommand::Request &req,
                  robot_mechanism_controllers::GetCommand::Response &resp);

  bool getActual(robot_mechanism_controllers::GetActual::Request &req,
                  robot_mechanism_controllers::GetActual::Response &resp);



private:
  LaserScannerQualification *c_;
};
}


