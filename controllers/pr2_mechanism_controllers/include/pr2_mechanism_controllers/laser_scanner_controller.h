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

#include <misc_utils/realtime_publisher.h>

// Messages
#include <pr2_mechanism_controllers/LaserScannerSignal.h>

// Services
#include <robot_mechanism_controllers/SetCommand.h>
#include <robot_mechanism_controllers/GetCommand.h>
#include <pr2_mechanism_controllers/SetProfile.h>

namespace controller
{

class LaserScannerController : public Controller
{
public:

  enum LaserControllerMode
  {
    MANUAL,SAWTOOTH,SINEWAVE,DYNAMIC_SAWTOOTH,DYNAMIC_SINEWAVE,AUTO_LEVEL
  };

  //! Used to specify which section of a profile we are currently in.
  enum ProfileExecutionState
  {
    NOT_APPLICABLE = 0,  //!< Implies that ProfileExecutionState doesn't make sense in our current control mode
    FIRST_HALF  = 1,     //!< Specifies that we're in the first half of our current profile
    SECOND_HALF = 2      //!< Specifies that we're in the second half of our current profile
  } ;
  
  /*!
   * \brief Default Constructor of the JointController class.
   *
   */
  LaserScannerController();

  /*!
   * \brief Destructor of the JointController class.
   */
  ~LaserScannerController();

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
   * \brief Set automatic profile to sawtooth
   *\param double period Period of signal
   *\param double amplitude Peak to peak amplitude of signal
   *\param int num_elements Number of points along one period of sawtooth wave
   *\param double offset Offset of minimum point of signal to zero
   *\param double current_time Used to determine start of cycle
   */

  void setSawtoothProfile(double period, double amplitude, int num_elements, double offset);

  /*!
   * \brief Set automatic profile to sawtooth, dynamically calculate desired position at each timestep
   *\param double period Period of signal
   *\param double amplitude Peak to peak amplitude of signal
   *\param double offset Offset of minimum point of signal to zero
   *\param double current_time Used to determine start of cycle
   */
  void setSawtoothProfile(double period, double amplitude, double offset);

  /*!
   * \brief Set automatic profile to sinewave
   *\param double period Period of signal
   *\param double amplitude Peak to peak amplitude of signal
   *\param int num_elements Number of points along one period of sine wave
   *\param double offset Offset of minimum point of signal to zero
   *\param double current_time Used to determine start of cycle
   */
  void setSinewaveProfile(double period, double amplitude, int num_elements, double offset);

  /*!
   * \brief Set automatic profile to sinewave, dynamically calculate desired position at each timestep
   *\param double period Period of signal
   *\param double amplitude Peak to peak amplitude of signal
   *\param double offset Offset of minimum point of signal to zero
   *\param double current_time Used to determine start of cycle
   */
  void setSinewaveProfile(double period, double amplitude,double offset);

  /*!
   * \brief Starts the process of auto-leveling
   */
  void startAutoLevelSequence();

  /*!
   * \brief Returns a value indicating whether auto leveling has finished
   */
  bool checkAutoLevelStatus();

  /*!
   * \brief Returns whether auto level completed successfully
   */
  bool checkAutoLevelResult();

  /*!
   * \brief Get which half of the current profile we're in.
   * \return The current profileExecutionState. Will be either first or second half. \
   *         Will return NotApplicable if we're not currently following a profile
   */
  ProfileExecutionState getProfileExecutionState() ;
  
  double getTime();
private:
  /*!
   * \brief Actually issue torque set command of the joint motor.
   */
  void setJointEffort(double torque);

  /*!
   * \brief Get dynamically calculated sinewave position based on time
   *\param double time_from_start Time elapsed since beginning of current period
   */
  void setDynamicSinewave(double time_from_start);

   /*!
   * \brief Get dynamically calculated sawtooth position based on time
   *\param double time_from_start Time elapsed since beginning of current period
   */
  void setDynamicSawtooth(double time_from_start);


  mechanism::JointState* joint_; /*!< Joint we're controlling>*/
  JointPositionController joint_position_controller_; /*!< Internal PID controller>*/
  double last_time_; /*!< Last time stamp of update> */
  double command_; /*!< Last commanded position> */
  mechanism::RobotState *robot_; /*!< Pointer to robot structure>*/
  double* profile_locations_; /**<Contains locations for profile>*/
  double* profile_dt_; /**<Contains timesteps for profile locations>*/
  int profile_index_; /**<Track location in profile>*/
  int profile_length_; /**<Number of points in one cycle>*/
  double cycle_start_time_; //**<Start of the last cycle for profile>*/

  double time_of_last_point_;/*!<Time of last setpoint>*/
  double period_;/*!<Period for use in dynamic profile calculation>*/
  double amplitude_;/*!<Amplitude for use in dynamic profile calculation>*/
  double offset_;/*!<Offset for use in dynamic profile calculation>*/


  LaserControllerMode current_mode_; /*!<Indicates the current status of the controller>*/
  bool auto_level_result_; /*!<Indicates whether the auto_level_routine finished correct>*/

};

class LaserScannerControllerNode : public Controller
{
public:
  /*!
   * \brief Default Constructor
   *
   */
  LaserScannerControllerNode();
  
  /*!
   * \brief Destructor
   */
  ~LaserScannerControllerNode();

  void update();

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

  // Services
  bool setCommand(robot_mechanism_controllers::SetCommand::request &req,
                  robot_mechanism_controllers::SetCommand::response &resp);


  bool setProfileCall(pr2_mechanism_controllers::SetProfile::request &req,
                      pr2_mechanism_controllers::SetProfile::response &resp);


  void setProfile(LaserScannerController::LaserControllerMode profile, double period, double amplitude, int num_elements=0, double offset=0.0);

private:
  LaserScannerController *c_;
  /*!
   * \brief service prefix
   */
  std::string service_prefix_;

  /*!
   * \brief A pointer to ros node
   */
  ros::node *node_;
  
  LaserScannerController::ProfileExecutionState prev_profile_exec_state_ ;       //!< Store the previous profileExecutionState. Need this to compare to the current state to detect transitions
  pr2_mechanism_controllers::LaserScannerSignal m_scanner_signal_ ;              //!< Stores the message that we want to send at the end of each sweep, and halfway through each sweep
  bool need_to_send_msg_ ;                                                       //!< Tracks whether we still need to send out the m_scanner_signal_ message.
  misc_utils::RealtimePublisher <pr2_mechanism_controllers::LaserScannerSignal>* publisher_ ;  //!< Publishes the m_scanner_signal msg from the update() realtime loop
};
}


