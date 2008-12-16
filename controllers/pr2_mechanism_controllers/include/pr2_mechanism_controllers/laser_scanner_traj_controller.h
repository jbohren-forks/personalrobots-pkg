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
#include <robot_mechanism_controllers/joint_position_smoothing_controller.h>

#include <misc_utils/realtime_publisher.h>

// Messages
#include <pr2_mechanism_controllers/LaserScannerSignal.h>
#include <pr2_mechanism_controllers/PeriodicCmd.h>

// Services
#include <robot_mechanism_controllers/SetCommand.h>
#include <robot_mechanism_controllers/GetCommand.h>
#include <pr2_mechanism_controllers/SetProfile.h>

#include "rosthread/mutex.h"
#include "trajectory/trajectory.h"

namespace controller
{

class LaserScannerTrajController : public Controller
{
public:
  LaserScannerTrajController() ;
  ~LaserScannerTrajController() ;

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

  virtual void update() ;

  void setPeriodicCmd(const pr2_mechanism_controllers::PeriodicCmd& cmd) ;

  void setTrajectory(const std::vector<trajectory::Trajectory::TPoint>& traj_points,
                     double max_rate, double max_acc, std::string interp ) ;

  //! \brief Returns what time we're currently at in the profile being executed
  inline double getCurProfileTime() ;
  //! \brief Returns the length (in seconds) of our current profile
  inline double getProfileDuration() ;

private:
  mechanism::RobotState *robot_ ;
  mechanism::JointState *joint_state_ ;                                 // Need this to check the calibrated flag on the joint

  ros::thread::mutex traj_lock_ ;                                       // Mutex for traj_
  trajectory::Trajectory traj_ ;                                        // Stores the current trajectory being executed

  JointPositionSmoothController joint_position_controller_ ;            // The PID position controller that is doing all the under-the-hood controls stuff

  double traj_start_time_ ;                                             // The time that the trajectory was started (in seconds)
  double traj_duration_ ;                                               // The length of the current profile (in seconds)

  double max_rate_ ;                                                    // Max allowable rate/velocity
  double max_acc_ ;                                                     // Max allowable acceleration
};

class LaserScannerTrajControllerNode : public Controller
{
public:
  LaserScannerTrajControllerNode() ;
  ~LaserScannerTrajControllerNode() ;

  void update() ;

  bool initXml(mechanism::RobotState *robot, TiXmlElement *config) ;

  // Message Callbacks
  void setPeriodicCmd() ;

private:
  ros::node *node_ ;
  LaserScannerTrajController c_ ;
  std::string service_prefix_ ;

  double prev_profile_time_ ;                                                    //!< The time in the current profile when update() was last called

  pr2_mechanism_controllers::PeriodicCmd cmd_ ;

  pr2_mechanism_controllers::LaserScannerSignal m_scanner_signal_ ;              //!< Stores the message that we want to send at the end of each sweep, and halfway through each sweep
  bool need_to_send_msg_ ;                                                       //!< Tracks whether we still need to send out the m_scanner_signal_ message.
  misc_utils::RealtimePublisher <pr2_mechanism_controllers::LaserScannerSignal>* publisher_ ;  //!< Publishes the m_scanner_signal msg from the update() realtime loop
};

}
