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

#include <pr2_controller_interface/controller.h>
#include <control_toolbox/pid.h>
//#include <robot_mechanism_controllers/joint_position_smoothing_controller.h>

#include <realtime_tools/realtime_publisher.h>
#include <tf/tf.h>

#include "filters/transfer_function.h"

// Messages
#include <pr2_msgs/LaserScannerSignal.h>
#include <pr2_msgs/PeriodicCmd.h>
#include <pr2_mechanism_controllers/TrackLinkCmd.h>
#include <pr2_msgs/LaserTrajCmd.h>

// Services
#include <pr2_mechanism_controllers/SetProfile.h>
#include <pr2_msgs/SetPeriodicCmd.h>
#include <pr2_msgs/SetLaserTrajCmd.h>

#include "boost/thread/mutex.hpp"
#include "trajectory/trajectory.h"

namespace controller
{

class LaserScannerTrajController : public Controller
{
public:
  LaserScannerTrajController() ;
  ~LaserScannerTrajController() ;

  bool initXml(pr2_mechanism::RobotState *robot, TiXmlElement *config);
  bool init(pr2_mechanism::RobotState *robot, const ros::NodeHandle &n);

  virtual void update() ;

  bool setPeriodicCmd(const pr2_msgs::PeriodicCmd& cmd) ;

  bool setTrajCmd(const pr2_msgs::LaserTrajCmd& traj_cmd) ;

  //bool setTrackLinkCmd(const pr2_mechanism_controllers::TrackLinkCmd& track_link_cmd) ;

  bool setTrajectory(const std::vector<trajectory::Trajectory::TPoint>& traj_points,
                     double max_rate, double max_acc, std::string interp ) ;

  //! \brief Returns what time we're currently at in the profile being executed
  inline double getCurProfileTime() ;
  //! \brief Returns the length (in seconds) of our current profile
  inline double getProfileDuration() ;
  //! \brief Returns the current trajectory segment we're executing in our current profile
  inline int getCurProfileSegment() ;

private:

  //void initDErrorFilter(double f) ;                                     // Initializes d_error_filter using a smoothing factor

  pr2_mechanism::RobotState *robot_ ;
  pr2_mechanism::JointState *joint_state_ ;                                 // Need this to check the calibrated flag on the joint

  boost::mutex traj_lock_ ;                                       // Mutex for traj_
  trajectory::Trajectory traj_ ;                                        // Stores the current trajectory being executed

  //boost::mutex track_link_lock_ ;
  //bool track_link_enabled_ ;
  //pr2_mechanism::LinkState* target_link_ ;
  //pr2_mechanism::LinkState* mount_link_ ;
  tf::Vector3 track_point_ ;

  //JointPositionSmoothController joint_position_controller_ ;            // The PID position controller that is doing all the under-the-hood controls stuff

  std::string name_ ;                                                   // The controller name. Used for ROS_INFO Messages

  ros::Time traj_start_time_ ;                                             // The time that the trajectory was started (in seconds)
  double traj_duration_ ;                                               // The length of the current profile (in seconds)


  double max_rate_ ;                                                    // Max allowable rate/velocity
  double max_acc_ ;                                                     // Max allowable acceleration

  // Control loop state
  control_toolbox::Pid pid_controller_ ;                                // Position PID Controller
  filters::TransferFunctionFilter<double> d_error_filter ;              // Filter on derivative term of error
  ros::Time last_time_ ;                                                   // The previous time at which the control loop was run
  double last_error_ ;                                                  // Error for the previous time at which the control loop was run
  double tracking_offset_ ;                                             // Position cmd generated by the track_link code
  double traj_command_ ;                                                // Position cmd generated by the trajectory code
};

class LaserScannerTrajControllerNode : public Controller
{
public:
  LaserScannerTrajControllerNode() ;
  ~LaserScannerTrajControllerNode() ;

  void update() ;

  bool initXml(pr2_mechanism::RobotState *robot, TiXmlElement *config) ;
  bool init(pr2_mechanism::RobotState *robot, const ros::NodeHandle &n);

  // Message Callbacks
  void setPeriodicCmd() ;
  void setTrajCmd() ;
  //void setTrackLinkCmd() ;
  bool setPeriodicSrv(pr2_msgs::SetPeriodicCmd::Request &req,
                      pr2_msgs::SetPeriodicCmd::Response &res);
  bool setTrajSrv(pr2_msgs::SetLaserTrajCmd::Request &req,
                  pr2_msgs::SetLaserTrajCmd::Response &res);


private:
  ros::Node *node_ ;
  LaserScannerTrajController c_ ;
  pr2_mechanism::RobotState *robot_ ;
  std::string service_prefix_ ;

  int prev_profile_segment_ ;                                                    //!< The segment in the current profile when update() was last called

  pr2_msgs::PeriodicCmd cmd_ ;
  pr2_msgs::LaserTrajCmd traj_cmd_ ;
  pr2_mechanism_controllers::TrackLinkCmd track_link_cmd_ ;

  pr2_msgs::LaserScannerSignal m_scanner_signal_ ;              //!< Stores the message that we want to send at the end of each sweep, and halfway through each sweep
  bool need_to_send_msg_ ;                                                       //!< Tracks whether we still need to send out the m_scanner_signal_ message.
  realtime_tools::RealtimePublisher <pr2_msgs::LaserScannerSignal>* publisher_ ; //!< Publishes the m_scanner_signal msg from the update() realtime loop
};

}
