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
#include <algorithm>
#include <pr2_mechanism_controllers/laser_scanner_traj_controller.h>
#include <angles/angles.h>

#include <math.h>

using namespace std ;
using namespace controller ;
using namespace filters ;

ROS_REGISTER_CONTROLLER(LaserScannerTrajController)

LaserScannerTrajController::LaserScannerTrajController() : traj_(1)
{
  tracking_offset_ = 0 ;
  track_link_enabled_ = false ;
  d_error_filter = NULL ;
}

LaserScannerTrajController::~LaserScannerTrajController()
{
  if (d_error_filter != NULL)
    delete d_error_filter ;
}

bool LaserScannerTrajController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  if (!robot || !config)
    return false ;
  robot_ = robot ;

  // ***** Name element *****
  const char* name = config->Attribute("name") ;
  if (!name)
  {
    ROS_ERROR("LaserScannerTrajController:: Name attribute not defined in controller tag") ;
    return false ;
  }
  name_ = name ;

  // ***** Joint Element *****
  TiXmlElement *j = config->FirstChildElement("joint") ;
  if (!j)
  {
    ROS_ERROR("%s:: joint element not defined inside controller", name_.c_str()) ;
    return false ;
  }
  const char *jn = j->Attribute("name") ;

  if (!jn)
  {
    ROS_ERROR("%s:: name attribute not defined insidejoint element", name_.c_str()) ;
    return false ;
  }
  joint_state_ = robot_->getJointState(string(jn)) ;  // Need joint state to check calibrated flag
  if (!joint_state_)
  {
    ROS_ERROR("%s:: Could not find joint \"%s\" in robot model", name_.c_str(), jn) ;
    return false ;
  }

  TiXmlElement *pid_elem = j->FirstChildElement("pid") ;
  if (!pid_elem)
  {
    ROS_ERROR("%s:: Could not find element \"pid\" in joint", name_.c_str()) ;
    return false ;
  }

  bool result ;
  result = pid_controller_.initXml(pid_elem) ;
  if (!result)
  {
    ROS_ERROR("%s:: Error initializing pid element", name_.c_str()) ;
    return false ;
  }
  last_time_ = robot->hw_->current_time_ ;
  last_error_ = 0.0 ;

  // ***** Derivate Error Filter Element *****
  TiXmlElement *filter_elem = config->FirstChildElement("d_error_filter");
  if(!filter_elem)
  {
    ROS_ERROR("%s:: d_error_filter element not defined inside controller", name_.c_str()) ;
    return false ;
  }

  double smoothing_factor ;
  if(filter_elem->QueryDoubleAttribute("smoothing_factor", & smoothing_factor)!=TIXML_SUCCESS)
  {
    ROS_ERROR("%s:: Error reading \"smoothing_factor\" element in d_error_filter element", name_.c_str()) ;
    return false ;
  }
  initDErrorFilter(smoothing_factor) ;

  // ***** Max Rate and Acceleration Elements *****
  TiXmlElement *max_rate_elem = config->FirstChildElement("max_rate") ;
  if (!max_rate_elem)
    return false ;
  if(max_rate_elem->QueryDoubleAttribute("value", &max_rate_) != TIXML_SUCCESS )
    return false ;

  TiXmlElement *max_acc_elem = config->FirstChildElement("max_acc") ;
  if (!max_acc_elem)
    return false ;
  if(max_acc_elem->QueryDoubleAttribute("value", &max_acc_) != TIXML_SUCCESS )
    return false ;

  return true ;
}

void LaserScannerTrajController::initDErrorFilter(double f)
{
  vector<double> a ;
  vector<double> b ;
  a.resize(2) ;
  b.resize(1) ;
  a[0] = 1.0 ;
  a[1] = - (1.0 - f) ;
  b[0] = 1.0 ;

  if(d_error_filter == NULL)
    delete d_error_filter ;
  d_error_filter = new TransferFunctionFilter<double>(b,a,1) ;
}

void LaserScannerTrajController::update()
{
  if (!joint_state_->calibrated_)
    return;

  // ***** Compute the offset from tracking a link *****
  //! \todo replace this link tracker with a KDL inverse kinematics solver
  if(track_link_lock_.trylock())
  {
    if (track_link_enabled_  && target_link_ && mount_link_)
    {
      // Compute the position of track_point_ in the world frame
      tf::Pose link_pose(target_link_->abs_orientation_, target_link_->abs_position_) ;
      tf::Point link_point_world ;
      link_point_world = link_pose*track_point_ ;

      // We're hugely approximating our inverse kinematics. This is probably good enough for now...
      double dx = link_point_world.x() - mount_link_->abs_position_.x() ;
      double dz = link_point_world.z() - mount_link_->abs_position_.z() ;
      tracking_offset_ = atan2(-dz,dx) ;
    }
    else
    {
      tracking_offset_ = 0.0 ;
    }
    track_link_lock_.unlock() ;
  }

  // ***** Compute the current command from the trajectory profile *****
  if (traj_lock_.trylock())
  {
    if (traj_duration_ > 1e-6)                                   // Short trajectories could make the mod_time calculation unstable
    {
      double profile_time = getCurProfileTime() ;

      trajectory::Trajectory::TPoint sampled_point ;
      sampled_point.dimension_ = 1 ;
      sampled_point.q_.resize(1) ;
      sampled_point.qdot_.resize(1) ;
      int result ;

      result = traj_.sample(sampled_point, profile_time) ;
      if (result > 0)
        traj_command_ = sampled_point.q_[0] ;
    }
    traj_lock_.unlock() ;
  }

  // ***** Run the position control loop *****
  double cmd = traj_command_ + tracking_offset_ ;

  double time = robot_->hw_->current_time_ ;
  double error(0.0) ;
  angles::shortest_angular_distance_with_limits(cmd, joint_state_->position_,
                                                joint_state_->joint_->joint_limit_min_,
                                                joint_state_->joint_->joint_limit_max_,
                                                error) ;
  double dt = time - last_time_ ;
  double d_error = (error-last_error_)/dt ;
  vector<double> filtered_d_error ;
  filtered_d_error.resize(1) ;

  vector<double> d_error_vec(1,d_error) ;

  d_error_filter->update(&d_error_vec, &filtered_d_error) ;

  // Add filtering step
  // Update pid with d_error added
  joint_state_->commanded_effort_ = pid_controller_.updatePid(error, filtered_d_error[0], dt) ;
  last_time_ = time ;
  last_error_ = error ;
}

double LaserScannerTrajController::getCurProfileTime()
{
  double time = robot_->hw_->current_time_ ;
  double time_from_start = time - traj_start_time_ ;
  double mod_time = time_from_start - floor(time_from_start/traj_.getTotalTime())*traj_.getTotalTime() ;
  return mod_time ;
}

double LaserScannerTrajController::getProfileDuration()
{
  return traj_duration_ ;
}

void LaserScannerTrajController::setTrajectory(const std::vector<trajectory::Trajectory::TPoint>& traj_points, double max_rate, double max_acc, std::string interp)
{
  while (!traj_lock_.trylock())
    usleep(100) ;

  vector<double> max_rates ;
  max_rates.push_back(max_rate) ;
  vector<double> max_accs ;
  max_accs.push_back(max_acc) ;

  traj_.setMaxRates(max_rates) ;
  traj_.setMaxAcc(max_accs) ;
  traj_.setInterpolationMethod(interp) ;

  traj_.setTrajectory(traj_points) ;

  traj_start_time_ = robot_->hw_->current_time_ ;

  traj_duration_ = traj_.getTotalTime() ;

  traj_lock_.unlock() ;
}

void LaserScannerTrajController::setPeriodicCmd(const pr2_mechanism_controllers::PeriodicCmd& cmd)
{
  if (cmd.profile == "linear" ||
      cmd.profile == "blended_linear")
  {
    double high_pt = cmd.amplitude + cmd.offset ;
    double low_pt = -cmd.amplitude + cmd.offset ;

    std::vector<trajectory::Trajectory::TPoint> tpoints ;

    trajectory::Trajectory::TPoint cur_point(1) ;

    cur_point.dimension_ = 1 ;

    cur_point.q_[0] = low_pt ;
    cur_point.time_ = 0.0 ;
    tpoints.push_back(cur_point) ;

    cur_point.q_[0] = high_pt ;
    cur_point.time_ = cmd.period/2.0 ;
    tpoints.push_back(cur_point) ;

    cur_point.q_[0] = low_pt ;
    cur_point.time_ = cmd.period ;
    tpoints.push_back(cur_point) ;

    setTrajectory(tpoints, max_rate_, max_acc_, cmd.profile) ;
    ROS_INFO("LaserScannerTrajController: Periodic Command set") ;
  }
  else
  {
    ROS_WARN("Unknown Periodic Trajectory Type. Not setting command.") ;
  }
}

void LaserScannerTrajController::setTrackLinkCmd(const pr2_mechanism_controllers::TrackLinkCmd& track_link_cmd)
{
  while (!track_link_lock_.trylock())
    usleep(100) ;

  if (track_link_cmd.enable)
  {
    ROS_INFO("LaserScannerTrajController:: Tracking link %s", track_link_cmd.link_name.c_str()) ;
    track_link_enabled_ = true ;
    string mount_link_name = "laser_tilt_mount_link" ;
    target_link_ = robot_->getLinkState(track_link_cmd.link_name) ;
    mount_link_  = robot_->getLinkState(mount_link_name) ;
    tf::PointMsgToTF(track_link_cmd.point, track_point_) ;

    if (target_link_ == NULL)
    {
      ROS_ERROR("LaserScannerTrajController:: Could not find target link:%s", track_link_cmd.link_name.c_str()) ;
      track_link_enabled_ = false ;
    }
    if (mount_link_ == NULL)
    {
      ROS_ERROR("LaserScannerTrajController:: Could not find mount link:%s", mount_link_name.c_str()) ;
      track_link_enabled_ = false ;
    }
  }
  else
  {
    track_link_enabled_ = false ;
    ROS_INFO("LaserScannerTrajController:: No longer tracking link") ;
  }

  track_link_lock_.unlock() ;
}


ROS_REGISTER_CONTROLLER(LaserScannerTrajControllerNode)
LaserScannerTrajControllerNode::LaserScannerTrajControllerNode(): node_(ros::node::instance()), c_()
{
  need_to_send_msg_ = false ;                                           // Haven't completed a sweep yet, so don't need to send a msg
  publisher_ = NULL ;                                                   // We don't know our topic yet, so we can't build it
}

LaserScannerTrajControllerNode::~LaserScannerTrajControllerNode()
{
  node_->unsubscribe(service_prefix_ + "/set_periodic_cmd") ;
  node_->unsubscribe(service_prefix_ + "/set_track_link_cmd") ;

  publisher_->stop() ;

  delete publisher_ ;    // Probably should wait on publish_->is_running() before exiting. Need to
                         //   look into shutdown semantics for realtime_publisher
}

void LaserScannerTrajControllerNode::update()
{
  c_.update() ;

  double cur_profile_time = c_.getCurProfileTime() ;

  // Check if we crossed the middle point of our profile
  if (cur_profile_time  >= c_.getProfileDuration()/2.0 &&
      prev_profile_time_ < c_.getProfileDuration()/2.0)
  {
    // Should we be populating header.stamp here? Or, we can simply let ros take care of the timestamp
    ros::Time cur_time ;
    cur_time.fromSec(robot_->hw_->current_time_) ;
    m_scanner_signal_.header.stamp = cur_time ;
    m_scanner_signal_.signal = 0 ;
    need_to_send_msg_ = true ;
  }
  else if (cur_profile_time < prev_profile_time_)        // Check if we wrapped around
  {
    ros::Time cur_time ;
    cur_time.fromSec(robot_->hw_->current_time_) ;
    m_scanner_signal_.header.stamp = cur_time ;
    m_scanner_signal_.signal = 1 ;
    need_to_send_msg_ = true ;
  }
  prev_profile_time_ = cur_profile_time ;

  // Use the realtime_publisher to try to send the message.
  //   If it fails sending, it's not a big deal, since we can just try again 1 ms later. No one will notice.
  if (need_to_send_msg_)
  {
    if (publisher_->trylock())
    {
      publisher_->msg_.header = m_scanner_signal_.header ;
      publisher_->msg_.signal = m_scanner_signal_.signal ;
      publisher_->unlockAndPublish() ;
      need_to_send_msg_ = false ;
    }
    //printf("tilt_laser: Signal trigger (%u)\n", m_scanner_signal_.signal) ;
    //std::cout << std::flush ;
  }
}

bool LaserScannerTrajControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  robot_ = robot ;      // Need robot in order to grab hardware time

  service_prefix_ = config->Attribute("name") ;

  if (!c_.initXml(robot, config))
  {
    ROS_ERROR("Error Loading LaserScannerTrajControllerNode XML") ;
    return false ;
  }

  node_->subscribe(service_prefix_ + "/set_periodic_cmd", cmd_, &LaserScannerTrajControllerNode::setPeriodicCmd, this, 1) ;
  node_->subscribe(service_prefix_ + "/set_track_link_cmd", track_link_cmd_, &LaserScannerTrajControllerNode::setTrackLinkCmd, this, 1) ;

  if (publisher_ != NULL)               // Make sure that we don't memory leak if initXml gets called twice
    delete publisher_ ;
  publisher_ = new misc_utils::RealtimePublisher <pr2_mechanism_controllers::LaserScannerSignal> (service_prefix_ + "/laser_scanner_signal", 1) ;

  prev_profile_time_ = 0.0 ;

  return true ;
}

void LaserScannerTrajControllerNode::setPeriodicCmd()
{
  c_.setPeriodicCmd(cmd_) ;
}

void LaserScannerTrajControllerNode::setTrackLinkCmd()
{
  c_.setTrackLinkCmd(track_link_cmd_) ;
}
