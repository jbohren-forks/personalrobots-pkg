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

using namespace std;
using namespace controller;

ROS_REGISTER_CONTROLLER(LaserScannerTrajController)

LaserScannerTrajController::LaserScannerTrajController() : traj_(1)
{

}

LaserScannerTrajController::~LaserScannerTrajController()
{

}

bool LaserScannerTrajController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  if (!robot || !config)
    return false ;
  robot_ = robot ;

  // Look through XML to grab the joint name
  TiXmlElement *j = config->FirstChildElement("joint") ;
  if (!j)
    return false ;

  const char *jn = j->Attribute("name") ;

  if (!jn)
    return false ;  
  std::string joint_name = jn ;

  joint_state_ = robot_->getJointState(joint_name) ;  // Need joint state to check calibrated flag
  
  joint_position_controller_.initXml(robot, config) ; //Pass down XML snippet to encapsulated joint_position_controller_
  return true ;
}

void LaserScannerTrajController::update()
{
  //if (!joint_state_->calibrated_)
  //  return;

  double time = robot_->hw_->current_time_ ;
  
  if (traj_lock_.trylock())
  {
    const double traj_duration = traj_.getTotalTime() ;
    if (traj_duration > 1e-6)                                   // Short trajectories make the mod_time calculation unstable
    {
      double time_from_start = time - traj_start_time_ ;
      double mod_time = time_from_start - floor(time_from_start/traj_.getTotalTime())*traj_.getTotalTime() ;
      
      trajectory::Trajectory::TPoint sampled_point ;
      sampled_point.dimension_ = 1 ;
      sampled_point.q_.resize(1) ;
      sampled_point.qdot_.resize(1) ;
      int result ;
      
      result = traj_.sample(sampled_point, mod_time) ;
      if (result > 0)
      {
        joint_position_controller_.setCommand(sampled_point.q_[0]) ;
        joint_position_controller_.update() ;
      }
  
    }
    traj_lock_.unlock() ;
  }
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
  
  //for (unsigned int i=0; i<traj_points.
  
  traj_.setTrajectory(traj_points) ;

  traj_start_time_ = robot_->hw_->current_time_ ;
  
  traj_lock_.unlock() ;
}

ROS_REGISTER_CONTROLLER(LaserScannerTrajControllerNode)
LaserScannerTrajControllerNode::LaserScannerTrajControllerNode(): node_(ros::node::instance())
{
  c_ = new LaserScannerTrajController();
}

LaserScannerTrajControllerNode::~LaserScannerTrajControllerNode()
{
  node_->unsubscribe(service_prefix_ + "/set_command");
  delete c_;
}

void LaserScannerTrajControllerNode::update()
{
  c_->update();
}


void LaserScannerTrajControllerNode::setCommand()
{
  if (cmd_.data >= -.5 && cmd_.data <= .5)
  {
    std::vector<trajectory::Trajectory::TPoint> tpoints ;
    
    trajectory::Trajectory::TPoint cur_point(1) ;
    
    cur_point.dimension_ = 1 ;
    
    cur_point.q_[0] = 0 ;
    cur_point.time_ = 0.0 ;
    tpoints.push_back(cur_point) ;
    
    cur_point.q_[0] = 0 ;
    cur_point.time_ = 50.0 ;
    tpoints.push_back(cur_point) ;
    
    cur_point.q_[0] = 0 ;
    cur_point.time_ = 100.0 ;
    tpoints.push_back(cur_point) ;
    
    double max_rate = 1 ;
    double max_acc = 1 ;
    
    c_->setTrajectory(tpoints, max_rate, max_acc, "linear") ;    
  }
  else if (cmd_.data >= -52.1 && cmd_.data <= -51.9)
  {
    std::vector<trajectory::Trajectory::TPoint> tpoints ;
    
    trajectory::Trajectory::TPoint cur_point(1) ;
    
    cur_point.dimension_ = 1 ;
    
    cur_point.q_[0] = .5 ;
    cur_point.time_ = 0.0 ;
    tpoints.push_back(cur_point) ;
    
    cur_point.q_[0] = -.5 ;
    cur_point.time_ = 5.0 ;
    tpoints.push_back(cur_point) ;
    
    cur_point.q_[0] = .5 ;
    cur_point.time_ = 10.0 ;
    tpoints.push_back(cur_point) ;
    
    double max_rate = 1 ;
    double max_acc = 1 ;
    
    c_->setTrajectory(tpoints, max_rate, max_acc, "linear") ;
  }
  else if (cmd_.data >= -53.1 && cmd_.data <= -52.9)
  {
    std::vector<trajectory::Trajectory::TPoint> tpoints ;
    
    trajectory::Trajectory::TPoint cur_point(1) ;
    
    cur_point.dimension_ = 1 ;
    
    cur_point.q_[0] = .5 ;
    cur_point.time_ = 0.0 ;
    tpoints.push_back(cur_point) ;
    
    cur_point.q_[0] = -.5 ;
    cur_point.time_ = 5.0 ;
    tpoints.push_back(cur_point) ;
    
    cur_point.q_[0] = .5 ;
    cur_point.time_ = 10.0 ;
    tpoints.push_back(cur_point) ;
    
    double max_rate = 1 ;
    double max_acc = 1 ;
    
    c_->setTrajectory(tpoints, max_rate, max_acc, "blended_linear") ;
  }
}

bool LaserScannerTrajControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  service_prefix_ = config->Attribute("name");

  if (!c_->initXml(robot, config))
  {
    return false;
  }
  node_->subscribe(service_prefix_ + "/set_command", cmd_, &LaserScannerTrajControllerNode::setCommand, this, 1);

  return true;
}
