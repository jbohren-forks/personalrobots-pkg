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
 *   * Neither the name of Willow Garage nor the names of its
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


#include <safety_core/action_tuck_arms.h>

using namespace safety_core;

TuckArmsAction::TuckArmsAction() :
  robot_actions::Action<std_msgs::Empty, std_msgs::Empty>("safety_tuck_arms"),
  action_name_("safety_tuck_arms"),
  node_(ros::Node::instance()),  
  traj_error_(false),
  which_arms_("right"),
  right_arm_controller_("r_arm_joint_trajectory_controller"),
  left_arm_controller_("l_arm_joint_trajectory_controller")
{
  // Check which arms need to be untucked.
  node_->param(action_name_ + "/which_arms", which_arms_, which_arms_);
  if(which_arms_ == "")
  {
    ROS_ERROR("%s: Aborted, which arms param was not set.", action_name_.c_str());
    return;
  }

  // Get the controller names
  if((which_arms_ == "both") || (which_arms_ == "right"))
  {
    node_->param(action_name_ + "/right_arm_controller", right_arm_controller_, right_arm_controller_); 
   
    if(right_arm_controller_ == "")
    {
      ROS_ERROR("%s: Aborted, right arm controller param was not set.", action_name_.c_str());
      return;
    }
  }
  
  if((which_arms_ == "both") || (which_arms_ == "left"))
  {
    node_->param(action_name_ + "/left_arm_controller", left_arm_controller_, left_arm_controller_);
    
    if(left_arm_controller_ == "")
    {
      ROS_ERROR("%s: Aborted, left arm controller param was not set.", action_name_.c_str());
      return;
    }
  }

  double right_arm_traj[2][7] = {{0.0,0.0,0.0,-2.25,0.0,0.0,0.0},{0.0,1.57,-1.57,-2.25,0.0,0.0,0.0}};
  double left_arm_traj[2][7] = {{0.0,0.0,0.0,-2.25,0.0,0.0,0.0}, {0.0,1.57,1.57,-2.25,0.0,0.0,0.0}};
                     
  right_traj_req_.hastiming = 0;
  right_traj_req_.requesttiming = 0;
  right_traj_req_.traj.set_points_size(2);
     
  left_traj_req_.hastiming = 0;
  left_traj_req_.requesttiming = 0;
  left_traj_req_.traj.set_points_size(2);  
                
  for (unsigned int i = 0 ; i < 2 ; ++i)
  {
    right_traj_req_.traj.points[i].set_positions_size(7);
    left_traj_req_.traj.points[i].set_positions_size(7);
    for (unsigned int j = 0 ; j < 7 ; ++j)
    {
      right_traj_req_.traj.points[i].positions[j] = right_arm_traj[i][j];
      left_traj_req_.traj.points[i].positions[j] = left_arm_traj[i][j];
      
    }
    right_traj_req_.traj.points[i].time = 0.0;
    left_traj_req_.traj.points[i].time = 0.0;
  }	
};

TuckArmsAction::~TuckArmsAction()
{
};

robot_actions::ResultStatus TuckArmsAction::execute(const std_msgs::Empty& empty, std_msgs::Empty& feedback)
{ 
  if((which_arms_ == "both") || (which_arms_ == "right"))
  {
    if(!ros::service::call(right_arm_controller_ + "/TrajectoryStart", right_traj_req_, traj_res_))
    {
      ROS_ERROR("%s: Aborted, failed to start right trajectory.", action_name_.c_str());
      return robot_actions::ABORTED;
    }  

    traj_id_ = traj_res_.trajectoryid;
    current_controller_name_ = right_arm_controller_;
    
    while(!isTrajectoryDone() && !traj_error_)
    {
      if (isPreemptRequested()){
	cancelTrajectory();
	ROS_INFO("%s: Preempted.", action_name_.c_str());
        return robot_actions::PREEMPTED;
      }

      sleep(0.5);
    }
  }
  
  
  if((which_arms_ == "both") || (which_arms_ == "left") && !traj_error_)
  {
    if(!ros::service::call(left_arm_controller_ + "/TrajectoryStart", left_traj_req_, traj_res_))
    {
      ROS_ERROR("%s: Aborted, failed to start left trajectory.", action_name_.c_str());
      return robot_actions::ABORTED;
    }  

    traj_id_ = traj_res_.trajectoryid;
    current_controller_name_ = left_arm_controller_;
    
    while(!isTrajectoryDone() && !traj_error_)
    {
      if (isPreemptRequested()){
	cancelTrajectory();
	ROS_INFO("%s: Preempted.", action_name_.c_str());
        return robot_actions::PREEMPTED;
      }

      sleep(0.5);
    }
  }
  
  if(traj_error_)
  {
    ROS_ERROR("%s: Aborted, trajectory controller failed to reach goal.", action_name_.c_str());
    return robot_actions::ABORTED;
  }

  return robot_actions::SUCCESS;
}

bool TuckArmsAction::isTrajectoryDone()
{
  bool done = false;
   
  pr2_mechanism_controllers::TrajectoryQuery::Request req;
  pr2_mechanism_controllers::TrajectoryQuery::Response res;
  
  req.trajectoryid = traj_id_;
  
  if(!ros::service::call(current_controller_name_ + "/TrajectoryQuery", req, res))
  {
    ROS_ERROR("%s: Error, failed to query trajectory completion", action_name_.c_str());
    traj_error_ = true;
    return done;
  }   
  else if(res.done == res.State_Done)
  {
    done = true;
  }
  else if(res.done == res.State_Failed)
  {
    ROS_ERROR("%s: Error, failed to complete trajectory", action_name_.c_str());
    traj_error_ = true;
  }
  
  return done;
}

void TuckArmsAction::cancelTrajectory()
{
  pr2_mechanism_controllers::TrajectoryCancel::Request cancel;
  pr2_mechanism_controllers::TrajectoryCancel::Response dummy;
  cancel.trajectoryid = traj_id_;
  
  if(!ros::service::call(current_controller_name_ + "/TrajectoryCancel", cancel, dummy))
  {
    ROS_ERROR("%s: Error, failed to cancel trajectory", action_name_.c_str());
  }
  return;

}





