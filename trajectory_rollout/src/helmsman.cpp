/*********************************************************************
*
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
#include <trajectory_rollout/helmsman.h>

//construct a helmsman
Helmsman::Helmsman(rosTFClient& tf, double sim_time, int sim_steps, int samples_per_dim,
    double robot_front_radius, double robot_side_radius, double max_occ_dist, 
    double pdist_scale, double gdist_scale, double dfast_scale, double occdist_scale, 
    double acc_lim_x, double acc_lim_y, double acc_lim_th):
  map_(MAP_SIZE_X, MAP_SIZE_Y), 
  tf_(tf), 
  tc_(map_, sim_time, sim_steps, samples_per_dim, robot_front_radius, robot_side_radius, max_occ_dist, 
      pdist_scale, gdist_scale, occdist_scale, dfast_scale, acc_lim_x, acc_lim_y, acc_lim_th, &tf_)
{
}

//compute the drive commands to send to the robot
bool Helmsman::computeVelocityCommands(const ObstacleMapAccessor& ma, const vector<std_msgs::Pose2DFloat32>& globalPlan,
				       double vel_x, double vel_y, double vel_theta, 
				       double& d_x, double& d_y, double& d_theta,
				       vector<std_msgs::Pose2DFloat32>& localPlan){
  localPlan.clear();

  libTF::TFPose2D drive_cmds;

  //pass pose and velocity information to the controller in robot space
  libTF::TFPose2D robot_pose;
  robot_pose.x = 0.0;
  robot_pose.y = 0.0;
  robot_pose.yaw = 0.0;
  robot_pose.frame = "base";
  robot_pose.time = 0;

  libTF::TFPose2D robot_vel;
  robot_vel.x = vel_x;
  robot_vel.y = vel_y;
  robot_vel.yaw = vel_theta;
  robot_vel.frame = "base";
  robot_vel.time = 0;

  //do we need to resize our map?
  double origin_x, origin_y;
  ma.getOriginInWorldCoordinates(origin_x, origin_y);
  map_.sizeCheck(ma.getWidth(), ma.getHeight(), origin_x, origin_y);
  map_.scale = 0.1;
  printf("Map scale: %.2f\n", map_.scale);

  // Temporary Transformation till api below changes
  std::vector<std_msgs::Point2DFloat32> copiedGlobalPlan;
  for(unsigned int i = 0; i< globalPlan.size(); i++){
    std_msgs::Point2DFloat32 p;
    p.x = globalPlan[i].x;
    p.y = globalPlan[i].y;
    copiedGlobalPlan.push_back(p);
  }

  tc_.updatePlan(copiedGlobalPlan);
  
  //compute what trajectory to drive along
  int path_index = tc_.findBestPath(ma, robot_pose, robot_vel, drive_cmds);

  //pass along drive commands
  d_x = drive_cmds.x;
  d_y = drive_cmds.y;
  d_theta = drive_cmds.yaw;

  //if we cannot move... tell someone
  if(path_index < 0)
    return false;

  // Fill out the local plan
  for(int i = 0; i < tc_.num_steps_; ++i){
    std_msgs::Pose2DFloat32 p;
    p.x = tc_.trajectory_pts_(0, path_index * tc_.num_steps_ + i); 
    p.y = tc_.trajectory_pts_(1, path_index * tc_.num_steps_ + i);
    p.th = tc_.trajectory_theta_(0, path_index * tc_.num_steps_ + i);
    localPlan.push_back(p);
  }

  return true;

}
