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
#ifndef HELMSMAN_H_
#define HELMSMAN_H_

#include <trajectory_rollout/trajectory_controller.h>
#include <trajectory_rollout/map_grid.h>
#include <rosTF/rosTF.h>

#define MAP_SIZE_X 100
#define MAP_SIZE_Y 100

//uses a trajectory controller to steer the robot
class Helmsman{
  public:
    Helmsman(rosTFClient& tf, double sim_time, int sim_steps, int samples_per_dim,
        double robot_front_radius, double robot_side_radius, double max_occ_dist, 
        double pdist_scale, double gdist_scale, double dfast_scale, double occdist_scale, 
        double acc_lim_x, double acc_lim_y, double acc_lim_th);

    bool computeVelocityCommands(const ObstacleMapAccessor& ma, const std::list<std_msgs::Pose2DFloat32>& globalPlan,
				 double vel_x, double vel_y, double vel_theta, 
				 double& d_x, double& d_y, double& d_theta,
				 std::list<std_msgs::Pose2DFloat32>& localPlan);

    std::vector<std_msgs::Point2DFloat32> drawFootprint(double x, double y, double th);

  private:

    //a map
    MapGrid map_;

    //transform client
    rosTFClient& tf_;

    //trajectory controller
    TrajectoryController tc_;

};
#endif
