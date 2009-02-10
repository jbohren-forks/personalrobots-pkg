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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include <trajectory_rollout/trajectory_controller_ros.h>
#include <ros/console.h>
#include <sys/time.h>

using namespace std;
using namespace std_msgs;
using namespace costmap_2d;

namespace trajectory_rollout {
  TrajectoryControllerROS::TrajectoryControllerROS(ros::Node& ros_node, 
      const costmap_2d::ObstacleMapAccessor& ma, 
      std::vector<std_msgs::Point2DFloat32> footprint_spec, double inscribed_radius, double circumscribed_radius) 
    : world_model_(NULL), tc_(NULL){
    double acc_lim_x, acc_lim_y, acc_lim_theta, sim_time, sim_granularity;
    int samples_per_dim;
    double pdist_scale, gdist_scale, occdist_scale, heading_lookahead, oscillation_reset_dist;
    bool holonomic_robot;
    double max_vel_x, min_vel_x, max_vel_th, min_vel_th, min_in_place_vel_th;

    bool freespace_model;
    
    ros_node.param("/trajectory_rollout/acc_lim_x", acc_lim_x, 2.5);
    ros_node.param("/trajectory_rollout/acc_lim_y", acc_lim_y, 2.5);
    ros_node.param("/trajectory_rollout/acc_lim_th", acc_lim_theta, 3.2);
    ros_node.param("/trajectory_rollout/sim_time", sim_time, 1.0);
    ros_node.param("/trajectory_rollout/sim_granularity", sim_granularity, 0.025);
    ros_node.param("/trajectory_rollout/samples_per_dim", samples_per_dim, 20);
    ros_node.param("/trajectory_rollout/path_distance_bias", pdist_scale, 0.6);
    ros_node.param("/trajectory_rollout/goal_distance_bias", gdist_scale, 0.8);
    ros_node.param("/trajectory_rollout/occdist_scale", occdist_scale, 0.2);
    ros_node.param("/trajectory_rollout/heading_lookahead", heading_lookahead, 0.325);
    ros_node.param("/trajectory_rollout/oscillation_reset_dist", oscillation_reset_dist, 0.05);
    ros_node.param("/trajectory_rollout/holonomic_robot", holonomic_robot, true);
    ros_node.param("/trajectory_rollout/max_vel_x", max_vel_x, 0.5);
    ros_node.param("/trajectory_rollout/min_vel_x", min_vel_x, 0.1);
    ros_node.param("/trajectory_rollout/max_vel_th", max_vel_th, 1.0);
    ros_node.param("/trajectory_rollout/min_vel_th", min_vel_th, -1.0);
    ros_node.param("/trajectory_rollout/min_in_place_vel_th", min_in_place_vel_th, 0.4);
    ros_node.param("/trajectory_rollout/freespace_model", freespace_model, false);

    if(freespace_model){
      Point2DFloat32 origin;
      origin.x = 0;
      origin.y = 0;
      world_model_ = new PointGrid(70.0, 70.0, 0.2, origin, 2.0, 2.0, 0.01);
      ROS_ERROR("Freespace\n");
    }
    else{
      world_model_ = new CostmapModel(ma); 
      ROS_ERROR("Costmap\n");
    }

    tc_ = new TrajectoryController(*world_model_, ma, footprint_spec, inscribed_radius, circumscribed_radius,
        acc_lim_x, acc_lim_y, acc_lim_theta, sim_time, sim_granularity, samples_per_dim, pdist_scale,
        gdist_scale, occdist_scale, heading_lookahead, oscillation_reset_dist, holonomic_robot,
        max_vel_x, min_vel_x, max_vel_th, min_vel_th, min_in_place_vel_th);
  }

  TrajectoryControllerROS::~TrajectoryControllerROS(){
    if(tc_ != NULL)
      delete tc_;
    if(world_model_ != NULL)
      delete world_model_;
  }

  vector<Point2DFloat32> TrajectoryControllerROS::drawFootprint(double x_i, double y_i, double theta_i){
    return tc_->drawFootprint(x_i, y_i, theta_i);
  }

  bool TrajectoryControllerROS::computeVelocityCommands(const std::list<deprecated_msgs::Pose2DFloat32>& global_plan, 
      const tf::Stamped<tf::Pose>& global_pose, 
      const std_msgs::PoseDot& global_vel, 
      std_msgs::PoseDot& cmd_vel,
      std::list<deprecated_msgs::Pose2DFloat32>& localPlan,
      const std::vector<costmap_2d::Observation>& observations){


    localPlan.clear();

    tf::Stamped<tf::Pose> drive_cmds;
    drive_cmds.frame_id_ = "base_link";

    tf::Stamped<tf::Pose> robot_vel;
    btQuaternion qt(global_vel.ang_vel.vz, 0, 0);
    robot_vel.setData(btTransform(qt, btVector3(global_vel.vel.vx, global_vel.vel.vy, 0)));
    robot_vel.frame_id_ = "base_link";
    robot_vel.stamp_ = ros::Time();

    //do we need to resize our map?
    //double origin_x, origin_y;
    //ma_.getOriginInWorldCoordinates(origin_x, origin_y);
    //map_.sizeCheck(ma_.getWidth(), ma_.getHeight(), origin_x, origin_y);

    // Temporary Transformation till api below changes
    std::vector<std_msgs::Point2DFloat32> copiedGlobalPlan;
    for(std::list<deprecated_msgs::Pose2DFloat32>::const_iterator it = global_plan.begin(); it != global_plan.end(); ++it){
      std_msgs::Point2DFloat32 p;
      p.x = it->x;
      p.y = it->y;
      copiedGlobalPlan.push_back(p);
    }

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */
    tc_->updatePlan(copiedGlobalPlan);

    //compute what trajectory to drive along
    Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds, observations);

    /* For timing uncomment
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_ERROR("Cycle time: %.9f", t_diff);
    */

    //pass along drive commands
    cmd_vel.vel.vx = drive_cmds.getOrigin().getX();
    cmd_vel.vel.vy = drive_cmds.getOrigin().getY();
    double uselessPitch, uselessRoll, yaw;
    drive_cmds.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);

    cmd_vel.ang_vel.vz = yaw;

    //if we cannot move... tell someone
    if(path.cost_ < 0)
      return false;

    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i){
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      deprecated_msgs::Pose2DFloat32 p;
      p.x = p_x; 
      p.y = p_y;
      p.th = p_th;
      localPlan.push_back(p);
    }

    return true;
  }

  void TrajectoryControllerROS::getLocalGoal(double& x, double& y){
    return tc_->getLocalGoal(x, y);
  }
};
