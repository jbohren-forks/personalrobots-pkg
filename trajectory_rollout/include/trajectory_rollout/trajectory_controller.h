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
#ifndef TRAJECTORY_CONTROLLER_H_
#define TRAJECTORY_CONTROLLER_H_

#include <vector>
#include <utility>
#include <iostream>
#include <math.h>
#include <sys/time.h>
#include <utility>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <rosconsole/rosconsole.h>

//For transform support
#include "tf/transform_listener.h"

#include <trajectory_rollout/map_cell.h>
#include <trajectory_rollout/map_grid.h>
#include <trajectory_rollout/trajectory.h>

// For obstacle data access
#include <costmap_2d/obstacle_map_accessor.h>

//we'll take in a path as a vector of points
#include <std_msgs/Point2DFloat32.h>
#include <std_msgs/Position2DInt.h>

//for computing path distance
#include <queue>

#define HEADING_LOOKAHEAD .325
#define OSCILLATION_RESET_DIST .05

//Based on the plan from the path planner, determine what velocities to send to the robot
class TrajectoryController {
  public:
    //create a controller given a map and a path
    TrajectoryController(MapGrid& mg, double sim_time, int num_steps, int samples_per_dim,
        double pdist_scale, double gdist_scale, double dfast_scale, double occdist_scale, 
        double acc_lim_x, double acc_lim_y, double acc_lim_theta, tf::TransformListener* tf,
        const costmap_2d::ObstacleMapAccessor& ma, std::vector<std_msgs::Point2DFloat32> footprint_spec);
    
    //given the current state of the robot, find a good trajectory
    Trajectory findBestPath(tf::Stamped<tf::Pose> global_pose, tf::Stamped<tf::Pose> global_vel,
			    tf::Stamped<tf::Pose>& drive_velocities);

    //compute the distance from each cell in the map grid to the planned path
    void computePathDistance(std::queue<MapCell*>& dist_queue);

    void computeGoalDistance(std::queue<MapCell*>& dist_queue);
    
    //given a trajectory in map space get the drive commands to send to the robot
    tf::Stamped<tf::Pose> getDriveVelocities(int t_num);

    //create the trajectories we wish to score
    Trajectory createTrajectories(double x, double y, double theta, double vx, double vy, double vtheta, 
        double acc_x, double acc_y, double acc_theta);

    //create a trajectory given the current pose of the robot and selected velocities
    void generateTrajectory(double x, double y, double theta, double vx, double vy, 
        double vtheta, double vx_samp, double vy_samp, double vtheta_samp, double acc_x, double acc_y,
        double acc_theta, double impossible_cost, Trajectory& traj);

    //pass a global plan to the controller
    void updatePlan(const std::vector<std_msgs::Point2DFloat32>& new_plan);

    std::vector<std_msgs::Point2DFloat32> drawFootprint(double x_i, double y_i, double theta_i);
    
    //for getting the cost of a given footprint
    double footprintCost(double x_i, double y_i, double theta_i);
    double lineCost(int x0, int x1, int y0, int y1);
    double pointCost(int x, int y);

    //for getting the cells for a given footprint
    std::vector<std_msgs::Position2DInt> getFootprintCells(double x_i, double y_i, double theta_i, bool fill);
    void getLineCells(int x0, int x1, int y0, int y1, std::vector<std_msgs::Position2DInt>& pts);
    void getFillCells(std::vector<std_msgs::Position2DInt>& footprint);
    
    //update what map cells are considered path based on the global_plan
    void setPathCells();

    //possible trajectories for this run
    std::vector<Trajectory> trajectories_;

    //the map passed on from the planner
    MapGrid& map_;
    
    //the number of points we generate for each trajectory
    int num_steps_;

    //the global plan for the robot to follow
    std::vector<std_msgs::Point2DFloat32> global_plan_;

    //to help the robot know what to do with rotations
    bool stuck_left, stuck_right;
    bool rotating_left, rotating_right;

    bool stuck_left_strafe, stuck_right_strafe;
    bool strafe_right, strafe_left;

    double goal_x_,goal_y_;

  private:
    //the simulation parameters for generating trajectories
    double sim_time_;
    int samples_per_dim_;
    double pdist_scale_, gdist_scale_, dfast_scale_, occdist_scale_;
    double acc_lim_x_, acc_lim_y_, acc_lim_theta_;
    double prev_x_, prev_y_;

    //transform client
    tf::TransformListener* tf_;
    
    //so that we can access obstacle information
    const costmap_2d::ObstacleMapAccessor& ma_;

    //for scoring trajectories
    Trajectory traj_one, traj_two;

    //for laying down the footprint of the robot
    std::vector<std_msgs::Point2DFloat32> footprint_spec_;

    inline void updatePathCell(MapCell* current_cell, MapCell* check_cell, 
        std::queue<MapCell*>& dist_queue){
      //mark the cell as visisted
      check_cell->path_mark = true;

      //if the cell is an obstacle set the max path distance
      if(!map_(check_cell->cx, check_cell->cy).within_robot && ma_.isDefinitelyBlocked(check_cell->cx, check_cell->cy)){
        check_cell->path_dist = map_.map_.size();
        return;
      }

      double new_path_dist = current_cell->path_dist + 1;
      if(new_path_dist < check_cell->path_dist)
        check_cell->path_dist = new_path_dist;

      dist_queue.push(check_cell);
    }

    inline void updateGoalCell(MapCell* current_cell, MapCell* check_cell, 
        std::queue<MapCell*>& dist_queue){
      ///mark the cell as visited
      check_cell->goal_mark = true;

      //if the cell is an obstacle set the max goal distance
      if(!map_(check_cell->cx, check_cell->cy).within_robot && ma_.isDefinitelyBlocked(check_cell->cx, check_cell->cy)){
        check_cell->goal_dist = map_.map_.size();
        return;
      }

      double new_goal_dist = current_cell->goal_dist + 1;
      if(new_goal_dist < check_cell->goal_dist)
        check_cell->goal_dist = new_goal_dist;

      dist_queue.push(check_cell);
    }

    //compute position based on velocity
    inline double computeNewXPosition(double xi, double vx, double vy, double theta, double dt){
      return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
    }

    //compute position based on velocity
    inline double computeNewYPosition(double yi, double vx, double vy, double theta, double dt){
      return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
    }

    //compute position based on velocity
    inline double computeNewThetaPosition(double thetai, double vth, double dt){
      return thetai + vth * dt;
    }

    //compute velocity based on acceleration
    inline double computeNewVelocity(double vg, double vi, double a_max, double dt){
      if(vg >= 0)
        return std::min(vg, vi + a_max * dt);
      return std::max(vg, vi - a_max * dt);
    }
    
};


#endif
