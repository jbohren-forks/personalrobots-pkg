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

//for matrix support
#include <newmat10/newmat.h>
#include <newmat10/newmatio.h>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

//For transform support
#include <rosTF/rosTF.h>

#include <trajectory_rollout/map_cell.h>
#include <trajectory_rollout/map_grid.h>
#include <trajectory_rollout/trajectory.h>
#include <trajectory_rollout/obstacle_map_accessor.h>

//we'll take in a path as a vector of points
#include <std_msgs/Point2DFloat32.h>

#define VALID_CELL(map, x, y) (((x) >= 0) && ((x) < ((int)(map).size_x_)) && ((y) >= 0) && ((y) < ((int)(map).size_y_)))

//convert from map to world coords
#define MX_WX(map, i) ((map).origin_x + (i) * (map).scale)
#define MY_WY(map, j) ((map).origin_y + (j) * (map).scale)


//convert from world to map coords
#define WX_MX(map, x) ((int)(((x) - (map).origin_x) / (map).scale + 0.5))
#define WY_MY(map, y) ((int)(((y) - (map).origin_y) / (map).scale + 0.5))

//Based on the plan from the path planner, determine what velocities to send to the robot
class TrajectoryController {
  public:
    //create a controller given a map and a path
    TrajectoryController(MapGrid& mg, double sim_time, int num_steps, int samples_per_dim,
        double robot_front_radius, double robot_side_radius, double max_occ_dist, 
        double pdist_scale, double gdist_scale, double dfast_scale, double occdist_scale, 
        double acc_lim_x, double acc_lim_y, double acc_lim_theta, rosTFClient* tf);
    
    //given the current state of the robot, find a good trajectory
    int findBestPath(const ObstacleMapAccessor& ma, libTF::TFPose2D global_pose, libTF::TFPose2D global_vel,
        libTF::TFPose2D& drive_velocities);

    //compute the distance from each cell in the map grid to the planned path
    void computePathDistance();
    
    //given a trajectory in map space get the drive commands to send to the robot
    libTF::TFPose2D getDriveVelocities(int t_num);

    //create the trajectories we wish to score
    void createTrajectories(double x, double y, double theta, double vx, double vy, double vtheta, 
        double acc_x, double acc_y, double acc_theta);

    //create a trajectory given the current pose of the robot and selected velocities
    Trajectory generateTrajectory(int t_num, double x, double y, double theta, double vx, double vy, 
        double vtheta, double vx_samp, double vy_samp, double vtheta_samp, double acc_x, double acc_y,
        double acc_theta);

    //pass a global plan to the controller
    void updatePlan(std::vector<std_msgs::Point2DFloat32>& new_plan);

    std::vector<std_msgs::Point2DFloat32> drawFootprint(double x_i, double y_i, double theta_i);
    void drawLine(int x0, int x1, int y0, int y1, std::vector<std_msgs::Point2DFloat32>& pts);

    
    //possible trajectories for this run
    std::vector<Trajectory> trajectories_;

    //needed to convert between map and robot space efficiently
    boost::numeric::ublas::matrix<double> trajectory_pts_;
    boost::numeric::ublas::matrix<double> trajectory_theta_;

    //the number of trajectories we'll create
    int num_trajectories_;

    //the map passed on from the planner
    MapGrid& map_;
    
    //the number of points we generate for each trajectory
    int num_steps_;

    //the global plan for the robot to follow
    std::vector<std_msgs::Point2DFloat32> global_plan_;

  private:
    //update what map cells are considered path based on the global_plan
    void setPathCells();

    //convert the trajectories computed in robot space to world space
    void trajectoriesToWorld();
    void transformTrajects(double x_i, double y_i, double th_i);

    //compute the cost for a single trajectory
    double trajectoryCost(const ObstacleMapAccessor& ma, int t_index, double pdist_scale, 
        double gdist_scale, double occdist_scale, double dfast_scale, double safe_raidus);

    double footprintCost(const ObstacleMapAccessor& ma, double x_i, double y_i, double theta_i);
    double lineCost(const ObstacleMapAccessor& ma, int x0, int x1, int y0, int y1);
    double pointCost(const ObstacleMapAccessor& ma, int x, int y);
    void swap(int& a, int& b);

    //the simulation parameters for generating trajectories
    double sim_time_;
    int samples_per_dim_;
    double robot_front_radius_, robot_side_radius_, max_occ_dist_;
    double pdist_scale_, gdist_scale_, dfast_scale_, occdist_scale_;
    double acc_lim_x_, acc_lim_y_, acc_lim_theta_;

    //transform client
    rosTFClient* tf_;

    inline void updateCell(MapCell* current_cell, MapCell* check_cell){
      double new_path_dist = check_cell->path_dist + 1;
      double new_goal_dist = check_cell->goal_dist + 1;

      if(new_path_dist < current_cell->path_dist)
        current_cell->path_dist = new_path_dist;

      if(new_goal_dist < current_cell->goal_dist)
        current_cell->goal_dist = new_goal_dist;
    }

    //compute position based on velocity
    inline double computeNewXPosition(double xi, double vx, double vy, double theta, double dt){
      return xi + (vx * cos(theta) + vy * sin(theta)) * dt;
    }

    //compute position based on velocity
    inline double computeNewYPosition(double yi, double vx, double vy, double theta, double dt){
      return yi + (vx * sin(theta) + vy * cos(theta)) * dt;
    }

    //compute position based on velocity
    inline double computeNewThetaPosition(double thetai, double vth, double dt){
      return thetai + vth * dt;
    }

    //compute velocity based on acceleration
    inline double computeNewVelocity(double vg, double vi, double a_max, double dt){
      if(vg >= 0)
        return min(vg, vi + a_max * dt);
      return max(vg, vi - a_max * dt);
    }
    
};


#endif
