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
#include <utility>
#include <cstdlib>
#include <ctime>
#include <algorithm>

//for matrix support
#include <newmat10/newmat.h>
#include <newmat10/newmatio.h>

//For transform support
#include <rosTF/rosTF.h>

#include <trajectory_rollout/map_cell.h>
#include <trajectory_rollout/map_grid.h>
#include <trajectory_rollout/trajectory.h>

//we'll take in a path as a vector of points
#include <std_msgs/Position2DInt.h>

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
    TrajectoryController(MapGrid& mg, double sim_time, int num_steps, int samples_per_dim, rosTFClient* tf);
    
    //given the current state of the robot, find a good trajectory
    int findBestPath(libTF::TFPose2D global_pose, libTF::TFPose2D global_vel, libTF::TFPose2D global_acc);

    //compute the distance from each cell in the map grid to the planned path
    void computePathDistance();
    
    //given a trajectory in map space get the drive commands to send to the robot
    libTF::TFPose2D getDriveVelocities(Trajectory t);

    //create the trajectories we wish to score
    void createTrajectories(double x, double y, double theta, double vx, double vy, double vtheta, 
        double acc_x, double acc_y, double acc_theta);

    //create a trajectory given the current pose of the robot and selected velocities
    Trajectory generateTrajectory(int t_num, double x, double y, double theta, double vx, double vy, 
        double vtheta, double vx_samp, double vy_samp, double vtheta_samp, double acc_x, double acc_y,
        double acc_theta);

    void updatePlan(std::vector<std_msgs::Position2DInt>& new_plan);

    
    //possible trajectories for this run
    std::vector<Trajectory> trajectories_;

    //needed to convert between map and robot space efficiently
    NEWMAT::Matrix trajectory_pts_;
    NEWMAT::Matrix trajectory_theta_;

    //the number of trajectories we'll create
    int num_trajectories_;

    //the map passed on from the planner
    MapGrid& map_;
    
    //the number of points we generate for each trajectory
    int num_steps_;

    //the global plan for the robot to follow
    std::vector<std_msgs::Position2DInt> global_plan_;

  private:
    //compute the distance from an individual cell to the planned path
    void cellPathDistance(int cx, int cy, int dx, int dy);

    //update what map cells are considered path based on the global_plan
    void setPathCells();

    //update neighboring path distance
    void updateNeighbors(int cx, int cy);

    //convert the trajectories computed in robot space to world space
    void trajectoriesToWorld();

    //compute the cost for a single trajectory
    double trajectoryCost(int t_index, double pdist_scale, double gdist_scale, double dfast_scale);

    //compute position based on velocity
    double computeNewXPosition(double xi, double vx, double vy, double theta, double dt);
    double computeNewYPosition(double yi, double vx, double vy, double theta, double dt);
    double computeNewThetaPosition(double thetai, double vth, double dt);

    //compute velocity based on acceleration
    double computeNewVelocity(double vg, double vi, double amax, double dt);

    //the simulation parameters for generating trajectories
    double sim_time_;
    int samples_per_dim_;

    //transform client
    rosTFClient* tf_;
    
};


#endif
