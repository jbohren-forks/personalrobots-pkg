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

//For transform support
#include <rosTF/rosTF.h>

#include <trajectory_rollout/map_cell.h>
#include <trajectory_rollout/map_grid.h>
#include <trajectory_rollout/trajectory.h>

#define MAX(x, y) x > y ? x : y
#define MIN(x, y) x < y ? x : y
#define VALID_CELL(map, i, j) ((i >= 0) && (i < (int)map.rows_) && (j >= 0) && (j < (int)map.cols_))

//Based on the plan from the path planner, determine what velocities to send to the robot
class TrajectoryController {
  public:
    //create a controller given a map, path, and acceleration limits of the robot
    TrajectoryController(MapGrid& mg, double acc_x, double acc_y, double acc_theta,
        double sim_time, int num_steps, int samples_per_dim, rosTFClient* tf);

    //compute the distance from each cell in the map grid to the planned path
    void computePathDistance();
    
    //compute the distance from an individual cell to the planned path
    void cellPathDistance(MapCell* current, int di, int dj);

    //update neighboring path distance
    void updateNeighbors(MapCell* current);

    //given the current state of the robot, find a good trajectory
    Trajectory findBestPath(libTF::TFPose2D robot_pose, libTF::TFPose2D robot_vel);

    //compute the cost for a single trajectory
    double trajectoryCost(Trajectory t, double pdist_scale, double gdist_scale, double dfast_scale);

    //given a position (in map space) return the containing cell
    std::pair<int, int> getMapCell(double x, double y);

    //create the trajectories we wish to score
    void createTrajectories(double x, double y, double theta, double vx, double vy, double vtheta);

    //create a trajectory given the current pose of the robot and selected velocities
    Trajectory generateTrajectory(double x, double y, double theta, double vx, double vy, 
        double vtheta, double vx_samp, double vy_samp, double vtheta_samp);

    //given a trajectory in map space get the drive commands to send to the robot
    libTF::TFPose2D getDriveVelocities(Trajectory t);

    //compute position based on velocity
    double computeNewPosition(double xi, double v, double dt);

    //compute velocity based on acceleration
    double computeNewVelocity(double vg, double vi, double amax, double dt);
    
    //the map passed on from the planner
    MapGrid& map_;

    //the acceleration limits of the robot
    double acc_x_, acc_y_, acc_theta_;

    //the simulation parameters for generating trajectories
    double sim_time_;
    int num_steps_, samples_per_dim_;

    //possible trajectories for this run
    std::vector<Trajectory> trajectories_;

    //transform client
    rosTFClient* tf_;
    
};


#endif
