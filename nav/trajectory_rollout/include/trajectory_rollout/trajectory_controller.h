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
#ifndef TRAJECTORY_ROLLOUT_TRAJECTORY_CONTROLLER_H_
#define TRAJECTORY_ROLLOUT_TRAJECTORY_CONTROLLER_H_

#include <vector>
#include <math.h>
#include <ros/console.h>

//for creating a local cost grid
#include <trajectory_rollout/map_cell.h>
#include <trajectory_rollout/map_grid.h>

//for obstacle data access
#include <costmap_2d/obstacle_map_accessor.h>
#include <trajectory_rollout/world_model.h>

#include <trajectory_rollout/trajectory.h>

//we'll take in a path as a vector of points
#include <deprecated_msgs/Point2DFloat32.h>
#include <trajectory_rollout/Position2DInt.h>

//for computing path distance
#include <queue>

//for some datatypes
#include <tf/transform_datatypes.h>

namespace trajectory_rollout {
  /**
   * @class TrajectoryController
   * @brief Computes control velocities for a robot given a costmap, a plan, and the robot's position in the world. 
   */
  class TrajectoryController{
    friend class TrajectoryControllerTest; //Need this for gtest to work
    public:
      /**
       * @brief  Constructs a trajectory controller
       * @param world_model The WorldModel the trajectory controller uses to check for collisions 
       * @param ma An ObstacleMapAccessor that allows the trajectory controller to query a costmap
       * @param footprint_spec A polygon representing the footprint of the robot. (Must be convex)
       * @param inscribed_radius The radius of the inscribed circle of the robot
       * @param circumscribed_radius The radius of the circumscribed circle of the robot
       * @param acc_lim_x The acceleration limit of the robot in the x direction
       * @param acc_lim_y The acceleration limit of the robot in the y direction
       * @param acc_lim_theta The acceleration limit of the robot in the theta direction
       * @param sim_time The number of seconds to "roll-out" each trajectory
       * @param sim_granularity The distance between simulation points should be small enough that the robot doesn't hit things
       * @param samples_per_dim The number of trajectories to sample in each dimension
       * @param pdist_scale A scaling factor for how close the robot should stay to the path
       * @param gdist_scale A scaling factor for how aggresively the robot should pursue a local goal
       * @param occdist_scale A scaling factor for how much the robot should prefer to stay away from obstacles
       * @param heading_lookahead How far the robot should look ahead of itself when differentiating between different rotational velocities
       * @param oscillation_reset_dist The distance the robot must travel before it can explore rotational velocities that were unsuccessful in the past
       * @param holonomic_robot Set this to true if the robot being controlled can take y velocities and false otherwise
       * @param max_vel_x The maximum x velocity the controller will explore
       * @param min_vel_x The minimum x velocity the controller will explore
       * @param max_vel_th The maximum rotational velocity the controller will explore
       * @param min_vel_th The minimum rotational velocity the controller will explore
       * @param min_in_place_vel_th The absolute value of the minimum in-place rotational velocity the controller will explore
       * @param y_vels A vector of the y velocities the controller will explore
       */
      TrajectoryController(WorldModel& world_model, 
          const costmap_2d::ObstacleMapAccessor& ma, 
          std::vector<deprecated_msgs::Point2DFloat32> footprint_spec,
          double inscribed_radius, double circumscribed_radius,
          double acc_lim_x = 1.0, double acc_lim_y = 1.0, double acc_lim_theta = 1.0,
          double sim_time = 1.0, double sim_granularity = 0.025, int samples_per_dim = 20, 
          double pdist_scale = 0.6, double gdist_scale = 0.8, double occdist_scale = 0.2,
          double heading_lookahead = 0.325, double oscillation_reset_dist = 0.05, 
          bool holonomic_robot = true,
          double max_vel_x = 0.5, double min_vel_x = 0.1, 
          double max_vel_th = 1.0, double min_vel_th = -1.0, double min_in_place_vel_th = 0.4,
          std::vector<double> y_vels = std::vector<double>(4));

      /**
       * @brief  Destructs a trajectory controller
       */
      ~TrajectoryController();

      /**
       * @brief  Given the current position, orientation, and velocity of the robot, return a trajectory to follow
       * @param global_pose The current pose of the robot in world space 
       * @param global_vel The current velocity of the robot in world space
       * @param drive_velocities Will be set to velocities to send to the robot base
       * @param observations A vector of updates from the robot's sensors in world space, is sometimes unused depending on the model
       * @param base_scan The latest base scan taken... used to clear freespace in front of the robot depending on the model
       * @return The selected path or trajectory
       */
      Trajectory findBestPath(tf::Stamped<tf::Pose> global_pose, tf::Stamped<tf::Pose> global_vel,
          tf::Stamped<tf::Pose>& drive_velocities, 
          std::vector<costmap_2d::Observation> observations = std::vector<costmap_2d::Observation>(0),
          PlanarLaserScan base_scan = PlanarLaserScan());

      /**
       * @brief  Update the plan that the controller is following
       * @param new_plan A new plan for the controller to follow 
       */
      void updatePlan(const std::vector<deprecated_msgs::Point2DFloat32>& new_plan);

      /**
       * @brief  Used for display purposes, allows the footprint of the robot to be drawn in visualization tools
       * @param x_i The x position of the robot
       * @param y_i The y position of the robot
       * @param theta_i The orientation of the robot
       * @return A vector of points in world coordinates that correspond to the verticies of the robot's footprint 
       */
      std::vector<deprecated_msgs::Point2DFloat32> drawFootprint(double x_i, double y_i, double theta_i);

      /**
       * @brief  Accessor for the goal the robot is currently pursuing in world corrdinates
       * @param x Will be set to the x position of the local goal 
       * @param y Will be set to the y position of the local goal 
       */
      void getLocalGoal(double& x, double& y);

    private:
      /**
       * @brief  Compute the distance from each cell in the local map grid to the planned path
       * @param dist_queue A queue of the initial cells on the path 
       */
      void computePathDistance(std::queue<MapCell*>& dist_queue);

      /**
       * @brief  Compute the distance from each cell in the local map grid to the local goal point
       * @param goal_queue A queue containing the local goal cell 
       */
      void computeGoalDistance(std::queue<MapCell*>& dist_queue);

      /**
       * @brief  Create the trajectories we wish to explore, score them, and return the best option
       * @param x The x position of the robot  
       * @param y The y position of the robot  
       * @param theta The orientation of the robot
       * @param vx The x velocity of the robot
       * @param vy The y velocity of the robot
       * @param vtheta The theta velocity of the robot
       * @param acc_x The x acceleration limit of the robot
       * @param acc_y The y acceleration limit of the robot
       * @param acc_theta The theta acceleration limit of the robot
       * @return 
       */
      Trajectory createTrajectories(double x, double y, double theta, double vx, double vy, double vtheta, 
          double acc_x, double acc_y, double acc_theta);

      /**
       * @brief  Generate and score a single trajectory
       * @param x The x position of the robot  
       * @param y The y position of the robot  
       * @param theta The orientation of the robot
       * @param vx The x velocity of the robot
       * @param vy The y velocity of the robot
       * @param vtheta The theta velocity of the robot
       * @param vx_samp The x velocity used to seed the trajectory
       * @param vy_samp The y velocity used to seed the trajectory
       * @param vtheta_samp The theta velocity used to seed the trajectory
       * @param acc_x The x acceleration limit of the robot
       * @param acc_y The y acceleration limit of the robot
       * @param acc_theta The theta acceleration limit of the robot
       * @param impossible_cost The cost value of a cell in the local map grid that is considered impassable
       * @param traj Will be set to the generated trajectory with its associated score 
       */
      void generateTrajectory(double x, double y, double theta, double vx, double vy, 
          double vtheta, double vx_samp, double vy_samp, double vtheta_samp, double acc_x, double acc_y,
          double acc_theta, double impossible_cost, Trajectory& traj);

      /**
       * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
       * @param x_i The x position of the robot 
       * @param y_i The y position of the robot 
       * @param theta_i The orientation of the robot
       * @return 
       */
      double footprintCost(double x_i, double y_i, double theta_i);

      /**
       * @brief  Used to get the cells that make up the footprint of the robot
       * @param x_i The x position of the robot
       * @param y_i The y position of the robot
       * @param theta_i The orientation of the robot
       * @param  fill If true: returns all cells in the footprint of the robot. If false: returns only the cells that make up the outline of the footprint.
       * @return The cells that make up either the outline or entire footprint of the robot depending on fill
       */
      std::vector<trajectory_rollout::Position2DInt> getFootprintCells(double x_i, double y_i, double theta_i, bool fill);

      /**
       * @brief  Use Bresenham's algorithm to trace a line between two points in a grid
       * @param  x0 The x coordinate of the first point
       * @param  x1 The x coordinate of the second point
       * @param  y0 The y coordinate of the first point
       * @param  y1 The y coordinate of the second point
       * @param  pts Will be filled with the cells that lie on the line in the grid
       */
      void getLineCells(int x0, int x1, int y0, int y1, std::vector<trajectory_rollout::Position2DInt>& pts);

      /**
       * @brief Fill the outline of a polygon, in this case the robot footprint, in a grid
       * @param footprint The list of cells making up the footprint in the grid, will be modified to include all cells inside the footprint
       */
      void getFillCells(std::vector<trajectory_rollout::Position2DInt>& footprint);

      /**
       * @brief Update what cells are considered path based on the global plan 
       */
      void setPathCells();

      MapGrid map_; ///< @brief The local map grid where we propagate goal and path distance 
      const costmap_2d::ObstacleMapAccessor& ma_; ///< @brief Provides access to cost map information
      WorldModel& world_model_; ///< @brief The world model that the controller uses for collision detection

      std::vector<deprecated_msgs::Point2DFloat32> footprint_spec_; ///< @brief The footprint specification of the robot

      double inscribed_radius_, circumscribed_radius_; ///< @brief The inscribed and circumscribed radii of the robot

      std::vector<deprecated_msgs::Point2DFloat32> global_plan_; ///< @brief The global path for the robot to follow

      bool stuck_left, stuck_right; ///< @brief Booleans to keep the robot from oscillating during rotation
      bool rotating_left, rotating_right; ///< @brief Booleans to keep track of the direction of rotation for the robot

      bool stuck_left_strafe, stuck_right_strafe; ///< @brief Booleans to keep the robot from oscillating during strafing
      bool strafe_right, strafe_left; ///< @brief Booleans to keep track of strafe direction for the robot

      double goal_x_,goal_y_; ///< @brief Storage for the local goal the robot is pursuing


      double sim_time_; ///< @brief The number of seconds each trajectory is "rolled-out"
      double sim_granularity_; ///< @brief The distance between simulation points

      int samples_per_dim_; ///< @brief The number of samples we'll take in each dimension of the control space

      double pdist_scale_, gdist_scale_, occdist_scale_; ///< @brief Scaling factors for the controller's cost function
      double acc_lim_x_, acc_lim_y_, acc_lim_theta_; ///< @brief The acceleration limits of the robot

      double prev_x_, prev_y_; ///< @brief Used to calculate the distance the robot has traveled before reseting oscillation booleans

      Trajectory traj_one, traj_two; ///< @brief Used for scoring trajectories

      double heading_lookahead_; ///< @brief How far the robot should look ahead of itself when differentiating between different rotational velocities
      double oscillation_reset_dist_; ///< @brief The distance the robot must travel before it can explore rotational velocities that were unsuccessful in the past
      bool holonomic_robot_; ///< @brief Is the robot holonomic or not? 
      
      double max_vel_x_, min_vel_x_, max_vel_th_, min_vel_th_, min_in_place_vel_th_; ///< @brief Velocity limits for the controller
      std::vector<double> y_vels_; ///< @brief Y velocities to explore

      /**
       * @brief  Used to update the distance of a cell in path distance computation
       * @param  current_cell The cell we're currently in 
       * @param  check_cell The cell to be updated
       */
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

      /**
       * @brief  Used to update the distance of a cell in goal distance computation
       * @param  current_cell The cell we're currently in 
       * @param  check_cell The cell to be updated
       */
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

      /**
       * @brief  Compute x position based on velocity
       * @param  xi The current x position
       * @param  vx The current x velocity
       * @param  vy The current y velocity
       * @param  theta The current orientation
       * @param  dt The timestep to take
       * @return The new x position 
       */
      inline double computeNewXPosition(double xi, double vx, double vy, double theta, double dt){
        return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
      }

      /**
       * @brief  Compute y position based on velocity
       * @param  yi The current y position
       * @param  vx The current x velocity
       * @param  vy The current y velocity
       * @param  theta The current orientation
       * @param  dt The timestep to take
       * @return The new y position 
       */
      inline double computeNewYPosition(double yi, double vx, double vy, double theta, double dt){
        return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
      }

      /**
       * @brief  Compute orientation based on velocity
       * @param  thetai The current orientation
       * @param  vth The current theta velocity
       * @param  dt The timestep to take
       * @return The new orientation
       */
      inline double computeNewThetaPosition(double thetai, double vth, double dt){
        return thetai + vth * dt;
      }

      //compute velocity based on acceleration
      /**
       * @brief  Compute velocity based on acceleration
       * @param vg The desired velocity, what we're accelerating up to 
       * @param vi The current velocity
       * @param a_max An acceleration limit
       * @param  dt The timestep to take
       * @return The new velocity
       */
      inline double computeNewVelocity(double vg, double vi, double a_max, double dt){
        if(vg >= 0)
          return std::min(vg, vi + a_max * dt);
        return std::max(vg, vi - a_max * dt);
      }
  };
};

#endif
