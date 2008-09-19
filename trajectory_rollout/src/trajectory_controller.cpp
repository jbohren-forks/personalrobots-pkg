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
#include <trajectory_rollout/trajectory_controller.h>

using namespace std;
using namespace std_msgs;

TrajectoryController::TrajectoryController(MapGrid& mg, double sim_time, int num_steps, int samples_per_dim,
    double robot_front_radius, double robot_side_radius, double max_occ_dist, rosTFClient* tf)
  : map_(mg), num_steps_(num_steps), sim_time_(sim_time), samples_per_dim_(samples_per_dim), robot_front_radius_(robot_front_radius),
  robot_side_radius_(robot_side_radius), max_occ_dist_(max_occ_dist), tf_(tf)
{
  num_trajectories_ = samples_per_dim * samples_per_dim * samples_per_dim;
  int total_pts = num_trajectories_ * num_steps_;

  //even though we only have x,y we need to multiply by a 4x4 matrix
  trajectory_pts_ = NEWMAT::Matrix(4, total_pts);
  trajectory_pts_ = 0.0;

  //storage for our theta values
  trajectory_theta_ = NEWMAT::Matrix(1, total_pts);
  trajectory_theta_ = 0.0;
}

//update what map cells are considered path based on the global_plan
void TrajectoryController::setPathCells(){
  map_.resetPathDist();
  int local_goal_x = -1;
  int local_goal_y = -1;
  bool started_path = false;
  for(unsigned int i = 0; i < global_plan_.size(); ++i){
    int map_x = WX_MX(map_, global_plan_[i].x);
    int map_y = WY_MY(map_, global_plan_[i].y);
    if(VALID_CELL(map_, map_x, map_y)){
      map_(map_x, map_y).path_dist = 0.0;
      local_goal_x = map_x;
      local_goal_y = map_y;
      started_path = true;
    }
    else{
      if(started_path)
        break;
    }
  }

  if(local_goal_x >= 0 && local_goal_y >= 0){
    map_(local_goal_x, local_goal_y).goal_dist = 0.0;
  }
}

//compute the distance from each cell in the map grid to the planned path
void TrajectoryController::computePathDistance(){
  //make sure that we update our path based on the global plan
  setPathCells();

  //two sweeps are needed to compute path distance for the grid

  //sweep bottom-top / left-right
  for(unsigned int i = 0; i < map_.size_x_; ++i)
    for(unsigned int j = 0; j < map_.size_y_; ++j)
      updateNeighbors(i, j);

  //sweep top-bottom / right-left
  for(int i = map_.size_x_ - 1; i >= 0; --i)
    for(int j = map_.size_y_ - 1; j >= 0; --j)
      updateNeighbors(i, j);

}

//update neighboring path distance
void TrajectoryController::updateNeighbors(int cx, int cy){
  //update the 8 neighbors of the current cell
  for(int dx = -1; dx <= 1; ++dx){
    for(int dy = -1; dy <= 1; ++dy){
      if(!(dx == 0 && dy == 0))
        cellPathDistance(cx, cy, dx, dy);
    }
  }
}

//compute the distance from an individual cell to the planned path
void TrajectoryController::cellPathDistance(int cx, int cy, int dx, int dy){
  int nx = cx + dx;
  int ny = cy + dy;

  double new_path_dist, new_goal_dist;

  MapCell& current_cell = map_(cx, cy);

  //determine whether to add 1 to distance or sqrt(2) (for corner neighbors)
  if(dx && dy){
    new_path_dist = current_cell.path_dist + 1.41;
    new_goal_dist = current_cell.goal_dist + 1.41;
  }
  else{
    new_path_dist = current_cell.path_dist + 1;
    new_goal_dist = current_cell.goal_dist + 1;
  }


  //make sure we are looking at a cell that exists
  if(!VALID_CELL(map_, nx, ny))
    return;

  //update path distance
  MapCell& new_cell = map_(nx, ny);
  if(new_path_dist < new_cell.path_dist){
    new_cell.path_dist = new_path_dist;
  }
  
  //update goal distance
  if(new_goal_dist < new_cell.goal_dist){
    new_cell.goal_dist = new_goal_dist;
  }
}

//create a trajectory given the current pose of the robot and selected velocities
Trajectory TrajectoryController::generateTrajectory(int t_num, double x, double y, double theta, double vx, double vy, 
    double vtheta, double vx_samp, double vy_samp, double vtheta_samp, double acc_x, double acc_y, double acc_theta){
  double x_i = x;
  double y_i = y;
  double theta_i = theta;
  double vx_i = vx;
  double vy_i = vy;
  double vtheta_i = vtheta;
  double dt = sim_time_ / num_steps_;

  //get the index for this trajectory in the matricies
  int mat_index = t_num * num_steps_;

  Trajectory traj(vx_samp, vy_samp, vtheta_samp);

  for(int i = 0; i < num_steps_; ++i){
    //add the point to the matrix
    trajectory_pts_.element(0, mat_index) = x_i;
    trajectory_pts_.element(1, mat_index) = y_i;
    trajectory_pts_.element(2, mat_index) = 0;
    trajectory_pts_.element(3, mat_index) = 1;

    //add theta to the matrix
    trajectory_theta_.element(0, mat_index) = theta_i;

    ++mat_index;

    //calculate velocities
    vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
    vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
    vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

    //calculate positions
    x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
    y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
    theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);
    
  }
  
  return traj;
}

void TrajectoryController::updatePlan(vector<Point2DFloat32>& new_plan){
  global_plan_.resize(new_plan.size());
  for(unsigned int i = 0; i < new_plan.size(); ++i){
    global_plan_[i] = new_plan[i];
  }
}

void TrajectoryController::trajectoriesToWorld(){
  NEWMAT::Matrix transform;

  if(tf_){
    try
    {
      transform = tf_->getMatrix("FRAMEID_MAP", "FRAMEID_ROBOT", 0);
    }
    catch(libTF::TransformReference::LookupException& ex)
    {
      puts("no global->local Tx yet");
      printf("%s\n", ex.what());
      return;
    }
    catch(libTF::TransformReference::ConnectivityException& ex)
    {
      puts("no global->local Tx yet");
      printf("%s\n", ex.what());
      return;
    }
    catch(libTF::TransformReference::ExtrapolateException& ex)
    {
      //      puts("extrapolation required");
      //      printf("%s\n", ex.what());
      return;
    }
  }
  trajectory_pts_ = transform * trajectory_pts_;

  //next transform theta
  NEWMAT::Matrix unit_vec_robot(4, 1);
  unit_vec_robot << 1.0 << 0.0 << 0.0 << 1.0;

  NEWMAT::Matrix unit_vec_map;
  unit_vec_map = transform * unit_vec_robot;
  
  //compute the angle between the two vectors
  double angle = atan2(unit_vec_map.element(0, 0) - unit_vec_robot.element(0, 0),
      unit_vec_map.element(1, 0) - unit_vec_robot.element(1,0));

  //add the angle to theta values to apply the rotation
  trajectory_theta_ = trajectory_theta_ + angle;

}


//compute position based on velocity
double TrajectoryController::computeNewXPosition(double xi, double vx, double vy, double theta, double dt){
  return xi + (vx * cos(theta) + vy * sin(theta)) * dt;
}

//compute position based on velocity
double TrajectoryController::computeNewYPosition(double yi, double vx, double vy, double theta, double dt){
  return yi + (vx * sin(theta) + vy * cos(theta)) * dt;
}

//compute position based on velocity
double TrajectoryController::computeNewThetaPosition(double thetai, double vth, double dt){
  return thetai + vth * dt;
}

//compute velocity based on acceleration
double TrajectoryController::computeNewVelocity(double vg, double vi, double a_max, double dt){
  if(vg >= 0)
    return min(vg, vi + a_max * dt);
  return max(vg, vi - a_max * dt);
}

//create the trajectories we wish to score
void TrajectoryController::createTrajectories(double x, double y, double theta, double vx, double vy, double vtheta,
    double acc_x, double acc_y, double acc_theta){
  //compute feasible velocity limits in robot space
  double max_vel_x, max_vel_y, max_vel_theta;
  double min_vel_x, min_vel_y, min_vel_theta;

  max_vel_x = min(1.0, vx + acc_x * sim_time_);
  //min_vel_x = vx - acc_x * sim_time_;
  min_vel_x = 0.0;

  max_vel_y = vy + acc_y * sim_time_;
  min_vel_y = vy - acc_y * sim_time_;

  //max_vel_theta = vtheta + acc_theta * sim_time_;
  //min_vel_theta = vtheta - acc_theta * sim_time_;

  max_vel_theta = 1.0;
  min_vel_theta = -1.0;

  //we want to sample the velocity space regularly
  double dvx = (max_vel_x - min_vel_x) / samples_per_dim_;
  double dvy = (max_vel_y - min_vel_y) / samples_per_dim_;
  double dvtheta = (max_vel_theta - min_vel_theta) / samples_per_dim_;

  double vx_samp = min_vel_x;
  double vtheta_samp = min_vel_theta;
  //double vy_samp = min_vel_y;
  double vy_samp = 0.0;

  //make sure to reset the list of trajectories
  trajectories_.clear();

  //keep track of which trajectory we're working on
  int t_num = 0;

  //generate trajectories for regularly sampled velocities
  for(int i = 0; i < samples_per_dim_; ++i){
    //vy_samp = min_vel_y;
    vtheta_samp = min_vel_theta;
    for(int j = 0; j < samples_per_dim_; ++j){
      //vy_samp = min_vel_y;
      //for(int k = 0; k < samples_per_dim_; ++k){
        trajectories_.push_back(generateTrajectory(t_num, x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, acc_x, acc_y, acc_theta));
        ++t_num;
        //vy_samp += dvy;
      //}
      vtheta_samp += dvtheta;
    }
    vx_samp += dvx;
  }
}

//given the current state of the robot, find a good trajectory
int TrajectoryController::findBestPath(libTF::TFPose2D global_pose, libTF::TFPose2D global_vel, libTF::TFPose2D global_acc){
  //first compute the path distance for all cells in our map grid
  computePathDistance();
  printf("Path distance computed\n");

  //next create the trajectories we wish to explore
  createTrajectories(global_pose.x, global_pose.y, global_pose.yaw, global_vel.x, global_vel.y, global_vel.yaw, 
      global_acc.x, global_acc.y, global_acc.yaw);
  printf("Trajectories created\n");

  //we need to transform the trajectories to world space for scoring
  trajectoriesToWorld();
  printf("Trajectories converted\n");

  //now we want to score the trajectories that we've created and return the best one
  double min_cost = DBL_MAX;

  //default to a trajectory that goes nowhere
  int best_index = -1;

  for(unsigned int i = 0; i < trajectories_.size(); ++i){
    double cost = trajectoryCost(i, .40, .10, .40, .10);

    //so we can draw with cost info
    trajectories_[i].cost_ = cost;

    //find the minimum cost path
    if(cost >= 0 && cost < min_cost){
      best_index = i;
      min_cost = cost;
    }
  }
  printf("Trajectories scored\n");

  return best_index;
}

//compute the cost for a single trajectory
double TrajectoryController::trajectoryCost(int t_index, double pdist_scale, double gdist_scale, double occdist_scale, double dfast_scale){
  Trajectory t = trajectories_[t_index];
  double path_dist = 0.0;
  double goal_dist = 0.0;
  double occ_dist = 0.0;
  for(int i = 0; i < num_steps_; ++i){
    int mat_index = t_index * num_steps_ + i;
    double x = trajectory_pts_.element(0, mat_index);
    double y = trajectory_pts_.element(1, mat_index);
    double theta = trajectory_theta_.element(0, mat_index);

    //we need to know in which cell the path ends
    int cell_x = WX_MX(map_, x);
    int cell_y = WY_MY(map_, y);

    //we don't want a path that ends off the known map or in an obstacle
    if(!VALID_CELL(map_, cell_x, cell_y) || map_(cell_x, cell_y).occ_state == 1){
      return -1.0;
    }

    path_dist += map_(cell_x, cell_y).path_dist;
    goal_dist += map_(cell_x, cell_y).goal_dist;

    //first we decide if we need to lay down the footprint of the robot
    if(map_(cell_x, cell_y).occ_dist < 2 * robot_front_radius_ + max_occ_dist_){
      //if we do compute the obstacle cost for each cell in the footprint
      double footprint_cost = footprintCost(x, y, theta);
      if(footprint_cost < 0)
        return -1.0;
      occ_dist += footprint_cost;
    }
  }
  double cost = pdist_scale * path_dist + gdist_scale * goal_dist + dfast_scale * (1.0 / ((.05 + t.xv_) * (.05 + t.xv_))) + occdist_scale *  (1 / (occ_dist + .05));
  //double cost = goal_dist;
  
  return cost;
}

//given a trajectory in map space get the drive commands to send to the robot
libTF::TFPose2D TrajectoryController::getDriveVelocities(Trajectory t){
  libTF::TFPose2D tVel;
  tVel.x = t.xv_;
  tVel.y = t.yv_;
  tVel.yaw = t.thetav_;
  tVel.frame = "FRAMEID_ROBOT";
  tVel.time = 0;
  return tVel;
}

void TrajectoryController::swap(int& a, int& b){
  int temp = a;
  a = b;
  b = temp;
}

double TrajectoryController::pointCost(int x, int y){
  //if the cell is in an obstacle the path is invalid
  if(map_(x, y).occ_state == 1){
    return -1;
  }

  //check if we need to add an obstacle distance
  if(map_(x, y).occ_dist < max_occ_dist_)
    return map_(x, y).occ_dist;

  return 0.0;
}

double TrajectoryController::lineCost(int x0, int x1, int y0, int y1){
  bool steep = abs(y1 - y0) > abs(x1 - x0);
  if(steep){
    swap(x0, y0);
    swap(x1, y1);
  }
  if(x0 > x1){
    swap(x0, x1);
    swap(y0, y1);
  }

  int delta_x = x1 - x0;
  int delta_y = abs(y1 - y0);
  int error = delta_x / 2;
  int ystep;
  int y = y0;

  double line_cost = 0.0;
  double point_cost = -1.0;

  if(y0 < y1)
    ystep = 1;
  else
    ystep = -1;

  for(int x = x0; x <= x1; ++x){
    if(steep)
      point_cost = pointCost(y, x);
    else
      point_cost = pointCost(x, y);

    if(point_cost < 0)
      return -1;

    line_cost += point_cost;

    error = error - delta_y;
    if(error < 0){
      y += ystep;
      error += delta_x;
    }
  }
  return line_cost;
}

double TrajectoryController::footprintCost(double x, double y, double theta){
  double x_diff = robot_front_radius_ * cos(theta) + robot_side_radius_ * sin(theta);
  double y_diff = robot_front_radius_ * sin(theta) + robot_side_radius_ * cos(theta);

  double footprint_dist = 0.0;
  double line_dist = -1.0;

  //upper right corner
  int x0 = WX_MX(map_, x + x_diff);
  int y0 = WY_MY(map_, y + y_diff);

  //lower right corner
  int x1 = WX_MX(map_, x + x_diff);
  int y1 = WY_MY(map_, y - y_diff);

  if(!VALID_CELL(map_, x0, y0) || !VALID_CELL(map_, x1, y1))
    return -1.0;

  //check the front line
  line_dist = lineCost(x0, x1, y0, y1);
  if(line_dist < 0)
    return -1;

  footprint_dist += line_dist;

  //lower left corner
  int x2 = WX_MX(map_, x - x_diff);
  int y2 = WY_MY(map_, y - y_diff);

  if(!VALID_CELL(map_, x2, y2))
    return -1.0;

  //check the right side line
  line_dist = lineCost(x1, x2, y1, y2);
  if(line_dist < 0)
    return -1;

  footprint_dist += line_dist;
  
  //upper left corner
  int x3 = WX_MX(map_, x - x_diff);
  int y3 = WY_MY(map_, y + y_diff);

  if(!VALID_CELL(map_, x3, y3))
    return -1.0;

  //check the back line
  line_dist = lineCost(x2, x3, y2, y3);
  if(line_dist < 0)
    return -1;

  footprint_dist += line_dist;

  //check the left side line
  line_dist = lineCost(x3, x0, y3, y0);
  if(line_dist < 0)
    return -1;

  footprint_dist += line_dist;

  return footprint_dist;
}
