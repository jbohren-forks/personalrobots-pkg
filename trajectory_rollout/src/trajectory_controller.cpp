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

namespace ublas = boost::numeric::ublas;
using namespace std;
using namespace std_msgs;

TrajectoryController::TrajectoryController(MapGrid& mg, double sim_time, int num_steps, int samples_per_dim,
    double robot_front_radius, double robot_side_radius, double max_occ_dist, double pdist_scale, double gdist_scale,
    double dfast_scale, double occdist_scale, double acc_lim_x, double acc_lim_y, double acc_lim_theta, rosTFClient* tf)
  : map_(mg), num_steps_(num_steps), sim_time_(sim_time), samples_per_dim_(samples_per_dim), robot_front_radius_(robot_front_radius),
  robot_side_radius_(robot_side_radius), max_occ_dist_(max_occ_dist), 
  pdist_scale_(pdist_scale), gdist_scale_(gdist_scale), dfast_scale_(dfast_scale), occdist_scale_(occdist_scale), 
  acc_lim_x_(acc_lim_x), acc_lim_y_(acc_lim_y), acc_lim_theta_(acc_lim_theta), tf_(tf)
{
  //regularly sample the forward velocity space... sample rotational vel space... sample backward vel space
  num_trajectories_ = samples_per_dim * samples_per_dim * samples_per_dim + samples_per_dim + samples_per_dim;
  int total_pts = num_trajectories_ * num_steps_;

  //even though we only have x,y we need to multiply by a 4x4 matrix
  trajectory_pts_ = ublas::zero_matrix<double>(4, total_pts);

  //storage for our theta values
  trajectory_theta_ = ublas::zero_matrix<double>(1, total_pts);
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

  unsigned int map_size = map_.map_.size();
  MapCell* current_cell, *check_cell;

  unsigned int last_col = map_.size_x_ - 1;
  for(unsigned int i = 0; i < map_size; ++i){
    current_cell = &(map_.map_[i]);
    check_cell = current_cell;

    //check to see if we are in the first col
    if(current_cell->cx > 0){
      --check_cell;
      updateCell(current_cell, check_cell);
    }

    //if we are in the first row we want to continue
    if(current_cell->cy == 0)
      continue;

    //check to see if we are in the first col
    if(current_cell->cx > 0){
      check_cell -= map_.size_x_;
      updateCell(current_cell, check_cell);
      ++check_cell;
    }
    else
      check_cell -= map_.size_x_;


    updateCell(current_cell, check_cell);

    //only check cell to bottom right if we're not in last col
    if(current_cell->cx < last_col){
      ++check_cell;
      updateCell(current_cell, check_cell);
    }
  }

  unsigned int end = map_size - 1;
  unsigned int last_row = map_.size_y_ - 1;
  for(int i = end; i >= 0; --i){
    current_cell = &(map_.map_[i]);
    check_cell = current_cell;

    //check to see if we are in the last col
    if(current_cell->cx < end){
      ++check_cell;
      updateCell(current_cell, check_cell);
    }

    //if we are in the last row we want to continue
    if(current_cell->cy == last_row)
      continue;

    //check to see if we are in the last col
    if(current_cell->cx < end){
      check_cell += map_.size_x_;
      updateCell(current_cell, check_cell);
      --check_cell;
    }
    else
      check_cell += map_.size_x_;


    updateCell(current_cell, check_cell);

    //only check cell to upper left if we're not in first col
    if(current_cell->cx > 0){
      ++check_cell;
      updateCell(current_cell, check_cell);
    }
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
    trajectory_pts_(0, mat_index) = x_i;
    trajectory_pts_(1, mat_index) = y_i;
    trajectory_pts_(2, mat_index) = 0;
    trajectory_pts_(3, mat_index) = 1;

    //add theta to the matrix
    trajectory_theta_(0, mat_index) = theta_i;

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
  libTF::TFPose2D robot_pose, global_pose;
  robot_pose.x = 0;
  robot_pose.y = 0;
  robot_pose.yaw = 0;
  robot_pose.frame = "base";
  robot_pose.time = 0;

  if(tf_){
    try
    {
      global_pose = tf_->transformPose2D("map", robot_pose);
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

  struct timeval start;
  struct timeval end;
  double start_t, end_t, t_diff;
  gettimeofday(&start,NULL);

  transformTrajects(global_pose.x, global_pose.y, global_pose.yaw);

  gettimeofday(&end,NULL);
  start_t = start.tv_sec + double(start.tv_usec) / 1e6;
  end_t = end.tv_sec + double(end.tv_usec) / 1e6;
  t_diff = end_t - start_t;
  fprintf(stderr, "Matrix Time: %.3f\n", t_diff);

}

//transform trajectories from robot space to map space
void TrajectoryController::transformTrajects(double x_i, double y_i, double th_i){
  double cos_th = cos(th_i);
  double sin_th = sin(th_i);
  printf("th: %.2f, cos: %.2f, sin: %.2f\n", th_i, cos_th, sin_th);

  double new_x, new_y, old_x, old_y;
  for(unsigned int i = 0; i < trajectory_pts_.size2(); ++i){
    old_x = trajectory_pts_(0, i);
    old_y = trajectory_pts_(1, i);
    new_x = x_i + old_x * cos_th - old_y * sin_th;
    new_y = y_i + old_x * sin_th + old_y * cos_th;
    trajectory_pts_(0, i) = new_x;
    trajectory_pts_(1, i) = new_y;
    trajectory_theta_(0, i) += th_i;
  }
}

//create the trajectories we wish to score
void TrajectoryController::createTrajectories(double x, double y, double theta, double vx, double vy, double vtheta,
    double acc_x, double acc_y, double acc_theta){
  //compute feasible velocity limits in robot space
  double max_vel_x, max_vel_y, max_vel_theta;
  double min_vel_x, min_vel_y, min_vel_theta;

  max_vel_x = min(1.0, vx + acc_x * sim_time_);
  min_vel_x = max(0.1, vx - acc_x * sim_time_);

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

  //next we want to generate trajectories for rotating in place
  vtheta_samp = min_vel_theta;
  vx_samp = 0.0;
  vy_samp = 0.0;
  for(int i = 0; i < samples_per_dim_; ++i){
    trajectories_.push_back(generateTrajectory(t_num, x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, acc_x, acc_y, acc_theta));
    ++t_num;
    vtheta_samp += dvtheta;
  }

  //and finally we want to generate trajectories that move backwards slowly
  vtheta_samp = min_vel_theta;
  vx_samp = -0.05;
  vy_samp = 0.0;
  for(int i = 0; i < samples_per_dim_; ++i){
    trajectories_.push_back(generateTrajectory(t_num, x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, acc_x, acc_y, acc_theta));
    ++t_num;
    vtheta_samp += dvtheta;
  }
  
}

//given the current state of the robot, find a good trajectory
int TrajectoryController::findBestPath(const ObstacleMapAccessor& ma, libTF::TFPose2D global_pose, libTF::TFPose2D global_vel, 
    libTF::TFPose2D& drive_velocities){
  //first compute the path distance for all cells in our map grid
  computePathDistance();
  //printf("Path distance computed\n");

  //next create the trajectories we wish to explore
  createTrajectories(global_pose.x, global_pose.y, global_pose.yaw, global_vel.x, global_vel.y, global_vel.yaw, 
      acc_lim_x_, acc_lim_y_, acc_lim_theta_);
  //printf("Trajectories created\n");

  //we need to transform the trajectories to world space for scoring
  trajectoriesToWorld();
  //printf("Trajectories converted\n");

  //now we want to score the trajectories that we've created and return the best one
  double min_cost = DBL_MAX;

  //default to a trajectory that goes nowhere
  int best_index = -1;

  //create a cirle containing the robot so we don't always have to lay down a footprint
  double safe_radius = sqrt(robot_front_radius_ * robot_front_radius_ + robot_side_radius_ * robot_side_radius_);

  //we know that everything except the last 2 sets of trajectories are in the forward direction
  unsigned int forward_traj_end = trajectories_.size() - 2 * samples_per_dim_;
  for(unsigned int i = 0; i < forward_traj_end; ++i){
    double cost = trajectoryCost(ma, i, pdist_scale_, gdist_scale_, occdist_scale_, dfast_scale_, safe_radius);

    //so we can draw with cost info
    trajectories_[i].cost_ = cost;

    //find the minimum cost path
    if(cost >= 0 && cost < min_cost){
      best_index = i;
      min_cost = cost;
    }
  }

  //if we have a valid path... return
  if(best_index >= 0){
    drive_velocities = getDriveVelocities(best_index);
    return best_index;
  }

  //the second to last set of trajectories is rotation only
  unsigned int rot_traj_end = trajectories_.size() - samples_per_dim_;
  for(unsigned int i = forward_traj_end; i < rot_traj_end; ++i){
    double cost = trajectoryCost(ma, i, pdist_scale_, gdist_scale_, occdist_scale_, dfast_scale_, safe_radius);

    //so we can draw with cost info
    trajectories_[i].cost_ = cost;

    //find the minimum cost path
    if(cost >= 0 && cost < min_cost){
      best_index = i;
      min_cost = cost;
    }
  }

  //if we have a valid path... return
  if(best_index >= 0){
    drive_velocities = getDriveVelocities(best_index);
    return best_index;
  }

  //the last set of trajectories is for moving backwards
  for(unsigned int i = rot_traj_end; i < trajectories_.size(); ++i){
    double cost = trajectoryCost(ma, i, pdist_scale_, gdist_scale_, occdist_scale_, dfast_scale_, safe_radius);

    //so we can draw with cost info
    trajectories_[i].cost_ = cost;

    //find the minimum cost path
    if(cost >= 0 && cost < min_cost){
      best_index = i;
      min_cost = cost;
    }
  }
  //printf("Trajectories scored\n");

  drive_velocities = getDriveVelocities(best_index);
  return best_index;
}

//compute the cost for a single trajectory
double TrajectoryController::trajectoryCost(const ObstacleMapAccessor& ma, int t_index, double pdist_scale,
    double gdist_scale, double occdist_scale, double dfast_scale, double safe_radius){
  Trajectory t = trajectories_[t_index];
  double path_dist = 0.0;
  double goal_dist = 0.0;
  double occ_dist = 0.0;
  for(int i = 0; i < num_steps_; ++i){
    int mat_index = t_index * num_steps_ + i;
    double x = trajectory_pts_(0, mat_index);
    double y = trajectory_pts_(1, mat_index);
    double theta = trajectory_theta_(0, mat_index);

    //we need to know in which cell the path ends
    int cell_x = WX_MX(map_, x);
    int cell_y = WY_MY(map_, y);

    //we don't want a path that ends off the known map or in an obstacle
    if(!VALID_CELL(map_, cell_x, cell_y) || ma.isOccupied(cell_x, cell_y)){
      return -1.0;
    }

    path_dist += map_(cell_x, cell_y).path_dist;
    goal_dist += map_(cell_x, cell_y).goal_dist;

    //first we decide if we need to lay down the footprint of the robot
    if(map_(cell_x, cell_y).occ_dist < safe_radius){
      //if we do compute the obstacle cost for each cell in the footprint
      double footprint_cost = footprintCost(ma, x, y, theta);
      if(footprint_cost < 0)
        return -1.0;
      occ_dist += footprint_cost;
    }
  }
  double cost = pdist_scale * path_dist + gdist_scale * goal_dist + dfast_scale * (1.0 / ((.05 + t.xv_) * (.05 + t.xv_))) + occdist_scale *  (1 / ((occ_dist + .05) * (occ_dist + .05)));
  
  return cost;
}

//given a trajectory in map space get the drive commands to send to the robot
libTF::TFPose2D TrajectoryController::getDriveVelocities(int t_num){
  libTF::TFPose2D tVel;
  //if no legal trajectory was found... stop the robot
  if(t_num < 0){
    tVel.x = 0.0;
    tVel.y = 0.0;
    tVel.yaw = 0.0;
    tVel.frame = "base";
    tVel.time = 0;
    return tVel;
  }

  //otherwise return the velocity cmds
  const Trajectory& t = trajectories_[t_num];
  tVel.x = t.xv_;
  tVel.y = t.yv_;
  tVel.yaw = t.thetav_;
  tVel.frame = "base";
  tVel.time = 0;
  return tVel;
}

void TrajectoryController::swap(int& a, int& b){
  int temp = a;
  a = b;
  b = temp;
}

double TrajectoryController::pointCost(const ObstacleMapAccessor& ma, int x, int y){
  //if the cell is in an obstacle the path is invalid
  if(ma.isOccupied(x, y)){
    return -1;
  }

  //check if we need to add an obstacle distance
  if(map_(x, y).occ_dist < max_occ_dist_)
    return map_(x, y).occ_dist;

  return 0.0;
}

//calculate the cost of a ray-traced line
double TrajectoryController::lineCost(const ObstacleMapAccessor& ma, int x0, int x1, int y0, int y1){
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
      point_cost = pointCost(ma, y, x);
    else
      point_cost = pointCost(ma, x, y);

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

//we need to take the footprint of the robot into account when we calculate cost to obstacles
double TrajectoryController::footprintCost(const ObstacleMapAccessor& ma, double x_i, double y_i, double theta_i){
  double cos_th = cos(theta_i);
  double sin_th = sin(theta_i);

  double footprint_dist = 0.0;
  double line_dist = -1.0;

  //upper right corner
  double old_x = 0.0 + robot_front_radius_;
  double old_y = 0.0 + robot_side_radius_;
  double new_x = x_i + old_x * cos_th - old_y * sin_th;
  double new_y = y_i + old_x * sin_th + old_y * cos_th;

  int x0 = WX_MX(map_, new_x);
  int y0 = WY_MY(map_, new_y);

  //lower right corner
  old_x = 0.0 + robot_front_radius_;
  old_y = 0.0 - robot_side_radius_;
  new_x = x_i + old_x * cos_th - old_y * sin_th;
  new_y = y_i + old_x * sin_th + old_y * cos_th;
  int x1 = WX_MX(map_, new_x);
  int y1 = WY_MY(map_, new_y);

  if(!VALID_CELL(map_, x0, y0) || !VALID_CELL(map_, x1, y1))
    return -1.0;

  //check the front line
  line_dist = lineCost(ma, x0, x1, y0, y1);
  if(line_dist < 0)
    return -1;

  footprint_dist += line_dist;

  //lower left corner
  old_x = 0.0 - robot_front_radius_;
  old_y = 0.0 - robot_side_radius_;
  new_x = x_i + old_x * cos_th - old_y * sin_th;
  new_y = y_i + old_x * sin_th + old_y * cos_th;
  int x2 = WX_MX(map_, new_x);
  int y2 = WY_MY(map_, new_y);

  if(!VALID_CELL(map_, x2, y2))
    return -1.0;

  //check the right side line
  line_dist = lineCost(ma, x1, x2, y1, y2);
  if(line_dist < 0)
    return -1;

  footprint_dist += line_dist;
  
  //upper left corner
  old_x = 0.0 - robot_front_radius_;
  old_y = 0.0 + robot_side_radius_;
  new_x = x_i + old_x * cos_th - old_y * sin_th;
  new_y = y_i + old_x * sin_th + old_y * cos_th;
  int x3 = WX_MX(map_, new_x);
  int y3 = WY_MY(map_, new_y);

  if(!VALID_CELL(map_, x3, y3))
    return -1.0;

  //check the back line
  line_dist = lineCost(ma, x2, x3, y2, y3);
  if(line_dist < 0)
    return -1;

  footprint_dist += line_dist;

  //check the left side line
  line_dist = lineCost(ma, x3, x0, y3, y0);
  if(line_dist < 0)
    return -1;

  footprint_dist += line_dist;

  return footprint_dist;
}

void TrajectoryController::drawLine(int x0, int x1, int y0, int y1, vector<std_msgs::Point2DFloat32>& pts){
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

  std_msgs::Point2DFloat32 pt;
  if(y0 < y1)
    ystep = 1;
  else
    ystep = -1;

  for(int x = x0; x <= x1; ++x){
    if(steep){
      pt.x = MX_WX(map_, y);
      pt.y = MY_WY(map_, x);
      pts.push_back(pt);
    }
    else{
      pt.x = MX_WX(map_, x);
      pt.y = MY_WY(map_, y);
      pts.push_back(pt);
    }

    error = error - delta_y;
    if(error < 0){
      y += ystep;
      error += delta_x;
    }
  }
}

//its nice to be able to draw a footprint for a particular point for debugging info
vector<std_msgs::Point2DFloat32> TrajectoryController::drawFootprint(double x_i, double y_i, double theta_i){
  vector<std_msgs::Point2DFloat32> footprint_pts;
  Point2DFloat32 pt;
  double cos_th = cos(theta_i);
  double sin_th = sin(theta_i);

  //upper right corner
  double old_x = 0.0 + robot_front_radius_;
  double old_y = 0.0 + robot_side_radius_;
  double new_x = x_i + old_x * cos_th - old_y * sin_th;
  double new_y = y_i + old_x * sin_th + old_y * cos_th;

  int x0 = WX_MX(map_, new_x);
  int y0 = WY_MY(map_, new_y);
  pt.x = MX_WX(map_, x0);
  pt.y = MY_WY(map_, y0);

  //lower right corner
  old_x = 0.0 + robot_front_radius_;
  old_y = 0.0 - robot_side_radius_;
  new_x = x_i + old_x * cos_th - old_y * sin_th;
  new_y = y_i + old_x * sin_th + old_y * cos_th;
  int x1 = WX_MX(map_, new_x);
  int y1 = WY_MY(map_, new_y);
  pt.x = MX_WX(map_, x1);
  pt.y = MY_WY(map_, y1);

  //check the front line
  drawLine(x0, x1, y0, y1, footprint_pts);

  //lower left corner
  old_x = 0.0 - robot_front_radius_;
  old_y = 0.0 - robot_side_radius_;
  new_x = x_i + old_x * cos_th - old_y * sin_th;
  new_y = y_i + old_x * sin_th + old_y * cos_th;
  int x2 = WX_MX(map_, new_x);
  int y2 = WY_MY(map_, new_y);
  pt.x = MX_WX(map_, x2);
  pt.y = MY_WY(map_, y2);

  //check the right side line
  drawLine(x1, x2, y1, y2, footprint_pts);
  
  //upper left corner
  old_x = 0.0 - robot_front_radius_;
  old_y = 0.0 + robot_side_radius_;
  new_x = x_i + old_x * cos_th - old_y * sin_th;
  new_y = y_i + old_x * sin_th + old_y * cos_th;
  int x3 = WX_MX(map_, new_x);
  int y3 = WY_MY(map_, new_y);
  pt.x = MX_WX(map_, x3);
  pt.y = MY_WY(map_, y3);

  //check the back line
  drawLine(x2, x3, y2, y3, footprint_pts);

  //check the left side line
  drawLine(x3, x0, y3, y0, footprint_pts);

  return footprint_pts;
}
