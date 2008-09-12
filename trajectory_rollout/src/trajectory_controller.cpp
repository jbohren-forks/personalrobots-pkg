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

TrajectoryController::TrajectoryController(MapGrid& mg, double sim_time, int num_steps, int samples_per_dim, rosTFClient* tf)
  : map_(mg), sim_time_(sim_time), num_steps_(num_steps), samples_per_dim_(samples_per_dim), tf_(tf)
{
}

//compute the distance from each cell in the map grid to the planned path
void TrajectoryController::computePathDistance(){
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
  //update the 6 neighbors of the current cell
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

  //determine whether to add 1 to distance or sqrt(2) (for corner neighbors)
  double new_dist = map_(cx, cy).path_dist + sqrt( dx * dx + dy * dy);


  //make sure we are looking at a cell that exists
  if(!VALID_CELL(map_, nx, ny))
    return;

  //only modify the neighboring cell if its distance from the path will shrink
  //also, do nothing with cells that contain obstacles
  if(map_(nx, ny).occ_state == 1 || map_(nx, ny).path_dist < new_dist){
    return;
  }

  //update path distance
  map_(nx, ny).path_dist = new_dist;
}

//create a trajectory given the current pose of the robot and selected velocities
Trajectory TrajectoryController::generateTrajectory(double x, double y, double theta, double vx, double vy, 
    double vtheta, double vx_samp, double vy_samp, double vtheta_samp, double acc_x, double acc_y, double acc_theta){
  double x_i = x;
  double y_i = y;
  double theta_i = theta;
  double vx_i = vx;
  double vy_i = vy;
  double vtheta_i = vtheta;
  double dt = sim_time_ / num_steps_;

  Trajectory traj(vx_samp, vy_samp, vtheta_samp, num_steps_);

  for(int i = 0; i < num_steps_; ++i){
    traj.addPoint(i, TrajectoryPoint(i * dt, x_i, y_i, theta_i, vx_i, vy_i, vtheta_i));

    //calculate velocities
    vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
    vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
    vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

    //calculate positions
    x_i = computeNewPosition(x_i, vx_i, dt);
    y_i = computeNewPosition(y_i, vy_i, dt);
    theta_i = computeNewPosition(theta_i, vtheta_i, dt);
    
  }
  
  return traj;
}

//convert the trajectories computed in robot space to world space
//TODO: This is WAYYY SLOW need to change this to a matrix multiplication
void TrajectoryController::trajectoriesToWorld(){
  for(unsigned int i = 0; i < trajectories_.size(); ++i){
    for(unsigned int j = 0; j < trajectories_[i].points_.size(); ++j){
      libTF::TFPose2D pos;
      libTF::TFPose2D vel;
      pos.x = trajectories_[i].points_[j].x_;
      vel.x = trajectories_[i].points_[j].xv_;

      pos.y = trajectories_[i].points_[j].y_;
      vel.y = trajectories_[i].points_[j].yv_;

      pos.yaw = trajectories_[i].points_[j].theta_;
      vel.yaw = trajectories_[i].points_[j].thetav_;
      
      pos.frame = "FRAMEID_ROBOT";
      vel.frame = "FRAMEID_ROBOT";

      pos.time = 0;
      vel.time = 0;

      if(tf_){
        try
        {
          //convert the trajectory point to map space
          pos = tf_->transformPose2D("FRAMEID_MAP", pos);
          vel = tf_->transformPose2D("FRAMEID_MAP", vel);
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

      trajectories_[i].points_[j].x_ = pos.x;
      trajectories_[i].points_[j].xv_ = vel.x;

      trajectories_[i].points_[j].y_ = pos.y;
      trajectories_[i].points_[j].yv_ = vel.y;

      trajectories_[i].points_[j].theta_ = pos.yaw;
      trajectories_[i].points_[j].thetav_ = vel.yaw;
    }
  }
}

//compute position based on velocity
double TrajectoryController::computeNewPosition(double xi, double v, double dt){
  return xi + v * dt;
}

//compute velocity based on acceleration
double TrajectoryController::computeNewVelocity(double vg, double vi, double a_max, double dt){
  if(vg >= 0)
    return MIN(vg, vi + a_max * dt);
  return MAX(vg, vi - a_max * dt);
}


//create the trajectories we wish to score
void TrajectoryController::createTrajectories(double x, double y, double theta, double vx, double vy, double vtheta,
    double acc_x, double acc_y, double acc_theta){
  //compute feasible velocity limits in robot space
  double max_vel_x, max_vel_y, max_vel_theta;
  double min_vel_x, min_vel_y, min_vel_theta;

  max_vel_x = vx + acc_x * sim_time_;
  min_vel_x = vx - acc_x * sim_time_;

  max_vel_y = vy + acc_y * sim_time_;
  min_vel_y = vy - acc_y * sim_time_;

  max_vel_theta = vtheta + acc_theta * sim_time_;
  min_vel_theta = vtheta - acc_theta * sim_time_;

  //we want to sample the velocity space regularly
  double dvx = (max_vel_x - min_vel_x) / samples_per_dim_;
  double dvy = (max_vel_y - min_vel_y) / samples_per_dim_;
  double dvtheta = (max_vel_theta - min_vel_theta) / samples_per_dim_;

  double vx_samp = min_vel_x;
  double vy_samp = min_vel_y;
  double vtheta_samp = min_vel_theta;

  //make sure to reset the list of trajectories
  trajectories_.clear();

  //generate trajectories for regularly sampled velocities
  for(int i = 0; i < samples_per_dim_; ++i){
    for(int j = 0; j < samples_per_dim_; ++j){
      for(int k = 0; k < samples_per_dim_; ++k){
        trajectories_.push_back(generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, acc_x, acc_y, acc_theta));
        vtheta_samp += dvtheta;
      }
    vy_samp += dvy;
    }
    vx_samp += dvx;
  }
}

//given the current state of the robot, find a good trajectory
Trajectory TrajectoryController::findBestPath(libTF::TFPose2D global_pose, libTF::TFPose2D global_vel, libTF::TFPose2D global_acc){
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
  Trajectory best(0, 0, 0, 1);

  for(unsigned int i = 0; i < trajectories_.size(); ++i){
    double cost = trajectoryCost(trajectories_[i], .33, .33, .33);

    //so we can draw with cost info
    trajectories_[i].cost_ = cost;

    //find the minimum cost path
    if(cost < min_cost){
      best = trajectories_[i];
      min_cost = cost;
    }
  }
  printf("Trajectories scored\n");

  return best;
}

//compute the cost for a single trajectory
double TrajectoryController::trajectoryCost(Trajectory t, double pdist_scale, double gdist_scale, double dfast_scale){
  //we need to know in which cell the path ends
  pair<int, int> point_cell = getMapCell(t.points_.back().x_, t.points_.back().y_);

  //we don't want a path that ends off the known map
  if(!VALID_CELL(map_, point_cell.first, point_cell.second)){
    return DBL_MAX;
  }

  double path_dist = map_(point_cell.first, point_cell.second).path_dist;
  double goal_dist = map_(point_cell.first, point_cell.second).goal_dist;

  double cost = pdist_scale * path_dist + gdist_scale * goal_dist + dfast_scale * (1.0 / (t.xv_ * t.xv_ + t.yv_ * t.yv_));
  
  return cost;
}

//given a position (in map space) return the containing cell
pair<int, int> TrajectoryController::getMapCell(double x, double y){
  //size of each cell meters/cell
  double scale = map_.scale;

  int i = (int)(x / scale);
  int j = (int)(y / scale);

  return pair<int, int>(i, j);
}

//given a trajectory in map space get the drive commands to send to the robot
libTF::TFPose2D TrajectoryController::getDriveVelocities(Trajectory t){
  libTF::TFPose2D tVel;
  tVel.x = t.xv_;
  tVel.y = t.yv_;
  tVel.yaw = t.thetav_;
  tVel.frame = "FRAMEID_ROBOT";
  tVel.time = 0;

  /*
  //might not have a transform client with unit tests
  if(tf_){
    try
    {
      tVel = tf_->transformPose2D("FRAMEID_ROBOT", tVel);
    }
    catch(libTF::TransformReference::LookupException& ex)
    {
      puts("no global->local Tx yet");
      printf("%s\n", ex.what());
      return libTF::TFPose2D();
    }
    catch(libTF::TransformReference::ConnectivityException& ex)
    {
      puts("no global->local Tx yet");
      printf("%s\n", ex.what());
      return libTF::TFPose2D();
    }
    catch(libTF::TransformReference::ExtrapolateException& ex)
    {
      //      puts("extrapolation required");
      //      printf("%s\n", ex.what());
      return libTF::TFPose2D();
    }
  }
  */

  return tVel;
}
