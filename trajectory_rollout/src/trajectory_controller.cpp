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

TrajectoryController::TrajectoryController(MapGrid& mg, double acc_x, double acc_y, double acc_theta,
    double sim_time, int num_steps, int samples_per_dim, rosTFClient* tf)
  : map_(mg), acc_x_(acc_x), acc_y_(acc_y), acc_theta_(acc_theta), sim_time_(sim_time),
  num_steps_(num_steps), samples_per_dim_(samples_per_dim), tf_(tf)
{
}

//compute the distance from each cell in the map grid to the planned path
void TrajectoryController::computePathDistance(){
  //two sweeps are needed to compute path distance for the grid

  //sweep bottom-top / left-right
  for(unsigned int i = 0; i < map_.rows_; ++i)
    for(unsigned int j = 0; j < map_.cols_; ++j)
      updateNeighbors(&map_(i, j));

  //sweep top-bottom / right-left
  for(int i = map_.rows_ - 1; i >= 0; --i)
    for(int j = map_.cols_ - 1; j >= 0; --j)
      updateNeighbors(&map_(i, j));

}

//update neighboring path distance
void TrajectoryController::updateNeighbors(MapCell* current){
  //update the 6 neighbors of the current cell
  for(int di = -1; di <= 1; ++di){
    for(int dj = -1; dj <= 1; ++dj){
      if(!(di == 0 && dj == 0))
        cellPathDistance(current, di, dj);
    }
  }
}

//compute the distance from an individual cell to the planned path
void TrajectoryController::cellPathDistance(MapCell* current, int di, int dj){
  int ni = current->ci + di;
  int nj = current->cj + dj;

  //determine whether to add 1 to distance or 1.5 (for corner neighbors)
  double new_dist = current->path_dist + sqrt( di * di + dj * dj);

  //make sure we are looking at a cell that exists
  if(!VALID_CELL(map_, ni, nj))
    return;

  //only modify the neighboring cell if its distance from the path will shrink
  //also, do nothing with cells that contain obstacles
  if(map_(ni, nj).occ_state == 1 || map_(ni, nj).path_dist < new_dist){
    return;
  }

  //update path distance
  map_(ni, nj).path_dist = new_dist;
}

//create a trajectory given the current pose of the robot and selected velocities
Trajectory TrajectoryController::generateTrajectory(double x, double y, double theta, double vx, double vy, 
    double vtheta, double vx_samp, double vy_samp, double vtheta_samp){
  double x_i = x;
  double y_i = y;
  double theta_i = theta;
  double vx_i = vx;
  double vy_i = vy;
  double vtheta_i = vtheta;
  double dt = sim_time_ / num_steps_;

  Trajectory traj(vx, vy, vtheta, num_steps_);
  for(int i = 0; i < num_steps_; ++i){
    traj.addPoint(i, TrajectoryPoint(i * dt, x_i, y_i, theta_i, vx_i, vy_i, vtheta_i));

    //calculate velocities
    vx_i = computeNewVelocity(vx_samp, vx_i, acc_x_, dt);
    vy_i = computeNewVelocity(vy_samp, vy_i, acc_y_, dt);
    vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta_, dt);

    //calculate positions
    x_i = computeNewPosition(x_i, vx_i, dt);
    y_i = computeNewPosition(y_i, vy_i, dt);
    theta_i = computeNewPosition(theta_i, vtheta_i, dt);
    
  }
  return traj;
}

//compute position based on velocity
double TrajectoryController::computeNewPosition(double xi, double v, double dt){
  return xi + v * dt;
}

//compute velocity based on acceleration
double TrajectoryController::computeNewVelocity(double vg, double vi, double a_max, double dt){
  if(vg >= 0)
    return MIN(vg, vi + a_max * dt);
  return MAX(vg, vi + a_max * dt);
}


//create the trajectories we wish to score
void TrajectoryController::createTrajectories(double x, double y, double theta, double vx, double vy, double vtheta){
  //compute feasible velocity limits in robot space
  libTF::TFPose2D max_vel;
  libTF::TFPose2D min_vel;

  max_vel.x = vx + acc_x_ * sim_time_;
  min_vel.x = vx - acc_x_ * sim_time_;

  max_vel.y = vy + acc_y_ * sim_time_;
  min_vel.y = vy - acc_y_ * sim_time_;

  max_vel.yaw = vtheta + acc_theta_ * sim_time_;
  min_vel.yaw = vtheta - acc_theta_ * sim_time_;

  max_vel.frame = "FRAMEID_ROBOT";
  min_vel.frame = "FRAMEID_ROBOT";

  //we could lack transforms in unit tests
  if(tf_){
    //convert our min and max velocities to map space
    max_vel = tf_->transformPose2D("FRAMEID_MAP", max_vel);
    min_vel = tf_->transformPose2D("FRAMEID_MAP", min_vel);
  }

  //we want to sample the velocity space regularly
  double dvx = (max_vel.x - min_vel.x) / samples_per_dim_;
  double dvy = (max_vel.y - min_vel.y) / samples_per_dim_;
  double dvtheta = (max_vel.yaw - min_vel.yaw) / samples_per_dim_;

  double vx_samp = min_vel.x;
  double vy_samp = min_vel.y;
  double vtheta_samp = min_vel.yaw;

  //generate trajectories for regularly sampled velocities
  for(int i = 0; i < samples_per_dim_; ++i){
    for(int j = 0; j < samples_per_dim_; ++j){
      for(int k = 0; k < samples_per_dim_; ++k){
        trajectories_.push_back(generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp));
        vtheta_samp += dvtheta;
      }
    vy_samp += dvy;
    }
    vx_samp += dvx;
  }
}

//given the current state of the robot, find a good trajectory
Trajectory TrajectoryController::findBestPath(libTF::TFPose2D robot_pose, libTF::TFPose2D robot_vel){
  //first compute the path distance for all cells in our map grid
  computePathDistance();

  //next create the trajectories we wish to explore
  createTrajectories(robot_pose.x, robot_pose.y, robot_pose.yaw, robot_vel.x, robot_vel.y, robot_vel.yaw);

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

  return best;
}

//compute the cost for a single trajectory
double TrajectoryController::trajectoryCost(Trajectory t, double pdist_scale, double gdist_scale, double dfast_scale){
  //we need to know in which cell the path ends
  pair<int, int> point_cell = getMapCell(t.points_.back().x_, t.points_.back().y_);

  //we don't want a path that ends off the known map
  if(!VALID_CELL(map_, point_cell.first, point_cell.second))
    return DBL_MAX;

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
  tVel.frame = "FRAMEID_MAP";

  //might not have a transform client with unit tests
  if(tf_)
    tVel = tf_->transformPose2D("FRAMEID_ROBOT", tVel);

  return tVel;
}
