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
#include "trajectory_controller.h"

using namespace std;

TrajectoryController::TrajectoryController(MapGrid mg, vector<pair<int, int> > path, double acc_x, double acc_y, double acc_theta)
  : map_(mg), planned_path_(path), acc_x_(acc_x), acc_y_(acc_y), acc_theta_(acc_theta)
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
void TrajectoryController::generateTrajectory(double x, double y, double theta, double vx, double vy, 
    double vtheta, double num_steps, double sim_time){
  double x_i = x;
  double y_i = y;
  double theta_i = theta;
  double vx_i = vx;
  double vy_i = vy;
  double vtheta_i = vtheta;
  double dt = sim_time / num_steps;

  Trajectory traj(vx, vy, vtheta, num_steps);
  for(int i = 0; i < num_steps; ++i){
    traj.addPoint(i, TrajectoryPoint(i * dt, x_i, y_i, theta_i, vx_i, vy_i, vtheta_i));

    //calculate velocities
    vx_i = computeNewVelocity(vx, vx_i, acc_x_, dt);
    vy_i = computeNewVelocity(vy, vy_i, acc_y_, dt);
    vtheta_i = computeNewVelocity(vtheta, vtheta_i, acc_theta_, dt);

    //calculate positions
    x_i = computeNewPosition(x_i, vx_i, dt);
    y_i = computeNewPosition(y_i, vy_i, dt);
    theta_i = computeNewPosition(theta_i, vtheta_i, dt);
    
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
  return MAX(vg, vi + a_max * dt);
}


/*TODO: Implement this for real
void TrajectoryController::findPath(){
  //compute feasible velocity limits
  double max_vel_x = vel_x_ + acc_x_;
  double min_vel_x = vel_x_ - acc_x_;

  double max_vel_y = vel_y_ + acc_y_;
  double min_vel_y = vel_y_ - acc_y_;

  double max_vel_theta = vel_theta_ + acc_theta_;
  double min_vel_theta = vel_theta_ - acc_theta_;
}

void TrajectoryController::sampleSpace(){
  srand((unsigned) time(0));
  int random_int;
  int highest, lowest;

  for(int i = 0; i < num_samples; ++i){
    rand_int = lowest + int(range * rand() / RAND_MAX + 1.0);
  }
}
*/
