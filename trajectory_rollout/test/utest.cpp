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
#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include <utility>
#include <trajectory_rollout/map_cell.h>
#include <trajectory_rollout/map_grid.h>
#include <trajectory_rollout/trajectory.h>
#include <trajectory_rollout/trajectory_controller.h>
#include <math.h>

#include <std_msgs/Point2DFloat32.h>


using namespace std;
using namespace std_msgs;

vector<Point2DFloat32> generatePlan(){
  //create a global plan
  vector<Point2DFloat32> path;
  Point2DFloat32 point;
  point.x = 0; point.y = 2;
  path.push_back(point);
  point.x = 1; point.y = 2;
  path.push_back(point);
  point.x = 1; point.y = 3;
  path.push_back(point);
  point.x = 1; point.y = 4;
  path.push_back(point);
  point.x = 2; point.y = 4;
  path.push_back(point);
  point.x = 3; point.y = 4;
  path.push_back(point);
  point.x = 3; point.y = 3;
  path.push_back(point);
  point.x = 3; point.y = 2;
  path.push_back(point);
  point.x = 3; point.y = 1;
  path.push_back(point);
  point.x = 4; point.y = 1;
  path.push_back(point);
  point.x = 5; point.y = 1;
  path.push_back(point);
  point.x = 5; point.y = 2;
  path.push_back(point);
  point.x = 5; point.y = 3;
  path.push_back(point);

  return path;
}

//make sure that we are getting the path distance map expected
TEST(TrajectoryController, correctPathDistance){
  MapGrid mg(6, 6);
  mg.scale = 1.0;
  //place some obstacles
  mg(2, 3).occ_state = 1;
  mg(3, 5).occ_state = 1;
  mg(4, 2).occ_state = 1;
  mg(5, 0).occ_state = 1;
  
  //create a trajectory_controller
  TrajectoryController tc(mg, 2.0, 20, 20, NULL);

  vector<Point2DFloat32> path = generatePlan();
  tc.updatePlan(path);

  tc.computePathDistance();

  //test enough of the 36 cell grid to be convinced
  EXPECT_FLOAT_EQ(tc.map_(0, 0).path_dist, 2.0);
  EXPECT_FLOAT_EQ(tc.map_(0, 1).path_dist, 1.0);
  EXPECT_FLOAT_EQ(tc.map_(4, 2).path_dist, DBL_MAX);
  EXPECT_FLOAT_EQ(tc.map_(0, 5).path_dist, sqrt(2));
  EXPECT_FLOAT_EQ(tc.map_(4, 0).path_dist, 1.0);
  EXPECT_FLOAT_EQ(tc.map_(3, 3).path_dist, 0.0);
  EXPECT_FLOAT_EQ(tc.map_(1, 0).path_dist, 2.0);
  EXPECT_FLOAT_EQ(tc.map_(5, 0).path_dist, DBL_MAX);

  //check for goal dist as well
  EXPECT_FLOAT_EQ(tc.map_(5, 3).goal_dist, 0.0);
  EXPECT_FLOAT_EQ(tc.map_(5, 2).goal_dist, 1.0);

  mg(5,5).occ_state = 1;

  tc.computePathDistance();

  EXPECT_FLOAT_EQ(tc.map_(5,5).path_dist, DBL_MAX);

  //print the results
  /*
  cout.precision(2);
  for(int k = tc.map_.rows_ - 1 ; k >= 0; --k){
    for(unsigned int m = 0; m < tc.map_.cols_; ++m){
      cout << tc.map_(k, m).path_dist << " | ";
    }
    cout << endl;
  }
  */
}

/* 
//convince ourselves that trajectories generate as expected
TEST(TrajectoryController, properIntegration){
  MapGrid mg(6, 6);

  //place some obstacles
  mg(2, 3).occ_state = 1;
  mg(3, 5).occ_state = 1;
  mg(4, 2).occ_state = 1;
  mg(5, 0).occ_state = 1;

  //create a trajectory_controller
  TrajectoryController tc(mg, 2.0, 20, 20, NULL);

  vector<Position2DInt> path = generatePlan();
  tc.updatePlan(path);

  tc.computePathDistance();

  Trajectory t1 = tc.generateTrajectory(0, 2, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1);

  int mat_index = tc.num_steps_ - 1;

  //check x integration fo position
  EXPECT_FLOAT_EQ(tc.trajectory_pts_.element(0, mat_index), 3.45);

  //check y integration fo position
  EXPECT_FLOAT_EQ(tc.trajectory_pts_.element(1, mat_index), 2.45);

  //check theta integration fo position and velocity
  EXPECT_FLOAT_EQ(tc.trajectory_theta_.element(0, mat_index), 1.45);

}
*/

//sanity check to make sure the grid functions correctly
TEST(MapGrid, properGridConstruction){
  MapGrid mg(10, 10);
  mg.scale = 1.0;
  MapCell mc;

  for(int i = 0; i < 10; ++i){
    for(int j = 0; j < 10; ++j){
      mc.cx = i;
      mc.cy = j;
      mg(i, j) = mc;
    }
  }

  for(int i = 0; i < 10; ++i){
    for(int j = 0; j < 10; ++j){
      EXPECT_FLOAT_EQ(mg(i, j).cx, i);
      EXPECT_FLOAT_EQ(mg(i, j).cy, j);
    }
  }
}

//test some stuff
int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
