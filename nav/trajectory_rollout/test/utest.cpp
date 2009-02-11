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
#include <costmap_2d/obstacle_map_accessor.h>
#include <trajectory_rollout/governor_node.h>
#include <math.h>

#include <std_msgs/Point2DFloat32.h>
#include <trajectory_rollout/Position2DInt.h>


using namespace std;
using namespace std_msgs;
using namespace trajectory_rollout;

namespace trajectory_rollout {
  class TrajectoryControllerTest : public testing::Test {
    public:
    TrajectoryControllerTest(MapGrid& g, WavefrontMapAccessor& wave, std::vector<deprecated_msgs::Point2DFloat32> footprint_spec);
    void correctFootprint();
    void footprintObstacles();
    void checkGoalDistance();
    void checkPathDistance();
    virtual void TestBody(){}

    MapGrid& map_;
    WavefrontMapAccessor& wa;
    CostmapModel cm;
    TrajectoryController tc;
  };

  TrajectoryControllerTest::TrajectoryControllerTest(MapGrid& g, WavefrontMapAccessor& wave, std::vector<deprecated_msgs::Point2DFloat32> footprint_spec) 
    : map_(g), wa(wave), cm(wa), tc(cm, wa, footprint_spec, 0.0, 1.0, 1.0, 1.0, 1.0, 2.0) 
  {}

  void TrajectoryControllerTest::correctFootprint(){
    //just create a basic footprint
    vector<trajectory_rollout::Position2DInt> footprint = tc.getFootprintCells(4.5, 4.5, 0, false);

    //we expect the front line to be first
    EXPECT_EQ(footprint[0].x, 6); EXPECT_EQ(footprint[0].y, 6);
    EXPECT_EQ(footprint[1].x, 6); EXPECT_EQ(footprint[1].y, 5);
    EXPECT_EQ(footprint[2].x, 6); EXPECT_EQ(footprint[2].y, 4);
    EXPECT_EQ(footprint[3].x, 6); EXPECT_EQ(footprint[3].y, 3);
    EXPECT_EQ(footprint[4].x, 6); EXPECT_EQ(footprint[4].y, 2);

    //next the right line
    EXPECT_EQ(footprint[5].x, 6); EXPECT_EQ(footprint[5].y, 2);
    EXPECT_EQ(footprint[6].x, 5); EXPECT_EQ(footprint[6].y, 2);
    EXPECT_EQ(footprint[7].x, 4); EXPECT_EQ(footprint[7].y, 2);
    EXPECT_EQ(footprint[8].x, 3); EXPECT_EQ(footprint[8].y, 2);
    EXPECT_EQ(footprint[9].x, 2); EXPECT_EQ(footprint[9].y, 2);

    //next the back line
    EXPECT_EQ(footprint[10].x, 2); EXPECT_EQ(footprint[10].y, 2);
    EXPECT_EQ(footprint[11].x, 2); EXPECT_EQ(footprint[11].y, 3);
    EXPECT_EQ(footprint[12].x, 2); EXPECT_EQ(footprint[12].y, 4);
    EXPECT_EQ(footprint[13].x, 2); EXPECT_EQ(footprint[13].y, 5);
    EXPECT_EQ(footprint[14].x, 2); EXPECT_EQ(footprint[14].y, 6);

    //finally the left line
    EXPECT_EQ(footprint[15].x, 2); EXPECT_EQ(footprint[15].y, 6);
    EXPECT_EQ(footprint[16].x, 3); EXPECT_EQ(footprint[16].y, 6);
    EXPECT_EQ(footprint[17].x, 4); EXPECT_EQ(footprint[17].y, 6);
    EXPECT_EQ(footprint[18].x, 5); EXPECT_EQ(footprint[18].y, 6);
    EXPECT_EQ(footprint[19].x, 6); EXPECT_EQ(footprint[19].y, 6);

    //check that rotation of the footprint works
    footprint = tc.getFootprintCells(4.5, 4.5, M_PI_2, false);

    //first the left line
    EXPECT_EQ(footprint[0].x, 2); EXPECT_EQ(footprint[0].y, 6);
    EXPECT_EQ(footprint[1].x, 3); EXPECT_EQ(footprint[1].y, 6);
    EXPECT_EQ(footprint[2].x, 4); EXPECT_EQ(footprint[2].y, 6);
    EXPECT_EQ(footprint[3].x, 5); EXPECT_EQ(footprint[3].y, 6);
    EXPECT_EQ(footprint[4].x, 6); EXPECT_EQ(footprint[4].y, 6);

    //next the front line
    EXPECT_EQ(footprint[5].x, 6); EXPECT_EQ(footprint[5].y, 6);
    EXPECT_EQ(footprint[6].x, 6); EXPECT_EQ(footprint[6].y, 5);
    EXPECT_EQ(footprint[7].x, 6); EXPECT_EQ(footprint[7].y, 4);
    EXPECT_EQ(footprint[8].x, 6); EXPECT_EQ(footprint[8].y, 3);
    EXPECT_EQ(footprint[9].x, 6); EXPECT_EQ(footprint[9].y, 2);

    //next the right line
    EXPECT_EQ(footprint[10].x, 6); EXPECT_EQ(footprint[10].y, 2);
    EXPECT_EQ(footprint[11].x, 5); EXPECT_EQ(footprint[11].y, 2);
    EXPECT_EQ(footprint[12].x, 4); EXPECT_EQ(footprint[12].y, 2);
    EXPECT_EQ(footprint[13].x, 3); EXPECT_EQ(footprint[13].y, 2);
    EXPECT_EQ(footprint[14].x, 2); EXPECT_EQ(footprint[14].y, 2);

    //next the back line
    EXPECT_EQ(footprint[15].x, 2); EXPECT_EQ(footprint[15].y, 2);
    EXPECT_EQ(footprint[16].x, 2); EXPECT_EQ(footprint[16].y, 3);
    EXPECT_EQ(footprint[17].x, 2); EXPECT_EQ(footprint[17].y, 4);
    EXPECT_EQ(footprint[18].x, 2); EXPECT_EQ(footprint[18].y, 5);
    EXPECT_EQ(footprint[19].x, 2); EXPECT_EQ(footprint[19].y, 6);
  }

  void TrajectoryControllerTest::footprintObstacles(){
    //place an obstacle
    map_(4, 6).occ_state = 1;
    wa.synchronize();
    Trajectory traj(0, 0, 0, 30);
    //tc->generateTrajectory(4.5, 4.5, M_PI_2, 0, 0, 0, 4, 0, 0, 4, 0, 0, DBL_MAX, traj, 2, 30);
    tc.generateTrajectory(4.5, 4.5, M_PI_2, 0, 0, 0, 4, 0, 0, 4, 0, 0, DBL_MAX, traj);
    //we expect this path to hit the obstacle
    EXPECT_FLOAT_EQ(traj.cost_, -1.0);

    //place a wall next to the footprint of the robot
    map_(7, 1).occ_state = 1;
    map_(7, 3).occ_state = 1;
    map_(7, 4).occ_state = 1;
    map_(7, 5).occ_state = 1;
    map_(7, 6).occ_state = 1;
    map_(7, 7).occ_state = 1;
    wa.synchronize();

    //try to rotate into it
    //tc->generateTrajectory(4.5, 4.5, M_PI_2, 0, 0, 0, 0, 0, M_PI_2, 0, 0, M_PI_4, 100, traj, 2, 30);
    tc.generateTrajectory(4.5, 4.5, M_PI_2, 0, 0, 0, 0, 0, M_PI_2, 0, 0, M_PI_4, 100, traj);
    //we expect this path to hit the obstacle
    EXPECT_FLOAT_EQ(traj.cost_, -1.0);
  }

  void TrajectoryControllerTest::checkGoalDistance(){
    //let's box a cell in and make sure that its distance gets set to max
    map_(1, 2).occ_state = 1;
    map_(1, 1).occ_state = 1;
    map_(1, 0).occ_state = 1;
    map_(2, 0).occ_state = 1;
    map_(3, 0).occ_state = 1;
    map_(3, 1).occ_state = 1;
    map_(3, 2).occ_state = 1;
    map_(2, 2).occ_state = 1;
    wa.synchronize();

    //set a goal
    tc.map_.resetPathDist();
    queue<MapCell*> goal_dist_queue;
    MapCell& current = tc.map_(4, 9);
    current.goal_dist = 0.0;
    current.goal_mark = true;
    goal_dist_queue.push(&current);
    tc.computeGoalDistance(goal_dist_queue);

    EXPECT_FLOAT_EQ(tc.map_(4, 8).goal_dist, 1.0);
    EXPECT_FLOAT_EQ(tc.map_(4, 7).goal_dist, 2.0);
    EXPECT_FLOAT_EQ(tc.map_(4, 6).goal_dist, 100.0); //there's an obstacle here placed above
    EXPECT_FLOAT_EQ(tc.map_(4, 5).goal_dist, 6.0);
    EXPECT_FLOAT_EQ(tc.map_(4, 4).goal_dist, 7.0);
    EXPECT_FLOAT_EQ(tc.map_(4, 3).goal_dist, 8.0);
    EXPECT_FLOAT_EQ(tc.map_(4, 2).goal_dist, 9.0);
    EXPECT_FLOAT_EQ(tc.map_(4, 1).goal_dist, 10.0);
    EXPECT_FLOAT_EQ(tc.map_(4, 0).goal_dist, 11.0);
    EXPECT_FLOAT_EQ(tc.map_(5, 8).goal_dist, 2.0);
    EXPECT_FLOAT_EQ(tc.map_(9, 4).goal_dist, 10.0);

    //check the boxed in cell
    EXPECT_FLOAT_EQ(tc.map_(2, 2).goal_dist, 100.0);

  }

  void TrajectoryControllerTest::checkPathDistance(){
    tc.map_.resetPathDist();
    queue<MapCell*> path_dist_queue;
    MapCell& current = tc.map_(4, 9);
    current.path_dist = 0.0;
    current.path_mark = true;
    path_dist_queue.push(&current);
    tc.computePathDistance(path_dist_queue);

    EXPECT_FLOAT_EQ(tc.map_(4, 8).path_dist, 1.0);
    EXPECT_FLOAT_EQ(tc.map_(4, 7).path_dist, 2.0);
    EXPECT_FLOAT_EQ(tc.map_(4, 6).path_dist, 100.0); //there's an obstacle here placed above
    EXPECT_FLOAT_EQ(tc.map_(4, 5).path_dist, 6.0);
    EXPECT_FLOAT_EQ(tc.map_(4, 4).path_dist, 7.0);
    EXPECT_FLOAT_EQ(tc.map_(4, 3).path_dist, 8.0);
    EXPECT_FLOAT_EQ(tc.map_(4, 2).path_dist, 9.0);
    EXPECT_FLOAT_EQ(tc.map_(4, 1).path_dist, 10.0);
    EXPECT_FLOAT_EQ(tc.map_(4, 0).path_dist, 11.0);
    EXPECT_FLOAT_EQ(tc.map_(5, 8).path_dist, 2.0);
    EXPECT_FLOAT_EQ(tc.map_(9, 4).path_dist, 10.0);

    //check the boxed in cell
    EXPECT_FLOAT_EQ(tc.map_(2, 2).path_dist, 100.0);

  }
};

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

TrajectoryControllerTest* tct = NULL;

TEST(TrajectoryControllerTest, correctFootprint){
  tct->correctFootprint();
}

//make sure that trajectories that intersect obstacles are invalidated
TEST(TrajectoryControllerTest, footprintObstacles){
  tct->footprintObstacles();
}

//make sure that goal distance is being computed as expected
TEST(TrajectoryControllerTest, checkGoalDistance){
  tct->checkGoalDistance();
}

//make sure that path distance is being computed as expected
TEST(TrajectoryControllerTest, checkPathDistance){
  tct->checkPathDistance();
}



//test some stuff
int main(int argc, char** argv){
  MapGrid mg(10, 10, 1, 0, 0);
  WavefrontMapAccessor wa(mg, .25);
  std::vector<deprecated_msgs::Point2DFloat32> footprint_spec;
  deprecated_msgs::Point2DFloat32 pt;
  //create a square footprint
  pt.x = 2;
  pt.y = 2;
  footprint_spec.push_back(pt);
  pt.x = 2;
  pt.y = -2;
  footprint_spec.push_back(pt);
  pt.x = -2;
  pt.y = -2;
  footprint_spec.push_back(pt);
  pt.x = -2;
  pt.y = 2;
  footprint_spec.push_back(pt);

  tct = new TrajectoryControllerTest(mg, wa, footprint_spec);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
