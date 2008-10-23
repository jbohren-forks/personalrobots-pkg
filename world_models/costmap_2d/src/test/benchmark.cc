/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author Conor McGann
 * @file Benchmark Testing of the Cost Map
 */

#include <costmap_2d/costmap_2d.h>

const unsigned int GRID_WIDTH(1000);
const unsigned int GRID_HEIGHT(1000);
const double RESOLUTION(1);
const double WINDOW_LENGTH(10);
const unsigned char THRESHOLD(100);
const double MAX_Z(1.0);
const double ROBOT_RADIUS(1.0);

using namespace costmap_2d;

int main(int argc, char** argv){
  srand(0);

  // Initialize the grid
  std::vector<unsigned char> mapData;
  for(unsigned int i = 0; i< GRID_WIDTH * GRID_HEIGHT; i++)
    mapData.push_back(0);

  CostMap2D costMap(GRID_WIDTH, GRID_HEIGHT, mapData, RESOLUTION, WINDOW_LENGTH, THRESHOLD, MAX_Z * 2, MAX_Z, 
		    ROBOT_RADIUS*3, ROBOT_RADIUS * 2, ROBOT_RADIUS);

  for(unsigned int i = 0; i < 100; i++){
    std_msgs::PointCloudFloat32 cloud;
    cloud.set_pts_size(1000);
    for(unsigned int j = 0; j < 1000; j++){
      unsigned int mx = 100 + (rand() % 100);
      unsigned int my = 100 + (rand() % 100);
      cloud.pts[j].x = mx;
      cloud.pts[j].y = my;
      cloud.pts[j].z = 0;
    }

    std::vector<unsigned int> updates;
    costMap.updateDynamicObstacles(i, cloud, updates);
  }
}
