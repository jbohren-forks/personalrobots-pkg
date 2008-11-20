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


#include <ros/node.h>
#include <rosconsole/rosconsole.h>
#include <std_srvs/StaticMap.h>
#include "topological_map/bottleneck_graph.h"

class BottleneckGraphRos: public ros::node
{
public:
  BottleneckGraphRos(int size, int skip, int radius, int distanceMin, int distanceMax);
};

BottleneckGraphRos::BottleneckGraphRos(int size, int skip, int radius, int distanceMin, int distanceMax) : ros::node("bottleneckgraph_ros")
{

  std_srvs::StaticMap::request req;
  std_srvs::StaticMap::response resp;
  ROS_INFO ("Requesting map... \n");
  while (!ros::service::call("static_map", req, resp))
  {
    sleep(2);
    ROS_INFO ("Request failed: trying again...\n");
    usleep(1000000);
  }
  sleep(2);
  ROS_INFO ("Received a %d by %d map at %f m/pix\n", resp.map.width, resp.map.height, resp.map.resolution);
  int sx = resp.map.width;
  int sy = resp.map.height;
  
  topological_map::GridArray grid(boost::extents[sy][sx]);
  int i = 0;
  for (int r=0; r<sy; r++) {
    for (int c=0; c<sx; c++) {
      int val = resp.map.data[i++];
      grid[r][c] = (val == 100);
      if ((val != 0) && (val != 100) && (val != 255)) {
        ROS_WARN ("Treating unexpected val %d in returned static map as occupied\n", val);
      }
    }
  }
  
  topological_map::BottleneckGraph g = topological_map::makeBottleneckGraph (grid, size, skip, radius, distanceMin, distanceMax);
  topological_map::printBottlenecks (g, grid);
}  


  

int main(int argc, char** argv)
{
  ros::init(argc, argv);
  assert (argc >= 6);
  BottleneckGraphRos node (atoi(argv[1]), atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), atoi(argv[5]));
  node.shutdown();
}
