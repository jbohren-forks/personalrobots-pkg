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
 * \file
 * 
 * \author Bhaskara Marthi
 */


#include <topological_map/ros_topological_map.h>
#include <fstream>


using std::string;

namespace topological_map
{


void RosTopologicalMap::loadMap (void)
{
    
  robot_srvs::StaticMap::Request req;
  robot_srvs::StaticMap::Response resp;
  ROS_INFO ("Requesting map... \n");
  while (!ros::service::call("static_map", req, resp))
  {
    sleep(2);
    ROS_INFO ("Request failed: trying again...\n");
    usleep(1000000);
  }
  sleep(2);
  ROS_INFO ("Received a %d by %d map at %f m/pix.  Origin is %f, %f, %f.  \n", resp.map.width, resp.map.height, resp.map.resolution, resp.map.origin.x, resp.map.origin.y, resp.map.origin.th);

  num_cols_ = resp.map.width;
  num_rows_ = resp.map.height;
  resolution_ = resp.map.resolution;

  occupancy_grid_.resize(boost::extents[num_rows_][num_cols_]);
  
  uint i = 0;
  bool expected[255];
  for (uint j=0; j<255; j++) expected[j]=false;
  for (uint r=0; r<num_rows_; r++) {
    for (uint c=0; c<num_cols_; c++) {
      int val = resp.map.data[i++];
      occupancy_grid_[r][c] = (val != 0);
      if (!expected[val]) {
        ROS_DEBUG ("Saw map cell value %d", val);
        expected[val]=true;
        if ((val != 0) && (val != 100) && (val != -1)) {
          ROS_INFO ("Treating unexpected val %d in returned static map as occupied\n", val);
        }
      }
    }
  }
  ROS_INFO ("Finished reading map");
}



RosTopologicalMap::RosTopologicalMap (uint bottleneck_size, uint bottleneck_width, uint bottleneck_skip, uint inflation_radius, const string& pgm_output_dir)
{
  loadMap();

  topological_map_ = topologicalMapFromGrid(occupancy_grid_, 1.0, 10.0, bottleneck_size, bottleneck_width, bottleneck_skip, inflation_radius, pgm_output_dir);
}




} // namespace topological_map








void exitWithUsage(void)
{
  exit(EX_USAGE);
}


// Parse command line params and initialize node
int main(int argc, char** argv)
{
  unsigned bottleneck_size=0;
  unsigned bottleneck_skip=1;
  unsigned bottleneck_width=1;
  unsigned inflation_radius=0;
  char* ppm_output_dir=0;

  while (1) {
    static struct option options[] =
      {{"bottleneck-size", required_argument, 0, 'b'},
       {"bottleneck-width", required_argument, 0, 'w'},
       {"bottleneck-skip", required_argument, 0, 'k'},
       {"inflation-radius", required_argument, 0, 'r'},
       {"ppm-output-dir", required_argument, 0, 'p'},
       {0, 0, 0, 0}};

    int option_index=0;
    int c = getopt_long (argc, argv, "b:w:k:r:p:", options, &option_index);
    if (c==-1) {
      break;
    }
    else {
      switch (c) {
      case 'b':
        bottleneck_size=atoi(optarg);
        break;
      case 'k':
        bottleneck_skip=atoi(optarg);
        break;
      case 'w':
        bottleneck_width=atoi(optarg);
        break;
      case 'r':
        inflation_radius=atoi(optarg);
        break;
      case 'p':
        ppm_output_dir=optarg;
        break;
      default:
        exitWithUsage();
      }
    }
  }
  
  if (!bottleneck_size) {
    exitWithUsage();
  }

  ros::init(argc, argv);
  ros::Node node("ros_topological_map");
  topological_map::RosTopologicalMap ros_top_map(bottleneck_size, bottleneck_width, bottleneck_skip, inflation_radius, ppm_output_dir ? string(ppm_output_dir) : string());

  return 0;
}


