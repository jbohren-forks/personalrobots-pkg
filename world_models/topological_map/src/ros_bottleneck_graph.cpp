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


#include <getopt.h>
#include <sysexits.h>
#include <ros/node.h>
#include <rosconsole/rosconsole.h>
#include <std_srvs/StaticMap.h>
#include <std_msgs/RobotBase2DOdom.h>
#include "topological_map/bottleneck_graph.h"


using namespace std;

namespace topological_map
{

enum NodeStatus { WAITING_FOR_MAP, CREATING_BOTTLENECK_GRAPH, READY };

class BottleneckGraphRos: public ros::node
{
public:
  BottleneckGraphRos(int size, int skip, int radius, int distanceMin, int distanceMax);
  BottleneckGraphRos(char* filename);

  void loadMap(void);
  void computeBottleneckGraph(void);
  void setupTopics(void);
  void writeToFile(char*);
  void setResolution(double r) { resolution_=r; }

  void poseCallback(void);

  void convertToMapIndices(double, double, int*, int*);


private:


  IndexedBottleneckGraph bottleneckGraph_;
  NodeStatus nodeStatus_;
  GridArray* grid_;

  int sx_, sy_;
  int region_;
  double resolution_;

  int size_, skip_, radius_, distanceMin_, distanceMax_;

  std_msgs::RobotBase2DOdom pose_;
};





/************************************************************
 * Constructors
 ************************************************************/

BottleneckGraphRos::BottleneckGraphRos(int size, int skip, int radius, int distanceMin, int distanceMax) : 
  ros::node("bottleneck_graph_ros"), nodeStatus_(WAITING_FOR_MAP), size_(size), skip_(skip), radius_(radius),
  distanceMin_(distanceMin), distanceMax_(distanceMax) 
{
}
 
BottleneckGraphRos::BottleneckGraphRos(char* filename) :
  ros::node("bottleneck_graph_ros")
{
  bottleneckGraph_.readFromFile(filename);
}





/************************************************************
 * Callbacks
 ************************************************************/

void BottleneckGraphRos::poseCallback (void)
{
  int r, c;
  convertToMapIndices (pose_.pos.x, pose_.pos.y, &r,&c);
  int region = bottleneckGraph_.regionId(r,c);
  if (region!=region_) {
    region_=region;
    BottleneckVertex v;
    bottleneckGraph_.lookupVertex (r, c, &v);
    if (bottleneckGraph_.vertexDescription(v).type == BOTTLENECK)
      ROS_DEBUG ("Moving into bottleneck vertex %d", region);
    else
      ROS_DEBUG ("Moving into open vertex %d", region);
  }
}



/************************************************************
 * Ops called by main
 ************************************************************/

void BottleneckGraphRos::computeBottleneckGraph (void)
{
  ROS_INFO ("Computing bottleneck graph... (this could take a while)\n");
  bottleneckGraph_.initializeFromGrid(*grid_, size_, skip_, radius_, distanceMin_, distanceMax_);
  nodeStatus_ = READY;
  ROS_INFO ("Done computing bottleneck graph\n");
  bottleneckGraph_.printBottlenecks();
}  

void BottleneckGraphRos::loadMap (void)
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
  ROS_INFO ("Received a %d by %d map at %f m/pix.  Origin is %f, %f, %f.  \n", resp.map.width, resp.map.height, resp.map.resolution, resp.map.origin.x, resp.map.origin.y, resp.map.origin.th);
  nodeStatus_ = CREATING_BOTTLENECK_GRAPH;
  sx_ = resp.map.width;
  sy_ = resp.map.height;
  resolution_ = resp.map.resolution;

  grid_ = new GridArray(boost::extents[sy_][sx_]);
  
  int i = 0;
  bool expected[255];
  for (int r=0; r<sy_; r++) {
    for (int c=0; c<sx_; c++) {
      int val = resp.map.data[i++];
      (*grid_)[r][c] = (val == 100);
      if ((val != 0) && (val != 100) && (val != 255) && !expected[val]) {
        expected[val] = true;
        ROS_WARN ("Treating unexpected val %d in returned static map as occupied\n", val);
      }
    }
  }
}

void BottleneckGraphRos::setupTopics (void)
{
  // Subscribe to 2d position
  ROS_INFO ("Setting up node topics");
  subscribe("localizedpose",  pose_,  &BottleneckGraphRos::poseCallback, 100);

} // namespace topological_map


void BottleneckGraphRos::writeToFile (char* filename)
{
  ROS_INFO ("Writing bottleneck graph to file %s", filename);
  bottleneckGraph_.printBottlenecks(filename);
  ROS_INFO ("Done writing");
}




/************************************************************
 * Internal
 ************************************************************/

void BottleneckGraphRos::convertToMapIndices (double x, double y, int* r, int* c)
{
  // Almost certainly wrong, as it ignores origin
  // Also need to check if x and y should be reversed
 
  *r = (int)floor(y/resolution_);
  *c = (int)floor(x/resolution_);



}


} // End topological_map namespace



void exitWithUsage(void)
{
  cout << "Usage 1:\n Required:\n  --bottleneck-size, -b\n  --inflation-radius, -r\n Optional:\n"
    "  --bottleneck-skip, -k\n  --distance-lower-bound, -d\n  --distance-upper-bound, -D\n  --output-file, -o\n"
    "Usage 2:\n Required:\n  --input-file, -i\n  --map-resolution, -m";
  exit(EX_USAGE);
}



int main(int argc, char** argv)
{
  int bottleneckSize=-1;
  int bottleneckSkip=-1;
  int inflationRadius=-1;
  int distanceLower=1;
  int distanceUpper=2;
  char* inputFile=0;
  char* outputFile=0;


  while (1) {
    static struct option options[] =
      {{"bottleneck-size", required_argument, 0, 'b'},
       {"bottleneck-skip", required_argument, 0, 'k'},
       {"inflation-radius", required_argument, 0, 'r'},
       {"distance-lower-bound", required_argument, 0, 'd'},
       {"distance-upper-bound", required_argument, 0, 'D'},
       {"input-file", required_argument, 0, 'i'},
       {"output-file", required_argument, 0, 'o'},
       {0, 0, 0, 0}};

    int option_index=0;
    int c = getopt_long (argc, argv, "b:k:r:d:D:i:o:", options, &option_index);
    if (c==-1) {
      break;
    }
    else {
      switch (c) {
      case 'b':
        bottleneckSize=atoi(optarg);
        if (bottleneckSkip<0) {
          bottleneckSkip = 1+bottleneckSize/3;
        }
        break;
      case 'k':
        bottleneckSkip=atoi(optarg);
        break;
      case 'r':
        inflationRadius=atoi(optarg);
        break;
      case 'd':
        distanceLower=atoi(optarg);
        break;
      case 'D':
        distanceUpper=atoi(optarg);
        break;
      case 'i':
        inputFile=optarg;
        break;
      case 'o':
        outputFile=optarg;
        break;
      case '?':
        exitWithUsage();
      default:
        exitWithUsage();
      }
    }
  }
  if (!inputFile && ((bottleneckSkip<0) || (bottleneckSize<0) || (inflationRadius<0))) {
    exitWithUsage();
  }
  
  ros::init(argc, argv);
  topological_map::BottleneckGraphRos* node;

  if (inputFile) {
    ROS_INFO ("Reading bottleneck graph from %s", inputFile);
    node = new topological_map::BottleneckGraphRos(inputFile);
    node->loadMap();
    node->setupTopics();
  }
  else {
    ROS_INFO ("Creating bottleneck graph with bottleneck size %d, skip %d, inflation %d, distance lower bound %d, distance upper bound %d\n",
              bottleneckSize, bottleneckSkip, inflationRadius, distanceLower, distanceUpper);
    node = new topological_map::BottleneckGraphRos(bottleneckSize, bottleneckSkip, inflationRadius, distanceLower, distanceUpper);
    node->setupTopics();
    node->loadMap();
    node->computeBottleneckGraph();
    if (outputFile) {
      node->writeToFile(outputFile);
    }
  }

  ROS_INFO ("Entering main loop");
  node->spin();
  node->shutdown();
  delete node;

  return 0;
}
