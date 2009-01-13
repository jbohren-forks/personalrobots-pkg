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



/** \author Bhaskara Marthi */

/**

   @mainpage

   @htmlinclude ../manifest.html

   @b ros_bottleneck_graph is a node that computes a topological graph from the static map.  Currently just keeps track of which vertex in the graph the robot is at any point.

   <hr>

   @section usage Usage

   @subsection usage1 Usage 1

   Required:
   - --bottleneck-size, -b
   - --inflation-radius, -r

   Optional:
   - --bottleneck-skip, -k
   - --distance-lower-bound, -d
   - --distance-upper-bound, -D
   - --output-file, -o

   @subsection usage2 Usage 2

   Required:
   - --input-file, -i


   <hr>

   @section topic ROS topics

   Subscribes to:
   - @b localizedpose

   Services used:
   - @b static_map.  Note: in the case of using the -i parameter to load the topological graph from a file, we're assuming the static map being provided has the same resolution as the one that was used to compute the saved graph.

   <hr>

   <hr>
**/


#include <topological_map/ros_bottleneck_graph.h>

using namespace std;

namespace topological_map
{



/************************************************************
 * Constructors/destructors
 ************************************************************/

BottleneckGraphRos::BottleneckGraphRos(int size, int skip, int radius, int distanceMin, int distanceMax) : 
  ros::node("bottleneck_graph_ros"), node_status_(WAITING_FOR_MAP), num_added_roadmap_points_(0), 
  size_(size), skip_(skip), radius_(radius), distanceMin_(distanceMin), distanceMax_(distanceMax) 
{
}
 

BottleneckGraphRos::BottleneckGraphRos(char* filename) :
  ros::node("bottleneck_graph_ros"), node_status_ (READY), num_added_roadmap_points_(0)
{
  bottleneck_graph_.readFromFile(filename);
}

BottleneckGraphRos::~BottleneckGraphRos()
{}





/************************************************************
 * Callbacks
 ************************************************************/

void BottleneckGraphRos::poseCallback (void)
{
  int r, c;
  convertToMapIndices (pose_.pos.x, pose_.pos.y, &r,&c);
  int region = bottleneck_graph_.regionId(r,c);
  if (region!=region_id_) {
    region_id_=region;
    BottleneckVertex v;
    bottleneck_graph_.lookupVertex (r, c, &v);
    if (bottleneck_graph_.vertexDescription(v).type == BOTTLENECK) {
      ROS_DEBUG ("Moving into bottleneck vertex %d", region);
    }
    else {
      ROS_DEBUG ("Moving into open vertex %d", region);
    }

    // Remove low-level cells from previous region and add the new ones
    bottleneck_graph_.switchToRegion (region_id_);

  }
}




/************************************************************
 * Ops called by main
 ************************************************************/

void BottleneckGraphRos::computeBottleneckGraph (void)
{
  ROS_INFO ("Computing bottleneck graph... (this could take a while)\n");
  bottleneck_graph_.initializeFromGrid(grid_, size_, skip_, radius_, distanceMin_, distanceMax_);
  node_status_ = READY;
  ROS_INFO ("Done computing bottleneck graph\n");
  bottleneck_graph_.printBottlenecks();
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
  node_status_ = CREATING_BOTTLENECK_GRAPH;
  sx_ = resp.map.width;
  sy_ = resp.map.height;
  resolution_ = resp.map.resolution;

  grid_.resize(boost::extents[sy_][sx_]);
  
  int i = 0;
  bool expected[255];
  for (int r=0; r<sy_; r++) {
    for (int c=0; c<sx_; c++) {
      int val = resp.map.data[i++];
      grid_[r][c] = (val == 100);
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

}

void BottleneckGraphRos::generateRoadmap (void)
{
  node_status_ = COMPUTING_ROADMAP;
  bottleneck_graph_.initializeRoadmap();
}


void BottleneckGraphRos::writeToFile (char* filename)
{
  ROS_INFO ("Writing bottleneck graph to file %s", filename);
  bottleneck_graph_.printBottlenecks(filename);
  ROS_INFO ("Done writing");
}




/************************************************************
 * Internal
 ************************************************************/

void BottleneckGraphRos::convertToMapIndices (double x, double y, int* r, int* c)
{
  // Wrong in general, as it ignores origin pose
  *r = (int)floor(y/resolution_);
  *c = (int)floor(x/resolution_);
}


} // namespace topological_map






void exitWithUsage(void)
{
  cout << "Usage 1:\n Required:\n  --bottleneck-size, -b\n  --inflation-radius, -r\n Optional:\n"
    "  --bottleneck-skip, -k\n  --distance-lower-bound, -d\n  --distance-upper-bound, -D\n  --output-file, -o\n"
    "Usage 2:\n Required:\n  --input-file, -i";
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
    node->loadMap();
    node->setupTopics();
    node->computeBottleneckGraph();
    if (outputFile) {
      node->writeToFile(outputFile);
    }
  }

  ROS_INFO ("Generating roadmap");
  node->generateRoadmap();
  ROS_INFO ("Entering main loop");
  node->spin();
  node->shutdown();
  delete node;

  return 0;
}


