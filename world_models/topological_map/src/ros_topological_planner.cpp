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
   - --resolution, -R


   <hr>

   @section Topics and services

   Subscribes to:
   - @b localizedpose

   Services used:
   - @b static_map (in the case when the input file is not provided).

   Services provided:
   - @b connector_costs.  

   <hr>

   <hr>
**/

#include <fstream>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <topological_map/ros_topological_planner.h>

using namespace std;

namespace topological_map
{



/************************************************************
 * Constructors/destructors
 ************************************************************/

BottleneckGraphRos::BottleneckGraphRos(int size, int skip, int radius, int distanceMin, int distanceMax) : 
  ros::Node("topological_planner"), node_status_(WAITING_FOR_MAP), costmap_(0), num_added_roadmap_points_(0), 
  size_(size), skip_(skip), radius_(radius), distanceMin_(distanceMin), distanceMax_(distanceMax)
{
}
 

BottleneckGraphRos::BottleneckGraphRos(char* filename, double resolution) :
  ros::Node("topological_planner"), costmap_(0), num_added_roadmap_points_(0), resolution_(resolution)
{
  graph_.readFromFile(filename);
  sx_ = graph_.numCols();
  sy_ = graph_.numRows();
  node_status_ = READY;
  costmap_ = new unsigned char[sy_*sx_]; // deleted in destructor
  ROS_DEBUG ("Read from file %d by %d graph with resolution %f", sy_, sx_, resolution_);
}

BottleneckGraphRos::~BottleneckGraphRos()
{
  if (costmap_) {
    delete[] costmap_;
  }
}


void BottleneckGraphRos::updateCostMap ()
{
  robot_srvs::StaticMap::Request  costmap_req;
  robot_srvs::StaticMap::Response costmap_resp;
  while (!ros::service::call("costmap", costmap_req, costmap_resp))
  {
    ROS_WARN("Costmap request failed; trying again...\n");
    usleep(1000000);
  }

  // Check array bounds and copy costmap
  ROS_ASSERT (((int)costmap_resp.map.height==sy_)&&((int)costmap_resp.map.width==sx_));
  copy (costmap_resp.map.data.begin(), costmap_resp.map.data.end(), costmap_);
  lock_.lock();
  graph_.setCostmap(costmap_);
  lock_.unlock();
}




struct PopulateConnectorCostResponse 
{
  PopulateConnectorCostResponse (BottleneckGraphRos* bgraph, ConnectorResponse& resp) : graph(bgraph), response(resp), ind(0) {}
  void operator() (const ConnectorCostPair& pair)
  {
    double x, y;
    graph->convertFromMapIndices(pair.first, &x, &y);
    response.connectors[ind].x=x;
    response.connectors[ind].y=y;
    response.connectors[ind].th=0;
    response.costs[ind]=pair.second;
    ind++;
  }

  BottleneckGraphRos* graph;
  ConnectorResponse& response;
  int ind;
};





/************************************************************
 * Callbacks
 ************************************************************/

bool BottleneckGraphRos::connectorCostsCallback (topological_map::ConnectorCosts::Request& req, ConnectorResponse& resp)
{
  GridCell start, goal;
  convertToMapIndices (req.start.x, req.start.y, &start);
  convertToMapIndices (req.goal.x, req.goal.y, &goal);

  updateCostMap();

  ConnectorCostVector connector_costs=graph_.evaluateConnectors(start, goal);
  resp.set_connectors_size(connector_costs.size());
  resp.set_costs_size(connector_costs.size());

  for_each(connector_costs.begin(), connector_costs.end(), PopulateConnectorCostResponse(this, resp));
  ROS_DEBUG_STREAM ("Connector costs callback called and added " << resp.costs.size() << " points");
  return true;
}

    



void BottleneckGraphRos::poseCallback (void)
{
  // Will eventually do caching of exit costs here
  int r, c;
  if ((node_status_==READY) && graph_.isReady()) {
    lock_.lock();

    convertToMapIndices (pose_.pos.x, pose_.pos.y, &r,&c);
    int region = graph_.regionId(r,c);
    if (region!=region_id_) {
      region_id_=region;
      BottleneckVertex v;
      graph_.lookupVertex (r, c, &v);
      if (graph_.vertexDescription(v).type == BOTTLENECK) {
        ROS_DEBUG ("Moving into bottleneck vertex %d", region);
      }
      else {
        ROS_DEBUG ("Moving into open vertex %d", region);
      }
    }
    lock_.unlock();
  }
}






/************************************************************
 * Ops called by main
 ************************************************************/

void BottleneckGraphRos::computeBottleneckGraph (void)
{
  lock_.lock();
  ROS_INFO ("Computing bottleneck graph... (this could take a while)\n");
  graph_.initializeFromGrid(grid_, size_, skip_, radius_, distanceMin_, distanceMax_);
  ROS_INFO ("Done computing bottleneck graph\n");
  //graph_.printBottlenecks();
  node_status_=READY;
  lock_.unlock();
}  

void BottleneckGraphRos::loadMap (void)
{
    
  lock_.lock();
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
  node_status_ = CREATING_BOTTLENECK_GRAPH;
  sx_ = resp.map.width;
  sy_ = resp.map.height;
  resolution_ = resp.map.resolution;

  grid_.resize(boost::extents[sy_][sx_]);
  costmap_ = new unsigned char[sy_*sx_]; // deleted in destructor
  
  int i = 0;
  bool expected[255];
  for (int j=0; j<255; j++) expected[j]=false;
  for (int r=0; r<sy_; r++) {
    for (int c=0; c<sx_; c++) {
      int val = resp.map.data[i++];
      grid_[r][c] = (val != 0);
      if (!expected[val]) {
        ROS_DEBUG ("Saw map cell value %d", val);
        expected[val]=true;
        if ((val != 0) && (val != 100) && (val != -1)) {
          ROS_INFO ("Treating unexpected val %d in returned static map as occupied\n", val);
        }
      }
    }
  }
  lock_.unlock();
}

void BottleneckGraphRos::setupTopics (void)
{
  // Subscribe to 2d position
  ROS_INFO ("Setting up node topics");
  subscribe("localizedpose",  pose_,  &BottleneckGraphRos::poseCallback, 10);

  // Advertise connector costs
  advertiseService("connector_costs", &BottleneckGraphRos::connectorCostsCallback);
}


void BottleneckGraphRos::generateRoadmap (void)
{
  node_status_ = COMPUTING_ROADMAP;
  graph_.initializeRoadmap();
}


void BottleneckGraphRos::writeToFile (char* filename)
{
  ROS_INFO ("Writing bottleneck graph to file %s", filename);
  lock_.lock();
  ROS_DEBUG ("Writing acquired lock");
  graph_.printBottlenecks(filename);
  lock_.unlock();
  ROS_INFO ("Done writing");
}


void BottleneckGraphRos::outputPpm (char* filename)
{
  ofstream stream;
  stream.open (filename);
  graph_.outputPpm(stream, 3);
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

void BottleneckGraphRos::convertToMapIndices (double x, double y, GridCell* c)
{
  int row, col;
  convertToMapIndices (x, y, &row, &col);
  c->first=row;
  c->second=col;
}

void BottleneckGraphRos::convertFromMapIndices (const GridCell& c, double* x, double* y)
{
  *y=c.first*resolution_;
  *x=c.second*resolution_;
}


} // namespace topological_map








void exitWithUsage(void)
{
  cout << "Usage 1:\n Required:\n  --bottleneck-size, -b\n  --inflation-radius, -r\n Optional:\n"
    "  --bottleneck-skip, -k\n  --distance-lower-bound, -d\n  --distance-upper-bound, -D\n  --output-file, -o\n --ppm-output-file, -p\n  --spin, -s\n"
    "Usage 2:\n Required:\n  --input-file, -i\n  --resolution, -R\n";
  exit(EX_USAGE);
}





int main(int argc, char** argv)
{
  int bottleneckSize=-1;
  int bottleneckSkip=-1;
  int inflationRadius=-1;
  int distanceLower=1;
  int distanceUpper=2;
  double resolution=0.0;
  char* inputFile=0;
  char* outputFile=0;
  char* ppmOutputFile=0;
  int spin=1;


  while (1) {
    static struct option options[] =
      {{"bottleneck-size", required_argument, 0, 'b'},
       {"bottleneck-skip", required_argument, 0, 'k'},
       {"inflation-radius", required_argument, 0, 'r'},
       {"distance-lower-bound", required_argument, 0, 'd'},
       {"distance-upper-bound", required_argument, 0, 'D'},
       {"input-file", required_argument, 0, 'i'},
       {"output-file", required_argument, 0, 'o'},
       {"ppm-output-file", required_argument, 0, 'p'},
       {"spin", required_argument, 0, 's'},
       {"resolution", required_argument, 0, 'R'},
       {0, 0, 0, 0}};

    int option_index=0;
    int c = getopt_long (argc, argv, "b:k:r:s:d:D:i:o:R:p:", options, &option_index);
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
      case 's':
        spin=atoi(optarg);
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
      case 'p':
        ppmOutputFile=optarg;
        break;
      case 'R':
        resolution=atof(optarg);
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
    if (resolution==0.0) {
      exitWithUsage();
    }
    else {
      ROS_INFO ("Reading bottleneck graph from %s", inputFile);
      node = new topological_map::BottleneckGraphRos(inputFile, resolution);
      node->setupTopics();
    }
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

  if (ppmOutputFile) {
    node->outputPpm(ppmOutputFile);
  }

  ROS_INFO ("Entering main loop");
  if (spin) {
    node->spin();
  }
  node->shutdown();
  delete node;

  return 0;
}


