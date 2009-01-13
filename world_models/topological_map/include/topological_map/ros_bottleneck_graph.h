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




#ifndef TOPOLOGICAL_MAP_ROS_BOTTLENECK_GRAPH_H
#define TOPOLOGICAL_MAP_ROS_BOTTLENECK_GRAPH_h


#include <getopt.h>
#include <sysexits.h>
#include <ros/node.h>
#include <rosconsole/rosconsole.h>
#include <std_srvs/StaticMap.h>
#include <std_msgs/RobotBase2DOdom.h>
#include "topological_map/roadmap_bottleneck_graph.h"



namespace topological_map
{


enum NodeStatus { WAITING_FOR_MAP, CREATING_BOTTLENECK_GRAPH, COMPUTING_ROADMAP, READY };

class BottleneckGraphRos: public ros::node
{
public:
  BottleneckGraphRos(int size, int skip, int radius, int distanceMin, int distanceMax);
  BottleneckGraphRos(char* filename);
  ~BottleneckGraphRos();

  void loadMap(void);
  void computeBottleneckGraph(void);
  void generateRoadmap(void);
  void setupTopics(void);
  void writeToFile(char*);
  void setResolution(double r) { resolution_=r; }

  void poseCallback(void);

  void convertToMapIndices(double, double, int*, int*);


private:

  // Disallow copy and assignment
  BottleneckGraphRos (const BottleneckGraphRos&);
  BottleneckGraphRos& operator= (const BottleneckGraphRos&);


  RoadmapBottleneckGraph bottleneck_graph_;
  NodeStatus node_status_;
  GridArray grid_;

  int sx_, sy_;
  int region_id_;
  int num_added_roadmap_points_;
  double resolution_;

  int size_, skip_, radius_, distanceMin_, distanceMax_;

  std_msgs::RobotBase2DOdom pose_;
};



} // namespace topological_map




#endif // TOPOLOGICAL_MAP_ROS_BOTTLENECK_GRAPH_H
