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


#include <topological_map/visualization.h>
#include <ros/node.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <robot_msgs/Door.h>
#include <robot_msgs/Point.h>
#include <visualization_msgs/Marker.h>

namespace topological_map
{

using ros::Node;
using robot_msgs::Door;
using robot_msgs::Point;
using visualization_msgs::Marker;


Visualizer::Visualizer (const TopologicalMap& tmap) : tmap(tmap) 
{
  Node::instance()->advertise<Marker>("visualization_marker", 0);
}

Visualizer::~Visualizer () 
{
  Node::instance()->unadvertise("visualization_marker");
}



// Functor for drawing individual doors and publishing them as a group to visualization_marker
struct DrawDoors
{
  DrawDoors (const TopologicalMap& tmap) : tmap(tmap) 
  {
    marker.header.frame_id = "map";
    marker.ns = "topological_map";
    marker.id=1;
    marker.type=Marker::LINE_LIST;
    marker.action=Marker::ADD;
    marker.color.a=1.0;
    marker.scale.x=0.2;
    marker.scale.y=0.2;
    marker.scale.z=0.2;
    marker.pose.orientation.w=1.0;
    marker.color.r=0;
    marker.color.g=0;
    marker.color.b=1.0;
  }

  void operator() (const RegionId id)
  {
    if (tmap.regionType(id)==DOORWAY) {
      
      Door d = tmap.regionDoor(id);
      Point p1, p2;
      p1.x = d.frame_p1.x;
      p1.y = d.frame_p1.y;
      p2.x = d.frame_p2.x;
      p2.y = d.frame_p2.y;
      
      marker.points.push_back(p1);
      marker.points.push_back(p2);
    }
  }

  ~DrawDoors ()
  {
    Node::instance()->publish("visualization_marker", marker);
  }

  const TopologicalMap& tmap;
  Marker marker;
};




void Visualizer::visualize ()
{
  const RegionIdSet& r = tmap.allRegions();
  for_each(r.begin(), r.end(), DrawDoors(tmap));
}


















} // end namespace
