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
#include <string>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <ros/node.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <door_msgs/Door.h>
#include <robot_msgs/Point.h>
#include <visualization_msgs/Marker.h>

namespace topological_map
{

using ros::NodeHandle;
using ros::Publisher;
using door_msgs::Door;
using robot_msgs::Point;
using visualization_msgs::Marker;
using std::string;
using boost::bind;
using boost::ref;


const string MARKER_TOPIC("visualization_marker");
const string MARKER_NS("topological_map");
const string MARKER_FRAME("map");


Visualizer::Visualizer (const TopologicalMap& tmap) : tmap_(tmap), node_(), marker_pub_(node_.advertise<Marker>(MARKER_TOPIC, 0))
{
}



// Functor for drawing individual doors and publishing them as a group to visualization_marker
struct DrawDoors
{
  DrawDoors (const TopologicalMap& tmap, const Publisher& pub) : tmap(tmap), pub(pub)
  {
    marker.header.frame_id = MARKER_FRAME;
    marker.ns = MARKER_NS;
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
    pub.publish(marker);
  }

  const TopologicalMap& tmap;
  Marker marker;
  const Publisher& pub;
};



void drawOutlet (const OutletId id, const TopologicalMap& m, const Publisher& pub)
{
  OutletInfo outlet = m.outletInfo(id);
  Marker marker;
  marker.id = id+100;
  marker.ns = MARKER_NS;
  marker.header.frame_id=MARKER_FRAME;
  marker.type = Marker::ARROW;
  marker.action=Marker::ADD;
  marker.color.a=1.0;
  marker.color.g=1.0;
  marker.scale.x=1.0;
  marker.scale.y=0.1;
  marker.scale.z=0.1;
  marker.pose.position.x=outlet.x;
  marker.pose.position.y=outlet.y;
  marker.pose.position.z=outlet.z;
  marker.pose.orientation.x=outlet.qx;
  marker.pose.orientation.y=outlet.qy;
  marker.pose.orientation.z=outlet.qz;
  marker.pose.orientation.w=outlet.qw;
  pub.publish(marker);
}




void Visualizer::visualize ()
{
  const RegionIdSet& regions = tmap_.allRegions();
  for_each(regions.begin(), regions.end(), DrawDoors(tmap_, marker_pub_));

  const OutletIdSet& outlets = tmap_.allOutlets();
  for_each(outlets.begin(), outlets.end(), bind(drawOutlet, _1, ref(tmap_), marker_pub_));
}


















} // end namespace
