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
#include <set>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/foreach.hpp>
#include <ros/node.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <door_msgs/Door.h>
#include <robot_msgs/Point.h>
#include <robot_msgs/Point32.h>
#include <robot_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>

#define foreach BOOST_FOREACH

namespace topological_map
{

using ros::NodeHandle;
using ros::Publisher;
using door_msgs::Door;
using robot_msgs::Point;
using robot_msgs::Point32;
using robot_msgs::PointCloud;
using visualization_msgs::Marker;
using std::set;
using std::string;
using boost::bind;
using boost::ref;


const string MARKER_TOPIC("visualization_marker");
const string CONNECTOR_TOPIC("~connectors");
const string MARKER_NS("topological_map");
const string VISUALIZER_FRAME("map");


typedef set<ConnectorId> ConnectorSet;


/************************************************************
 * Helpers
 ************************************************************/

// Functor for drawing individual doors and publishing them as a group to visualization_marker
struct DrawDoors
{
  DrawDoors (const TopologicalMap& tmap, const Publisher& pub) : tmap(tmap), pub(pub)
  {
    marker.header.frame_id = VISUALIZER_FRAME;
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
  marker.header.frame_id=VISUALIZER_FRAME;
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



ConnectorSet getConnectors (const TopologicalMap& tmap)
{
  ConnectorSet connectors;
  foreach (RegionId r, tmap.allRegions()) {
    foreach (ConnectorId c, tmap.adjacentConnectors(r)) {
      connectors.insert(c);
    }
  }
  return connectors;
}


struct DrawConnectors
{
  DrawConnectors (const TopologicalMap& tmap, const Publisher& pub) : tmap(tmap), pub(pub)
  {
    cloud.header.frame_id=VISUALIZER_FRAME;
  }
  void operator() (const ConnectorId id)
  {
    Point2D p = tmap.connectorPosition(id);
    Point32 p2;
    p2.x=p.x;
    p2.y=p.y;
    cloud.pts.push_back(p2);
  }
  ~DrawConnectors ()
  {
    pub.publish(cloud);
  }

  const TopologicalMap& tmap;
  const Publisher& pub;
  PointCloud cloud;
};




/************************************************************
 * Top level 
 ************************************************************/

void Visualizer::visualize ()
{
  const RegionIdSet regions = tmap_.allRegions();
  for_each(regions.begin(), regions.end(), DrawDoors(tmap_, marker_pub_));

  foreach (OutletId id, tmap_.allOutlets()) 
    drawOutlet (id, tmap_, marker_pub_);

  const ConnectorSet connectors = getConnectors(tmap_);
  for_each (connectors.begin(), connectors.end(), DrawConnectors(tmap_, connector_pub_));
}


Visualizer::Visualizer (const TopologicalMap& tmap) 
  : tmap_(tmap), node_(), marker_pub_(node_.advertise<Marker>(MARKER_TOPIC, 0)),
    connector_pub_(node_.advertise<PointCloud>(CONNECTOR_TOPIC, 0))
{
}









} // end namespace
