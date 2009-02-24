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
 * \mainpage
 *
 * \htmlinclude manifest.html
 *
 * \author Bhaskara Marthi
 *
 *
 **/



#include <string>
#include <iostream>
#include <map>
#include <boost/filesystem.hpp>
#include <ros/node.h>
#include <ros/time.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <visual_nav/visual_nav.h>
#include <deprecated_msgs/RobotBase2DOdom.h>
#include <robot_msgs/Planner2DGoal.h>
#include <robot_msgs/VisualizationMarker.h>
#include <vslam/Roadmap.h>
#include <visual_nav/exceptions.h>

using std::string;
using robot_msgs::Planner2DGoal;
using deprecated_msgs::RobotBase2DOdom;
using ros::Duration;
using vslam::Roadmap;
using ros::Node;
using vslam::Edge;
using std::map;

namespace visual_nav
{

using robot_msgs::VisualizationMarker; // the message class name


typedef map<int, NodeId> IdMap;

/************************************************************
 * Node class
 ************************************************************/

class RosVisualNavigator
{
public:

  /// Constructor that listens to visual slam message
  RosVisualNavigator (double exit_point_radius, int goal_id);

  // Subscribe topics
  void setupTopics();

  // Spin
  void run();

  // Callbacks
  void odomCallback();
  void roadmapCallback();

private:

  /******************************
   * Nested classes
   ******************************/

  // RAII class to encapsulate lock on roadmap message and object, start_id_, and nav_odom_transform_
  struct RoadmapLock
  {
    RoadmapLock (RosVisualNavigator* nav) : nav(nav) 
    { 
      ROS_DEBUG_NAMED("node", "Acquiring RoadmapLock...");
      nav->roadmap_message_.lock(); 
      ROS_DEBUG_NAMED("node", "Acquired RoadmapLock"); 
    }
    ~RoadmapLock () { nav->roadmap_message_.unlock(); ROS_DEBUG_NAMED("node", "Released RoadmapLock"); }
    RosVisualNavigator* nav;
  };
  friend struct RoadmapLock;


  // Used when converting ros message
  struct AddNodeToRoadmap
  {
    AddNodeToRoadmap (RosVisualNavigator* nav) : nav(nav), external_id(0) {}
    void operator() (vslam::Node n)
    {
      NodeId id=nav->roadmap_->addNode(n.x, n.y, n.theta);
      nav->id_map_[external_id++]=id;
    }
    RosVisualNavigator* nav;
    uint external_id;
  };
  friend struct AddNodeToRoadmap;


  // Used when converting ros message
  struct AddEdgeToRoadmap
  {
    AddEdgeToRoadmap (RosVisualNavigator* nav): nav(nav) {}
    void operator() (Edge e)
    {
      nav->roadmap_->addEdge(nav->getInternalId(e.node0), nav->getInternalId(e.node1));
    }
    RosVisualNavigator* nav;
  };
  friend struct AddEdgeToRoadmap;

    

  /******************************
   * Ops
   ******************************/

  // Send the current map, robot position, and goal to visualizer
  void publishVisualization ();

  // Given goal pose in nav frame, generate a goal message (in odom frame)
  Planner2DGoal createGoalMessage (const Pose& goal_pose) const;

  // Update the nav-to-odom transform given the most recent roadmap and odom messages
  void updateOdomTransform ();

  /// \returns Internally used id corresponding to external (as defined by ros message from vslam) id \a id
  /// \throws UnknownExternalId
  NodeId getInternalId(uint id);


  // Node object
  Node node_;

  // Roadmap, and roadmap message used to populate it
  // Access these only in scopes where a RoadmapLock has been declared
  RoadmapPtr roadmap_;
  Roadmap roadmap_message_;
  
  // Has at least one map message been received?
  bool map_received_;

  // Map between external id's and our internal node ids
  IdMap id_map_;

  // Last odom message
  RobotBase2DOdom odom_message_;

  // has at least one odometry message been received?
  bool odom_received_;

  // the radius of the window used to find the exit point of the path 
  double exit_point_radius_;

  // Goal node's (external) id
  NodeId goal_id_;

  // Start node's (internal) id.  Access requires RoadmapLock.
  NodeId start_id_;

  // Transform between the nav frame (used by the visual roadmap) and the odometry frame (used to give goals to local controller)
  // Access requires RoadmapLock
  Transform2D nav_odom_transform_;

  // Number of visualization markers currently being displayed by this node
  uint num_active_markers_;

};



/************************************************************
 * top level
 ************************************************************/

// Constructor
RosVisualNavigator::RosVisualNavigator (double exit_point_radius, const NodeId goal_id) : 
  node_("visual_navigator"), roadmap_(new VisualNavRoadmap), map_received_(false), 
  odom_received_(false), exit_point_radius_(exit_point_radius), goal_id_(goal_id), num_active_markers_(0)
{
}


void RosVisualNavigator::setupTopics ()
{
  node_.subscribe("odom", odom_message_, &RosVisualNavigator::odomCallback, this, 1);
  node_.subscribe("roadmap", roadmap_message_, &RosVisualNavigator::roadmapCallback, this, 1);
  node_.advertise<Planner2DGoal>("goal", 1);
  node_.advertise<VisualizationMarker>( "visualizationMarker", 0 );
}


void RosVisualNavigator::run ()
{
  Duration d(1);

  // Wait for map and odom messages
  while (!map_received_ || !odom_received_) {
    d.sleep();
    ROS_INFO_COND_NAMED (!map_received_, "node", "Waiting for map message");
    ROS_INFO_COND_NAMED (!odom_received_, "node", "Waiting for odom message");
  }

  
  // Main loop: at each iteration, generate a plan to goal and publish exit point
  while (true) {
    d.sleep();

    updateOdomTransform();
    publishVisualization();

    PathPtr path = roadmap_->pathToGoal(start_id_, goal_id_);
    Pose exit_point = roadmap_->pathExitPoint(path, exit_point_radius_);
    node_.publish("goal", createGoalMessage(exit_point));
  }
}



/************************************************************
 * callbacks
 ************************************************************/

void RosVisualNavigator::odomCallback ()
{
  odom_received_=true;
}


void RosVisualNavigator::roadmapCallback ()
{
  vector<vslam::Node> nodes;
  vector<Edge> edges;

  ROS_DEBUG_NAMED ("node", "In roadmap callback");
  // No need to acquire lock since callback does this automatically

  roadmap_message_.get_nodes_vec(nodes);
  roadmap_message_.get_edges_vec(edges);

  for_each (nodes.begin(), nodes.end(), AddNodeToRoadmap(this));
  for_each (edges.begin(), edges.end(), AddEdgeToRoadmap(this));

  ROS_ASSERT (roadmap_message_.localization>=0);
  start_id_=id_map_[roadmap_message_.localization];
  map_received_=true;

}





/************************************************************
 * Visualization
 ************************************************************/

// Functor for publishing each edge of the roadmap
struct PublishGuiEdge
{
  PublishGuiEdge(RoadmapPtr roadmap, const Transform2D& nav_odom_transform, uint id, uint* num_active_markers) : 
    roadmap(roadmap), nav_odom_transform(nav_odom_transform), id(id), num_active_markers(num_active_markers) 
  {
    Pose transformed_pose = transform(nav_odom_transform, roadmap->nodePose(id));
    marker.header.frame_id="odom";
    marker.type=VisualizationMarker::LINE_STRIP;
    marker.action=VisualizationMarker::ADD;
    marker.alpha=255;
    marker.set_points_size(2);
    marker.points[0].x=transformed_pose.x;
    marker.points[0].y=transformed_pose.y;
    marker.xScale=.3;
    marker.r=0;
    marker.g=0;
    marker.b=255;
  }

  void operator() (NodeId j)
  {
    Pose transformed_pose = transform(nav_odom_transform, roadmap->nodePose(j));
    marker.header.stamp = ros::Time::now();
    marker.id=(*num_active_markers)++;
    marker.points[1].x=transformed_pose.x;
    marker.points[1].y=transformed_pose.y;
    Node::instance()->publish("visualizationMarker", marker);
  }

  RoadmapPtr roadmap;
  const Transform2D& nav_odom_transform;
  uint id;
  uint* num_active_markers;
  VisualizationMarker marker;
};
    


// Publish the full roadmap
void RosVisualNavigator::publishVisualization () 
{
  ROS_DEBUG_NAMED ("rviz", "Publishing visualization");
  RoadmapLock lock(this);

  // First, delete the old ones
  for (uint i=0; i<num_active_markers_; ++i) {
    VisualizationMarker marker;
    marker.id=i;
    marker.action=VisualizationMarker::DELETE;
    node_.publish("visualizationMarker", marker);
  }
  num_active_markers_=0;


  /// Now add the new ones
  /// \todo remove dependency on node id's going from 0 to numNodes-1
  for (uint i=0; i<roadmap_->numNodes(); ++i) {
    Pose p1 = roadmap_->nodePose(i);
    vector<NodeId> neighbors = roadmap_->neighbors(i);
    for_each (neighbors.begin(), neighbors.end(), PublishGuiEdge(roadmap_, nav_odom_transform_, i, &num_active_markers_));
  }

  // Publish a marker for current position
  VisualizationMarker marker;
  Pose transformed_pose = transform(nav_odom_transform_, roadmap_->nodePose(start_id_));
  ROS_DEBUG_STREAM_NAMED ("rviz", "Odom coordinates of start are " << transformed_pose);
  marker.header.frame_id="odom";
  marker.header.stamp = ros::Time::now();
  marker.id=num_active_markers_++;
  marker.type=VisualizationMarker::SPHERE;
  marker.action=VisualizationMarker::ADD;
  marker.x=0;
  marker.y=0;
  marker.alpha=255;
  marker.r=255;
  marker.g=0;
  marker.b=0;
  marker.xScale=5;
  marker.yScale=5;
  marker.zScale=5;
  node_.publish("visualizationMarker", marker);


  ROS_DEBUG_STREAM_NAMED ("rviz", "Finished publishing roadmap with " << num_active_markers_ << " edges");
}





/************************************************************
 * internal
 ************************************************************/

NodeId RosVisualNavigator::getInternalId(const uint id)
{
  IdMap::iterator pos = id_map_.find(id);
  if (pos==id_map_.end()) {
    throw UnknownExternalId(id);
  }
  return pos->second;
}


Planner2DGoal RosVisualNavigator::createGoalMessage (const Pose& pose) const
{
  Planner2DGoal m;
  Pose transformed_pose=transform(nav_odom_transform_, pose);

  m.goal.x = transformed_pose.x;
  m.goal.y = transformed_pose.y;
  m.goal.th = transformed_pose.theta;
  m.header.frame_id = "odom";
  m.enable = 1;
  return m;
}


// Update nav_odom_transform_
void RosVisualNavigator::updateOdomTransform ()
{
  RoadmapLock lock(this);
  ROS_ASSERT (odom_received_&&map_received_);

  Pose odom_pose(odom_message_.pos.x, odom_message_.pos.y, odom_message_.pos.th);
  nav_odom_transform_ = getTransformBetween(roadmap_->nodePose(start_id_), odom_pose);
}




} // namespace visual_nav



/************************************************************
 * Main
 ************************************************************/


using visual_nav::RosVisualNavigator;



int main(int argc, char** argv)
{
  ros::init (argc, argv);

  // Eventually, parse the command line arguments

  RosVisualNavigator nav(5.0, 5);

  nav.setupTopics();

  nav.run();

}


