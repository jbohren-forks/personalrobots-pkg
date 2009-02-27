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
#include <boost/program_options.hpp>
#include <ros/node.h>
#include <ros/time.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <tf/transform_listener.h>
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

namespace po=boost::program_options;

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
  void roadmapCallback();

private:

  /******************************
   * Internal classes
   ******************************/

  // RAII class to encapsulate lock on roadmap message and object, start_id_, and nav_odom_transform_
  struct RoadmapLock
  {
    RoadmapLock (RosVisualNavigator* nav) : nav(nav) 
    { 
      ROS_DEBUG_NAMED("lock", "Acquiring RoadmapLock...");
      nav->roadmap_message_.lock(); 
      ROS_DEBUG_NAMED("lock", "Acquired RoadmapLock"); 
    }
    ~RoadmapLock () { nav->roadmap_message_.unlock(); ROS_DEBUG_NAMED("lock", "Released RoadmapLock"); }
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

  // Publish a sphere for a given pose and color
  void publishNodeMarker (const Pose& pose, uint r, uint g, uint b);

  // Publish a goal message to the controller
  void publishGoalMessage (const Pose& goal_pose);

  // Update the nav-to-odom transform 
  void updateOdom ();

  /// \returns Internally used id corresponding to external (as defined by ros message from vslam) id \a id
  /// \throws UnknownExternalId
  NodeId getInternalId(uint id);


  /******************************
   * Fields
   ******************************/

  // Node object
  Node node_;

  // Listens to transforms
  tf::TransformListener tf_;

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

  // The current exit point we're aiming for (in the odom frame)
  Pose exit_point_;

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
  node_("visual_navigator"), tf_(node_), map_received_(false), odom_received_(false), 
  exit_point_radius_(exit_point_radius), goal_id_(goal_id), num_active_markers_(0)
{
}


void RosVisualNavigator::setupTopics ()
{
  node_.subscribe("roadmap", roadmap_message_, &RosVisualNavigator::roadmapCallback, this, 1);
  node_.advertise<Planner2DGoal>("goal", 1);
  node_.advertise<VisualizationMarker>( "visualizationMarker", 0 );
}


void RosVisualNavigator::run ()
{
  Duration d(2);

  // Wait for map and odom messages
  while (!map_received_ || !odom_received_) {
    d.sleep();
    updateOdom();
    ROS_INFO_COND_NAMED (!map_received_, "node", "Waiting for map message");
    ROS_INFO_COND_NAMED (!odom_received_, "node", "Waiting for odom message");
  }

  
  // Main loop: at each iteration, generate a plan to goal and publish exit point
  while (true) {
    d.sleep();
    updateOdom();

    try {
      PathPtr path = roadmap_->pathToGoal(start_id_, goal_id_);
      exit_point_ = transform(nav_odom_transform_, roadmap_->pathExitPoint(path, exit_point_radius_));
      publishGoalMessage(exit_point_);

      publishVisualization();
    }

    // Ignore goals that don't exist yet (to deal with the case where we start from an empty map and build up)
    catch (UnknownNodeIdException& r) 
    {
      if (r.id == goal_id_) {
        ROS_INFO_NAMED ("node", "The goal id %u was unknown, so ignoring", r.id);
      }
      else {
        throw;
      }
    }
  }
}



/************************************************************
 * callbacks
 ************************************************************/

void RosVisualNavigator::roadmapCallback ()
{
  vector<vslam::Node> nodes;
  vector<Edge> edges;

  ROS_DEBUG_NAMED ("node", "In roadmap callback");
  // No need to acquire lock since callback does this automatically

  roadmap_message_.get_nodes_vec(nodes);
  roadmap_message_.get_edges_vec(edges);

  roadmap_ = RoadmapPtr(new VisualNavRoadmap);

  for_each (nodes.begin(), nodes.end(), AddNodeToRoadmap(this));
  for_each (edges.begin(), edges.end(), AddEdgeToRoadmap(this));

  // We're treating loc>=0 as denoting an actual map, loc<0 as initial empty map, and everything else as an error
  int localization=roadmap_message_.localization;
  if (localization<0) {
    ROS_DEBUG_NAMED ("node", "Treating localization %d as denoting empty roadmap", localization);
  }
  else {
    start_id_=id_map_[localization];
    map_received_=true;
  }
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


void RosVisualNavigator::publishGoalMessage (const Pose& pose) 
{
  Planner2DGoal m;

  m.goal.x = pose.x;
  m.goal.y = pose.y;
  m.goal.th =pose.theta;
  m.header.frame_id = "odom";
  m.enable = 1;
  node_.publish("goal", m);
}


// Update nav_odom_transform_
void RosVisualNavigator::updateOdom ()
{
  RoadmapLock lock(this);

  // Use tf to get the odom pose
  StampedPose identity, odom_pose;
  identity.setIdentity();
  identity.frame_id_ = "base_link";
  identity.stamp_ = ros::Time();

  
  try {
    tf_.transformPose("odom", identity, odom_pose);
    odom_received_=true;
    
    // If we have a roadmap as well, figure out the transform between them 
    if (roadmap_) {
      nav_odom_transform_ = getTransformBetween(roadmap_->nodePose(start_id_), Pose(odom_pose));
      ROS_DEBUG_STREAM_NAMED ("transform", "Updating odom pose to " << Pose(odom_pose) << " resulting in nav_odom_transform " << nav_odom_transform_);
    }
  }
  catch (UnknownNodeIdException& e) {
    ROS_WARN_STREAM_NAMED ("node", "Unexpectedly found that the node id " << e.id << " was unknown in the roadmap, so not updating nav_odom_transform");
  }
  catch (tf::TransformException& e) {
    ROS_INFO_STREAM_NAMED ("node", "Received tf exception " << e.what() << " when attempting to get odom pose, so skipping");
  }
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
    ROS_DEBUG_STREAM_NAMED ("rviz", "Published an edge marker with id " << marker.id << " with timestamp " << marker.header.stamp);
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
  publishNodeMarker(transform(nav_odom_transform_, roadmap_->nodePose(start_id_)), 255, 0, 0);

  // Publish one for exit point
  publishNodeMarker(exit_point_, 0, 255, 0);

  // And one for the goal
  publishNodeMarker(transform(nav_odom_transform_, roadmap_->nodePose(goal_id_)), 255, 255, 0);

  ROS_DEBUG_STREAM_NAMED ("rviz", "Finished publishing roadmap with " << num_active_markers_ << " edges");
}


void RosVisualNavigator::publishNodeMarker (const Pose& pose, const uint r, const uint g, const uint b)
{
  VisualizationMarker marker;
  marker.header.frame_id="odom";
  marker.header.stamp = ros::Time::now();
  marker.id=num_active_markers_++;
  marker.type=VisualizationMarker::SPHERE;
  marker.action=VisualizationMarker::ADD;
  marker.x=pose.x;
  marker.y=pose.y;
  marker.yaw=pose.theta;
  marker.alpha=255;
  marker.r=r;
  marker.g=g;
  marker.b=b;
  marker.xScale=1;
  marker.yScale=.4;
  marker.zScale=.2;
  node_.publish("visualizationMarker", marker);
  ROS_DEBUG_STREAM_NAMED ("rviz", "Just published node marker " << marker.id << " with timestamp " << marker.header.stamp);
}


} // namespace visual_nav



/************************************************************
 * Main
 ************************************************************/


using visual_nav::RosVisualNavigator;



int main(int argc, char** argv)
{
  double radius;
  ros::init (argc, argv);

  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("exit_radius,r", po::value<double>(&radius)->default_value(2.0), "exit radius of path");
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);    

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }

  
  ROS_DEBUG_NAMED ("node", "Using exit radius %f", radius);
  

  RosVisualNavigator nav(radius, 2);

  nav.setupTopics();

  nav.run();

}


