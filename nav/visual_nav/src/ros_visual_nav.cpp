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
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <laser_scan/laser_scan.h>
#include <ros/ros.h>
#include <ros/node.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_notifier.h>
#include <visual_nav/visual_nav.h>
#include <visualization_msgs/Polyline.h>
#include <visualization_msgs/Marker.h>
#include <vslam/Roadmap.h>
#include <visual_nav/exceptions.h>
#include <visual_nav/VisualNavGoal.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

using std::string;
using std::vector;
using geometry_msgs::PoseStamped;
using geometry_msgs::Point;
using geometry_msgs::PointStamped;
using geometry_msgs::Point32;
using ros::Duration;
using vslam::Roadmap;
using visualization_msgs::Polyline;
using ros::Node;
using vslam::Edge;
using std::map;
using visual_nav::VisualNavGoal;
using tf::MessageNotifier;
using boost::shared_ptr;
using boost::mutex;
using std_msgs::String;

namespace po=boost::program_options;

namespace visual_nav
{

using visualization_msgs::Marker; // the message class name
using ros::Time;
using std::string;
using ros::Node;

typedef map<int, NodeId> IdMap;
typedef shared_ptr<PointSet> PointsPtr;
typedef MessageNotifier<sensor_msgs::LaserScan> Notifier;
typedef shared_ptr<Notifier> NotifierPtr;
typedef map<string, NodeId> NodeNameMap;



/************************************************************
 * Node class
 ************************************************************/

class RosVisualNavigator
{
public:

  RosVisualNavigator (double exit_point_radius, uint scan_period);
  ~RosVisualNavigator ();

  // Spin
  void run();

  // Callbacks
  void roadmapCallback();
  void goalCallback();
  void nameCallback();
  void poseCallback();
  void baseScanCallback(const Notifier::MessagePtr& msg);

private:

  /******************************
   * Internal classes
   ******************************/

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
      nav->roadmap_->addEdge(nav->getInternalId(e.node0), nav->getInternalId(e.node1), e.length);
    }
    RosVisualNavigator* nav;
  };
  friend struct AddEdgeToRoadmap;

  // Observed scans at a pose
  struct ObservedScan
  {
    ObservedScan (NodeId id, PointsPtr points, const Pose& pose) : id(id), points(points), pose(pose) {}
    NodeId id;
    PointsPtr points;
    Pose pose;
  };
  typedef vector<ObservedScan> ObsScanVector;


  /******************************
   * Ops
   ******************************/

  // Send the current map, robot position, and goal to visualizer
  void publishVisualization ();

  // Get rid of existing markers
  void deleteOldMarkers ();

  // Publish a sphere for a given pose and color
  void publishNodeMarker (const Pose& pose, uint r, uint g, uint b);

  // Compute and publish exit point
  void publishExitPoint ();

  /// \returns Internally used id corresponding to external (as defined by ros message from vslam) id \a id
  /// \throws UnknownExternalId
  NodeId getInternalId(uint id);


  /******************************
   * Fields
   ******************************/

  // Node object
  Node node_;

  // Listens to transforms
  tf::TransformListener tf_listener_;

  // Sends them
  tf::TransformBroadcaster tf_sender_;

  // For now just have a single mutex
  mutex node_mutex_;

  // Roadmap, and roadmap message used to populate it
  // Requires lock to access
  RoadmapPtr roadmap_;
  Roadmap roadmap_message_;

  // Goal messages
  VisualNavGoal goal_message_;

  // Name messages
  String node_name_message_;
  

  // Has at least one map message been received?
  bool map_received_;

  // Map between external id's and our internal node ids
  IdMap id_map_;

  // Timestamp of last received roadmap
  Time roadmap_timestamp_;

  // the radius of the window used to find the exit point of the path
  double exit_point_radius_;

  // The current exit point we're aiming for
  Pose exit_point_;

  // Goal node's (external) id
  NodeId goal_id_;

  // Is there a goal right now?  Set by goal callback and main loop.
  bool have_goal_;

  // Start node's (internal) id.  Access requires lock.
  NodeId start_id_;

  // Number of visualization markers currently being displayed by this node
  uint num_active_markers_;

  // Name of vslam roadmap frame
  string vslam_frame_;

  // Used to project laser scans
  laser_scan::LaserProjection projector_;

  // Previously observed scans.  Requires lock.
  ObsScanVector observed_scans_;

  // How often to save scans
  uint scan_counter_, scan_period_;

  // Message notifier to ensure that we only deal with scans when we can transform them
  NotifierPtr base_scan_notifier_;

  // How close we need to be to declare success on goal
  double goal_distance_threshold_;

  // Maps names to node ids
  NodeNameMap node_name_map_;


};



/************************************************************
 * top level
 ************************************************************/

RosVisualNavigator::RosVisualNavigator (double exit_point_radius, uint scan_period) :
  node_("visual_navigator"), tf_listener_(node_), map_received_(false),
  exit_point_radius_(exit_point_radius), have_goal_(false),
  num_active_markers_(0), scan_counter_(1), scan_period_(scan_period)
{
  node_.subscribe("roadmap", roadmap_message_, &RosVisualNavigator::roadmapCallback, this, 1);
  node_.subscribe("visual_nav_goal", goal_message_, &RosVisualNavigator::goalCallback, this, 1);
  node_.subscribe("name_node", node_name_message_, &RosVisualNavigator::nameCallback, this, 1);
  node_.param("~vslam_frame", vslam_frame_, string("base_link"));
  base_scan_notifier_ = NotifierPtr(new Notifier(&tf_listener_, ros::Node::instance(),  bind(&RosVisualNavigator::baseScanCallback, this, _1), "base_scan", vslam_frame_, 50));
  node_.advertise<PoseStamped>("/move_base/activate", 1);
  node_.advertise<Marker>( "visualization_marker", 0 );
  node_.advertise<Polyline> ("vslam_laser", 1);
  node_.param("~goal_distance_threshold", goal_distance_threshold_, .5);

  ROS_INFO_STREAM ("Started RosVisualNavigator with exit_point_radius=" << exit_point_radius_ << ", goal_id=" << goal_id_) ;

}


RosVisualNavigator::~RosVisualNavigator ()
{
  deleteOldMarkers();
}


void RosVisualNavigator::run ()
{
  Duration d(0.1);

  while (true) {

    if (!map_received_) {
      ROS_DEBUG_NAMED ("node", "Waiting for roadmap message");
    }

    // If map received, send exit point goal if necessary, and publish visualization
    else {
      if (have_goal_) {
        
        if (distance(roadmap_->nodePose(start_id_), roadmap_->nodePose(goal_id_)) < goal_distance_threshold_) {
          have_goal_ = false;
          ROS_INFO_STREAM ("Achieved goal " << goal_id_);
        }
        else {
          publishExitPoint();
        }
      }
      publishVisualization();
    }

    d.sleep();
  }
}





/************************************************************
 * callbacks
 ************************************************************/

void RosVisualNavigator::roadmapCallback ()
{
  vector<vslam::Node> nodes;
  vector<Edge> edges;

  mutex::scoped_lock l(node_mutex_);

  roadmap_message_.get_nodes_vec(nodes);
  roadmap_message_.get_edges_vec(edges);

  roadmap_ = RoadmapPtr(new VisualNavRoadmap);
  roadmap_timestamp_ = roadmap_message_.header.stamp;

  if (roadmap_message_.header.frame_id != vslam_frame_) {
    ROS_FATAL_STREAM ("Received a roadmap message with frame id " << roadmap_message_.header.frame_id << 
                      " but expected vslam frame is " << vslam_frame_);
    exit(0);
  }

  for_each (nodes.begin(), nodes.end(), AddNodeToRoadmap(this));
  for_each (edges.begin(), edges.end(), AddEdgeToRoadmap(this));

  // We're treating loc>=0 as denoting an actual map and loc<0 as initial empty map
  int localization=roadmap_message_.localization;
  if (localization<0) {
    ROS_DEBUG_NAMED ("node", "Treating localization %d as denoting empty roadmap", localization);
    ROS_ASSERT (!map_received_);
  }
  else {
    start_id_=id_map_[localization];
    map_received_=true;
  }
}



void RosVisualNavigator::goalCallback()
{
  have_goal_ = false;
  if (goal_message_.named_goal.length() > 0) {
    const NodeNameMap::const_iterator pos = node_name_map_.find(goal_message_.named_goal);
    if (pos==node_name_map_.end()) {
      ROS_ERROR_STREAM ("Received unknown named goal " << goal_message_.named_goal);
    }
    else {
      goal_id_ = getInternalId(pos->second);
      have_goal_ = true;
    }
  }
  else if (goal_message_.goal>=0) {
    goal_id_ = getInternalId(goal_message_.goal);
    have_goal_=true;
  }

  if (have_goal_)
    ROS_INFO_STREAM ("New goal: node " << goal_id_);
  else 
    ROS_INFO_STREAM ("Goal cancelled");
}



void RosVisualNavigator::nameCallback()
{
  node_name_map_[node_name_message_.data] = start_id_;
}


// Convert from ros point message to internal point type
inline Point2D convertPoint (const Point32& p)
{
  return Point2D(p.x, p.y);
}

// Convert the scan to a set of points and store that, together with the pose and node id at the time the scan was taken
void RosVisualNavigator::baseScanCallback(const Notifier::MessagePtr& message)
{
  if (!(scan_counter_++%scan_period_)) {
    try {
      sensor_msgs::PointCloud point_cloud;
      projector_.transformLaserScanToPointCloud (vslam_frame_, point_cloud, *message, tf_listener_);

      vector<Point32> point_vec;
      point_cloud.get_pts_vec(point_vec);
      PointsPtr points(new PointSet());
      transform(point_vec.begin(), point_vec.end(), inserter(*points, points->begin()), convertPoint);

      // Acquire lock as we are changing observed_scans_
      mutex::scoped_lock l(node_mutex_);

      observed_scans_.push_back(ObservedScan(start_id_, points, roadmap_->nodePose(start_id_)));
      ROS_DEBUG_STREAM_NAMED ("scans", "Adding an observed scan at node " << start_id_ << " at pose " << roadmap_->nodePose(start_id_) << " with initial point " << *(points->begin()));
    }
    catch (tf::TransformException& e) {
      ROS_DEBUG_STREAM_NAMED ("scans", "Received tf exception " << e.what() << " when attempting to transform base scan, so skipping");
    }
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


void RosVisualNavigator::publishExitPoint ()
{
  try {

    mutex::scoped_lock l(node_mutex_);

    PathPtr path = roadmap_->pathToGoal(start_id_, goal_id_);
    exit_point_ = roadmap_->pathExitPoint(path, exit_point_radius_);

    // publish the goal message
    PoseStamped goal;

    goal.pose.position.x = exit_point_.x;
    goal.pose.position.y = exit_point_.y;
    tf::quaternionTFToMsg(tf::Quaternion(exit_point_.theta, 0, 0), goal.pose.orientation);
    goal.header.frame_id = vslam_frame_;

    node_.publish("/move_base/activate", goal);
    
    ROS_DEBUG_STREAM_NAMED("nav", "Publishing exit point " << goal.pose.position.x << ", " 
                           << goal.pose.position.y << ", " << acos(goal.pose.orientation.w/2)
                           << " in " << goal.header.frame_id << " frame");
  }
  catch (UnknownNodeIdException& r) {
    if (r.id == goal_id_) {
      ROS_DEBUG_NAMED ("node", "The goal id %u was unknown, so ignoring", r.id);
    }
    else {
      throw;
    }
  }
}



/************************************************************
 * Visualization
 ************************************************************/


struct DrawEdges
{
  DrawEdges (RoadmapPtr roadmap, uint* num_active_markers) : roadmap(roadmap)
  {
    marker.header.frame_id = vslam_frame_;
    marker.ns = "visual_nav";
    marker.id = (*num_active_markers)++;
    marker.type=Marker::LINE_LIST;
    marker.action=Marker::ADD;
    marker.color.a=1.0;
    marker.scale.x = 0.04;
    marker.pose.orientation.w = 1.0;
    marker.color.r=0;
    marker.color.g=100;
    marker.color.b=100;
    ROS_DEBUG_STREAM_NAMED ("rviz", "Creating roadmap marker for graph with " << roadmap->numNodes() << " nodes");
  }

  void operator() (NodeId n)
  {
    NodeVector neighbors = roadmap->neighbors(n);
    Point node;
    Pose pose = roadmap->nodePose(n);


    node.x = pose.x;
    node.y = pose.y;
    for (NodeVector::iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter) {
      if (*iter > n) {
        marker.points.push_back(node);
        Point neighbor;
        Pose neighbor_pose = roadmap->nodePose(*iter);
        neighbor.x = neighbor_pose.x;
        neighbor.y = neighbor_pose.y;
        marker.points.push_back(neighbor);
      }
    }
  }

  void publish ()
  {
    if (marker.get_points_size() > 0) {
      ROS_DEBUG_STREAM_NAMED ("rviz", "Publishing graph with " << marker.get_points_size() << " edges and " << roadmap->numNodes() << " nodes");
      marker.header.stamp = Time::now();
      Node::instance()->publish("visualization_marker", marker);
    }
    else {
      ROS_DEBUG_STREAM_NAMED ("rviz", "Not yet publishing roadmap visualization.  Num nodes is " << roadmap->numNodes());
    }
  }

  RoadmapPtr roadmap;
  Marker marker;
};



Point transformToPolylinePoint (tf::TransformListener* tf, const Point2D& p)
{
  Point visualized_point;
  visualized_point.x = p.x;
  visualized_point.y = p.y;
  return visualized_point;
}

void RosVisualNavigator::deleteOldMarkers ()
{
  for (uint i=0; i<num_active_markers_; ++i) {
    Marker marker;
    marker.ns = "visual_nav";
    marker.header.frame_id = vslam_frame_;
    marker.id=i;
    marker.action=Marker::DELETE;
    node_.publish("visualization_marker", marker);
  }

  ROS_DEBUG_STREAM_NAMED ("rviz", "Deleted " << num_active_markers_ << " old markers upon node destruction");

  num_active_markers_=0;
}

// Publish the full roadmap
void RosVisualNavigator::publishVisualization ()
{
  ROS_DEBUG_NAMED ("rviz", "Publishing visualization");

  // Figure out and publish transform from vslam to map frame based on known pose of node 0
  // Commenting out as it was inducing a cycle in transform tree when fake_localization was used


  deleteOldMarkers();


  /// Now add the new ones and publish, while holding lock
  mutex::scoped_lock l(node_mutex_);

  // First, use a linelist marker to draw the skeleton
  NodeVector nodes = roadmap_->nodes();
  for_each(nodes.begin(), nodes.end(), DrawEdges(roadmap_, &num_active_markers_)).publish();


  // Publish a marker for current position
  publishNodeMarker(roadmap_->nodePose(start_id_), 255, 0, 0);

  // One for node 0 of the roadmap
  publishNodeMarker(roadmap_->nodePose(0), 255, 0, 255);

  // Markers for goal and exit point if they exist
  if (have_goal_) {
    publishNodeMarker(roadmap_->nodePose(goal_id_), 255, 255, 0);
    publishNodeMarker(exit_point_, 0, 255, 0);
  }


  // Combine scans
  for (ObsScanVector::iterator iter=observed_scans_.begin(); iter!=observed_scans_.end(); ++iter) {
    roadmap_->attachScan(iter->id, *(iter->points), iter->pose);
  }
  PointSet points = roadmap_->overlayScans();

  // Publish as polyline
  Polyline scans;
  scans.header.frame_id=vslam_frame_;
  scans.set_points_size(points.size());
  scans.color.b=1.0;
  transform (points.begin(), points.end(), scans.points.begin(), bind(transformToPolylinePoint, &tf_listener_, _1));
  ROS_DEBUG_STREAM_COND_NAMED (points.size()>0, "scans", "First scan point in vslam frame is " << *(points.begin())
                               << " and transformed version is " << scans.points[0].x << ", " << scans.points[0].y << ", " << scans.points[0].z);
  ROS_DEBUG_STREAM_NAMED ("rviz", "Publishing scans with " << scans.get_points_size() << " points");
  node_.publish("vslam_laser", scans);

  ROS_DEBUG_STREAM_NAMED ("rviz", "Finished publishing roadmap with " << roadmap_->numNodes() << " nodes");
}




void RosVisualNavigator::publishNodeMarker (const Pose& pose, const uint r, const uint g, const uint b)
{
  Marker marker;
  marker.header.frame_id=vslam_frame_;
  marker.header.stamp = Time::now();
  marker.ns="visual_nav";
  marker.id=num_active_markers_++;
  marker.type=Marker::SPHERE;
  marker.action=Marker::ADD;
  marker.pose.position.x=pose.x;
  marker.pose.position.y=pose.y;

  btQuaternion orient(pose.theta, 0, 0);
  marker.pose.orientation.x=orient.getX();
  marker.pose.orientation.y=orient.getY();
  marker.pose.orientation.z=orient.getZ();
  marker.pose.orientation.w=orient.getW();

  marker.color.a=1.0;
  marker.color.r=r;
  marker.color.g=g;
  marker.color.b=b;
  marker.scale.x=1;
  marker.scale.y=.4;
  marker.scale.z=.2;
  node_.publish("visualization_marker", marker);
  ROS_DEBUG_STREAM_NAMED ("rviz", "Just published node marker " << marker.id << " with timestamp " << marker.header.stamp << " with vslam pose " << pose);
}


} // namespace visual_nav



/************************************************************
 * Main
 ************************************************************/


using visual_nav::RosVisualNavigator;

int main(int argc, char** argv)
{
  double radius;
  uint scan_period;

  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("exit_radius,r", po::value<double>(&radius)->default_value(2.0), "exit radius of path")
    ("scan_period_,s", po::value<uint>(&scan_period)->default_value(50), "Store one in this many scans");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }

  ros::init (argc, argv);

  RosVisualNavigator nav(radius, scan_period);

  nav.run();

}


