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
#include <ros/node.h>
#include <deprecated_msgs/RobotBase2DOdom.h>
#include <robot_msgs/Point.h>
#include <robot_msgs/PointCloud.h>
#include <robot_msgs/PointStamped.h>
#include <laser_scan/laser_scan.h>
#include <ros/time.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_notifier.h>
#include <visual_nav/visual_nav.h>
#include <robot_msgs/Polyline.h>
#include <pr2_robot_actions/Pose2D.h>
#include <robot_msgs/VisualizationMarker.h>
#include <vslam/Roadmap.h>
#include <visual_nav/exceptions.h>
#include <visual_nav/VisualNavGoal.h>


using std::string;
using std::vector;
using pr2_robot_actions::Pose2D;
using robot_msgs::Point;
using robot_msgs::PointStamped;
using robot_msgs::Point32;
using laser_scan::LaserScan;
using ros::Duration;
using vslam::Roadmap;
using robot_msgs::Polyline;
using ros::Node;
using vslam::Edge;
using std::map;
using visual_nav::VisualNavGoal;
using tf::MessageNotifier;
using boost::shared_ptr;
using boost::mutex;

namespace po=boost::program_options;

namespace visual_nav
{

using robot_msgs::VisualizationMarker; // the message class name
using ros::Time;
using deprecated_msgs::RobotBase2DOdom;
using std::string;


typedef map<int, NodeId> IdMap;
typedef shared_ptr<PointSet> PointsPtr;
typedef MessageNotifier<LaserScan> Notifier;
typedef shared_ptr<Notifier> NotifierPtr;




/************************************************************
 * Node class
 ************************************************************/

class RosVisualNavigator
{
public:

  // Constructor
  RosVisualNavigator (double exit_point_radius, const Pose& init_pose, uint scan_period);

  // Subscribe topics
  void setupTopics();

  // Spin
  void run();

  // Callbacks
  void roadmapCallback();
  void goalCallback();
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
      nav->roadmap_->addEdge(nav->getInternalId(e.node0), nav->getInternalId(e.node1));
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

  // Publish a sphere for a given pose and color
  void publishNodeMarker (const Pose& pose, uint r, uint g, uint b);

  // Compute and publish exit point
  void publishExitPoint ();

  // Update the nav-to-odom transform and publish it to tf
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
  tf::TransformListener tf_listener_;

  // Sends them
  tf::TransformBroadcaster tf_sender_;

  // For now just have a single mutex
  mutex node_mutex_;

  // Roadmap, and roadmap message used to populate it
  // Requires lock to access
  RoadmapPtr roadmap_;
  Roadmap roadmap_message_;

  // Temporary
  RobotBase2DOdom pose_message_;

  // Goal messages
  VisualNavGoal goal_message_;

  // Has at least one map message been received?
  bool map_received_;

  // Map between external id's and our internal node ids
  IdMap id_map_;

  // has at least one odometry message been received?
  bool odom_received_;

  // Timestamp of last received roadmap
  Time roadmap_timestamp_;

  // the radius of the window used to find the exit point of the path 
  double exit_point_radius_;

  // The current exit point we're aiming for
  Pose exit_point_;

  // Goal node's (external) id
  NodeId goal_id_;

  // Is there a goal right now?
  bool have_goal_;

  // Start node's (internal) id.  Access requires lock.
  NodeId start_id_;

  // Transform between the nav frame (used by the visual roadmap) and the odometry frame (used to give goals to local controller)
  // Access requires lock
  Transform2D nav_odom_transform_;

  // Number of visualization markers currently being displayed by this node
  uint num_active_markers_;

  // Initial pose in map frame
  const Pose init_map_pose_;

  // Name of odom frame
  string odom_frame_;

  // Used to project laser scans
  laser_scan::LaserProjection projector_;

  // Previously observed scans.  Requires lock.
  ObsScanVector observed_scans_;

  // How often to save scans
  uint scan_counter_, scan_period_;

  // Message notifier to ensure that we only deal with scans when we can transform them
  NotifierPtr base_scan_notifier_;

};



/************************************************************
 * top level
 ************************************************************/

// Constructor
RosVisualNavigator::RosVisualNavigator (double exit_point_radius, const Pose& init_pose, uint scan_period) : 
  node_("visual_navigator"), tf_listener_(node_), tf_sender_(node_), map_received_(false), 
  odom_received_(false), exit_point_radius_(exit_point_radius), have_goal_(false),
  num_active_markers_(0), init_map_pose_(init_pose), scan_counter_(1), scan_period_(scan_period)
{
}


void RosVisualNavigator::setupTopics ()
{
  node_.subscribe("roadmap", roadmap_message_, &RosVisualNavigator::roadmapCallback, this, 1);
  node_.subscribe("localizedpose", pose_message_, &RosVisualNavigator::poseCallback, this, 1);
  node_.subscribe("visual_nav_goal", goal_message_, &RosVisualNavigator::goalCallback, this, 1);
  base_scan_notifier_ = NotifierPtr(new Notifier(&tf_listener_, ros::Node::instance(),  bind(&RosVisualNavigator::baseScanCallback, this, _1), "base_scan", "vslam", 50));
  node_.advertise<Pose2D>("/move_base_node/activate", 1);
  node_.advertise<VisualizationMarker>( "visualizationMarker", 0 );
  node_.advertise<Polyline> ("vslam_laser", 1);
  node_.param("~odom_frame", odom_frame_, string("odom"));

  ROS_INFO_STREAM ("Started RosVisualNavigator with exit_point_radius=" << exit_point_radius_ << ", goal_id=" << goal_id_ << ", init_pose=" << init_map_pose_ << ", odom_frame=" << odom_frame_);
  
}



void RosVisualNavigator::run ()
{
  Duration d(0.1); 

  // Wait for map and odom messages
  for (uint i=0; ; ++i) {

    // Wait till we have an initial value for odom and map
    updateOdom();
    if (!map_received_ || !odom_received_) {
      ROS_INFO_COND_NAMED ((!map_received_) && !(i%10), "node", "Waiting for map message");
      ROS_INFO_COND_NAMED (!(i%10) && (!odom_received_), "node", "Waiting for odom message");
    }

    // Otherwise, send exit point goal if necessary, and publish visualization
    else {
      if (have_goal_) {
        publishExitPoint();
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

void RosVisualNavigator::poseCallback ()
{
  // ROS_INFO_STREAM ("Time is " << Time::now() << " and pose stamp is " << pose_message_.header.stamp);
}


void RosVisualNavigator::goalCallback()
{
  if (goal_message_.goal>=0) {
    goal_id_ = getInternalId(goal_message_.goal);
    have_goal_=true;
  }
  else {
    have_goal_=false;
  }
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
      robot_msgs::PointCloud point_cloud;
      projector_.transformLaserScanToPointCloud ("vslam", point_cloud, *message, tf_listener_);

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
    Pose2D m;

    m.x = exit_point_.x;
    m.y = exit_point_.y;
    m.th = exit_point_.theta;
    m.header.frame_id = "vslam";
    node_.publish("/move_base_node/activate", m);
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



// Update nav_odom_transform_ and publish it over tf_message
void RosVisualNavigator::updateOdom ()
{
  mutex::scoped_lock l(node_mutex_);
  
  try {
    if (map_received_ && tf_listener_.canTransform(odom_frame_, "base_footprint", roadmap_timestamp_)) {

      // Use tf to get the odom pose
      StampedPose identity, odom_pose;
      identity.setIdentity();
      identity.frame_id_ = "base_footprint";
      identity.stamp_ = roadmap_timestamp_;
      tf_listener_.transformPose(odom_frame_, identity, odom_pose);
      odom_received_=true;
    
      nav_odom_transform_ = getTransformBetween(roadmap_->nodePose(start_id_), Pose(odom_pose));
      ROS_DEBUG_STREAM_NAMED ("transform", "Odom pose is " << Pose(odom_pose) << " and vslam pose is " << roadmap_->nodePose(start_id_) 
                              << " resulting in nav_odom_transform " << nav_odom_transform_);
      tf_sender_.sendTransform (nav_odom_transform_.convertToTf(), Time::now(), "vslam", odom_frame_);
    }
    else {
      ROS_DEBUG_STREAM_NAMED ("transform", "Did not update nav-odom transform because roadmap either not received or transform does not exist at time " << roadmap_timestamp_);
    }
  }


  catch (UnknownNodeIdException& e) {
    ROS_WARN_STREAM_NAMED ("node", "Unexpectedly found that the node id " << e.id << " was unknown in the roadmap, so not updating nav_odom_transform");
  }
  catch (tf::TransformException& e) {
    ROS_ERROR_STREAM_NAMED ("node", "Received tf exception " << e.what() << " when attempting to get odom pose. This shouldn't have happened as we checked beforehand that transform is possible.");
  }
}



/************************************************************
 * Visualization
 ************************************************************/
    

struct DrawEdges
{
  DrawEdges (RoadmapPtr roadmap, uint* num_active_markers) : roadmap(roadmap)
  {
    marker.header.frame_id="vslam";
    marker.id = (*num_active_markers)++;
    marker.type=VisualizationMarker::LINE_LIST;
    marker.action=VisualizationMarker::ADD;
    marker.alpha=255;
    marker.xScale=.1;
    marker.r=0;
    marker.g=0;
    marker.b=255;
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
    ROS_INFO_STREAM_NAMED ("rviz", "Publishing graph with " << marker.get_points_size() << " edges and " << roadmap->numNodes() << " nodes");
    marker.header.stamp = Time::now();
    Node::instance()->publish("visualizationMarker", marker);
  }

  RoadmapPtr roadmap;
  VisualizationMarker marker;
};



Point transformToMapFrame (tf::TransformListener* tf, const Point2D& p, const string& odom_frame)
{
  PointStamped point, transformed_point;
  point.point.x = p.x;
  point.point.y = p.y;
  point.header.frame_id = "vslam";
  tf->transformPoint(odom_frame, point, transformed_point);
  Point visualized_point;
  visualized_point.x = transformed_point.point.x;
  visualized_point.y = transformed_point.point.y;
  visualized_point.z = transformed_point.point.z;
  return visualized_point;
}

// Publish the full roadmap
void RosVisualNavigator::publishVisualization () 
{
  ROS_DEBUG_NAMED ("rviz", "Publishing visualization");

  // Figure out and publish transform from vslam to map frame based on known pose of node 0
  // Commenting out as it was inducing a cycle in transform tree when fake_localization was used
  // tf_sender_.sendTransform(getTransformBetween(roadmap_->nodePose(0), init_map_pose_).convertToTf(), Time::now(), "map", "vslam");


  // First, delete the old ones
  for (uint i=0; i<num_active_markers_; ++i) {
    VisualizationMarker marker;
    marker.id=i;
    marker.action=VisualizationMarker::DELETE;
    node_.publish("visualizationMarker", marker);
  }
  num_active_markers_=0;


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

  // Transform to the odom frame (note this is no longer necessary since rviz polylines now accept headers)
  PointSet points = roadmap_->overlayScans();
  Polyline scans;
  scans.header.frame_id=odom_frame_;
  scans.set_points_size(points.size());
  scans.color.b=1.0;
  transform (points.begin(), points.end(), scans.points.begin(), bind(transformToMapFrame, &tf_listener_, _1, odom_frame_));
  ROS_DEBUG_STREAM_COND_NAMED (points.size()>0, "scans", "First scan point in vslam frame is " << *(points.begin()) 
                               << " and transformed version is " << scans.points[0].x << ", " << scans.points[0].y << ", " << scans.points[0].z);
  ROS_DEBUG_STREAM_NAMED ("rviz", "Publishing scans with " << scans.get_points_size() << " points");
  node_.publish("vslam_laser", scans);

  ROS_DEBUG_STREAM_NAMED ("rviz", "Finished publishing roadmap with " << roadmap_->numNodes() << " nodes");
}




void RosVisualNavigator::publishNodeMarker (const Pose& pose, const uint r, const uint g, const uint b)
{
  VisualizationMarker marker;
  marker.header.frame_id="vslam";
  marker.header.stamp = Time::now();
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
  ROS_DEBUG_STREAM_NAMED ("rviz", "Just published node marker " << marker.id << " with timestamp " << marker.header.stamp << " with vslam pose " << pose);
}


} // namespace visual_nav



/************************************************************
 * Main
 ************************************************************/


using visual_nav::RosVisualNavigator;
using ros::Node;


int main(int argc, char** argv)
{
  double radius;
  ros::init (argc, argv);
  string init_pose_str;
  visual_nav::Pose init_pose;
  uint scan_period;

  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("initial_map_pose,i", po::value<string>(&init_pose_str), "The initial pose in map frame.  Used to publish the vslam-map transform (purely for visualization).")
    ("exit_radius,r", po::value<double>(&radius)->default_value(2.0), "exit radius of path")
    ("scan_period_,s", po::value<uint>(&scan_period)->default_value(50), "Store one in this many scans");
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);   

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }

  if (vm.count("initial_map_pose")) {
    typedef boost::tokenizer<boost::char_separator<char> > Tokenizer;
    boost::char_separator<char> sep(",");
    Tokenizer tok(init_pose_str,sep);
    vector<string> tokens(tok.begin(), tok.end());
    ROS_ASSERT(tokens.size()==3);
    init_pose.x = atof(tokens[0].c_str());
    init_pose.y = atof(tokens[1].c_str());
    init_pose.theta = atof(tokens[2].c_str());
  }
  
  RosVisualNavigator nav(radius, init_pose, scan_period);

  nav.setupTopics();

  nav.run();

}


