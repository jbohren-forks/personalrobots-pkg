/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 * @mainpage
 *
 * @htmlinclude manifest.html
 *
 * @b move_base is a highlevel controller for moving the base to a target pose (x, y, theta)
 *
 * This node uses a global path planner to plan a path from the current position to the goal. It
 * uses a local trajectory planner to select trajectories to follow the global plan in light of
 * local observations and dynamics constraints. Obstacle data is integrated in a cost map which supports
 * both global and local planning. Subclasses of this node operate with specific global planners.
 *
 * <hr>
 *
 *  @section usage Usage
 *  @verbatim
 *  $ This base class is abstract and must be subclassed for use.
 *  @endverbatim
 *
 * <hr>
 *
 * @section topic ROS topics
 *
 * Subscribes to (name/type):
 * - @b "base_scan"/laser_scan::LaserScan : the base laser scan, used for obstacle detection
 * - @b "tilt_laser_cloud_filtered"/robot_msgs::PointCloud : the tilt laser cloud, used for obstacle detection
 * - @b "dcam/cloud"/robot_msgs::PointCloud : point clouds from a stereo camera, used for obstacle detection
 * - @b "ground_plane"/robot_msgs::PointCloud : used to filter out bogus obstacles that are really just ground hits
 * - @b "obstacle_cloud"/robot_msgs::PointCloud : low obstacles near the ground

 * Publishes to (name / type):
 * - @b "raw_obstacles"robot_msgs::Polyline : contains all workspace obstacles in a window around the robot
 * - @b "inflated_obstacles"/robot_msgs::Polyline : contains c-space expansion, up to the inscribed radius of the robot
 *  <hr>
 *
 * @section parameters ROS parameters (in addition to base class parameters):
 * - @b "move_base/map_update_frequency" : controls the rate in Hz, for updating the cost map.
 * - @b "/global_frame_id" : determines the global frame to transform to for input messages
 * - @b "~costmap_2d/base_laser_max_range" : buonds the range for base laser data
 * - @b "~costmap_2d/tilt_laser_max_range" : bounds the range for tilt laser data
 * - @b "~costmap_2d/lethal_obstacle_threshold" : defines a cost value indicating a workspace obstacle
 * - @b "~costmap_2d/no_information_value" : defines a cost value indicating an unknown cost.
 * - @b "~costmap_2d/z_threshold_max" : obstacles above this height will be filtered out
 * - @b "~costmap_2d/z_threshold_min" : obstacles below this point will be filtered out
 * - @b "~costmap_2d/inflation_radius" : the maximum distance for inflating a cost function from an obstacle
 * - @b "~costmap_2d/circumscribed_radius" : the smallest radius enclosing the robot footprint
 * - @b "~costmap_2d/inscribed_radius" : the largest radius fully contained in the robot footprint
 * - @b "~costmap_2d/robot_radius"
 * - @b "~costmap_2d/weight" : a multiplier to scale cost values, normalizing them with path length costs.
 * - @b "~costmap_2d/base_laser_update_rate" : a minimum update rate (Hz) for base laser data
 * - @b "~costmap_2d/tilt_laser_update_rate" : a minimum update rate (Hz) for tilt laser data
 * - @b "~costmap_2d/low_obstacle_update_rate" : a minimum update rate (Hz) for low obstacle observations
 * - @b "~costmap_2d/stereo_update_rate" : a minimum update rate (Hz) for stereo data
 * - @b "~costmap_2d/base_laser_keepalive" : an upper bound on the lifetime of observations for this buffer (seconds)
 * - @b "~costmap_2d/tilt_laser_keepalive" : an upper bound on the lifetime of observations for this buffer (seconds)
 * - @b "~costmap_2d/low_obstacle_keepalive" : an upper bound on the lifetime of observations for this buffer (seconds)
 * - @b "~costmap_2d/stereo_keepalive" : an upper bound on the lifetime of observations for this buffer (seconds)
 * - @b "~costmap_2d/body_part_scale"
 * - @b "~costmap_2d/robot_name"
 * - @b "~costmap_2d/filter_robot_points" : true if the robot model should be used to filter out its own body.
 * - @b "~costmap_2d/zLB" : an upper bound on the height of observations used for free space projection
 * - @b "~costmap_2d/zUB" : a upper bound on the height of observations used for free space projection
 * - @b "~costmap_2d/raytrace_window" : the size of the window in which free space will be cleared
 * - @b "~costmap_2d/raytrace_range" : the range after which points will be considered
 * - @b "~costmap_2d/obstacle_range" : obstacles outside this range will not be included in the cost map
 * - None
 **/

#include <old_costmap_2d/costmap_node.h>
#include <robot_msgs/PoseDot.h>
#include <robot_msgs/PointCloud.h>
#include <deprecated_msgs/Pose2DFloat32.h>
#include <robot_msgs/Polyline.h>
#include <robot_srvs/StaticMap.h>
#include <robot_msgs/PointStamped.h>
#include <algorithm>
#include <iterator>
#include <angles/angles.h>
#include <boost/thread.hpp>
#include <robot_filter/RobotFilter.h>

namespace old_costmap_2d
{
  CostMapNode::CostMapNode()
    : use_base_scan_(true), use_tilt_scan_(true), use_stereo_(true), use_low_obstacles_(true),
      tf_(*ros::Node::instance(), true, ros::Duration(10)), // cache for 10 sec, no extrapolation
      costMap_(NULL),
      global_map_accessor_(NULL),
      local_map_accessor_(NULL),
      baseLaserMaxRange_(10.0),
      tiltLaserMaxRange_(10.0),
      minZ_(0.10), maxZ_(2.0), robotWidth_(0.0), active_(true) , map_update_frequency_(10.0),
      trans_stopped_velocity_(1e-2), rot_stopped_velocity_(1e-2), min_abs_theta_vel_(0.4),
      yaw_goal_tolerance_(0.1), xy_goal_tolerance_(robotWidth_ / 2), reset_cost_map_(false)
  {
    // Initialize global pose. Will be set in control loop based on actual data.
    global_pose_.setIdentity();

    // Update rate for the cost map
    ros::Node::instance()->param("~costmap_2d/map_update_frequency", map_update_frequency_, map_update_frequency_);

    // Costmap parameters
    unsigned char lethalObstacleThreshold(100);
    unsigned char noInformation(CostMap2D::NO_INFORMATION);
    double inflationRadius(0.55);
    double robotRadius(0.325);
    double circumscribedRadius(0.46);
    double inscribedRadius(0.325);
    double weight(0.1); // Scale costs down by a factor of 10
    // Which frame is "global"
    ros::Node::instance()->param("/global_frame_id", global_frame_, std::string("/map"));
    ros::Node::instance()->param("~costmap_2d/base_laser_max_range", baseLaserMaxRange_, baseLaserMaxRange_);
    ros::Node::instance()->param("~costmap_2d/tilt_laser_max_range", tiltLaserMaxRange_, tiltLaserMaxRange_);

    ros::Node::instance()->param("~costmap_2d/use_base_scan",use_base_scan_, true);
    ros::Node::instance()->param("~costmap_2d/use_tilt_scan",use_tilt_scan_, true);
    ros::Node::instance()->param("~costmap_2d/use_stereo",use_stereo_, true);
    ros::Node::instance()->param("~costmap_2d/use_low_obstacles",use_low_obstacles_, true);

    // Unsigned chars cannot be stored in parameter server
    int tmpLethalObstacleThreshold;
    ros::Node::instance()->param("~costmap_2d/lethal_obstacle_threshold", tmpLethalObstacleThreshold, int(lethalObstacleThreshold));
    if (tmpLethalObstacleThreshold > 255)
      tmpLethalObstacleThreshold = 255;
    else if (tmpLethalObstacleThreshold < 0)
      tmpLethalObstacleThreshold = 0;

    lethalObstacleThreshold = tmpLethalObstacleThreshold;

    int tmpNoInformation;
    ros::Node::instance()->param("~costmap_2d/no_information_value", tmpNoInformation, int(noInformation));
    if (tmpNoInformation > 255)
      tmpNoInformation = 255;
    else if (tmpNoInformation < 0)
      tmpNoInformation = 0;

    noInformation = tmpNoInformation;

    ros::Node::instance()->param("~costmap_2d/z_threshold_max", maxZ_, maxZ_);
    ros::Node::instance()->param("~costmap_2d/z_threshold_min", minZ_, minZ_);
    ros::Node::instance()->param("~costmap_2d/inflation_radius", inflationRadius, inflationRadius);
    ros::Node::instance()->param("~costmap_2d/circumscribed_radius", circumscribedRadius, circumscribedRadius);
    ros::Node::instance()->param("~costmap_2d/inscribed_radius", inscribedRadius, inscribedRadius);
    ros::Node::instance()->param("~costmap_2d/robot_radius", robotRadius, robotRadius);
    ros::Node::instance()->param("~costmap_2d/weight", weight, weight);

    robotWidth_ = inscribedRadius * 2;

    // Obtain parameters for sensors and allocate observation buffers accordingly. Rates are in Hz.
    double base_laser_update_rate(2.0);
    double tilt_laser_update_rate(2.0);
    double low_obstacle_update_rate(0.2);
    double stereo_update_rate(2.0);
    ros::Node::instance()->param("~costmap_2d/base_laser_update_rate", base_laser_update_rate , base_laser_update_rate);
    ros::Node::instance()->param("~costmap_2d/tilt_laser_update_rate", tilt_laser_update_rate , tilt_laser_update_rate);
    ros::Node::instance()->param("~costmap_2d/low_obstacle_update_rate", low_obstacle_update_rate , low_obstacle_update_rate);
    ros::Node::instance()->param("~costmap_2d/stereo_update_rate", stereo_update_rate , stereo_update_rate);
    double base_laser_keepalive(0.0);
    double tilt_laser_keepalive(3.0);
    double low_obstacle_keepalive(2.0);
    double stereo_keepalive(0.0);
    ros::Node::instance()->param("~costmap_2d/base_laser_keepalive", base_laser_keepalive, base_laser_keepalive);
    ros::Node::instance()->param("~costmap_2d/tilt_laser_keepalive", tilt_laser_keepalive, tilt_laser_keepalive);
    ros::Node::instance()->param("~costmap_2d/low_obstacle_keepalive", low_obstacle_keepalive, low_obstacle_keepalive);
    ros::Node::instance()->param("~costmap_2d/stereo_keepalive", stereo_keepalive, stereo_keepalive);

    //Create robot filter
    std::string robotName = "/robotdesc/pr2";
    double bodypartScale = 2.4;
    bool useFilter = false;
    ros::Node::instance()->param("~costmap_2d/body_part_scale", bodypartScale, bodypartScale);
    ros::Node::instance()->param("~costmap_2d/robot_name", robotName, robotName);
    ros::Node::instance()->param("~costmap_2d/filter_robot_points", useFilter, useFilter);

    if (useFilter) {
      filter_ = new robot_filter::RobotFilter((ros::Node*)this, robotName, true, bodypartScale);
      filter_->loadRobotDescription();
      filter_->waitForState();
    } else {
      filter_ = NULL;
    }



    // Then allocate observation buffers
    if(use_base_scan_)
    {
      baseScanBuffer_ = new old_costmap_2d::BasicObservationBuffer(std::string("base_laser"), global_frame_, tf_,
                                                               ros::Duration().fromSec(base_laser_keepalive),
                                                               old_costmap_2d::BasicObservationBuffer::computeRefreshInterval(base_laser_update_rate),
                                                               inscribedRadius, minZ_, maxZ_, filter_);
    }
    if(use_base_scan_)
    {
      baseCloudBuffer_ = new old_costmap_2d::BasicObservationBuffer(std::string("base_laser"), global_frame_, tf_,
                                                                ros::Duration().fromSec(base_laser_keepalive),
                                                                old_costmap_2d::BasicObservationBuffer::computeRefreshInterval(base_laser_update_rate),
                                                                inscribedRadius, minZ_, maxZ_, filter_);
    }
    if(use_tilt_scan_)
    {
      tiltScanBuffer_ = new old_costmap_2d::BasicObservationBuffer(std::string("laser_tilt_link"), global_frame_, tf_,
                                                               ros::Duration().fromSec(tilt_laser_keepalive),
                                                               old_costmap_2d::BasicObservationBuffer::computeRefreshInterval(tilt_laser_update_rate),
                                                               inscribedRadius, minZ_, maxZ_, filter_);
    }
    if(use_low_obstacles_)
    {
      lowObstacleBuffer_ = new old_costmap_2d::BasicObservationBuffer(std::string("odom_combined"), global_frame_, tf_,
                                                                  ros::Duration().fromSec(low_obstacle_keepalive),
                                                                  old_costmap_2d::BasicObservationBuffer::computeRefreshInterval(low_obstacle_update_rate),
                                                                  inscribedRadius, -10.0, maxZ_, filter_);
    }
    if(use_stereo_)
    {
      stereoCloudBuffer_ = new old_costmap_2d::BasicObservationBuffer(std::string("stereo_link"), global_frame_, tf_,
                                                                  ros::Duration().fromSec(stereo_keepalive),
                                                                  old_costmap_2d::BasicObservationBuffer::computeRefreshInterval(stereo_update_rate),
                                                                  inscribedRadius, minZ_, maxZ_, filter_);
    }

    // get map via RPC
    robot_srvs::StaticMap::Request  req;
    robot_srvs::StaticMap::Response resp;
    ROS_DEBUG("Requesting the map...\n");
    while(!ros::service::call("static_map", req, resp))
    {
      ROS_INFO("Map Request failed; trying again...\n");
      usleep(1000000);
    }

    ROS_DEBUG("Received a %d X %d map at %f m/pix\n",
              resp.map.info.width, resp.map.info.height, resp.map.info.resolution);

    // We are treating cells with no information as lethal obstacles based on the input data. This is not ideal but
    // our planner and controller do not reason about the no obstacle case
    std::vector<unsigned char> inputData;
    unsigned int numCells = resp.map.info.width * resp.map.info.height;
    for(unsigned int i = 0; i < numCells; i++){
      inputData.push_back((unsigned char) resp.map.data[i]);
    }

    // Now allocate the cost map and its sliding window used by the controller
    double zLB, zUB, raytraceWindow, obstacleRange, rayTraceRange;
    ros::Node::instance()->param("~costmap_2d/zLB", zLB, 0.25);
    ros::Node::instance()->param("~costmap_2d/zUB", zUB, 0.35);
    ros::Node::instance()->param("~costmap_2d/raytrace_window", raytraceWindow, 2.5);
    ros::Node::instance()->param("~costmap_2d/raytrace_range", rayTraceRange, 10.0);
    ros::Node::instance()->param("~costmap_2d/obstacle_range", obstacleRange, 10.0);

    costMap_ = new CostMap2D((unsigned int)resp.map.info.width, (unsigned int)resp.map.info.height,
                             inputData , resp.map.info.resolution,
                             lethalObstacleThreshold, maxZ_, zLB, zUB,
                             inflationRadius, circumscribedRadius, inscribedRadius, weight,
                             obstacleRange, rayTraceRange, raytraceWindow);

    // set up costmap service response
    costmap_response_.map.info.width=resp.map.info.width;
    costmap_response_.map.info.height=resp.map.info.height;
    costmap_response_.map.info.resolution=resp.map.info.resolution;
    costmap_response_.map.info.origin.position.x = 0.0;
    costmap_response_.map.info.origin.position.y = 0.0;
    costmap_response_.map.info.origin.position.z = 0.0;
    costmap_response_.map.info.origin.orientation.x = 0.0;
    costmap_response_.map.info.origin.orientation.y = 0.0;
    costmap_response_.map.info.origin.orientation.z = 0.0;
    costmap_response_.map.info.origin.orientation.w = 1.0;

    costmap_response_.map.set_data_size(resp.map.info.width*resp.map.info.height);

    ros::Node::instance()->param("~costmap_2d/local_access_mapsize", local_access_mapsize_, 4.0);
    global_map_accessor_ = new CostMapAccessor(*costMap_);
    local_map_accessor_ = new CostMapAccessor(*costMap_, local_access_mapsize_, 0.0, 0.0);

    // Advertize messages to publish cost map updates
    ros::Node::instance()->advertise<robot_msgs::Polyline>("raw_obstacles", 1);
    ros::Node::instance()->advertise<robot_msgs::Polyline>("inflated_obstacles", 1);

    // Advertise costmap service
    // Might be worth eventually having a dedicated node provide this service and all
    // nodes including move_base access the costmap through it, but for now leaving costmap
    // in move_base for fast access
    ros::Node::instance()->advertiseService("costmap", &CostMapNode::costmapCallback, this);

    // The cost map is populated with either laser scans in the case that we are unable to use a
    // world model   source, or point clouds if we are. We shall pick one, and will be dominated by
    // point clouds
    if(use_base_scan_)
    {
      baseScanNotifier_ = new tf::MessageNotifier<laser_scan::LaserScan>(&tf_, ros::Node::instance(),
                                                                         boost::bind(&CostMapNode::baseScanCallback, this, _1),
                                                                         "base_scan", global_frame_, 50);
    }
    if(use_base_scan_)
    {
      baseCloudNotifier_ = new tf::MessageNotifier<robot_msgs::PointCloud>(&tf_, ros::Node::instance(),
                                                                           boost::bind(&CostMapNode::baseCloudCallback, this, _1),
                                                                           "base_scan_filtered", global_frame_, 50);
    }
    if(use_tilt_scan_)
    {
      tiltLaserNotifier_ = new tf::MessageNotifier<robot_msgs::PointCloud>(&tf_, ros::Node::instance(),
                                                                           boost::bind(&CostMapNode::tiltCloudCallback, this, _1),
                                                                           "tilt_laser_cloud_filtered", global_frame_, 50);
    }
    if(use_stereo_)
    {
      ros::Node::instance()->subscribe("dcam/cloud",  stereoCloudMsg_, &CostMapNode::stereoCloudCallback, this, 1);
    }
    if(use_low_obstacles_)
    {
      ros::Node::instance()->subscribe("ground_plane",  groundPlaneMsg_, &CostMapNode::groundPlaneCallback, this, 1);
      ros::Node::instance()->subscribe("cloud_ground_filtered",  groundPlaneCloudMsg_, &CostMapNode::groundPlaneCloudCallback, this, 1);
    }
    // Spawn map update thread
    map_update_thread_ = new boost::thread(boost::bind(&CostMapNode::mapUpdateLoop, this));

    // Note: derived classes must initialize.
  }

  CostMapNode::~CostMapNode(){

    active_ = false;

    delete map_update_thread_;

    if(local_map_accessor_ != NULL)
      delete local_map_accessor_;

    if(global_map_accessor_ != NULL)
      delete global_map_accessor_;

    if(costMap_ != NULL)
      delete costMap_;

    if(use_base_scan_)
    {
      delete baseScanBuffer_;
      delete baseCloudBuffer_;
    }
    if(use_low_obstacles_)
    {
      delete lowObstacleBuffer_;
    }
    if(use_tilt_scan_)
    {
      delete tiltScanBuffer_;
    }
    if(use_stereo_)
    {
      delete stereoCloudBuffer_;
    }
    delete filter_;
    if(use_base_scan_)
    {
      delete baseScanNotifier_;
    }
    if(use_tilt_scan_)
    {
      delete tiltLaserNotifier_;
    }
  }

  void CostMapNode::updateGlobalPose()
  {
    tf::Stamped<tf::Pose> robotPose;
    robotPose.setIdentity();
    robotPose.frame_id_ = "base_link";
    robotPose.stamp_ = ros::Time();

    try{
      tf_.transformPose(global_frame_, robotPose, global_pose_);
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    }

    // Update the cost map window
    double uselessPitch, uselessRoll, yaw;
    global_pose_.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);
    ROS_DEBUG("Received new position (x=%f, y=%f, th=%f)", global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), yaw);

    local_map_accessor_->updateForRobotPosition(global_pose_.getOrigin().x(), global_pose_.getOrigin().y());
  }


  void CostMapNode::baseScanCallback(const tf::MessageNotifier<laser_scan::LaserScan>::MessagePtr& message)
  {
    // Project laser into point cloud
    robot_msgs::PointCloud local_cloud;
    local_cloud.header = message->header;
    projector_.projectLaser(*message, local_cloud, -1.0, true);
    lock();
    baseScanBuffer_->buffer_cloud(local_cloud);
    unlock();
  }

  void CostMapNode::tiltScanCallback()
  {
    // Project laser into point cloud
    robot_msgs::PointCloud local_cloud;
    local_cloud.header = tiltScanMsg_.header;
    projector_.projectLaser(tiltScanMsg_, local_cloud, tiltLaserMaxRange_);
    lock();
    tiltScanBuffer_->buffer_cloud(local_cloud);
    unlock();
  }

  void CostMapNode::tiltCloudCallback(const tf::MessageNotifier<robot_msgs::PointCloud>::MessagePtr& message)
  {
    lock();
    tiltScanBuffer_->buffer_cloud(*message);
    unlock();
  }

  void CostMapNode::baseCloudCallback(const tf::MessageNotifier<robot_msgs::PointCloud>::MessagePtr& message)
  {
    lock();
    baseCloudBuffer_->buffer_cloud(*message);
    unlock();
  }

  //updates the point and normal that define the ground plane
  void CostMapNode::groundPlaneCallback()
  {
    lock();
    ground_plane_ = groundPlaneMsg_;
    unlock();
  }

  void CostMapNode::groundPlaneCloudCallback()
  {
    lock();
    lowObstacleBuffer_->buffer_cloud(groundPlaneCloudMsg_);
    unlock();
  }

  void CostMapNode::stereoCloudCallback()
  {
    lock();
    stereoCloudBuffer_->buffer_cloud(stereoCloudMsg_);
    unlock();
  }

  /**
   * The conjunction of all observation buffers must be current
   */
  bool CostMapNode::checkWatchDog() const {

    bool ok =  (baseScanBuffer_->isCurrent() || !use_base_scan_) && (tiltScanBuffer_->isCurrent() ||!use_tilt_scan_) && (stereoCloudBuffer_->isCurrent() || !use_stereo_) && (lowObstacleBuffer_->isCurrent() || !use_low_obstacles_);

    if(!ok)
      ROS_WARN("Missed required cost map update. Should not allow commanding now. Check cost map data source.\n");

    return ok;
  }

  /**
   * @brief Utility to output local obstacles. Make the local cost map accessor. It is very cheap :-) Then
   * render the obstacles. Note that the rendered window is typically larger than the local map for control
   */
  void CostMapNode::publishLocalCostMap() {
    double mapSize = std::min(costMap_->getWidth()/2, costMap_->getHeight()/2);
    CostMapAccessor cm(*costMap_, std::min(10.0, mapSize), global_pose_.getOrigin().x(), global_pose_.getOrigin().y());

    // Publish obstacle data for each obstacle cell
    std::vector< std::pair<double, double> > rawObstacles, inflatedObstacles;
    double origin_x, origin_y;
    cm.getOriginInWorldCoordinates(origin_x, origin_y);
    for(unsigned int i = 0; i<cm.getWidth(); i++){
      for(unsigned int j = 0; j<cm.getHeight();j++){
        double wx, wy;
        wx = i * cm.getResolution() + origin_x;
        wy = j * cm.getResolution() + origin_y;
        std::pair<double, double> p(wx, wy);

        if(cm.getCost(i, j) == CostMap2D::LETHAL_OBSTACLE)
          rawObstacles.push_back(p);
        else if(cm.getCost(i, j) == CostMap2D::INSCRIBED_INFLATED_OBSTACLE)
          inflatedObstacles.push_back(p);
      }
    }

    // First publish raw obstacles in red
    robot_msgs::Polyline pointCloudMsg;
    pointCloudMsg.header.frame_id = global_frame_;
    unsigned int pointCount = rawObstacles.size();
    pointCloudMsg.set_points_size(pointCount);
    pointCloudMsg.color.a = 0.0;
    pointCloudMsg.color.r = 1.0;
    pointCloudMsg.color.b = 0.0;
    pointCloudMsg.color.g = 0.0;

    for(unsigned int i=0;i<pointCount;i++){
      pointCloudMsg.points[i].x = rawObstacles[i].first;
      pointCloudMsg.points[i].y = rawObstacles[i].second;
      pointCloudMsg.points[i].z = 0;
    }

    if (!ros::Node::instance()->ok()) {
      return;
    }
    ros::Node::instance()->publish("raw_obstacles", pointCloudMsg);

    // Now do inflated obstacles in blue
    pointCount = inflatedObstacles.size();
    pointCloudMsg.set_points_size(pointCount);
    pointCloudMsg.color.a = 0.0;
    pointCloudMsg.color.r = 0.0;
    pointCloudMsg.color.b = 1.0;
    pointCloudMsg.color.g = 0.0;

    for(unsigned int i=0;i<pointCount;i++){
      pointCloudMsg.points[i].x = inflatedObstacles[i].first;
      pointCloudMsg.points[i].y = inflatedObstacles[i].second;
      pointCloudMsg.points[i].z = 0;
    }

    if (!ros::Node::instance()->ok()) {
      return;
    }
    ros::Node::instance()->publish("inflated_obstacles", pointCloudMsg);
  }


  /**
   * @brief Utility to output local obstacles. Make the local cost map accessor. It is very cheap :-) Then
   * render the obstacles.
   */
  void CostMapNode::publishFreeSpaceAndObstacles() {
    double mapSize = std::min(costMap_->getWidth()/2, costMap_->getHeight()/2);
    CostMapAccessor cm(*costMap_, std::min(10.0, mapSize), global_pose_.getOrigin().x(), global_pose_.getOrigin().y());

    // Publish obstacle data for each obstacle cell
    std::vector< std::pair<double, double> > rawObstacles, inflatedObstacles;
    double origin_x, origin_y;
    cm.getOriginInWorldCoordinates(origin_x, origin_y);
    for(unsigned int i = 0; i<cm.getWidth(); i++){
      for(unsigned int j = 0; j<cm.getHeight();j++){
        double wx, wy;
        wx = i * cm.getResolution() + origin_x;
        wy = j * cm.getResolution() + origin_y;
        std::pair<double, double> p(wx, wy);

        if(cm.getCost(i, j) > 0)
          rawObstacles.push_back(p);
        else if(cm.getCost(i, j) == 0)
          inflatedObstacles.push_back(p);
      }
    }

    // First publish raw obstacles in red
    robot_msgs::Polyline pointCloudMsg;
    pointCloudMsg.header.frame_id = global_frame_;
    unsigned int pointCount = rawObstacles.size();
    pointCloudMsg.set_points_size(pointCount);
    pointCloudMsg.color.a = 0.0;
    pointCloudMsg.color.r = 1.0;
    pointCloudMsg.color.b = 0.0;
    pointCloudMsg.color.g = 0.0;

    for(unsigned int i=0;i<pointCount;i++){
      pointCloudMsg.points[i].x = rawObstacles[i].first;
      pointCloudMsg.points[i].y = rawObstacles[i].second;
      pointCloudMsg.points[i].z = 0;
    }

    ros::Node::instance()->publish("raw_obstacles", pointCloudMsg);

    // Now do inflated obstacles in blue
    pointCount = inflatedObstacles.size();
    pointCloudMsg.set_points_size(pointCount);
    pointCloudMsg.color.a = 0.0;
    pointCloudMsg.color.r = 0.0;
    pointCloudMsg.color.b = 1.0;
    pointCloudMsg.color.g = 0.0;

    for(unsigned int i=0;i<pointCount;i++){
      pointCloudMsg.points[i].x = inflatedObstacles[i].first;
      pointCloudMsg.points[i].y = inflatedObstacles[i].second;
      pointCloudMsg.points[i].z = 0;
    }

    ros::Node::instance()->publish("inflated_obstacles", pointCloudMsg);
  }


  CostMapNode::footprint_t const & CostMapNode::getFootprint() const{
    return footprint_;
  }

  void CostMapNode::updateCostMap() {
    if (reset_cost_map_) {
      costMap_->revertToStaticMap(global_pose_.getOrigin().x(), global_pose_.getOrigin().y());
    }

    ROS_DEBUG("Starting cost map update/n");

    lock();
    // Aggregate buffered observations across all sources. Must be thread safe
    std::vector<old_costmap_2d::Observation> observations, raytrace_obs;
    if(use_base_scan_)
    {
      baseCloudBuffer_->get_observations(observations);

      //get the observations from the base scan to use for raytracing
      baseScanBuffer_->get_observations(raytrace_obs);

      //if we are not getting filtered base scans... we'll add the base scans as obstacle points
      if(observations.empty())
        baseScanBuffer_->get_observations(observations);
    }
    if(use_tilt_scan_)
    {
      tiltScanBuffer_->get_observations(observations);
    }

    if(use_low_obstacles_)
    {
      lowObstacleBuffer_->get_observations(observations);
    }

    if(use_stereo_)
    {
      stereoCloudBuffer_->get_observations(observations);
    }

    std::vector<robot_msgs::PointCloud> points_storage(observations.size()); //needed to deep copy observations
    std::vector<old_costmap_2d::Observation> stored_observations(observations.size());

    //we need to perform a deep copy on the observations to be thread safe
    for(unsigned int i = 0; i < observations.size(); ++i){
      points_storage[i] = *(observations[i].cloud_);
      stored_observations[i] = Observation(observations[i].origin_, &points_storage[i]);
    }

    unlock();

    ROS_DEBUG("Applying update with %d observations/n", stored_observations.size());
    // Apply to cost map
    ros::Time start = ros::Time::now();
    if(raytrace_obs.empty() || raytrace_obs.front().cloud_ == NULL)
      costMap_->updateDynamicObstacles(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), stored_observations);
    else
      costMap_->updateDynamicObstacles(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), stored_observations, &raytrace_obs.front());
    double t_diff = (ros::Time::now() - start).toSec();
    ROS_DEBUG("Updated map in %f seconds for %d observations/n", t_diff, stored_observations.size());

    // Finally, we must extract the cost data that we have computed and:
    // 1. Refresh the local_map_accessor for the controller
    // 2. Refresh the global_map accessor for the planner
    // 3. Publish the local cost map window
    lock();
    local_map_accessor_->refresh();
    global_map_accessor_->refresh();
    publishLocalCostMap();
    reset_cost_map_ = false;
    unlock();
  }


  /** Callback invoked when someone requests costmap */
  bool CostMapNode::costmapCallback(robot_srvs::StaticMap::Request &req, robot_srvs::StaticMap::Response &res )
  {
    const unsigned char* costmap = getCostMap().getMap();
    res = costmap_response_;
    copy(costmap, costmap+res.map.info.width*res.map.info.height, res.map.data.begin());
    return true;
  }


  /**
   * Each update loop will query all observations and aggregate them and then apply
   * a batch update to the cost map
   */
  void CostMapNode::mapUpdateLoop()
  {
    ros::Duration d;
    d.fromSec(1.0/map_update_frequency_);

    while (active_){
      updateGlobalPose();
      updateCostMap();
      d.sleep();
    }
  }
}
