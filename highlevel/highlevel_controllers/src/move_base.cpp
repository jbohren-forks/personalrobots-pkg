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


#include <highlevel_controllers/move_base.hh>
#include <robot_msgs/PoseDot.h>
#include <robot_msgs/PointCloud.h>
#include <deprecated_msgs/Pose2DFloat32.h>
#include <robot_msgs/Polyline2D.h>
#include <robot_srvs/StaticMap.h>
#include <robot_msgs/PointStamped.h>
#include <algorithm>
#include <iterator>
#include <angles/angles.h>
#include <boost/thread.hpp>
#include <robot_filter/RobotFilter.h>

using namespace costmap_2d;

namespace ros {
  namespace highlevel_controllers {

    MoveBase::MoveBase()
      : HighlevelController<robot_msgs::Planner2DState, robot_msgs::Planner2DGoal>("move_base", "state", "goal"),
        tf_(*ros::Node::instance(), true, ros::Duration(10)), // cache for 10 sec, no extrapolation
      controller_(NULL),
      costMap_(NULL),
      global_map_accessor_(NULL),
      local_map_accessor_(NULL),
      baseLaserMaxRange_(10.0),
      tiltLaserMaxRange_(10.0),
      minZ_(0.02), maxZ_(2.0), robotWidth_(0.0), active_(true) , map_update_frequency_(10.0),
      yaw_goal_tolerance_(0.1), xy_goal_tolerance_(robotWidth_ / 2), reset_cost_map_(false)
    {
      // Initialize global pose. Will be set in control loop based on actual data.
      global_pose_.setIdentity();

      // Initialize odometry
      base_odom_.vel.x = 0;
      base_odom_.vel.y = 0;
      base_odom_.vel.th = 0;

      // Initialize state message parameters that are unused
      stateMsg.waypoint.x = 0.0;
      stateMsg.waypoint.y = 0.0;
      stateMsg.waypoint.th = 0.0;
      stateMsg.set_waypoints_size(0);
      stateMsg.waypoint_idx = -1;

      // Update rate for the cost map
      local_param("map_update_frequency", map_update_frequency_, map_update_frequency_);

      // Costmap parameters
      unsigned char lethalObstacleThreshold(100);
      unsigned char noInformation(CostMap2D::NO_INFORMATION);
      double freeSpaceProjectionHeight(0.19);
      double inflationRadius(0.55);
      double robotRadius(0.325);
      double circumscribedRadius(0.46);
      double inscribedRadius(0.325);
      double weight(0.1); // Scale costs down by a factor of 10
      // Which frame is "global"
      ros::Node::instance()->param("/global_frame_id", global_frame_, std::string("/map"));
      ros::Node::instance()->param("/costmap_2d/base_laser_max_range", baseLaserMaxRange_, baseLaserMaxRange_);
      ros::Node::instance()->param("/costmap_2d/tilt_laser_max_range", tiltLaserMaxRange_, tiltLaserMaxRange_);
       
      //thresholds for ground plane detection
      ros::Node::instance()->param("/ransac_ground_plane_extraction/distance_threshold", ransac_distance_threshold_, 0.03);

      // Unsigned chars cannot be stored in parameter server
      int tmpLethalObstacleThreshold;
      ros::Node::instance()->param("/costmap_2d/lethal_obstacle_threshold", tmpLethalObstacleThreshold, int(lethalObstacleThreshold));
      if (tmpLethalObstacleThreshold > 255)
        tmpLethalObstacleThreshold = 255;
      else if (tmpLethalObstacleThreshold < 0)
        tmpLethalObstacleThreshold = 0;

      lethalObstacleThreshold = tmpLethalObstacleThreshold;

      int tmpNoInformation;
      ros::Node::instance()->param("/costmap_2d/no_information_value", tmpNoInformation, int(noInformation));
      if (tmpNoInformation > 255)
        tmpNoInformation = 255;
      else if (tmpNoInformation < 0)
        tmpNoInformation = 0;

      noInformation = tmpNoInformation;

      ros::Node::instance()->param("/costmap_2d/z_threshold_max", maxZ_, maxZ_);
      ros::Node::instance()->param("/costmap_2d/z_threshold_min", minZ_, minZ_);
      ros::Node::instance()->param("/costmap_2d/freespace_projection_height", freeSpaceProjectionHeight, freeSpaceProjectionHeight);
      ros::Node::instance()->param("/costmap_2d/inflation_radius", inflationRadius, inflationRadius);
      ros::Node::instance()->param("/costmap_2d/circumscribed_radius", circumscribedRadius, circumscribedRadius);
      ros::Node::instance()->param("/costmap_2d/inscribed_radius", inscribedRadius, inscribedRadius);
      ros::Node::instance()->param("/costmap_2d/robot_radius", robotRadius, robotRadius);
      ros::Node::instance()->param("/costmap_2d/weight", weight, weight);

      robotWidth_ = inscribedRadius * 2;
      xy_goal_tolerance_ = robotWidth_ / 2;

      // Obtain parameters for sensors and allocate observation buffers accordingly. Rates are in Hz. 
      double base_laser_update_rate(2.0);
      double tilt_laser_update_rate(2.0);
      double low_obstacle_update_rate(0.2);
      double stereo_update_rate(2.0);
      ros::Node::instance()->param("/costmap_2d/base_laser_update_rate", base_laser_update_rate , base_laser_update_rate);
      ros::Node::instance()->param("/costmap_2d/tilt_laser_update_rate", tilt_laser_update_rate , tilt_laser_update_rate);
      ros::Node::instance()->param("/costmap_2d/low_obstacle_update_rate", low_obstacle_update_rate , low_obstacle_update_rate);
      ros::Node::instance()->param("/costmap_2d/stereo_update_rate", stereo_update_rate , stereo_update_rate);
      double base_laser_keepalive(0.0);
      double tilt_laser_keepalive(3.0);
      double low_obstacle_keepalive(2.0);
      double stereo_keepalive(0.0);
      ros::Node::instance()->param("/costmap_2d/base_laser_keepalive", base_laser_keepalive, base_laser_keepalive);
      ros::Node::instance()->param("/costmap_2d/tilt_laser_keepalive", tilt_laser_keepalive, tilt_laser_keepalive);
      ros::Node::instance()->param("/costmap_2d/low_obstacle_keepalive", low_obstacle_keepalive, low_obstacle_keepalive);
      ros::Node::instance()->param("/costmap_2d/stereo_keepalive", stereo_keepalive, stereo_keepalive);

      //Create robot filter
      std::string robotName = "/robotdesc/pr2";
      double bodypartScale = 2.4;
      bool useFilter = false;
      ros::Node::instance()->param("/costmap_2d/body_part_scale", bodypartScale, bodypartScale);
      ros::Node::instance()->param("/costmap_2d/robot_name", robotName, robotName);
      ros::Node::instance()->param("/costmap_2d/filter_robot_points", useFilter, useFilter);
      
      if (useFilter) {
	filter_ = new robot_filter::RobotFilter((ros::Node*)this, robotName, true, bodypartScale);
	filter_->loadRobotDescription();
	filter_->waitForState();
      } else {
	filter_ = NULL;
      }

      // Then allocate observation buffers
      baseScanBuffer_ = new costmap_2d::BasicObservationBuffer(std::string("base_laser"), global_frame_, tf_, 
							       ros::Duration().fromSec(base_laser_keepalive), 
							       costmap_2d::BasicObservationBuffer::computeRefreshInterval(base_laser_update_rate),
							       inscribedRadius, minZ_, maxZ_, filter_);
      tiltScanBuffer_ = new costmap_2d::BasicObservationBuffer(std::string("laser_tilt_link"), global_frame_, tf_, 
							       ros::Duration().fromSec(tilt_laser_keepalive), 
							       costmap_2d::BasicObservationBuffer::computeRefreshInterval(tilt_laser_update_rate),
							       inscribedRadius, minZ_, maxZ_, filter_);
      lowObstacleBuffer_ = new costmap_2d::BasicObservationBuffer(std::string("odom_combined"), global_frame_, tf_, 
								  ros::Duration().fromSec(low_obstacle_keepalive), 
								  costmap_2d::BasicObservationBuffer::computeRefreshInterval(low_obstacle_update_rate),
								  inscribedRadius, -10.0, maxZ_, filter_);
      stereoCloudBuffer_ = new costmap_2d::BasicObservationBuffer(std::string("stereo_link"), global_frame_, tf_, 
								  ros::Duration().fromSec(stereo_keepalive), 
								  costmap_2d::BasicObservationBuffer::computeRefreshInterval(stereo_update_rate),
								  inscribedRadius, minZ_, maxZ_, filter_);


      // get map via RPC
      robot_srvs::StaticMap::Request  req;
      robot_srvs::StaticMap::Response resp;
      ROS_INFO("Requesting the map...\n");
      while(!ros::service::call("static_map", req, resp))
      {
        ROS_INFO("Request failed; trying again...\n");
        usleep(1000000);
      }

      ROS_INFO("Received a %d X %d map at %f m/pix\n",
          resp.map.width, resp.map.height, resp.map.resolution);

      // We are treating cells with no information as lethal obstacles based on the input data. This is not ideal but
      // our planner and controller do not reason about the no obstacle case
      std::vector<unsigned char> inputData;
      unsigned int numCells = resp.map.width * resp.map.height;
      for(unsigned int i = 0; i < numCells; i++){
        inputData.push_back((unsigned char) resp.map.data[i]);
      }

      // Now allocate the cost map and its sliding window used by the controller
      double zLB, zUB, raytraceWindow, obstacleRange, rayTraceRange;
      ros::Node::instance()->param("/costmap_2d/zLB", zLB, 0.15);
      ros::Node::instance()->param("/costmap_2d/zUB", zUB, 0.25);
      ros::Node::instance()->param("/costmap_2d/raytrace_window", raytraceWindow, 2.5);
      ros::Node::instance()->param("/costmap_2d/raytrace_range", rayTraceRange, 10.0);
      ros::Node::instance()->param("/costmap_2d/obstacle_range", obstacleRange, 10.0);

      costMap_ = new CostMap2D((unsigned int)resp.map.width, (unsigned int)resp.map.height,
			       inputData , resp.map.resolution, 
			       lethalObstacleThreshold, maxZ_, zLB, zUB,
			       inflationRadius, circumscribedRadius, inscribedRadius, weight, 
			       obstacleRange, rayTraceRange, raytraceWindow);

      // set up costmap service response
      costmap_response_.map.width=resp.map.width;
      costmap_response_.map.height=resp.map.height;
      costmap_response_.map.resolution=resp.map.resolution;
      costmap_response_.map.origin.x = 0.0;
      costmap_response_.map.origin.y = 0.0;
      costmap_response_.map.origin.th = 0.0;
      costmap_response_.map.set_data_size(resp.map.width*resp.map.height);


      // Allocate Velocity Controller
      double mapSize(2.0);
      ros::Node::instance()->param("/trajectory_rollout/map_size", mapSize, 2.0);
      ros::Node::instance()->param("/trajectory_rollout/yaw_goal_tolerance", yaw_goal_tolerance_, yaw_goal_tolerance_);
      ros::Node::instance()->param("/trajectory_rollout/xy_goal_tolerance", xy_goal_tolerance_, xy_goal_tolerance_);

      ROS_ASSERT(mapSize <= costMap_->getWidth());
      ROS_ASSERT(mapSize <= costMap_->getHeight());


      global_map_accessor_ = new CostMapAccessor(*costMap_);
      local_map_accessor_ = new CostMapAccessor(*costMap_, mapSize, 0.0, 0.0);

      deprecated_msgs::Point2DFloat32 pt;
      //create a square footprint
      pt.x = robotRadius + .01;
      pt.y = -1 * (robotRadius + .01);
      footprint_.push_back(pt);
      pt.x = -1 * (robotRadius + .01);
      pt.y = -1 * (robotRadius + .01);
      footprint_.push_back(pt);
      pt.x = -1 * (robotRadius + .01);
      pt.y = robotRadius + .01;
      footprint_.push_back(pt);
      pt.x = robotRadius + .01;
      pt.y = robotRadius + .01;
      footprint_.push_back(pt);

      //give the robot a nose
      pt.x = circumscribedRadius;
      pt.y = 0;
      footprint_.push_back(pt);

      controller_ = new trajectory_rollout::TrajectoryControllerROS(*ros::Node::instance(), tf_, global_frame_, *local_map_accessor_, 
          footprint_, inscribedRadius, circumscribedRadius);

      // Advertize messages to publish cost map updates
      ros::Node::instance()->advertise<robot_msgs::Polyline2D>("raw_obstacles", 1);
      ros::Node::instance()->advertise<robot_msgs::Polyline2D>("inflated_obstacles", 1);

      // Advertize message to publish the global plan
      ros::Node::instance()->advertise<robot_msgs::Polyline2D>("gui_path", 1);

      // Advertize message to publish local plan
      ros::Node::instance()->advertise<robot_msgs::Polyline2D>("local_path", 1);

      // Advertize message to publish robot footprint
      ros::Node::instance()->advertise<robot_msgs::Polyline2D>("robot_footprint", 1);

      // Advertize message to publish velocity cmds
      ros::Node::instance()->advertise<robot_msgs::PoseDot>("cmd_vel", 1);

      //Advertize message to publish local goal for head to track
      ros::Node::instance()->advertise<robot_msgs::PointStamped>("head_controller/head_track_point", 1);

      // Advertise costmap service
      // Might be worth eventually having a dedicated node provide this service and all
      // nodes including move_base access the costmap through it, but for now leaving costmap
      // in move_base for fast access
      ros::Node::instance()->advertiseService("costmap", &MoveBase::costmapCallback);

      // The cost map is populated with either laser scans in the case that we are unable to use a
      // world model   source, or point clouds if we are. We shall pick one, and will be dominated by
      // point clouds
      baseScanNotifier_ = new tf::MessageNotifier<laser_scan::LaserScan>(&tf_, ros::Node::instance(),  
                                 boost::bind(&MoveBase::baseScanCallback, this, _1), 
                                "base_scan", global_frame_, 50); 
      tiltLaserNotifier_ = new tf::MessageNotifier<robot_msgs::PointCloud>(&tf_, ros::Node::instance(), 
				 boost::bind(&MoveBase::tiltCloudCallback, this, _1),
				 "tilt_laser_cloud_filtered", global_frame_, 50);
      ros::Node::instance()->subscribe("dcam/cloud",  stereoCloudMsg_, &MoveBase::stereoCloudCallback, this, 1);
      ros::Node::instance()->subscribe("ground_plane",  groundPlaneMsg_, &MoveBase::groundPlaneCallback, this, 1);
      ros::Node::instance()->subscribe("obstacle_cloud",  groundPlaneCloudMsg_, &MoveBase::groundPlaneCloudCallback, this, 1);

      // Subscribe to odometry messages to get global pose
      ros::Node::instance()->subscribe("odom", odomMsg_, &MoveBase::odomCallback, this, 1);

      // Spawn map update thread
      map_update_thread_ = new boost::thread(boost::bind(&MoveBase::mapUpdateLoop, this));

      // Note: derived classes must initialize.
    }

    MoveBase::~MoveBase(){

      active_ = false;

      delete map_update_thread_;

      if(controller_ != NULL)
        delete controller_;

      if(local_map_accessor_ != NULL)
        delete local_map_accessor_;
 
      if(global_map_accessor_ != NULL)
        delete global_map_accessor_;


      if(costMap_ != NULL)
        delete costMap_;

      delete baseScanBuffer_;
      delete lowObstacleBuffer_;
      delete tiltScanBuffer_;
      delete stereoCloudBuffer_;
      delete filter_;
      delete baseScanNotifier_;
      delete tiltLaserNotifier_;
    }

    void MoveBase::updateGlobalPose(){ 
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


    /**
     * @brief Called by the main control loop in the base class. Lock already aquired
     */
    void MoveBase::updateGoalMsg(){
      // Revert to static map on new goal. May result in oscillation, but requested by Eitan for the milestone
      reset_cost_map_ = true;

      tf::Stamped<tf::Pose> goalPose, transformedGoalPose;
      btQuaternion qt;
      qt.setEulerZYX(goalMsg.goal.th, 0, 0);
      goalPose.setData(btTransform(qt, btVector3(goalMsg.goal.x, goalMsg.goal.y, 0)));
      goalPose.frame_id_ = goalMsg.header.frame_id;
      goalPose.stamp_ = ros::Time();

      try{
        tf_.transformPose(global_frame_, goalPose, transformedGoalPose);
      }
      catch(tf::LookupException& ex){
        ROS_ERROR("No transform available from %s to map. This may be because the frame_id of the goalMsg is wrong.\n", goalMsg.header.frame_id.c_str());
        ROS_ERROR("The details of the LookupException are: %s\n", ex.what());
      }
      catch(tf::ConnectivityException& ex){
        ROS_ERROR("No transform available from %s to map. This may be because the frame_id of the goalMsg is wrong.\n", goalMsg.header.frame_id.c_str());
        ROS_ERROR("The details of the ConnectivityException are: %s\n", ex.what());
      }
      catch(tf::ExtrapolationException& ex){
        ROS_ERROR("No transform available from %s to map. This may be because the frame_id of the goalMsg is wrong.\n", goalMsg.header.frame_id.c_str());
        ROS_ERROR("The details of the ExtrapolationException are: %s\n", ex.what());
      }

      stateMsg.goal.x = transformedGoalPose.getOrigin().x();
      stateMsg.goal.y = transformedGoalPose.getOrigin().y();
      double uselessPitch, uselessRoll, yaw;
      transformedGoalPose.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);
      stateMsg.goal.th = (float)yaw;

      ROS_DEBUG("Received new goal (x=%f, y=%f, th=%f)\n", stateMsg.goal.x, stateMsg.goal.y, stateMsg.goal.th);
    }

    void MoveBase::updateStateMsg(){
      // Get the current robot pose in the map frame
      updateGlobalPose();

      // Assign state data 
      stateMsg.pos.x = global_pose_.getOrigin().x();
      stateMsg.pos.y = global_pose_.getOrigin().y();
      double uselessPitch, uselessRoll, yaw;
      global_pose_.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);
      stateMsg.pos.th = (float)yaw;
    }

    void MoveBase::baseScanCallback(const tf::MessageNotifier<laser_scan::LaserScan>::MessagePtr& message)
    {
      // Project laser into point cloud
      robot_msgs::PointCloud local_cloud;
      local_cloud.header = message->header;
      projector_.projectLaser(*message, local_cloud, baseLaserMaxRange_);
      lock();
      baseScanBuffer_->buffer_cloud(local_cloud);
      unlock();
    }

    void MoveBase::tiltScanCallback()
    {
      // Project laser into point cloud
      robot_msgs::PointCloud local_cloud;
      local_cloud.header = tiltScanMsg_.header;
      projector_.projectLaser(tiltScanMsg_, local_cloud, tiltLaserMaxRange_);
      lock();
      tiltScanBuffer_->buffer_cloud(local_cloud);
      unlock();
    }

    void MoveBase::tiltCloudCallback(const tf::MessageNotifier<robot_msgs::PointCloud>::MessagePtr& message)
    {
      lock();
      tiltScanBuffer_->buffer_cloud(*message);
      unlock();
    }

    //updates the point and normal that define the ground plane
    void MoveBase::groundPlaneCallback()
    {
      lock();
      ground_plane_ = groundPlaneMsg_;
      unlock();
    }

    void MoveBase::groundPlaneCloudCallback()
    {
      lock();
      lowObstacleBuffer_->buffer_cloud(groundPlaneCloudMsg_);
      unlock();
    }

    void MoveBase::stereoCloudCallback()
    {
      lock();
      stereoCloudBuffer_->buffer_cloud(stereoCloudMsg_);
      unlock();
    }

    /**
     * The odomMsg_ will be updated and we will do the transform to update the odometry data in the base frame
     */
    void MoveBase::odomCallback(){
      if(isTerminated())
        return;

      base_odom_.lock();

      try
      {
        tf::Stamped<btVector3> v_in(btVector3(odomMsg_.vel.x, odomMsg_.vel.y, 0), ros::Time(), odomMsg_.header.frame_id), v_out;
        tf_.transformVector("base_link", ros::Time(), v_in, odomMsg_.header.frame_id, v_out);	 
        base_odom_.vel.x = v_in.x();
        base_odom_.vel.y = v_in.y();
        base_odom_.vel.th = odomMsg_.vel.th;
      }
      catch(tf::LookupException& ex)
      {
        puts("no odom->base Tx yet");
        ROS_DEBUG("%s\n", ex.what());
      }
      catch(tf::ConnectivityException& ex)
      {
        puts("no odom->base Tx yet");
        ROS_DEBUG("%s\n", ex.what());
      }
      catch(tf::ExtrapolationException& ex)
      {
        puts("Extrapolation exception");
      }

      base_odom_.unlock();
    }

    /**
     * A lock will already be aquired here, so just revert the cost map
     */
    void MoveBase::handlePlanningFailure(){
      ROS_DEBUG("No plan found. Handling planning failure");
      reset_cost_map_ = true;
      stopRobot();
    }

    void MoveBase::updatePlan(const std::list<deprecated_msgs::Pose2DFloat32>& newPlan){
      lock();

      // If we have a valid plan then only swap in the new plan if it is shorter.
      if(!isValid() || plan_.size() > newPlan.size()){
        plan_.clear();
        plan_ = newPlan;
        publishPath(true, plan_);
      }

      unlock();
    }

    /** \todo Some code duplication wrt MoveBase::updatePlan(const std::list<deprecated_msgs::Pose2DFloat32>&). */
    void MoveBase::updatePlan(mpglue::waypoint_plan_t const & newPlan) {
      sentry<MoveBase> guard(this);
      if (!isValid() || plan_.size() > newPlan.size()){
        plan_.clear();
        std::copy(newPlan.begin(), newPlan.end(), std::back_inserter(plan_));
        publishPath(true, plan_);
      }
    }

    /**
     * This is used as a validation check and is only called from within dispatchCommands where the lock has already been
     * applied to protect access to the plan.
     */
    bool MoveBase::inCollision() const {
      for(std::list<deprecated_msgs::Pose2DFloat32>::const_iterator it = plan_.begin(); it != plan_.end(); ++it){
        const deprecated_msgs::Pose2DFloat32& w = *it;
        unsigned int ind = costMap_->WC_IND(w.x, w.y);
        if((*costMap_)[ind] >= CostMap2D::INSCRIBED_INFLATED_OBSTACLE){
          ROS_DEBUG("path in collision at <%f, %f>\n", w.x, w.y);
          return true;
        }
      }

      return false;
    }

    void MoveBase::publishFootprint(double x, double y, double th){
      std::vector<deprecated_msgs::Point2DFloat32> footprint = controller_->drawFootprint(x, y, th);
      robot_msgs::Polyline2D footprint_msg;
      footprint_msg.set_points_size(footprint.size());
      footprint_msg.color.r = 1.0;
      footprint_msg.color.g = 0;
      footprint_msg.color.b = 0;
      footprint_msg.color.a = 0;
      for(unsigned int i = 0; i < footprint.size(); ++i){
        footprint_msg.points[i].x = footprint[i].x;
        footprint_msg.points[i].y = footprint[i].y;
      }
      ros::Node::instance()->publish("robot_footprint", footprint_msg);
    }

    void MoveBase::publishPath(bool isGlobal, const std::list<deprecated_msgs::Pose2DFloat32>& path) {
      robot_msgs::Polyline2D guiPathMsg;
      guiPathMsg.set_points_size(path.size());

      unsigned int i = 0;
      for(std::list<deprecated_msgs::Pose2DFloat32>::const_iterator it = path.begin(); it != path.end(); ++it){
        const deprecated_msgs::Pose2DFloat32& w = *it;
        guiPathMsg.points[i].x = w.x;
        guiPathMsg.points[i].y = w.y;
        i++;
      }

      if(isGlobal){
        guiPathMsg.color.r = 0;
        guiPathMsg.color.g = 1.0;
        guiPathMsg.color.b = 0;
        guiPathMsg.color.a = 0;
        ros::Node::instance()->publish("gui_path", guiPathMsg);
      }
      else {
        guiPathMsg.color.r = 0;
        guiPathMsg.color.g = 0;
        guiPathMsg.color.b = 1.0;
        guiPathMsg.color.a = 0;
        ros::Node::instance()->publish("local_path", guiPathMsg);
      }

    }

    bool MoveBase::goalReached(){
      // We assume the plan is valid if we are checking the goal. This should be ensured in the base class
      ROS_ASSERT(isValid());

      // Publish the global plan
      publishPath(true, plan_);

      // If the plan has been executed (i.e. empty) and we are within a required distance of the target orientation,
      // and we have stopped the robot, then we are done
      double uselessPitch, uselessRoll, yaw;
      global_pose_.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);
      if(plan_.empty() && 
          fabs(angles::shortest_angular_distance(yaw , stateMsg.goal.th)) < this->yaw_goal_tolerance_){ /// @todo: this is still wrong, should use bt or similar to check shortest angular distance of roll/pitch/yaw.

        ROS_DEBUG("Goal achieved at: (%f, %f, %f) for (%f, %f, %f)\n",
            global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), yaw,
            stateMsg.goal.x, stateMsg.goal.y, stateMsg.goal.th);

        // The last act will issue stop command
        stopRobot();

        return true;
      }

      // If we have reached the end of the path then clear the plan
      if(!plan_.empty() &&
          withinDistance(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), yaw,
            stateMsg.goal.x, stateMsg.goal.y, yaw)){
        ROS_DEBUG("Last waypoint achieved at: (%f, %f, %f) for (%f, %f, %f)\n",
            global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), yaw,
            stateMsg.goal.x, stateMsg.goal.y, stateMsg.goal.th);

        plan_.clear();
      }

      return false;
    }

    /**
     * The conjunction of all observation buffers must be current
     */
    bool MoveBase::checkWatchDog() const {
      bool ok =  baseScanBuffer_->isCurrent() && tiltScanBuffer_->isCurrent() && stereoCloudBuffer_->isCurrent() && lowObstacleBuffer_->isCurrent();

      if(!ok) 
        ROS_INFO("Missed required cost map update. Should not allow commanding now. Check cost map data source.\n");

      return ok;
    }

    /**
     * Note that we have tried doing collision checks on the plan, and replanning in that case at the global level. With ARA*
     * at least, this leads to poor performance since planning ks slow (seconds) and thus the robot stops alot. If we leave the
     * velocity controller flexibility to work around the path in collision then that seems to work better. Note that this is still
     * sensitive to the goal (exit point of the path in the window) being in collision which would require an alternate metric
     * to allow more flexibility to get near the goal - essentially treating the goal as a waypoint. 
     */
    bool MoveBase::dispatchCommands(){
      // First criteria is that we have had a sufficiently recent sensor update to trust perception and that we have a valid plan. This latter
      // case is important since we can end up with an active controller that becomes invalid through the planner looking ahead. 
      // We want to be able to stop the robot in that case
      bool planOk = checkWatchDog() && isValid();
      robot_msgs::PoseDot cmdVel; // Commanded velocities      

      // if we have achieved all our waypoints but have yet to achieve the goal, then we know that we wish to accomplish our desired
      // orientation
      if(planOk && plan_.empty()){
        double uselessPitch, uselessRoll, yaw;
        global_pose_.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);
        ROS_DEBUG("Moving to desired goal orientation\n");
        cmdVel.vel.vx = 0;
        cmdVel.vel.vy = 0;
        double ang_diff =  fmod((double)stateMsg.goal.th, 2 * M_PI) - fmod(yaw, 2 * M_PI);
        if(ang_diff < 0){
          if(ang_diff < -1.0 * M_PI)
            cmdVel.ang_vel.vz = .6;
          else
            cmdVel.ang_vel.vz = -.6;
        }
        else{
          if(ang_diff > M_PI)
            cmdVel.ang_vel.vz = -.6;
          else
            cmdVel.ang_vel.vz = .6;
        }
      }
      else {
        // Refine the plan to reflect progress made. If no part of the plan is in the local cost window then
        // the global plan has failed since we are nowhere near the plan. We also prune parts of the plan that are behind us as we go. We determine this
        // by assuming that we start within a certain distance from the beginning of the plan and we can stay within a maximum error of the planned
        // path
        std::list<deprecated_msgs::Pose2DFloat32>::iterator it = plan_.begin();
        while(it != plan_.end()){
          const deprecated_msgs::Pose2DFloat32& w = *it;
          // Fixed error bound of 2 meters for now. Can reduce to a portion of the map size or based on the resolution
          if(fabs(global_pose_.getOrigin().x() - w.x) < 2 && fabs(global_pose_.getOrigin().y() - w.y) < 2){
            ROS_DEBUG("Nearest waypoint to <%f, %f> is <%f, %f>\n", global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), w.x, w.y);
            break;
          }

          it = plan_.erase(it);
        }

        // The plan is bogus if it is empty
        if(planOk && plan_.empty()){
          planOk = false;
          ROS_DEBUG("No path points in local window.\n");
        }

        // Set current velocities from odometry
        robot_msgs::PoseDot currentVel;
        currentVel.vel.vx = base_odom_.vel.x;
        currentVel.vel.vy = base_odom_.vel.y;
        currentVel.ang_vel.vz = base_odom_.vel.th;

	ros::Time start = ros::Time::now();
        // Create a window onto the global cost map for the velocity controller
        std::list<deprecated_msgs::Pose2DFloat32> localPlan; // Capture local plan for display

        lock();
        // Aggregate buffered observations across all sources. Must be thread safe
        std::vector<costmap_2d::Observation> observations;
        baseScanBuffer_->get_observations(observations);
        tiltScanBuffer_->get_observations(observations);
        lowObstacleBuffer_->get_observations(observations);
        stereoCloudBuffer_->get_observations(observations);
	//unlock(); COMMENTED AS PER BUG #971

        if(planOk && !controller_->computeVelocityCommands(plan_, global_pose_, currentVel, cmdVel, localPlan, observations)){
          ROS_DEBUG("Velocity Controller could not find a valid trajectory.\n");
          planOk = false;
        }
	unlock(); //ADDED AS PER BUG #971
        ROS_DEBUG("Cycle Time: %.3f\n", (ros::Time::now() - start).toSec());

        if(!planOk){
          // Zero out the velocities
          cmdVel.vel.vx = 0;
          cmdVel.vel.vy = 0;
          cmdVel.ang_vel.vz = 0;
        }
        else {
          publishPath(false, localPlan);
        }
      }

      ros::Node::instance()->publish("cmd_vel", cmdVel);
      double uselessPitch, uselessRoll, yaw;
      global_pose_.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);
      publishFootprint(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), yaw);

      //publish a point that the head can track
      double ptx, pty;
      controller_->getLocalGoal(ptx, pty);
      robot_msgs::PointStamped target_point;
      target_point.point.x = ptx;
      target_point.point.y = pty;
      target_point.point.z = 1;
      target_point.header.stamp = ros::Time::now();
      target_point.header.frame_id = global_frame_;
      ros::Node::instance()->publish("head_controller/head_track_point", target_point);
      return planOk;
    }

    /**
     * @todo Make based on loaded tolerances
     */
    bool MoveBase::withinDistance(double x1, double y1, double th1, double x2, double y2, double th2) const {
      if(fabs(x1 - x2) < this->xy_goal_tolerance_ && fabs(y1 - y2) < this->xy_goal_tolerance_)
        return true;

      return false;
    }

    /**
     * @brief Utility to output local obstacles. Make the local cost map accessor. It is very cheap :-) Then
     * render the obstacles. Note that the rendered window is typically larger than the local map for control
     */
    void MoveBase::publishLocalCostMap() {
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
      robot_msgs::Polyline2D pointCloudMsg;
      unsigned int pointCount = rawObstacles.size();
      pointCloudMsg.set_points_size(pointCount);
      pointCloudMsg.color.a = 0.0;
      pointCloudMsg.color.r = 1.0;
      pointCloudMsg.color.b = 0.0;
      pointCloudMsg.color.g = 0.0;

      for(unsigned int i=0;i<pointCount;i++){
        pointCloudMsg.points[i].x = rawObstacles[i].first;
        pointCloudMsg.points[i].y = rawObstacles[i].second;
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
    void MoveBase::publishFreeSpaceAndObstacles() {
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
      robot_msgs::Polyline2D pointCloudMsg;
      unsigned int pointCount = rawObstacles.size();
      pointCloudMsg.set_points_size(pointCount);
      pointCloudMsg.color.a = 0.0;
      pointCloudMsg.color.r = 1.0;
      pointCloudMsg.color.b = 0.0;
      pointCloudMsg.color.g = 0.0;

      for(unsigned int i=0;i<pointCount;i++){
        pointCloudMsg.points[i].x = rawObstacles[i].first;
        pointCloudMsg.points[i].y = rawObstacles[i].second;
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
      }

      ros::Node::instance()->publish("inflated_obstacles", pointCloudMsg);
    }

    void MoveBase::stopRobot(){
      ROS_DEBUG("Stopping the robot now!\n");
      robot_msgs::PoseDot cmdVel; // Commanded velocities
      cmdVel.vel.vx = 0.0;
      cmdVel.vel.vy = 0.0;
      cmdVel.ang_vel.vz = 0.0;
      ros::Node::instance()->publish("cmd_vel", cmdVel);
    }

    void MoveBase::handleDeactivation(){
      stopRobot();
    }

    MoveBase::footprint_t const & MoveBase::getFootprint() const{
      return footprint_;
    }
    
    void MoveBase::updateCostMap() {
      if (reset_cost_map_) {
	costMap_->revertToStaticMap(global_pose_.getOrigin().x(), global_pose_.getOrigin().y());
      }

      ROS_DEBUG("Starting cost map update/n");
      
      lock();
      // Aggregate buffered observations across all sources. Must be thread safe
      std::vector<costmap_2d::Observation> observations;
      baseScanBuffer_->get_observations(observations);
      tiltScanBuffer_->get_observations(observations);
      lowObstacleBuffer_->get_observations(observations);
      stereoCloudBuffer_->get_observations(observations);
      //unlock(); COMMENTED AS PER BUG #971
      
      ROS_DEBUG("Applying update with %d observations/n", observations.size());
      // Apply to cost map
      ros::Time start = ros::Time::now();
      costMap_->updateDynamicObstacles(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), observations);
      double t_diff = (ros::Time::now() - start).toSec();
      ROS_DEBUG("Updated map in %f seconds for %d observations/n", t_diff, observations.size());
      
      // Finally, we must extract the cost data that we have computed and:
      // 1. Refresh the local_map_accessor for the controller
      // 2. Refresh the global_map accessor for the planner
      // 3. Publish the local cost map window
      //lock(); COMMENTED AS PER BUG #971
      local_map_accessor_->refresh();
      global_map_accessor_->refresh();
      publishLocalCostMap();
      reset_cost_map_ = false;
      unlock();
    }


  /** Callback invoked when someone requests costmap */
  bool MoveBase::costmapCallback(robot_srvs::StaticMap::Request &req, robot_srvs::StaticMap::Response &res )
  {
    const unsigned char* costmap = getCostMap().getMap();
    res = costmap_response_;
    copy(costmap, costmap+res.map.width*res.map.height, res.map.data.begin());
    return true;
  }




    /**
     * Each update loop will query all observations and aggregate them and then apply
     * a batch update to the cost map
     */
    void MoveBase::mapUpdateLoop()
    {
      ros::Duration d;
      d.fromSec(1.0/map_update_frequency_);

      while (active_){
        //Avoids laser race conditions.
        if (isInitialized()) {
	  updateCostMap();
        }

        d.sleep();
      }
    }
  }
}
