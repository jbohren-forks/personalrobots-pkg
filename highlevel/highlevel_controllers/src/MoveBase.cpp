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


#include <MoveBase.hh>
#include <costmap_2d/basic_observation_buffer.h>
#include <std_msgs/BaseVel.h>
#include <std_msgs/PointCloud.h>
#include <std_msgs/Pose2DFloat32.h>
#include <std_msgs/Polyline2D.h>
#include <std_srvs/StaticMap.h>
#include <std_msgs/PointStamped.h>
#include <algorithm>
#include <iterator>
#include <angles/angles.h>
#include <boost/thread.hpp>

namespace ros {
  namespace highlevel_controllers {

    MoveBase::MoveBase()
      : HighlevelController<robot_msgs::Planner2DState, robot_msgs::Planner2DGoal>("move_base", "state", "goal"),
      tf_(*this, true, 10000000000ULL), // cache for 10 sec, no extrapolation
      controller_(NULL),
      costMap_(NULL),
      ma_(NULL),
      baseLaserMaxRange_(10.0),
      tiltLaserMaxRange_(10.0),
      minZ_(0.02), maxZ_(2.0), robotWidth_(0.0), active_(true) , map_update_frequency_(10.0),
      yaw_goal_tolerance_(0.1),
      xy_goal_tolerance_(robotWidth_ / 2)
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
      param("/costmap_2d/base_laser_max_range", baseLaserMaxRange_, baseLaserMaxRange_);
      param("/costmap_2d/tilt_laser_max_range", tiltLaserMaxRange_, tiltLaserMaxRange_);
       
      //thresholds for ground plane detection
      param("/ransac_ground_plane_extraction/distance_threshold", ransac_distance_threshold_, 0.03);

      // Unsigned chars cannot be stored in parameter server
      int tmpLethalObstacleThreshold;
      param("/costmap_2d/lethal_obstacle_threshold", tmpLethalObstacleThreshold, int(lethalObstacleThreshold));
      if (tmpLethalObstacleThreshold > 255)
        tmpLethalObstacleThreshold = 255;
      else if (tmpLethalObstacleThreshold < 0)
        tmpLethalObstacleThreshold = 0;

      lethalObstacleThreshold = tmpLethalObstacleThreshold;

      int tmpNoInformation;
      param("/costmap_2d/no_information_value", tmpNoInformation, int(noInformation));
      if (tmpNoInformation > 255)
        tmpNoInformation = 255;
      else if (tmpNoInformation < 0)
        tmpNoInformation = 0;

      noInformation = tmpNoInformation;

      param("/costmap_2d/z_threshold_max", maxZ_, maxZ_);
      param("/costmap_2d/z_threshold_min", minZ_, minZ_);
      param("/costmap_2d/freespace_projection_height", freeSpaceProjectionHeight, freeSpaceProjectionHeight);
      param("/costmap_2d/inflation_radius", inflationRadius, inflationRadius);
      param("/costmap_2d/circumscribed_radius", circumscribedRadius, circumscribedRadius);
      param("/costmap_2d/inscribed_radius", inscribedRadius, inscribedRadius);
      param("/costmap_2d/weight", weight, weight);

      robotWidth_ = inscribedRadius * 2;
      xy_goal_tolerance_ = robotWidth_ / 2;

      // Obtain parameters for sensors and allocate observation buffers accordingly. Rates are in Hz. 
      double base_laser_update_rate(2.0);
      double tilt_laser_update_rate(2.0);
      double low_obstacle_update_rate(0.2);
      double stereo_update_rate(2.0);
      param("/costmap_2d/base_laser_update_rate", base_laser_update_rate , base_laser_update_rate);
      param("/costmap_2d/tilt_laser_update_rate", tilt_laser_update_rate , tilt_laser_update_rate);
      param("/costmap_2d/low_obstacle_update_rate", low_obstacle_update_rate , low_obstacle_update_rate);
      param("/costmap_2d/stereo_update_rate", stereo_update_rate , stereo_update_rate);
      double base_laser_keepalive(0.0);
      double tilt_laser_keepalive(3.0);
      double low_obstacle_keepalive(2.0);
      double stereo_keepalive(0.0);
      param("/costmap_2d/base_laser_keepalive", base_laser_keepalive, base_laser_keepalive);
      param("/costmap_2d/tilt_laser_keepalive", tilt_laser_keepalive, tilt_laser_keepalive);
      param("/costmap_2d/low_obstacle_keepalive", low_obstacle_keepalive, low_obstacle_keepalive);
      param("/costmap_2d/stereo_keepalive", stereo_keepalive, stereo_keepalive);
      // Then allocate observation buffers
      baseScanBuffer_ = new costmap_2d::BasicObservationBuffer(std::string("base_laser"), tf_, 
							       ros::Duration().fromSec(base_laser_keepalive), 
							       costmap_2d::BasicObservationBuffer::computeRefreshInterval(base_laser_update_rate),
							       inscribedRadius, minZ_, maxZ_);
      tiltScanBuffer_ = new costmap_2d::BasicObservationBuffer(std::string("laser_tilt_link"), tf_, 
							       ros::Duration().fromSec(tilt_laser_keepalive), 
							       costmap_2d::BasicObservationBuffer::computeRefreshInterval(tilt_laser_update_rate),
							       inscribedRadius, minZ_, maxZ_);
      lowObstacleBuffer_ = new costmap_2d::BasicObservationBuffer(std::string("odom_combined"), tf_, 
								  ros::Duration().fromSec(low_obstacle_keepalive), 
								  costmap_2d::BasicObservationBuffer::computeRefreshInterval(low_obstacle_update_rate),
								  inscribedRadius, -10.0, maxZ_);
      stereoCloudBuffer_ = new costmap_2d::BasicObservationBuffer(std::string("stereo_link"), tf_, 
								  ros::Duration().fromSec(stereo_keepalive), 
								  costmap_2d::BasicObservationBuffer::computeRefreshInterval(stereo_update_rate),
								  inscribedRadius, minZ_, maxZ_);


      // get map via RPC
      std_srvs::StaticMap::request  req;
      std_srvs::StaticMap::response resp;
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
      param("/costmap_2d/zLB", zLB, 0.15);
      param("/costmap_2d/zUB", zUB, 0.25);
      param("/costmap_2d/raytrace_window", raytraceWindow, 2.5);
      param("/costmap_2d/raytrace_range", rayTraceRange, 10.0);
      param("/costmap_2d/obstacle_range", obstacleRange, 10.0);

      costMap_ = new CostMap2D((unsigned int)resp.map.width, (unsigned int)resp.map.height,
			       inputData , resp.map.resolution, 
			       lethalObstacleThreshold, maxZ_, zLB, zUB,
			       inflationRadius, circumscribedRadius, inscribedRadius, weight, 
			       obstacleRange, rayTraceRange, raytraceWindow);

      // Allocate Velocity Controller
      double mapSize(2.0);
      double pathDistanceBias(0.0);
      double goalDistanceBias(1.0);
      double accLimit_x(1.0);
      double accLimit_y(1.0);
      double accLimit_th(1.0);
      double sim_time = 1.0;
      //Note: sim_steps and samples_per_dim should be unsigned
      //ints but the param function does not like it
      int sim_steps = 20; 
      int samples_per_dim = 25;
      double dfast_scale = 0;
      double occdist_scale = 0.2;
      param("/trajectory_rollout/map_size", mapSize, 2.0);
      param("/trajectory_rollout/path_distance_bias", pathDistanceBias, 0.0);
      param("/trajectory_rollout/goal_distance_bias", goalDistanceBias, 1.0);
      param("/trajectory_rollout/acc_limit_x", accLimit_x, 1.0);
      param("/trajectory_rollout/acc_limit_y", accLimit_y, 1.0);
      param("/trajectory_rollout/acc_limit_th", accLimit_th, 1.0);
      param("/trajectory_rollout/occdist_scale", occdist_scale, occdist_scale);
      param("/trajectory_rollout/dfast_scale", dfast_scale, dfast_scale);
      param("/trajectory_rollout/samples_per_dim", samples_per_dim, samples_per_dim);
      param("/trajectory_rollout/sim_steps", sim_steps, sim_steps);
      param("/trajectory_rollout/sim_time", sim_time, sim_time);
      param("/trajectory_rollout/yaw_goal_tolerance", yaw_goal_tolerance_, yaw_goal_tolerance_);
      param("/trajectory_rollout/xy_goal_tolerance", xy_goal_tolerance_, xy_goal_tolerance_);

      ROS_ASSERT(mapSize <= costMap_->getWidth());
      ROS_ASSERT(mapSize <= costMap_->getHeight());

      ma_ = new CostMapAccessor(*costMap_, mapSize, 0.0, 0.0);

      std_msgs::Point2DFloat32 pt;
      //create a square footprint
      pt.x = robotRadius;
      pt.y = -1 * robotRadius;
      footprint_.push_back(pt);
      pt.x = -1 * robotRadius;
      pt.y = -1 * robotRadius;
      footprint_.push_back(pt);
      pt.x = -1 * robotRadius;
      pt.y = robotRadius;
      footprint_.push_back(pt);
      pt.x = robotRadius;
      pt.y = robotRadius;
      footprint_.push_back(pt);

      //give the robot a nose
      pt.x = circumscribedRadius;
      pt.y = 0;
      footprint_.push_back(pt);

      controller_ = new ros::highlevel_controllers::TrajectoryRolloutController(&tf_, *ma_,
          sim_time,
          sim_steps,
	  samples_per_dim,
          pathDistanceBias,
          goalDistanceBias,
          dfast_scale,
          occdist_scale,
          accLimit_x,
          accLimit_y,
          accLimit_th, 
          footprint_);

      // Advertize messages to publish cost map updates
      advertise<std_msgs::Polyline2D>("raw_obstacles", 1);
      advertise<std_msgs::Polyline2D>("inflated_obstacles", 1);

      // Advertize message to publish the global plan
      advertise<std_msgs::Polyline2D>("gui_path", 1);

      // Advertize message to publish local plan
      advertise<std_msgs::Polyline2D>("local_path", 1);

      // Advertize message to publish robot footprint
      advertise<std_msgs::Polyline2D>("robot_footprint", 1);

      // Advertize message to publish velocity cmds
      advertise<std_msgs::BaseVel>("cmd_vel", 1);

      //Advertize message to publish local goal for head to track
      advertise<std_msgs::PointStamped>("head_controller/head_track_point", 1);

      // The cost map is populated with either laser scans in the case that we are unable to use a
      // world model   source, or point clouds if we are. We shall pick one, and will be dominated by
      // point clouds
      subscribe("base_scan",  baseScanMsg_,  &MoveBase::baseScanCallback, 1);
      //subscribe("tilt_scan",  tiltScanMsg_,  &MoveBase::tiltScanCallback, 1);
      subscribe("tilt_laser_cloud_filtered", tiltCloudMsg_, &MoveBase::tiltCloudCallback, 1);
      subscribe("dcam/cloud",  stereoCloudMsg_,  &MoveBase::stereoCloudCallback, 1);
      subscribe("ground_plane",  groundPlaneMsg_,  &MoveBase::groundPlaneCallback, 1);
      subscribe("obstacle_cloud",  groundPlaneCloudMsg_,  &MoveBase::groundPlaneCloudCallback, 1);

      // Subscribe to odometry messages to get global pose
      subscribe("odom", odomMsg_, &MoveBase::odomCallback, 1);

      // Spawn map update thread
      map_update_thread_ = new boost::thread(boost::bind(&MoveBase::mapUpdateLoop, this));

      // Note: derived classes must initialize.
    }

    MoveBase::~MoveBase(){

      active_ = false;

      delete map_update_thread_;

      if(controller_ != NULL)
        delete controller_;

      if(ma_ != NULL)
        delete ma_;

      if(costMap_ != NULL)
        delete costMap_;

      delete baseScanBuffer_;
      delete lowObstacleBuffer_;
      delete tiltScanBuffer_;
      delete stereoCloudBuffer_;
    }

    void MoveBase::updateGlobalPose(){ 
      tf::Stamped<tf::Pose> robotPose;
      robotPose.setIdentity();
      robotPose.frame_id_ = "base_link";
      robotPose.stamp_ = ros::Time();

      try{
        tf_.transformPose("map", robotPose, global_pose_);
      }
      catch(tf::LookupException& ex) {
        ROS_INFO("No Transform available Error\n");
      }
      catch(tf::ConnectivityException& ex) {
        ROS_INFO("Connectivity Error\n");
      }
      catch(tf::ExtrapolationException& ex) {
        ROS_INFO("Extrapolation Error\n");
      }

      // Update the cost map window


      double uselessPitch, uselessRoll, yaw;
      global_pose_.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);
      ROS_DEBUG("Received new position (x=%f, y=%f, th=%f)", global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), yaw);

      ma_->updateForRobotPosition(global_pose_.getOrigin().x(), global_pose_.getOrigin().y());
    }


    /**
     * @brief Called by the main control loop in the base class. Lock already aquired
     */
    void MoveBase::updateGoalMsg(){
      // Revert to static map on new goal. May result in oscillation, but requested by Eitan for the milestone
      updateCostMap(true);

      tf::Stamped<tf::Pose> goalPose, transformedGoalPose;
      btQuaternion qt;
      qt.setEulerZYX(goalMsg.goal.th, 0, 0);
      goalPose.setData(btTransform(qt, btVector3(goalMsg.goal.x, goalMsg.goal.y, 0)));
      goalPose.frame_id_ = goalMsg.header.frame_id;
      goalPose.stamp_ = ros::Time();

      try{
        tf_.transformPose("map", goalPose, transformedGoalPose);
      }
      catch(tf::LookupException& ex){
        ROS_ERROR("No transform available from %s to map. This may be because the frame_id of the goalMsg is wrong.\n", goalMsg.header.frame_id.c_str());
        ROS_ERROR("The details of the LookupException are: %s\n", ex.what());
      }
      catch(tf::ConnectivityException& ex){
        ROS_ERROR("No transform available from %s to map. This may be because the frame_id of the goalMsg is wrong.\n", goalMsg.header.frame_id.c_str());
        ROS_ERROR("The details of the LookupException are: %s\n", ex.what());
      }
      catch(tf::ExtrapolationException& ex){
        ROS_ERROR("No transform available from %s to map. This may be because the frame_id of the goalMsg is wrong.\n", goalMsg.header.frame_id.c_str());
        ROS_ERROR("The details of the LookupException are: %s\n", ex.what());
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

    void MoveBase::baseScanCallback()
    {
      // Project laser into point cloud
      std_msgs::PointCloud local_cloud;
      local_cloud.header = baseScanMsg_.header;
      projector_.projectLaser(baseScanMsg_, local_cloud, baseLaserMaxRange_);
      lock();
      baseScanBuffer_->buffer_cloud(local_cloud);
      unlock();
    }

    void MoveBase::tiltScanCallback()
    {
      // Project laser into point cloud
      std_msgs::PointCloud local_cloud;
      local_cloud.header = tiltScanMsg_.header;
      projector_.projectLaser(tiltScanMsg_, local_cloud, tiltLaserMaxRange_);
      lock();
      tiltScanBuffer_->buffer_cloud(local_cloud);
      unlock();
    }

    void MoveBase::tiltCloudCallback()
    {
      lock();
      tiltScanBuffer_->buffer_cloud(tiltCloudMsg_);
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
      updateCostMap(true);
      stopRobot();
    }

    void MoveBase::updatePlan(const std::list<std_msgs::Pose2DFloat32>& newPlan){
      lock();

      // If we have a valid plan then only swap in the new plan if it is shorter.
      if(!isValid() || plan_.size() > newPlan.size()){
        plan_.clear();
        plan_ = newPlan;
        publishPath(true, plan_);
      }

      unlock();
    }

    /** \todo Some code duplication wrt MoveBase::updatePlan(const std::list<std_msgs::Pose2DFloat32>&). */
    void MoveBase::updatePlan(ompl::waypoint_plan_t const & newPlan) {
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
      for(std::list<std_msgs::Pose2DFloat32>::const_iterator it = plan_.begin(); it != plan_.end(); ++it){
        const std_msgs::Pose2DFloat32& w = *it;
        unsigned int ind = costMap_->WC_IND(w.x, w.y);
        if((*costMap_)[ind] >= CostMap2D::INSCRIBED_INFLATED_OBSTACLE){
          ROS_DEBUG("path in collision at <%f, %f>\n", w.x, w.y);
          return true;
        }
      }

      return false;
    }

    void MoveBase::publishFootprint(double x, double y, double th){
      std::vector<std_msgs::Point2DFloat32> footprint = controller_->drawFootprint(x, y, th);
      std_msgs::Polyline2D footprint_msg;
      footprint_msg.set_points_size(footprint.size());
      footprint_msg.color.r = 1.0;
      footprint_msg.color.g = 0;
      footprint_msg.color.b = 0;
      footprint_msg.color.a = 0;
      for(unsigned int i = 0; i < footprint.size(); ++i){
        footprint_msg.points[i].x = footprint[i].x;
        footprint_msg.points[i].y = footprint[i].y;
      }
      publish("robot_footprint", footprint_msg);
    }

    void MoveBase::publishPath(bool isGlobal, const std::list<std_msgs::Pose2DFloat32>& path) {
      std_msgs::Polyline2D guiPathMsg;
      guiPathMsg.set_points_size(path.size());

      unsigned int i = 0;
      for(std::list<std_msgs::Pose2DFloat32>::const_iterator it = path.begin(); it != path.end(); ++it){
        const std_msgs::Pose2DFloat32& w = *it;
        guiPathMsg.points[i].x = w.x;
        guiPathMsg.points[i].y = w.y;
        i++;
      }

      if(isGlobal){
        guiPathMsg.color.r = 0;
        guiPathMsg.color.g = 1.0;
        guiPathMsg.color.b = 0;
        guiPathMsg.color.a = 0;
        publish("gui_path", guiPathMsg);
      }
      else {
        guiPathMsg.color.r = 0;
        guiPathMsg.color.g = 0;
        guiPathMsg.color.b = 1.0;
        guiPathMsg.color.a = 0;
        publish("local_path", guiPathMsg);
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
      std_msgs::BaseVel cmdVel; // Commanded velocities      

      // Update the cost map window
      ma_->updateForRobotPosition(global_pose_.getOrigin().getX(), global_pose_.getOrigin().getY());

      // if we have achieved all our waypoints but have yet to achieve the goal, then we know that we wish to accomplish our desired
      // orientation
      if(planOk && plan_.empty()){
        double uselessPitch, uselessRoll, yaw;
        global_pose_.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);
        ROS_DEBUG("Moving to desired goal orientation\n");
        cmdVel.vx = 0;
        cmdVel.vy = 0;
        cmdVel.vw =  stateMsg.goal.th - yaw;
        cmdVel.vw = cmdVel.vw >= 0.0 ? cmdVel.vw + .4 : cmdVel.vw - .4;
      }
      else {
        // Refine the plan to reflect progress made. If no part of the plan is in the local cost window then
        // the global plan has failed since we are nowhere near the plan. We also prune parts of the plan that are behind us as we go. We determine this
        // by assuming that we start within a certain distance from the beginning of the plan and we can stay within a maximum error of the planned
        // path
        std::list<std_msgs::Pose2DFloat32>::iterator it = plan_.begin();
        while(it != plan_.end()){
          const std_msgs::Pose2DFloat32& w = *it;
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
        std_msgs::BaseVel currentVel;
        currentVel.vx = base_odom_.vel.x;
        currentVel.vy = base_odom_.vel.y;
        currentVel.vw = base_odom_.vel.th;

	ros::Time start = ros::Time::now();
        // Create a window onto the global cost map for the velocity controller
        std::list<std_msgs::Pose2DFloat32> localPlan; // Capture local plan for display
        if(planOk && !controller_->computeVelocityCommands(plan_, global_pose_, currentVel, cmdVel, localPlan)){
          ROS_DEBUG("Velocity Controller could not find a valid trajectory.\n");
          planOk = false;
        }
        ROS_DEBUG("Cycle Time: %.3f\n", (ros::Time::now() - start).toSec());

        if(!planOk){
          // Zero out the velocities
          cmdVel.vx = 0;
          cmdVel.vy = 0;
          cmdVel.vw = 0;
        }
        else {
          publishPath(false, localPlan);
        }
      }

      publish("cmd_vel", cmdVel);
      double uselessPitch, uselessRoll, yaw;
      global_pose_.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);
      publishFootprint(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), yaw);

      //publish a point that the head can track
      double ptx, pty;
      controller_->getLocalGoal(ptx, pty);
      std_msgs::PointStamped target_point;
      target_point.point.x = ptx;
      target_point.point.y = pty;
      target_point.point.z = 1;
      target_point.header.stamp = ros::Time::now();
      target_point.header.frame_id = "map";
      publish("head_controller/head_track_point", target_point);
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
     * render the obstacles.
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
      std_msgs::Polyline2D pointCloudMsg;
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

      if (!this->ok()) { 
	return; 
      }
      publish("raw_obstacles", pointCloudMsg);

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

      if (!this->ok()) { 
	return; 
      }
      publish("inflated_obstacles", pointCloudMsg);
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
      std_msgs::Polyline2D pointCloudMsg;
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

      publish("raw_obstacles", pointCloudMsg);

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

      publish("inflated_obstacles", pointCloudMsg);
    }

    void MoveBase::stopRobot(){
      ROS_DEBUG("Stopping the robot now!\n");
      std_msgs::BaseVel cmdVel; // Commanded velocities
      cmdVel.vx = 0.0;
      cmdVel.vy = 0.0;
      cmdVel.vw = 0.0;
      publish("cmd_vel", cmdVel);
    }

    void MoveBase::handleDeactivation(){
      stopRobot();
    }

    MoveBase::footprint_t const & MoveBase::getFootprint() const{
      return footprint_;
    }

    void MoveBase::updateCostMap(bool static_map_reset){
      ROS_DEBUG("Starting cost map update/n");
      if(static_map_reset)
        costMap_->revertToStaticMap(global_pose_.getOrigin().x(), global_pose_.getOrigin().y());

      // Aggregate buffered observations across 3 sources
      std::vector<costmap_2d::Observation> observations;
      baseScanBuffer_->get_observations(observations);
      tiltScanBuffer_->get_observations(observations);
      lowObstacleBuffer_->get_observations(observations);
      stereoCloudBuffer_->get_observations(observations);

      ROS_DEBUG("Applying update with %d observations/n", observations.size());
      // Apply to cost map
      ros::Time start = ros::Time::now();
      costMap_->updateDynamicObstacles(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), observations);
      double t_diff = (ros::Time::now() - start).toSec();
      publishLocalCostMap();
      ROS_DEBUG("Updated map in %f seconds for %d observations/n", t_diff, observations.size());
    }

    /**
     * Each update loop will query all observations and aggregate them and then apply
     * a batch update to the cost map
     */
    void MoveBase::mapUpdateLoop()
    {
      ros::Duration *d = new ros::Duration();
      d->fromSec(1.0/map_update_frequency_);

      while (active_){
        //Avoids laser race conditions.
        if (isInitialized()) {
          //update the cost map without resetting to static map
          lock();
          updateCostMap(false);
          unlock();
        }

        d->sleep();
      }

      delete d;
    }
  }
}
