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
#include <std_msgs/BaseVel.h>
#include <std_msgs/PointCloud.h>
#include <std_msgs/Pose2DFloat32.h>
#include <std_msgs/Polyline2D.h>
#include <std_srvs/StaticMap.h>
#include <std_msgs/PointStamped.h>
#include <deque>
#include <set>

namespace ros {
  namespace highlevel_controllers {

    MoveBase::MoveBase()
      : HighlevelController<std_msgs::Planner2DState, std_msgs::Planner2DGoal>("move_base", "state", "goal"),
	tf_(*this, true, 10000000000ULL, 0ULL), // cache for 10 sec, no extrapolation
	controller_(NULL),
	costMap_(NULL),
	ma_(NULL),
	baseLaserMaxRange_(10.0),
	tiltLaserMaxRange_(10.0),
	minZ_(0.03), maxZ_(2.0), robotWidth_(0.0) {
      // Initialize global pose. Will be set in control loop based on actual data.
      global_pose_.x = 0;
      global_pose_.y = 0;
      global_pose_.yaw = 0;

      // Initialize odometry
      base_odom_.vel.x = 0;
      base_odom_.vel.y = 0;
      base_odom_.vel.th = 0;

      // Initialize state message parameters that are unsused
      stateMsg.waypoint.x = 0.0;
      stateMsg.waypoint.y = 0.0;
      stateMsg.waypoint.th = 0.0;
      stateMsg.set_waypoints_size(0);
      stateMsg.waypoint_idx = -1;

      // This should become a static transform. For now we will simply allow it to be provided
      // as a parameter until we hear how static transforms are to be handled.
      double laser_x_offset(0.275);
      param("/laser_x_offset", laser_x_offset, laser_x_offset);
      //tf_.setWithEulers("base_laser", "base", laser_x_offset, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

      // Costmap parameters
      double windowLength(1.0);
      unsigned char lethalObstacleThreshold(100);
      unsigned char noInformation(CostMap2D::NO_INFORMATION);
      double freeSpaceProjectionHeight(0.5);
      double inflationRadius(0.50);
      double robotRadius(0.325);
      double circumscribedRadius(0.46);
      double inscribedRadius(0.325);
      param("/costmap_2d/base_laser_max_range", baseLaserMaxRange_, baseLaserMaxRange_);
      param("/costmap_2d/tilt_laser_max_range", tiltLaserMaxRange_, tiltLaserMaxRange_);
      param("/costmap_2d/dynamic_obstacle_window", windowLength, windowLength);
      param("/costmap_2d/lethal_obstacle_threshold", lethalObstacleThreshold, lethalObstacleThreshold);
      param("/costmap_2d/no_information_value", noInformation, noInformation);
      param("/costmap_2d/z_threshold", maxZ_, maxZ_);
      param("/costmap_2d/freespace_projection_height", freeSpaceProjectionHeight, freeSpaceProjectionHeight);
      param("/costmap_2d/inflation_radius", inflationRadius, inflationRadius);
      param("/costmap_2d/circumscribed_radius", circumscribedRadius, circumscribedRadius);
      param("/costmap_2d/inscribed_radius", inscribedRadius, inscribedRadius);

      robotWidth_ = inscribedRadius * 2;

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
      costMap_ = new CostMap2D((unsigned int)resp.map.width, (unsigned int)resp.map.height,
                               inputData , resp.map.resolution, 
			       windowLength, lethalObstacleThreshold, maxZ_, freeSpaceProjectionHeight,
			       inflationRadius, circumscribedRadius, inscribedRadius);

      // Allocate Velocity Controller
      double mapSize(2.0);
      double pathDistanceBias(0.4);
      double goalDistanceBias(0.6);
      double accLimit_x(0.15);
      double accLimit_y(1.0);
      double accLimit_th(1.0);
      const double SIM_TIME = 1.0;
      const unsigned int SIM_STEPS = 30;
      const unsigned int SAMPLES_PER_DIM = 25;
      const double DFAST_SCALE = 0;
      const double OCCDIST_SCALE = 0;
      param("/trajectory_rollout/map_size", mapSize, 2.0);
      param("/trajectory_rollout/path_distance_bias", pathDistanceBias, 0.4);
      param("/trajectory_rollout/goal_distance_bias", goalDistanceBias, 0.6);
      param("/trajectory_rollout/acc_limit_x", accLimit_x, 0.15);
      param("/trajectory_rollout/acc_limit_y", accLimit_y, 0.15);
      param("/trajectory_rollout/acc_limit_th", accLimit_th, 1.0);

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
										SIM_TIME,
										SIM_STEPS,
										SAMPLES_PER_DIM,
										pathDistanceBias,
										goalDistanceBias,
										DFAST_SCALE,
										OCCDIST_SCALE,
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
      
      //Advertize message to publis local goal for head to track
      advertise<std_msgs::PointStamped>("head_controller/head_track_point", 1);

      // The cost map is populated with either laser scans in the case that we are unable to use a
      // world model   source, or point clouds if we are. We shall pick one, and will be dominated by
      // point clouds
      subscribe("base_scan",  baseScanMsg_,  &MoveBase::baseScanCallback, 1);
      subscribe("tilt_scan",  tiltScanMsg_,  &MoveBase::tiltScanCallback, 1);
      subscribe("stereo_cloud",  stereoCloudMsg_,  &MoveBase::stereoCloudCallback, 1);

      // Subscribe to odometry messages to get global pose
      subscribe("odom", odomMsg_, &MoveBase::odomCallback, 1);

      // Note: derived classes must initialize.
    }

    MoveBase::~MoveBase(){

      if(controller_ != NULL)
	delete controller_;

      if(ma_ != NULL)
	delete ma_;

      if(costMap_ != NULL)
	delete costMap_;
    }

    void MoveBase::updateGlobalPose(){
      libTF::TFPose2D robotPose;
      robotPose.x = 0;
      robotPose.y = 0;
      robotPose.yaw = 0;
      robotPose.frame = "base";
      robotPose.time = 0; 

      try{
	global_pose_ = this->tf_.transformPose2D("map", robotPose);
      }
      catch(libTF::TransformReference::LookupException& ex){
	ROS_INFO("No Transform available Error\n");
      }
      catch(libTF::TransformReference::ConnectivityException& ex){
	ROS_INFO("Connectivity Error\n");
      }
      catch(libTF::TransformReference::ExtrapolateException& ex){
	ROS_INFO("Extrapolation Error\n");
      }

      // Update the cost map window
      ma_->updateForRobotPosition(global_pose_.x, global_pose_.y);
    }


    void MoveBase::updateGoalMsg(){
      libTF::TFPose2D goalPose, transformedGoalPose;
      goalPose.x = goalMsg.goal.x;
      goalPose.y = goalMsg.goal.y;
      goalPose.yaw = goalMsg.goal.th;
      goalPose.frame = goalMsg.header.frame_id;
      goalPose.time = 0;
	  
      try{
	transformedGoalPose = this->tf_.transformPose2D("map", goalPose);
      }
      catch(libTF::TransformReference::LookupException& ex){
	ROS_ERROR("No transform available from %s to map. This may be because the frame_id of the goalMsg is wrong.\n", goalMsg.header.frame_id.c_str());
	ROS_ERROR("The details of the LookupException are: %s\n", ex.what());
      }
      catch(libTF::TransformReference::ConnectivityException& ex){
	ROS_ERROR("No transform available from %s to map. This may be because the frame_id of the goalMsg is wrong.\n", goalMsg.header.frame_id.c_str());
	ROS_ERROR("The details of the LookupException are: %s\n", ex.what());
      }
      catch(libTF::TransformReference::ExtrapolateException& ex){
	ROS_ERROR("No transform available from %s to map. This may be because the frame_id of the goalMsg is wrong.\n", goalMsg.header.frame_id.c_str());
	ROS_ERROR("The details of the LookupException are: %s\n", ex.what());
      }

      stateMsg.goal.x = goalPose.x;
      stateMsg.goal.y = goalPose.y;
      stateMsg.goal.th = goalPose.yaw;

      ROS_DEBUG("Received new goal (x=%f, y=%f, th=%f)\n", goalMsg.goal.x, goalMsg.goal.y, goalMsg.goal.th);
    }

    void MoveBase::updateStateMsg(){
      // Get the current robot pose in the map frame
      updateGlobalPose();

      // Assign state data 
      stateMsg.pos.x = global_pose_.x;
      stateMsg.pos.y = global_pose_.y;
      stateMsg.pos.th = global_pose_.yaw;
    }
    
    void MoveBase::baseScanCallback()
    {
      ROS_INFO("Base Scan Callback");
      // Project laser into point cloud
      std_msgs::PointCloud local_cloud;
      local_cloud.header = baseScanMsg_.header;
      projector_.projectLaser(baseScanMsg_, local_cloud, baseLaserMaxRange_);
      ROS_INFO("Projected");
      processData(local_cloud);
    }

    void MoveBase::tiltScanCallback()
    {
      // Project laser into point cloud
      std_msgs::PointCloud local_cloud;
      local_cloud.header = tiltScanMsg_.header;
      projector_.projectLaser(tiltScanMsg_, local_cloud, tiltLaserMaxRange_);
      processData(local_cloud);
    }

    void MoveBase::stereoCloudCallback()
    {
      processData(stereoCloudMsg_);
    }
    
    void MoveBase::processData(const std_msgs::PointCloud& local_cloud)
    {
      bufferMutex_.lock();

      point_clouds_.push_back(local_cloud);

      std_msgs::PointCloud * newData = NULL;

      while(!point_clouds_.empty()){

	const std_msgs::PointCloud& point_cloud = point_clouds_.front();

	if(local_cloud.header.stamp - point_cloud.header.stamp > ros::Duration(9, 0)){
	  point_clouds_.pop_front();
	  continue;
	}

	std_msgs::PointCloud base_cloud;
	std_msgs::PointCloud map_cloud;
	
	/* Transform to the base frame */
	try
	  {
	    tf_.transformPointCloud("base", base_cloud, point_cloud);
	    newData = extractFootprintAndGround(base_cloud);
	    tf_.transformPointCloud("map", map_cloud, *newData);
	  }
	catch(libTF::TransformReference::LookupException& ex)
	  {
	    ROS_ERROR("Lookup exception: %s\n", ex.what());
	    break;
	  }
	catch(libTF::TransformReference::ExtrapolateException& ex)
	  {
	    ROS_DEBUG("No transform available yet - have to try later: %s . Buffer size is %d\n", ex.what(), point_clouds_.size());
	    break;
	  }
	catch(libTF::TransformReference::ConnectivityException& ex)
	  {
	    ROS_ERROR("Connectivity exception: %s\n", ex.what());
	    break;
	  }
	catch(...)
	  {
	    ROS_ERROR("Exception in point cloud computation\n");
	    break;
	  }

	point_clouds_.pop_front();

	if (newData == NULL){
	    delete newData;
	    newData = NULL;
	}

	// Is the time stamp copied when we do a tranform?
	// Verify what happens if we get many updates in the same update time step?
	// How can interleaving effect things?
	ROS_INFO("Processing point cloud with %d points\n", map_cloud.get_pts_size());
        updateDynamicObstacles(point_cloud.header.stamp.to_double(), map_cloud);
      }

      // In case we get thrown out on the second transform - clean up
      if (newData == NULL){
	delete newData;
	newData = NULL;
      }

      bufferMutex_.unlock();
    }

    /**
     * The point is in the footprint if its x and y values are in the range [0 robotWidth] in
     * the base frame.
     */
    bool MoveBase::inFootprint(double x, double y) const {
      bool result = fabs(x) <= robotWidth_/2 && fabs(y) <= robotWidth_/2;

      if(result){
	ROS_DEBUG("Discarding point <%f, %f> in footprint\n", x, y);
      }
      return result;
    }

    std_msgs::PointCloud * MoveBase::extractFootprintAndGround(const std_msgs::PointCloud& baseFrameCloud) const {
      std_msgs::PointCloud *copy = new std_msgs::PointCloud();
      copy->header = baseFrameCloud.header;

      unsigned int n = baseFrameCloud.get_pts_size();
      unsigned int j = 0;
      copy->set_pts_size(n);
      for (unsigned int k = 0 ; k < n ; ++k){

	ROS_DEBUG("Evaluating <%f, %f, %f>\n", 
		  baseFrameCloud.pts[k].x, baseFrameCloud.pts[k].y,   baseFrameCloud.pts[k].z);

	bool ok = baseFrameCloud.pts[k].z > minZ_ &&   baseFrameCloud.pts[k].z < maxZ_;
	ROS_DEBUG("Discarding point for height %f\n", baseFrameCloud.pts[k].z);
	if (ok && !inFootprint(baseFrameCloud.pts[k].x, baseFrameCloud.pts[k].y))
	  copy->pts[j++] = baseFrameCloud.pts[k];

      }

      copy->set_pts_size(j);
	
      ROS_DEBUG("Filter discarded %d points (%d left) \n", n - j, j);

      return copy;
    }

    /**
     * The odomMsg_ will be updates and we will do the transform to update the odom in the base frame
     */
    void MoveBase::odomCallback(){
      if(isTerminated())
	return;

      base_odom_.lock();

      try
	{
	  libTF::TFVector v_in, v_out;
	  v_in.x = odomMsg_.vel.x;
	  v_in.y = odomMsg_.vel.y;
	  v_in.z = odomMsg_.vel.th;	  
	  v_in.time = 0; // Gets the latest
	  v_in.frame = "odom";
	  v_out = tf_.transformVector("base", v_in);
	  base_odom_.vel.x = v_in.x;
	  base_odom_.vel.y = v_in.y;
	  base_odom_.vel.th = v_in.z;
	}
      catch(libTF::TransformReference::LookupException& ex)
	{
	  puts("no odom->base Tx yet");
	  ROS_DEBUG("%s\n", ex.what());
	}
      catch(libTF::TransformReference::ConnectivityException& ex)
	{
	  puts("no odom->base Tx yet");
	  ROS_DEBUG("%s\n", ex.what());
	}
      catch(libTF::TransformReference::ExtrapolateException& ex)
	{
	  puts("Extrapolation exception");
	}

      base_odom_.unlock();
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
    
    /**
     * This is used as a validation check and is only called from within dispatchCommands where the lock has already been
     * applied to protect access to the plan.
     */
    bool MoveBase::inCollision() const {
      for(std::list<std_msgs::Pose2DFloat32>::const_iterator it = plan_.begin(); it != plan_.end(); ++it){
	const std_msgs::Pose2DFloat32& w = *it;
	unsigned int ind = costMap_->WC_IND(w.x, w.y);
	if((*costMap_)[ind] > CostMap2D::CIRCUMSCRIBED_INFLATED_OBSTACLE){
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
      // Publish the global plan
      publishPath(true, plan_);

      // If the plan has been executed (i.e. empty) and we are within a required distance of the target orientation,
      // and we have stopped the robot, then we are done
      if(plan_.empty() && 
	 fabs(global_pose_.yaw - stateMsg.goal.th) < 0.1){

	ROS_DEBUG("Goal achieved at: (%f, %f, %f) for (%f, %f, %f)\n",
		  global_pose_.x, global_pose_.y, global_pose_.yaw,
		  stateMsg.goal.x, stateMsg.goal.y, stateMsg.goal.th);

	// The last act will issue stop command
	stopRobot();

	return true;
      }

      // If we have reached the end of the path then clear the plan
      if(!plan_.empty() &&
	 withinDistance(global_pose_.x, global_pose_.y, global_pose_.yaw,
			stateMsg.goal.x, stateMsg.goal.y, global_pose_.yaw)){
	ROS_DEBUG("Last waypoint achieved at: (%f, %f, %f) for (%f, %f, %f)\n",
	       global_pose_.x, global_pose_.y, global_pose_.yaw,
	       stateMsg.goal.x, stateMsg.goal.y, stateMsg.goal.th);

	plan_.clear();
      }

      return false;
    }

    void MoveBase::petTheWatchDog(){
      gettimeofday(&lastUpdated_, NULL);
    }

    /**
     * At most one second between updates
     */
    bool MoveBase::checkWatchDog() const {
      struct timeval curr;
      gettimeofday(&curr,NULL);
      double curr_t, last_t, t_diff;
      last_t = lastUpdated_.tv_sec + lastUpdated_.tv_usec / 1e6;
      curr_t = curr.tv_sec + curr.tv_usec / 1e6;
      t_diff = curr_t - last_t;
      bool ok = t_diff < 1.0;
      if(!ok) 
	ROS_DEBUG("Missed required cost map update. Should not allow commanding now. Check cost map data source.\n");
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
      bool planOk = checkWatchDog();
      std_msgs::BaseVel cmdVel; // Commanded velocities      

      // if we have achieved all our waypoints but have yet to achieve the goal, then we know that we wish to accomplish our desired
      // orientation
      if(plan_.empty()){
	ROS_DEBUG("Moving to desired goal orientation\n");
	cmdVel.vx = 0;
	cmdVel.vy = 0;
	cmdVel.vw =  .5;
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
	  if(fabs(global_pose_.x - w.x) < 2 && fabs(global_pose_.y - w.y) < 2){
	    ROS_DEBUG("Nearest waypoint to <%f, %f> is <%f, %f>\n", global_pose_.x, global_pose_.y, w.x, w.y);
	    break;
	  }

	  it = plan_.erase(it);
	}

	// The plan is bogus if it is empty
	if(planOk && plan_.empty()){
	  planOk = false;
	  ROS_ASSERT(inCollision());
	  ROS_DEBUG("No path points in local window.\n");
	}

	// Set current velocities from odometry
	std_msgs::BaseVel currentVel;
	currentVel.vx = base_odom_.vel.x;
	currentVel.vy = base_odom_.vel.y;
	currentVel.vw = base_odom_.vel.th;

	struct timeval start;
	struct timeval end;
	double start_t, end_t, t_diff;
	// Create a window onto the global cost map for the velocity controller
	std::list<std_msgs::Pose2DFloat32> localPlan; // Capture local plan for display
	gettimeofday(&start,NULL);
	if(planOk && !controller_->computeVelocityCommands(plan_, global_pose_, currentVel, cmdVel, localPlan)){
	  ROS_DEBUG("Velocity Controller could not find a valid trajectory.\n");
	  planOk = false;
	}
	gettimeofday(&end,NULL);
	start_t = start.tv_sec + double(start.tv_usec) / 1e6;
	end_t = end.tv_sec + double(end.tv_usec) / 1e6;
	t_diff = end_t - start_t;
	ROS_DEBUG("Cycle Time: %.3f\n", t_diff);

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

      ROS_INFO("Dispatching velocity vector: (%f, %f, %f)\n", cmdVel.vx, cmdVel.vy, cmdVel.vw);

      publish("cmd_vel", cmdVel);
      publishFootprint(global_pose_.x, global_pose_.y, global_pose_.yaw);

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
      if(fabs(x1 - x2) < 4 * getCostMap().getResolution() &&
	 fabs(y1 - y2) < 4 * getCostMap().getResolution())
	return true;

      return false;
    }

    /**
     * @brief Utility to output local obstacles. Make the local cost map accessor. It is very cheap :-) Then
     * render the obstacles.
     */
    void MoveBase::publishLocalCostMap() {
      double mapSize = std::min(costMap_->getWidth()/2, costMap_->getHeight()/2);
      CostMapAccessor cm(*costMap_, std::min(10.0, mapSize), global_pose_.x, global_pose_.y);

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
      ROS_INFO("Stopping the robot now!\n");
      std_msgs::BaseVel cmdVel; // Commanded velocities
      cmdVel.vx = 0.0;
      cmdVel.vy = 0.0;
      cmdVel.vw = 0.0;
      publish("cmd_vel", cmdVel);
    }

    void MoveBase::handleDeactivation(){
      stopRobot();
    }

    void MoveBase::updateDynamicObstacles(double ts, const std_msgs::PointCloud& cloud){
      //Avoids laser race conditions.
      if (!isInitialized()) {
	return;
      }

      std::vector<unsigned int> updates;
      lock();
      petTheWatchDog();
      struct timeval curr;
      gettimeofday(&curr,NULL);
      double curr_t, last_t, t_diff;
      curr_t = curr.tv_sec + curr.tv_usec / 1e6;
      costMap_->updateDynamicObstacles(ts, global_pose_.x, global_pose_.y, cloud, updates);
      gettimeofday(&curr,NULL);
      last_t = curr.tv_sec + curr.tv_usec / 1e6;
      t_diff = last_t - curr_t;
      handleMapUpdates(updates);
      publishLocalCostMap();
      unlock();
      ROS_INFO("Updated map in %f seconds/n", t_diff);
    }
    
    MoveBase::footprint_t const & MoveBase::getFootprint() const{
      return footprint_;
    }

  }
}
