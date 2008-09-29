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
#include <std_msgs/PointCloudFloat32.h>
#include <std_msgs/Pose2DFloat32.h>
#include <std_msgs/Polyline2D.h>
#include <std_srvs/StaticMap.h>
#include <set>

namespace ros {
  namespace highlevel_controllers {

    MoveBase::MoveBase(VelocityController& vc, double windowLength, unsigned char lethalObstacleThreshold, unsigned char noInformation, double maxZ, double inflationRadius)
      : HighlevelController<std_msgs::Planner2DState, std_msgs::Planner2DGoal>("move_base", "state", "goal"),
	controller_(vc),
	tf_(*this, true, 10000000000ULL), // cache for 10 sec, no extrapolation
	costMap_(NULL),
	laserMaxRange_(4.0) {

      // Initialize the velocity controller with the transform client
      controller_.initialize(tf_);

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

      // Set up transforms
      double laser_x_offset(0.05);
      //param("laser_x_offset", laser_x_offset, 0.05);
      tf_.setWithEulers("base_laser", "base", laser_x_offset, 0.0, 0.0, 0.0, 0.0, 0.0, 0);

      // get map via RPC
      std_srvs::StaticMap::request  req;
      std_srvs::StaticMap::response resp;
      std::cout << "Requesting the map..." << std::endl;
      while(!ros::service::call("static_map", req, resp))
	{
	  std::cout << "Request failed; trying again..." << std::endl;
	  usleep(1000000);
	}

      std::cout << "Received a " << resp.map.width << " X " << 
	resp.map.height << " map at " << 
	resp.map.resolution << "m/pix" << std::endl;

      // Convert to cost map
      unsigned char* mapdata;
      unsigned int numCells = resp.map.width*resp.map.height;
      mapdata = new unsigned char[numCells];
      for(unsigned int i=0;i<numCells;i++)
	{
	  // This is masking the case of no information since it does not seem to be
	  // represented in the input and the users of the cost map do not exploit it
	  if(resp.map.data[i] > lethalObstacleThreshold){
	    mapdata[i] = lethalObstacleThreshold; //CostMap2D::NO_INFORMATION;
	  }
	  else{
	    // Simply copy the actual cost value. It will get thresholded in the cost map
	    mapdata[i] = resp.map.data[i];
	  }
	}

      // Now allocate the cost map
      costMap_ = new CostMap2D(resp.map.width, resp.map.height, mapdata, resp.map.resolution, 
			       windowLength, lethalObstacleThreshold, maxZ, inflationRadius);

      // Finish by deleting the mapdata
      delete mapdata;

      // Subscribe to laser scan messages
      subscribe("scan", laserScanMsg_, &MoveBase::laserScanCallback, QUEUE_MAX());

      // Subscribe to odometry messages
      subscribe("odom", odomMsg_, &MoveBase::odomCallback, QUEUE_MAX());

      // Advertize messages to publish path updates
      advertise<std_msgs::Polyline2D>("gui_laser", QUEUE_MAX());

      // Advertize message to publish the global plan
      advertise<std_msgs::Polyline2D>("gui_path", QUEUE_MAX());

      // Advertize message to publish local plan
      advertise<std_msgs::Polyline2D>("local_path", QUEUE_MAX());

      // Advertize message to publish velocity cmds
      advertise<std_msgs::BaseVel>("cmd_vel", QUEUE_MAX());

      // Now initialize
      initialize();
    }

    MoveBase::~MoveBase(){

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
	std::cout << "No Transform available Error\n";
      }
      catch(libTF::TransformReference::ConnectivityException& ex){
	std::cout << "Connectivity Error\n";
      }
      catch(libTF::TransformReference::ExtrapolateException& ex){
	std::cout << "Extrapolation Error\n";
      }

      //std::cout << "Robot at (" << global_pose_.x << ", " << global_pose_.y << ", " << global_pose_.yaw << ")\n";
    }


    void MoveBase::updateGoalMsg(){
      lock();
      stateMsg.goal.x = goalMsg.goal.x;
      stateMsg.goal.y = goalMsg.goal.y;
      stateMsg.goal.th = goalMsg.goal.th;
      unlock();

      printf("Received new goal (x=%f, y=%f, th=%f)\n", goalMsg.goal.x, goalMsg.goal.y, goalMsg.goal.th);
    }

    void MoveBase::updateStateMsg(){
      // Get the current robot pose in the map frame
      updateGlobalPose();

      // Assign state data 
      stateMsg.pos.x = global_pose_.x;
      stateMsg.pos.y = global_pose_.y;
      stateMsg.pos.th = global_pose_.yaw;
    }

    /**
     * The laserScanMsg_ member will have been updated. It is locked already too.
     */
    void MoveBase::laserScanCallback(){

      // Assemble a point cloud, in the laser's frame
      std_msgs::PointCloudFloat32 local_cloud;
      projector_.projectLaser(laserScanMsg_, local_cloud, laserMaxRange_);
    
      // Convert to a point cloud in the map frame
      std_msgs::PointCloudFloat32 global_cloud;

      try
	{
	  global_cloud = this->tf_.transformPointCloud("map", local_cloud);
	}
      catch(libTF::TransformReference::LookupException& ex)
	{
	  puts("no global->local Tx yet");
	  printf("%s\n", ex.what());
	  return;
	}
      catch(libTF::TransformReference::ConnectivityException& ex)
	{
	  puts("no global->local Tx yet");
	  printf("%s\n", ex.what());
	  return;
	}
      catch(libTF::TransformReference::ExtrapolateException& ex)
	{
	  puts("Extrapolation exception");
	}

      /*
	std::cout << "Laser scan received (Laser Scan:" << laserScanMsg_.get_ranges_size() << 
	", localCloud:" << local_cloud.get_pts_size() << 
	", globalCloud:" << global_cloud.get_pts_size() << ")\n";
      */

      // Update the cost map
      const double ts = laserScanMsg_.header.stamp.to_double();
      std::vector<unsigned int> insertions, deletions;

      // Surround with a lock since it can interact with main planning and execution thread
      lock();
      costMap_->updateDynamicObstacles(ts, global_cloud, insertions, deletions);
      handleMapUpdates(insertions, deletions);
      publishLocalCostMap();
      unlock();
    }

    /**
     * The odomMsg_ will be updates and we will do the transform to update the odom in the base frame
     */
    void MoveBase::odomCallback(){
      lock();
      try
	{
	  libTF::TFVector v_in, v_out;
	  v_in.x = odomMsg_.vel.x;
	  v_in.y = odomMsg_.vel.y;
	  v_in.z = odomMsg_.vel.th;	  
	  v_in.time = 0; // Gets the latest
	  v_in.frame = "odom";
	  v_out = tf_.transformVector("base", v_in);
	  base_odom_.vel.x = v_out.x;
	  base_odom_.vel.y = v_out.y;
	  base_odom_.vel.th = v_out.z;
	}
      catch(libTF::TransformReference::LookupException& ex)
	{
	  puts("no odom->base Tx yet");
	  printf("%s\n", ex.what());
	}
      catch(libTF::TransformReference::ConnectivityException& ex)
	{
	  puts("no odom->base Tx yet");
	  printf("%s\n", ex.what());
	}
      catch(libTF::TransformReference::ExtrapolateException& ex)
	{
	  puts("Extrapolation exception");
	}

      unlock();
    }

    void MoveBase::updatePlan(const std::list<std_msgs::Pose2DFloat32>& newPlan){
      plan_.clear();
      plan_ = newPlan;
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
	guiPathMsg.color.r = 1;
	guiPathMsg.color.g = 0;
	guiPathMsg.color.b = 0;
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
	 fabs(global_pose_.yaw - stateMsg.goal.th) < 10 &&
	 fabs(base_odom_.vel.x - 0) < 0.001 &&
	 fabs(base_odom_.vel.y - 0) < 0.001 &&
	 fabs(base_odom_.vel.th - 0) < 0.001){

	printf("Goal achieved at: (%f, %f, %f) for (%f, %f, %f)\n",
	       global_pose_.x, global_pose_.y, global_pose_.yaw,
	       stateMsg.goal.x, stateMsg.goal.y, stateMsg.goal.th);

	return true;
      }

      // If we have reached the end of the path then clear the plan
      if(!plan_.empty() &&
	 withinDistance(global_pose_.x, global_pose_.y, global_pose_.yaw,
			stateMsg.goal.x, stateMsg.goal.y, global_pose_.yaw)){
	printf("Last waypoint achieved at: (%f, %f, %f) for (%f, %f, %f)\n",
	       global_pose_.x, global_pose_.y, global_pose_.yaw,
	       stateMsg.goal.x, stateMsg.goal.y, stateMsg.goal.th);

	plan_.clear();
      }

      return false;
    }

    bool MoveBase::dispatchCommands(){
      bool planOk = true; // Return value to trigger replanning or not
      std_msgs::BaseVel cmdVel; // Commanded velocities      

      // if we have achieved all our waypoints but have yet to achieve the goal, then we know that we wish to accomplish our desired
      // orientation
      if(plan_.empty()){
	std::cout << "Moving to desired goal orientation\n";
	cmdVel.vx = 0;
	cmdVel.vy = 0;
	cmdVel.vw = stateMsg.goal.th - global_pose_.yaw;
      }
      else {
	// Refine the plan to reflect progress made. If no part of the plan is in the local cost window then
	// the global plan has failed since we are nowhere near the plan. We also prune parts of the plan that are behind us as we go. We determine this
	// by assuming that we start within a certain distance from the beginning of the plan and we can stay within a maximum error of the planned
	// path
	std::list<std_msgs::Pose2DFloat32>::iterator it = plan_.begin();
	while(it != plan_.end()){
	  const std_msgs::Pose2DFloat32& w = *it;

	  // The path following constraint is twice the map resolution. Could be an external parameter
	  if(sqrt(pow(global_pose_.x - w.x, 2) + pow(global_pose_.y - w.y, 2)) <= (getCostMap().getResolution() * 2))
	    break;

	  it = plan_.erase(it);
	}

	// The plan is bogus if it is empty
	planOk = !plan_.empty();

	// Set current velocities from odometry
	std_msgs::BaseVel currentVel;
	currentVel.vx = base_odom_.vel.x;
	currentVel.vy = base_odom_.vel.y;
	currentVel.vw = base_odom_.vel.th;

	// Create a window onto the global cost map for the velocity controller
	CostMapAccessor ma(getCostMap(), controller_.getMapDeltaX(), controller_.getMapDeltaY(), global_pose_.x, global_pose_.y);
	std::list<std_msgs::Pose2DFloat32> localPlan; // Capture local plan for display
	planOk = planOk && controller_.computeVelocityCommands(ma, plan_, global_pose_, currentVel, cmdVel, localPlan);

	if(!planOk){
	  // Zero out the velocities
	  cmdVel.vx = 0;
	  cmdVel.vy = 0;
	  cmdVel.vw = 0;
	  std::cout << "Local planning has failed :-(\n";
	}

	publishPath(false, localPlan);
	publishPath(true, plan_);
      }

      printf("Dispatching velocity vector: (%f, %f, %f)\n", cmdVel.vx, cmdVel.vy, cmdVel.vw);

      publish("cmd_vel", cmdVel);


      return planOk;
    }

    /**
     * @todo Make based on loaded tolerances
     */
    bool MoveBase::withinDistance(double x1, double y1, double th1, double x2, double y2, double th2) const {
      if(fabs(x1 - x2) < getCostMap().getResolution() &&
	 fabs(y1 - y2) < getCostMap().getResolution() &&
	 fabs(th1- th2)< 10)
	return true;

      return false;
    }

    /**
     * @brief Utility to output local obstacles. Make the local cost map accessor. It is very cheap :-) Then
     * render the obstacles.
     */
    void MoveBase::publishLocalCostMap() {
      // Now we should have a valid origin to construct the local map as a window onto the global map
      CostMapAccessor ma(getCostMap(), 
			 controller_.getMapDeltaX(),
			 controller_.getMapDeltaY(), 
			 global_pose_.x, global_pose_.y);

      // Publish obstacle data for each obstacle cell
      std::vector< std::pair<double, double> > allObstacles;
      double origin_x, origin_y;
      ma.getOriginInWorldCoordinates(origin_x, origin_y);
      for(unsigned int i = 0; i<ma.getWidth(); i++)
	for(unsigned int j = 0; j<ma.getHeight();j++){
	  if(ma.isObstacle(i, j) || ma.isInflatedObstacle(i, j) ){
	      double wx, wy;
	      wx = i * ma.getResolution() + origin_x;
	      wy = j * ma.getResolution() + origin_y;
	      std::pair<double, double> p(wx, wy);
	      allObstacles.push_back(p);
	    }
	}


      std_msgs::Polyline2D pointCloudMsg;
      unsigned int pointCount = allObstacles.size();
      pointCloudMsg.set_points_size(pointCount);
      pointCloudMsg.color.a = 0.0;
      pointCloudMsg.color.r = 0.0;
      pointCloudMsg.color.b = 1.0;
      pointCloudMsg.color.g = 0.0;

      for(unsigned int i=0;i<pointCount;i++){
	pointCloudMsg.points[i].x = allObstacles[i].first;
	pointCloudMsg.points[i].y = allObstacles[i].second;
      }

      publish("gui_laser", pointCloudMsg);
    }
  }
}
