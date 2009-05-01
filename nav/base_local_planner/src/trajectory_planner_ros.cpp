/*********************************************************************
*
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include <base_local_planner/trajectory_planner_ros.h>
#include <ros/console.h>
#include <sys/time.h>

#include "visualization_msgs/Polyline.h"

using namespace std;
using namespace robot_msgs;
using namespace costmap_2d;
using namespace laser_scan;

namespace base_local_planner {
  TrajectoryPlannerROS::TrajectoryPlannerROS(ros::Node& ros_node, tf::TransformListener& tf, 
      Costmap2D& costmap, std::vector<Point> footprint_spec, const Costmap2D* planner_map) 
    : world_model_(NULL), tc_(NULL), costmap_(costmap), base_scan_notifier_(NULL), tf_(tf), ros_node_(ros_node), laser_scans_(2), 
    point_grid_(NULL), voxel_grid_(NULL), rot_stopped_velocity_(1e-2), trans_stopped_velocity_(1e-2), goal_reached_(true), costmap_publisher_(NULL){
    double acc_lim_x, acc_lim_y, acc_lim_theta, sim_time, sim_granularity;
    int vx_samples, vtheta_samples;
    double pdist_scale, gdist_scale, occdist_scale, heading_lookahead, oscillation_reset_dist;
    bool holonomic_robot, dwa, simple_attractor, heading_scoring;
    double heading_scoring_timestep;
    double max_vel_x, min_vel_x, max_vel_th, min_vel_th;
    string world_model_type;

    //adverstise the fact that we'll publish the robot footprint
    ros_node.advertise<visualization_msgs::Polyline>("~base_local_planner/robot_footprint", 1);
    ros_node.advertise<visualization_msgs::Polyline>("~base_local_planner/global_plan", 1);
    ros_node.advertise<visualization_msgs::Polyline>("~base_local_planner/local_plan", 1);

    ros_node.param("~base_local_planner/costmap/global_frame", global_frame_, string("map"));
    ros_node.param("~base_local_planner/costmap/robot_base_frame", robot_base_frame_, string("base_link"));
    ros_node.param("~base_local_planner/transform_tolerance", transform_tolerance_, 0.2);

    double map_publish_frequency;
    ros_node.param("~base_local_planner/costmap_visualization_rate", map_publish_frequency, 2.0);
    costmap_publisher_ = new costmap_2d::Costmap2DPublisher(ros_node, costmap, map_publish_frequency, global_frame_, "base_local_planner");

    //we need to make sure that the transform between the robot base frame and the global frame is available
    while(!tf_.canTransform(global_frame_, robot_base_frame_, ros::Time(), ros::Duration(5.0))){
      ROS_WARN("Waiting on transform from %s to %s to become available before running the controller", robot_base_frame_.c_str(), global_frame_.c_str());
    }

    ros_node.param("~base_local_planner/yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
    ros_node.param("~base_local_planner/xy_goal_tolerance", xy_goal_tolerance_, 0.10);

    string odom_topic;
    ros_node.param("~base_local_planner/odom_topic", odom_topic, string("odom"));
    // Subscribe to odometry messages to get global pose
    ros_node.subscribe(odom_topic, odom_msg_, &TrajectoryPlannerROS::odomCallback, this, 1);

    base_scan_notifier_ = new tf::MessageNotifier<LaserScan>(&tf_, &ros_node,
        boost::bind(&TrajectoryPlannerROS::baseScanCallback, this, _1),
        "base_scan", global_frame_, 50);

    tilt_scan_notifier_ = new tf::MessageNotifier<LaserScan>(&tf_, &ros_node,
        boost::bind(&TrajectoryPlannerROS::tiltScanCallback, this, _1),
        //"tilt_scan", global_frame_, 50);
        "tilt_scan", global_frame_, 50);

    //we'll get the parameters for the robot radius from the costmap we're associated with
    ros_node.param("~base_local_planner/costmap/inscribed_radius", inscribed_radius_, 0.325);
    ros_node.param("~base_local_planner/costmap/circumscribed_radius", circumscribed_radius_, 0.46);
    ros_node.param("~base_local_planner/costmap/inflation_radius", inflation_radius_, 0.55);
    
    ros_node.param("~base_local_planner/acc_lim_x", acc_lim_x, 2.5);
    ros_node.param("~base_local_planner/acc_lim_y", acc_lim_y, 2.5);
    ros_node.param("~base_local_planner/acc_lim_th", acc_lim_theta, 3.2);
    ros_node.param("~base_local_planner/sim_time", sim_time, 1.0);
    ros_node.param("~base_local_planner/sim_granularity", sim_granularity, 0.025);
    ros_node.param("~base_local_planner/vx_samples", vx_samples, 20);
    ros_node.param("~base_local_planner/vtheta_samples", vtheta_samples, 20);
    ros_node.param("~base_local_planner/path_distance_bias", pdist_scale, 0.6);
    ros_node.param("~base_local_planner/goal_distance_bias", gdist_scale, 0.8);
    ros_node.param("~base_local_planner/occdist_scale", occdist_scale, 0.2);
    ros_node.param("~base_local_planner/heading_lookahead", heading_lookahead, 0.325);
    ros_node.param("~base_local_planner/oscillation_reset_dist", oscillation_reset_dist, 0.05);
    ros_node.param("~base_local_planner/holonomic_robot", holonomic_robot, true);
    ros_node.param("~base_local_planner/max_vel_x", max_vel_x, 0.5);
    ros_node.param("~base_local_planner/min_vel_x", min_vel_x, 0.1);
    ros_node.param("~base_local_planner/max_vel_th", max_vel_th, 1.0);
    ros_node.param("~base_local_planner/min_vel_th", min_vel_th, -1.0);
    ros_node.param("~base_local_planner/min_in_place_vel_th", min_in_place_vel_th_, 0.4);
    ros_node.param("~base_local_planner/world_model", world_model_type, string("freespace"));
    ros_node.param("~base_local_planner/dwa", dwa, false);
    ros_node.param("~base_local_planner/heading_scoring", heading_scoring, false);
    ros_node.param("~base_local_planner/heading_scoring_timestep", heading_scoring_timestep, 0.1);
    ros_node.param("~base_local_planner/simple_attractor", simple_attractor, false);

    //parameters for using the freespace controller
    double min_pt_separation, max_obstacle_height, grid_resolution;
    ros_node.param("~base_local_planner/point_grid/max_sensor_range", max_sensor_range_, 2.0);
    ros_node.param("~base_local_planner/point_grid/min_pt_separation", min_pt_separation, 0.01);
    ros_node.param("~base_local_planner/point_grid/max_obstacle_height", max_obstacle_height, 2.0);
    ros_node.param("~base_local_planner/point_grid/grid_resolution", grid_resolution, 0.2);

    if(world_model_type == "freespace"){
      ROS_ASSERT_MSG(planner_map != NULL, "Until a rolling window version of the freespace controller is implemented... the costmap the planner uses must be passed in for sizing info.");
      Point origin;
      origin.x = planner_map->originX();
      origin.y = planner_map->originY();
      unsigned int cmap_width, cmap_height;
      cmap_width = planner_map->cellSizeX();
      cmap_height = planner_map->cellSizeY();
      point_grid_ = new PointGrid(cmap_width * planner_map->resolution(), cmap_height * planner_map->resolution(), grid_resolution, 
          origin, max_obstacle_height, max_sensor_range_, min_pt_separation);
      world_model_ = point_grid_;
      ROS_DEBUG("Freespace Origin: (%.4f, %.4f), Width: %.4f, Height: %.4f\n", origin.x, origin.y, cmap_width * planner_map->resolution(), cmap_height * planner_map->resolution());
      /*For Debugging
      ros_node.advertise<PointCloud>("point_grid", 1);
      */
    }
    else if(world_model_type == "voxel"){
      ROS_ASSERT_MSG(planner_map != NULL, "Until a rolling window version of the voxel controller is implemented... the costmap the planner uses must be passed in for sizing info.");
      double origin_x, origin_y;
      origin_x = planner_map->originX();
      origin_y = planner_map->originY();
      unsigned int cmap_width, cmap_height;
      cmap_width = planner_map->cellSizeX();
      cmap_height = planner_map->cellSizeY();
      voxel_grid_ = new VoxelGridModel(cmap_width, cmap_height, 10, planner_map->resolution(), max_obstacle_height / 10,
          origin_x, origin_y, 0.0, max_obstacle_height, max_sensor_range_);
      world_model_ = voxel_grid_;
      /*For Debugging
      ros_node.advertise<PointCloud>("point_grid", 1);
      */
    }
    else{
      world_model_ = new CostmapModel(costmap); 
      ROS_INFO("Costmap\n");
    }

    tc_ = new TrajectoryPlanner(*world_model_, costmap, footprint_spec, inscribed_radius_, circumscribed_radius_,
        acc_lim_x, acc_lim_y, acc_lim_theta, sim_time, sim_granularity, vx_samples, vtheta_samples, pdist_scale,
        gdist_scale, occdist_scale, heading_lookahead, oscillation_reset_dist, holonomic_robot,
        max_vel_x, min_vel_x, max_vel_th, min_vel_th, min_in_place_vel_th_,
        dwa, heading_scoring, heading_scoring_timestep, simple_attractor);
  }

  void TrajectoryPlannerROS::baseScanCallback(const tf::MessageNotifier<LaserScan>::MessagePtr& message){
    //project the laser into a point cloud
    PointCloud base_cloud;
    base_cloud.header = message->header;
    //we want all values... even those out of range
    projector_.projectLaser(*message, base_cloud, -1.0, true);
    tf::Stamped<btVector3> global_origin;

    obs_lock_.lock();
    laser_scans_[0].angle_min = message->angle_min;
    laser_scans_[0].angle_max = message->angle_max;
    laser_scans_[0].angle_increment = message->angle_increment;

    //we know the transform is available from the laser frame to the global frame 
    try{
      //transform the origin for the sensor
      tf::Stamped<btVector3> local_origin(btVector3(0, 0, 0), base_cloud.header.stamp, base_cloud.header.frame_id);
      tf_.transformPoint(global_frame_, local_origin, global_origin);
      laser_scans_[0].origin.x = global_origin.getX();
      laser_scans_[0].origin.y = global_origin.getY();
      laser_scans_[0].origin.z = global_origin.getZ();

      //transform the point cloud
      tf_.transformPointCloud(global_frame_, base_cloud, laser_scans_[0].cloud);
    }
    catch(tf::TransformException& ex){
      ROS_ERROR("TF Exception that should never happen %s", ex.what());
      return;
    }
    obs_lock_.unlock();
  }

  void TrajectoryPlannerROS::tiltScanCallback(const tf::MessageNotifier<LaserScan>::MessagePtr& message){
    //project the laser into a point cloud
    PointCloud tilt_cloud;
    tilt_cloud.header = message->header;
    //we want all values... even those out of range
    projector_.projectLaser(*message, tilt_cloud, -1.0, true);
    tf::Stamped<btVector3> global_origin;

    obs_lock_.lock();
    laser_scans_[1].angle_min = message->angle_min;
    laser_scans_[1].angle_max = message->angle_max;
    laser_scans_[1].angle_increment = message->angle_increment;

    //we know the transform is available from the laser frame to the global frame 
    try{
      //transform the origin for the sensor
      tf::Stamped<btVector3> local_origin(btVector3(0, 0, 0), tilt_cloud.header.stamp, tilt_cloud.header.frame_id);
      tf_.transformPoint(global_frame_, local_origin, global_origin);
      laser_scans_[1].origin.x = global_origin.getX();
      laser_scans_[1].origin.y = global_origin.getY();
      laser_scans_[1].origin.z = global_origin.getZ();

      //transform the point cloud
      tf_.transformPointCloud(global_frame_, tilt_cloud, laser_scans_[1].cloud);
    }
    catch(tf::TransformException& ex){
      ROS_ERROR("TF Exception that should never happen %s", ex.what());
      return;
    }
    obs_lock_.unlock();
  }

  TrajectoryPlannerROS::~TrajectoryPlannerROS(){
    if(costmap_publisher_ != NULL)
      delete costmap_publisher_;

    if(base_scan_notifier_ != NULL)
      delete base_scan_notifier_;

    if(tilt_scan_notifier_ != NULL)
      delete tilt_scan_notifier_;

    if(tc_ != NULL)
      delete tc_;

    if(world_model_ != NULL)
      delete world_model_;
  }

  vector<Point> TrajectoryPlannerROS::drawFootprint(double x_i, double y_i, double theta_i){
    return tc_->drawFootprint(x_i, y_i, theta_i);
  }

  bool TrajectoryPlannerROS::stopped(){
    return abs(base_odom_.vel.th) <= rot_stopped_velocity_ 
      && abs(base_odom_.vel.x) <= trans_stopped_velocity_
      && abs(base_odom_.vel.y) <= trans_stopped_velocity_;
  }

  double TrajectoryPlannerROS::distance(double x1, double y1, double x2, double y2){
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  }

  bool TrajectoryPlannerROS::goalPositionReached(const tf::Stamped<tf::Pose>& global_pose, double goal_x, double goal_y){
    double dist = distance(global_pose.getOrigin().x(), global_pose.getOrigin().y(), goal_x, goal_y);
    return fabs(dist) <= xy_goal_tolerance_;
  }

  bool TrajectoryPlannerROS::goalOrientationReached(const tf::Stamped<tf::Pose>& global_pose, double goal_th){
    double useless_pitch, useless_roll, yaw;
    global_pose.getBasis().getEulerZYX(yaw, useless_pitch, useless_roll);
    return fabs(angles::shortest_angular_distance(yaw, goal_th)) <= yaw_goal_tolerance_;
  }

  void TrajectoryPlannerROS::rotateToGoal(const tf::Stamped<tf::Pose>& global_pose, double goal_th, robot_msgs::PoseDot& cmd_vel){
    double uselessPitch, uselessRoll, yaw;
    global_pose.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);
    ROS_DEBUG("Moving to desired goal orientation\n");
    cmd_vel.vel.vx = 0;
    cmd_vel.vel.vy = 0;
    double ang_diff = angles::shortest_angular_distance(yaw, goal_th);
    cmd_vel.ang_vel.vz = ang_diff > 0.0 ? std::max(min_in_place_vel_th_, ang_diff) : std::min(-1.0 * min_in_place_vel_th_, ang_diff);
  }

  void TrajectoryPlannerROS::odomCallback(){
    base_odom_.lock();

    try
    {
      tf::Stamped<btVector3> v_in(btVector3(odom_msg_.vel.x, odom_msg_.vel.y, 0), ros::Time(), odom_msg_.header.frame_id), v_out;
      tf_.transformVector(robot_base_frame_, ros::Time(), v_in, odom_msg_.header.frame_id, v_out);
      base_odom_.vel.x = v_in.x();
      base_odom_.vel.y = v_in.y();
      base_odom_.vel.th = odom_msg_.vel.th;
    }
    catch(tf::LookupException& ex)
    {
      ROS_DEBUG("No odom->base Tx yet: %s", ex.what());
    }
    catch(tf::ConnectivityException& ex)
    {
      ROS_DEBUG("No odom->base Tx yet: %s\n", ex.what());
    }
    catch(tf::ExtrapolationException& ex)
    {
      ROS_DEBUG("Extrapolation exception");
    }

    base_odom_.unlock();
  }

  bool TrajectoryPlannerROS::goalReached(){
    return goal_reached_;
  }

  void TrajectoryPlannerROS::updatePlan(const std::vector<robot_msgs::PoseStamped>& orig_global_plan){
    //reset the global plan
    global_plan_.clear();

    //transform the plan into the frame of the controller
    try{
      for(unsigned int i = 0; i < orig_global_plan.size(); ++i){
        robot_msgs::PoseStamped new_pose;
        ros::Time current_time = ros::Time::now(); // save time for checking tf delay later
	new_pose = orig_global_plan[i];
	new_pose.header.stamp = ros::Time();
	tf_.transformPose(global_frame_, new_pose, new_pose);
        // check global_pose timeout
        if (current_time.toSec() - new_pose.header.stamp.toSec() > transform_tolerance_) {
          ROS_ERROR("TrajectoryPlannerROS transform timeout updating plan. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
              current_time.toSec() ,new_pose.header.stamp.toSec() ,transform_tolerance_);
          return;
        }
        global_plan_.push_back(new_pose);
      }
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      return;
    }
  }

  bool TrajectoryPlannerROS::computeVelocityCommands(robot_msgs::PoseDot& cmd_vel, const std::vector<costmap_2d::Observation>& observations){
    //assume at the beginning of our control cycle that we could have a new goal
    goal_reached_ = false;

    std::vector<robot_msgs::PoseStamped> local_plan;
    tf::Stamped<tf::Pose> robot_pose, global_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = robot_base_frame_;
    robot_pose.stamp_ = ros::Time();
    ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

    //get the global pose of the robot
    try{
      tf_.transformPose(global_frame_, robot_pose, global_pose);
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      return false;
    }
    // check global_pose timeout
    if (current_time.toSec() - global_pose.stamp_.toSec() > transform_tolerance_) {
      ROS_ERROR("TrajectoryPlannerROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
          current_time.toSec() ,global_pose.stamp_.toSec() ,transform_tolerance_);
      return false;
    }

    //now we'll prune the plan based on the position of the robot
    prunePlan(global_pose, global_plan_);

    //we also want to clear the robot footprint from the costmap we're using
    clearRobotFootprint(global_pose, costmap_);

    // Set current velocities from odometry
    robot_msgs::PoseDot global_vel;
    global_vel.vel.vx = base_odom_.vel.x;
    global_vel.vel.vy = base_odom_.vel.y;
    global_vel.ang_vel.vz = base_odom_.vel.th;

    tf::Stamped<tf::Pose> drive_cmds;
    drive_cmds.frame_id_ = robot_base_frame_;

    tf::Stamped<tf::Pose> robot_vel;
    btQuaternion qt(global_vel.ang_vel.vz, 0, 0);
    robot_vel.setData(btTransform(qt, btVector3(global_vel.vel.vx, global_vel.vel.vy, 0)));
    robot_vel.frame_id_ = robot_base_frame_;
    robot_vel.stamp_ = ros::Time();

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */

    //if the global plan passed in is empty... we won't do anything
    if(global_plan_.empty())
      return false;

    tf::Stamped<tf::Pose> goal_point;
    tf::PoseStampedMsgToTF(global_plan_.back(), goal_point);
    //we assume the global goal is the last point in the global plan
    double goal_x = goal_point.getOrigin().getX();
    double goal_y = goal_point.getOrigin().getY();

    double uselessPitch, uselessRoll, yaw;
    goal_point.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);

    double goal_th = yaw;

    //check to see if we've reached the goal position
    if(goalPositionReached(global_pose, goal_x, goal_y)){
      //check to see if the goal orientation has been reached
      if(goalOrientationReached(global_pose, goal_th)){
        //set the velocity command to zero
        cmd_vel.vel.vx = 0.0;
        cmd_vel.vel.vy = 0.0;
        cmd_vel.ang_vel.vz = 0.0;

        //make sure that we're actually stopped before returning success
        if(stopped())
          goal_reached_ = true;
      }
      else {
        //otherwise we need to rotate to the goal... compute the next velocity we should take
        rotateToGoal(global_pose, goal_th, cmd_vel);
      }

      //publish the robot footprint and an empty plan because we've reached our goal position
      publishFootprint(global_pose);
      publishPlan(global_plan_, "global_plan", 0.0, 1.0, 0.0, 0.0);
      publishPlan(local_plan, "local_plan", 0.0, 0.0, 1.0, 0.0);

      //we don't actually want to run the controller when we're just rotating to goal
      return true;
    }

    tc_->updatePlan(global_plan_);

    obs_lock_.lock();
    //compute what trajectory to drive along
    Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds, observations, laser_scans_);
    obs_lock_.unlock();

    /* For timing uncomment
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_INFO("Cycle time: %.9f", t_diff);
    */

    //pass along drive commands
    cmd_vel.vel.vx = drive_cmds.getOrigin().getX();
    cmd_vel.vel.vy = drive_cmds.getOrigin().getY();
    drive_cmds.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);

    cmd_vel.ang_vel.vz = yaw;

    //if we cannot move... tell someone
    if(path.cost_ < 0){
      local_plan.clear();
      publishFootprint(global_pose);
      publishPlan(global_plan_, "global_plan", 0.0, 1.0, 0.0, 0.0);
      publishPlan(local_plan, "local_plan", 0.0, 0.0, 1.0, 0.0);
      return false;
    }

    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i){
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(tf::Quaternion(p_th, 0.0, 0.0), tf::Point(p_x, p_y, 0.0)), ros::Time::now(), global_frame_);
      robot_msgs::PoseStamped pose;
      tf::PoseStampedTFToMsg(p, pose);
      local_plan.push_back(pose);
    }

    /* For Debugging
    if(point_grid_ != NULL){
      PointCloud grid_cloud;
      grid_cloud.header.frame_id = global_frame_;
      point_grid_->getPoints(grid_cloud);
      ros::Node::instance()->publish("point_grid", grid_cloud);
    }

    if(voxel_grid_ != NULL){
      PointCloud grid_cloud;
      grid_cloud.header.frame_id = global_frame_;
      voxel_grid_->getPoints(grid_cloud);
      ros::Node::instance()->publish("point_grid", grid_cloud);
    }
    */

    //publish information to the visualizer
    publishFootprint(global_pose);
    publishPlan(global_plan_, "global_plan", 0.0, 1.0, 0.0, 0.0);
    publishPlan(local_plan, "local_plan", 0.0, 0.0, 1.0, 0.0);
    return true;
  }

  void TrajectoryPlannerROS::getLocalGoal(double& x, double& y){
    return tc_->getLocalGoal(x, y);
  }

  void TrajectoryPlannerROS::prunePlan(const tf::Stamped<tf::Pose>& global_pose, std::vector<robot_msgs::PoseStamped>& plan){
    std::vector<robot_msgs::PoseStamped>::iterator it = plan.begin();
    while(it != plan.end()){
      const robot_msgs::PoseStamped& w = *it;
      // Fixed error bound of 2 meters for now. Can reduce to a portion of the map size or based on the resolution
      double x_diff = global_pose.getOrigin().x() - w.pose.position.x;
      double y_diff = global_pose.getOrigin().y() - w.pose.position.y;
      double distance = sqrt(x_diff * x_diff + y_diff * y_diff);
      if(distance < 1){
        ROS_DEBUG("Nearest waypoint to <%f, %f> is <%f, %f>\n", global_pose.getOrigin().x(), global_pose.getOrigin().y(), w.pose.position.x, w.pose.position.y);
        break;
      }
      it = plan.erase(it);
    }
  }

  void TrajectoryPlannerROS::clearRobotFootprint(Costmap2D& costmap){
    tf::Stamped<tf::Pose> robot_pose, global_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = robot_base_frame_;
    robot_pose.stamp_ = ros::Time();
    ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

    //get the global pose of the robot
    try{
      tf_.transformPose(global_frame_, robot_pose, global_pose);
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      return;
    }
    // check global_pose timeout
    if (current_time.toSec() - global_pose.stamp_.toSec() > transform_tolerance_) {
      ROS_ERROR("TrajcetoryPlannerROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
          current_time.toSec() ,global_pose.stamp_.toSec() ,transform_tolerance_);
      return;
    }

    clearRobotFootprint(global_pose, costmap);
  }

  void TrajectoryPlannerROS::clearRobotFootprint(const tf::Stamped<tf::Pose>& global_pose, Costmap2D& costmap){
    double useless_pitch, useless_roll, yaw;
    global_pose.getBasis().getEulerZYX(yaw, useless_pitch, useless_roll);

    //get the oriented footprint of the robot
    std::vector<robot_msgs::Point> oriented_footprint = drawFootprint(global_pose.getOrigin().x(), global_pose.getOrigin().y(), yaw);

    costmap.lock();
    //set the associated costs in the cost map to be free
    if(!costmap.setConvexPolygonCost(oriented_footprint, costmap_2d::FREE_SPACE))
      return;

    double max_inflation_dist = inflation_radius_ + circumscribed_radius_;

    //clear all non-lethal obstacles out to the maximum inflation distance of an obstacle in the robot footprint
    costmap.clearNonLethal(global_pose.getOrigin().x(), global_pose.getOrigin().y(), max_inflation_dist, max_inflation_dist);

    //make sure to re-inflate obstacles in the affected region... plus those obstalces that could 
    costmap.reinflateWindow(global_pose.getOrigin().x(), global_pose.getOrigin().y(), max_inflation_dist + inflation_radius_, max_inflation_dist + inflation_radius_, false);
    costmap.unlock();

  }

  void TrajectoryPlannerROS::publishFootprint(const tf::Stamped<tf::Pose>& global_pose){
    double useless_pitch, useless_roll, yaw;
    global_pose.getBasis().getEulerZYX(yaw, useless_pitch, useless_roll);
    std::vector<robot_msgs::Point> footprint = drawFootprint(global_pose.getOrigin().x(), global_pose.getOrigin().y(), yaw);
    visualization_msgs::Polyline footprint_msg;
    footprint_msg.header.frame_id = global_frame_;
    footprint_msg.set_points_size(footprint.size());
    footprint_msg.color.r = 1.0;
    footprint_msg.color.g = 0;
    footprint_msg.color.b = 0;
    footprint_msg.color.a = 0;
    for(unsigned int i = 0; i < footprint.size(); ++i){
      footprint_msg.points[i].x = footprint[i].x;
      footprint_msg.points[i].y = footprint[i].y;
      footprint_msg.points[i].z = footprint[i].z;
    }
    ros_node_.publish("~base_local_planner/robot_footprint", footprint_msg);
  }

  void TrajectoryPlannerROS::publishPlan(const std::vector<robot_msgs::PoseStamped>& path, std::string topic, double r, double g, double b, double a){
    visualization_msgs::Polyline gui_path_msg;
    gui_path_msg.header.frame_id = global_frame_;

    //given an empty path we won't do anything
    if(!path.empty()){
      // Extract the plan in world co-ordinates, we assume the path is all in the same frame
      gui_path_msg.header.stamp = path[0].header.stamp;
      gui_path_msg.set_points_size(path.size());
      for(unsigned int i=0; i < path.size(); i++){
        gui_path_msg.points[i].x = path[i].pose.position.x;
        gui_path_msg.points[i].y = path[i].pose.position.y;
        gui_path_msg.points[i].z = path[i].pose.position.z;
      }
    }

    gui_path_msg.color.r = r;
    gui_path_msg.color.g = g;
    gui_path_msg.color.b = b;
    gui_path_msg.color.a = a;

    ros_node_.publish("~base_local_planner/" + topic, gui_path_msg);
  }
};
