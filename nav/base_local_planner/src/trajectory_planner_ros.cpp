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

using namespace std;
using namespace robot_msgs;
using namespace costmap_2d;
using namespace laser_scan;

namespace base_local_planner {
  TrajectoryPlannerROS::TrajectoryPlannerROS(ros::Node& ros_node, tf::TransformListener& tf, 
      const Costmap2D& cost_map, std::vector<Point> footprint_spec, const Costmap2D* planner_map) 
    : world_model_(NULL), tc_(NULL), base_scan_notifier_(NULL), tf_(tf), laser_scans_(2), 
    point_grid_(NULL), voxel_grid_(NULL), rot_stopped_velocity_(1e-2), trans_stopped_velocity_(1e-2), goal_reached_(true){
    double acc_lim_x, acc_lim_y, acc_lim_theta, sim_time, sim_granularity;
    int vx_samples, vtheta_samples;
    double pdist_scale, gdist_scale, occdist_scale, heading_lookahead, oscillation_reset_dist;
    bool holonomic_robot, dwa, simple_attractor, heading_scoring;
    double heading_scoring_timestep;
    double max_vel_x, min_vel_x, max_vel_th, min_vel_th;
    string world_model_type;

    ros_node.param("~base_local_planner/global_frame", global_frame_, string("map"));
    ros_node.param("~base_local_planner/robot_base_frame", robot_base_frame_, string("base_link"));

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

    double inscribed_radius, circumscribed_radius;
    ros_node.param("~base_local_planner/inscribed_radius", inscribed_radius, 0.325);
    ros_node.param("~base_local_planner/circumscribed_radius", circumscribed_radius, 0.46);
    
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
      world_model_ = new CostmapModel(cost_map); 
      ROS_INFO("Costmap\n");
    }

    tc_ = new TrajectoryPlanner(*world_model_, cost_map, footprint_spec, inscribed_radius, circumscribed_radius,
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

  bool TrajectoryPlannerROS::computeVelocityCommands(const std::vector<robot_msgs::PoseStamped>& orig_global_plan, 
      robot_msgs::PoseDot& cmd_vel,
      std::vector<robot_msgs::PoseStamped>& local_plan,
      const std::vector<costmap_2d::Observation>& observations){

    tf::Stamped<tf::Pose> robot_pose, global_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = robot_base_frame_;
    robot_pose.stamp_ = ros::Time();

    std::vector<robot_msgs::PoseStamped> global_plan;

    //get the global pose of the robot
    try{
      tf_.transformPose(global_frame_, robot_pose, global_pose);
      //transform the plan for the robot into the global frame for the controller
      for(unsigned int i = 0; i < orig_global_plan.size(); ++i){
        robot_msgs::PoseStamped new_pose;
        tf_.transformPose(global_frame_, orig_global_plan[i], new_pose);
        global_plan.push_back(new_pose);
      }
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

    // Set current velocities from odometry
    robot_msgs::PoseDot global_vel;
    global_vel.vel.vx = base_odom_.vel.x;
    global_vel.vel.vy = base_odom_.vel.y;
    global_vel.ang_vel.vz = base_odom_.vel.th;


    local_plan.clear();

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
    if(global_plan.empty())
      return false;

    tf::Stamped<tf::Pose> goal_point;
    tf::PoseStampedMsgToTF(global_plan.back(), goal_point);
    //we assume the global goal is the last point in the global plan
    double goal_x = goal_point.getOrigin().getX();
    double goal_y = goal_point.getOrigin().getY();

    double uselessPitch, uselessRoll, yaw;
    goal_point.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);

    double goal_th = yaw;

    //assume at the beginning of our control cycle that we could have a new goal
    goal_reached_ = false;

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

      //we don't actually want to run the controller when we're just rotating to goal
      return true;
    }

    tc_->updatePlan(global_plan);

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
    if(path.cost_ < 0)
      return false;

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

    return true;
  }

  void TrajectoryPlannerROS::getLocalGoal(double& x, double& y){
    return tc_->getLocalGoal(x, y);
  }
};
