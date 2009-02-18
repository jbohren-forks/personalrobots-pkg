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

#include <trajectory_rollout/trajectory_controller_ros.h>
#include <ros/console.h>
#include <sys/time.h>

using namespace std;
using namespace deprecated_msgs;
using namespace robot_msgs;
using namespace costmap_2d;
using namespace tf;
using namespace laser_scan;

namespace trajectory_rollout {
  TrajectoryControllerROS::TrajectoryControllerROS(ros::Node& ros_node, TransformListener& tf, 
      string global_frame,
      const costmap_2d::CostMapAccessor& ma, 
      std::vector<deprecated_msgs::Point2DFloat32> footprint_spec, double inscribed_radius, double circumscribed_radius) 
    : world_model_(NULL), tc_(NULL), base_scan_notifier_(NULL), tf_(tf), global_frame_(global_frame), point_grid_(NULL){
    double acc_lim_x, acc_lim_y, acc_lim_theta, sim_time, sim_granularity;
    int vx_samples, vtheta_samples;
    double pdist_scale, gdist_scale, occdist_scale, heading_lookahead, oscillation_reset_dist;
    bool holonomic_robot, dwa, simple_attractor;
    double max_vel_x, min_vel_x, max_vel_th, min_vel_th, min_in_place_vel_th;

    base_scan_notifier_ = new MessageNotifier<LaserScan>(&tf_, &ros_node,
        boost::bind(&TrajectoryControllerROS::baseScanCallback, this, _1),
        "base_scan", global_frame_, 50);

    tilt_scan_notifier_ = new MessageNotifier<LaserScan>(&tf_, &ros_node,
        boost::bind(&TrajectoryControllerROS::tiltScanCallback, this, _1),
        //"tilt_scan", global_frame_, 50);
        "base_scan", global_frame_, 50);
    
    ros_node.param("/trajectory_rollout/acc_lim_x", acc_lim_x, 2.5);
    ros_node.param("/trajectory_rollout/acc_lim_y", acc_lim_y, 2.5);
    ros_node.param("/trajectory_rollout/acc_lim_th", acc_lim_theta, 3.2);
    ros_node.param("/trajectory_rollout/sim_time", sim_time, 1.0);
    ros_node.param("/trajectory_rollout/sim_granularity", sim_granularity, 0.025);
    ros_node.param("/trajectory_rollout/vx_samples", vx_samples, 20);
    ros_node.param("/trajectory_rollout/vtheta_samples", vtheta_samples, 20);
    ros_node.param("/trajectory_rollout/path_distance_bias", pdist_scale, 0.6);
    ros_node.param("/trajectory_rollout/goal_distance_bias", gdist_scale, 0.8);
    ros_node.param("/trajectory_rollout/occdist_scale", occdist_scale, 0.2);
    ros_node.param("/trajectory_rollout/heading_lookahead", heading_lookahead, 0.325);
    ros_node.param("/trajectory_rollout/oscillation_reset_dist", oscillation_reset_dist, 0.05);
    ros_node.param("/trajectory_rollout/holonomic_robot", holonomic_robot, true);
    ros_node.param("/trajectory_rollout/max_vel_x", max_vel_x, 0.5);
    ros_node.param("/trajectory_rollout/min_vel_x", min_vel_x, 0.1);
    ros_node.param("/trajectory_rollout/max_vel_th", max_vel_th, 1.0);
    ros_node.param("/trajectory_rollout/min_vel_th", min_vel_th, -1.0);
    ros_node.param("/trajectory_rollout/min_in_place_vel_th", min_in_place_vel_th, 0.4);
    ros_node.param("/trajectory_rollout/freespace_model", freespace_model_, false);
    ros_node.param("/trajectory_rollout/dwa", dwa, true);
    ros_node.param("/trajectory_rollout/simple_attractor", simple_attractor, false);

    //parameters for using the freespace controller
    double min_pt_separation, max_obstacle_height, grid_resolution;
    ros_node.param("/trajectory_rollout/point_grid/max_sensor_range", max_sensor_range_, 2.0);
    ros_node.param("/trajectory_rollout/point_grid/min_pt_separation", min_pt_separation, 0.01);
    ros_node.param("/trajectory_rollout/point_grid/max_obstacle_height", max_obstacle_height, 2.0);
    ros_node.param("/trajectory_rollout/point_grid/grid_resolution", grid_resolution, 0.2);

    if(freespace_model_){
      double origin_x, origin_y;
      ma.getOriginInWorldCoordinates(origin_x, origin_y);
      Point2DFloat32 origin;
      origin.x = origin_x;
      origin.y = origin_y;
      unsigned int cmap_width, cmap_height;
      ma.getCostmapDimensions(cmap_width, cmap_height);
      point_grid_ = new PointGrid(cmap_width * ma.getResolution(), cmap_height * ma.getResolution(), grid_resolution, 
          origin, max_obstacle_height, max_sensor_range_, min_pt_separation);
      world_model_ = point_grid_;
      ROS_INFO("Freespace Origin: (%.4f, %.4f), Width: %.4f, Height: %.4f\n", origin.x, origin.y, cmap_width * ma.getResolution(), cmap_height * ma.getResolution());
      /*For Debugging
      ros_node.advertise<PointCloud>("point_grid", 1);
      */
    }
    else{
      world_model_ = new CostmapModel(ma); 
      ROS_INFO("Costmap\n");
    }

    tc_ = new TrajectoryController(*world_model_, ma, footprint_spec, inscribed_radius, circumscribed_radius,
        acc_lim_x, acc_lim_y, acc_lim_theta, sim_time, sim_granularity, vx_samples, vtheta_samples, pdist_scale,
        gdist_scale, occdist_scale, heading_lookahead, oscillation_reset_dist, holonomic_robot,
        max_vel_x, min_vel_x, max_vel_th, min_vel_th, min_in_place_vel_th, dwa, simple_attractor);
  }

  void TrajectoryControllerROS::baseScanCallback(const MessageNotifier<LaserScan>::MessagePtr& message){
    //project the laser into a point cloud
    PointCloud base_cloud;
    base_cloud.header = message->header;
    //we want all values... even those out of range
    projector_.projectLaser(*message, base_cloud, -1.0, true);
    Stamped<btVector3> global_origin;

    lock_.lock();
    base_scan_.angle_min = message->angle_min;
    base_scan_.angle_max = message->angle_max;
    base_scan_.angle_increment = message->angle_increment;

    //we know the transform is available from the laser frame to the global frame 
    try{
      //transform the origin for the sensor
      Stamped<btVector3> local_origin(btVector3(0, 0, 0), base_cloud.header.stamp, base_cloud.header.frame_id);
      tf_.transformPoint(global_frame_, local_origin, global_origin);
      base_scan_.origin.x = global_origin.getX();
      base_scan_.origin.y = global_origin.getY();
      base_scan_.origin.z = global_origin.getZ();

      //transform the point cloud
      tf_.transformPointCloud(global_frame_, base_cloud, base_scan_.cloud);
    }
    catch(tf::TransformException& ex){
      ROS_ERROR("TF Exception that should never happen %s", ex.what());
      return;
    }
    lock_.unlock();
  }

  void TrajectoryControllerROS::tiltScanCallback(const MessageNotifier<LaserScan>::MessagePtr& message){
    //project the laser into a point cloud
    PointCloud tilt_cloud;
    tilt_cloud.header = message->header;
    //we want all values... even those out of range
    projector_.projectLaser(*message, tilt_cloud, -1.0, true);
    Stamped<btVector3> global_origin;
    PointCloud global_cloud;

    //we know the transform is available from the laser frame to the global frame 
    try{
      //transform the origin for the sensor
      Stamped<btVector3> local_origin(btVector3(0, 0, 0), tilt_cloud.header.stamp, tilt_cloud.header.frame_id);
      tf_.transformPoint(global_frame_, local_origin, global_origin);


      //transform the point cloud
      tf_.transformPointCloud(global_frame_, tilt_cloud, global_cloud);
    }
    catch(tf::TransformException& ex){
      ROS_ERROR("TF Exception that should never happen %s", ex.what());
      return;
    }

    lock_.lock();
    //now we want to create a polygonal approximation of the scan
    risk_poly_.clear();
    //for now... let's just make a triangle with the origin and the extreme points
    Point2DFloat32 pt;
    pt.x = global_origin.getX();
    pt.y = global_origin.getY();
    risk_poly_.push_back(pt);

    double x = global_cloud.pts[global_cloud.pts.size() - 1].x - global_origin.getX();
    double y = global_cloud.pts[global_cloud.pts.size() - 1].y - global_origin.getY();
    double length = sqrt(x*x + y*y);
    x = max_sensor_range_ * (x / length); 
    y = max_sensor_range_ * (y / length); 
    pt.x = x + global_origin.getX();
    pt.y = y + global_origin.getY();
    risk_poly_.push_back(pt);

    x = global_cloud.pts[3 * global_cloud.pts.size() / 4].x - global_origin.getX();
    y = global_cloud.pts[3 * global_cloud.pts.size() / 4].y - global_origin.getY();
    length = sqrt(x*x + y*y);
    x = max_sensor_range_ * (x / length); 
    y = max_sensor_range_ * (y / length); 
    pt.x = x + global_origin.getX();
    pt.y = y + global_origin.getY();
    risk_poly_.push_back(pt);

    x = global_cloud.pts[global_cloud.pts.size() / 2].x - global_origin.getX();
    y = global_cloud.pts[global_cloud.pts.size() / 2].y - global_origin.getY();
    length = sqrt(x*x + y*y);
    x = max_sensor_range_ * (x / length); 
    y = max_sensor_range_ * (y / length); 
    pt.x = x + global_origin.getX();
    pt.y = y + global_origin.getY();
    risk_poly_.push_back(pt);

    x = global_cloud.pts[global_cloud.pts.size() / 4].x - global_origin.getX();
    y = global_cloud.pts[global_cloud.pts.size() / 4].y - global_origin.getY();
    length = sqrt(x*x + y*y);
    x = max_sensor_range_ * (x / length); 
    y = max_sensor_range_ * (y / length); 
    pt.x = x + global_origin.getX();
    pt.y = y + global_origin.getY();
    risk_poly_.push_back(pt);

    x = global_cloud.pts[0].x - global_origin.getX();
    y = global_cloud.pts[0].y - global_origin.getY();
    length = sqrt(x*x + y*y);
    x = max_sensor_range_ * (x / length); 
    y = max_sensor_range_ * (y / length); 
    pt.x = x + global_origin.getX();
    pt.y = y + global_origin.getY();
    risk_poly_.push_back(pt);

    lock_.unlock();

  }

  TrajectoryControllerROS::~TrajectoryControllerROS(){
    if(tc_ != NULL)
      delete tc_;
    if(world_model_ != NULL)
      delete world_model_;
    if(base_scan_notifier_ != NULL)
      delete base_scan_notifier_;
  }

  vector<Point2DFloat32> TrajectoryControllerROS::drawFootprint(double x_i, double y_i, double theta_i){
    return tc_->drawFootprint(x_i, y_i, theta_i);
  }

  bool TrajectoryControllerROS::computeVelocityCommands(const std::list<deprecated_msgs::Pose2DFloat32>& global_plan, 
      const tf::Stamped<tf::Pose>& global_pose, 
      const robot_msgs::PoseDot& global_vel, 
      robot_msgs::PoseDot& cmd_vel,
      std::list<deprecated_msgs::Pose2DFloat32>& localPlan,
      const std::vector<costmap_2d::Observation>& observations){


    localPlan.clear();

    tf::Stamped<tf::Pose> drive_cmds;
    drive_cmds.frame_id_ = "base_link";

    tf::Stamped<tf::Pose> robot_vel;
    btQuaternion qt(global_vel.ang_vel.vz, 0, 0);
    robot_vel.setData(btTransform(qt, btVector3(global_vel.vel.vx, global_vel.vel.vy, 0)));
    robot_vel.frame_id_ = "base_link";
    robot_vel.stamp_ = ros::Time();

    //do we need to resize our map?
    //double origin_x, origin_y;
    //ma_.getOriginInWorldCoordinates(origin_x, origin_y);
    //map_.sizeCheck(ma_.getWidth(), ma_.getHeight(), origin_x, origin_y);

    // Temporary Transformation till api below changes
    std::vector<deprecated_msgs::Point2DFloat32> copiedGlobalPlan;
    for(std::list<deprecated_msgs::Pose2DFloat32>::const_iterator it = global_plan.begin(); it != global_plan.end(); ++it){
      deprecated_msgs::Point2DFloat32 p;
      p.x = it->x;
      p.y = it->y;
      copiedGlobalPlan.push_back(p);
    }

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */

    tc_->updatePlan(copiedGlobalPlan);

    lock_.lock();
    //compute what trajectory to drive along
    Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds, observations, base_scan_, risk_poly_);
    lock_.unlock();

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
    double uselessPitch, uselessRoll, yaw;
    drive_cmds.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);

    cmd_vel.ang_vel.vz = yaw;

    //if we cannot move... tell someone
    if(path.cost_ < 0)
      return false;

    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i){
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      deprecated_msgs::Pose2DFloat32 p;
      p.x = p_x; 
      p.y = p_y;
      p.th = p_th;
      localPlan.push_back(p);
    }

    /* For Debugging
    if(freespace_model_ && point_grid_ != NULL){
      PointCloud grid_cloud;
      grid_cloud.header.frame_id = global_frame_;
      point_grid_->getPoints(grid_cloud);
      ros::Node::instance()->publish("point_grid", grid_cloud);
    }
    */

    return true;
  }

  void TrajectoryControllerROS::getLocalGoal(double& x, double& y){
    return tc_->getLocalGoal(x, y);
  }
};
