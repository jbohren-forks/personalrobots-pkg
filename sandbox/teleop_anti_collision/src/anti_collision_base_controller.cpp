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
* Author: Sachin Chitta
*********************************************************************/

#include <teleop_anti_collision/anti_collision_base_controller.h>

using namespace std;
using namespace robot_msgs;
using namespace costmap_2d;
using namespace base_local_planner;

namespace anti_collision_base_controller
{
  AntiCollisionBaseController::AntiCollisionBaseController()
  {   
    //create the ros wrapper for the controller's costmap... and initialize a pointer we'll use with the underlying map
    costmap_ros_ = new costmap_2d::Costmap2DROS("", tf_);

    //we'll get the parameters for the robot radius from the costmap we're associated with
    inscribed_radius_ = costmap_ros_->inscribedRadius();
    circumscribed_radius_ = costmap_ros_->circumscribedRadius();
    inflation_radius_ = costmap_ros_->inflationRadius();
    global_frame_ = costmap_ros_->globalFrame();
    robot_base_frame_ = costmap_ros_->baseFrame();
    footprint_spec_ = costmap_ros_->robotFootprint();

    ros_node_.param("~controller_frequency", controller_frequency_, 10.0);

    ros_node_.param("~acc_lim_x", acc_lim_x_, 2.5);
    ros_node_.param("~acc_lim_y", acc_lim_y_, 2.5);
    ros_node_.param("~acc_lim_th", acc_lim_theta_, 3.2);
    ros_node_.param("~sim_time", sim_time_, 1.0);
    ros_node_.param("~sim_granularity", sim_granularity_, 0.025);
    ros_node_.param("~vx_samples", vx_samples_, 3);
    ros_node_.param("~vtheta_samples", vtheta_samples_, 20);
   
    ros_node_.param("~max_vel_x", max_vel_x_, 0.5);
    ros_node_.param("~min_vel_x", min_vel_x_, 0.1);
    ros_node_.param("~max_vel_th", max_vel_th_, 1.0);
    ros_node_.param("~min_vel_th", min_vel_th_, -1.0);
    ros_node_.param("~min_in_place_vel_th", min_in_place_vel_th_, 0.4);

    //initialize the copy of the costmap the controller will use
    costmap_ros_->clearNonLethalWindow(circumscribed_radius_ * 2, circumscribed_radius_ * 2);
    costmap_ros_->getCostmapCopy(costmap_);

    string odom_topic, base_cmd_topic, joy_listen_topic, world_model_type;
    ros_node_.param("~odom_topic", odom_topic, string("odom"));
    ros_node_.param("~joy_listen_topic", joy_listen_topic, string("joy_cmd_vel"));
    ros_node_.param("~base_cmd_topic", base_cmd_topic, string("cmd_vel"));
    ros_node_.param("~world_model", world_model_type, string("costmap"));
    ros_node_.param("~timeout", timeout_, 0.2);

    ROS_ASSERT_MSG(world_model_type == "costmap", "At this time, only costmap world models are supported by this controller");
    world_model_ = new CostmapModel(costmap_); 

    joy_sub_ = ros_node_.subscribe(joy_listen_topic, 1, &AntiCollisionBaseController::joyCallBack, this);
    odom_sub_ = ros_node_.subscribe(odom_topic, 1, &AntiCollisionBaseController::odomCallback, this);
    base_cmd_pub_ = ros_node_.advertise<robot_msgs::PoseDot>(base_cmd_topic,1);

    last_cmd_received_ = ros::Time();
  }

  AntiCollisionBaseController::~AntiCollisionBaseController()
  {
    delete costmap_ros_;
  }

  //create and score a trajectory given the current pose of the robot and selected velocities
  bool AntiCollisionBaseController::generateTrajectory(double x, double y, double theta, 
						       double vx, double vy, double vtheta, 
						       double vx_samp, double vy_samp, double vtheta_samp, 
                                                       double acc_x, double acc_y, double acc_theta, 
						       base_local_planner::Trajectory& traj)
  {
    double x_i = x;
    double y_i = y;
    double theta_i = theta;

    double vx_i, vy_i, vtheta_i;

    vx_i = vx;
    vy_i = vy;
    vtheta_i = vtheta;

    //compute the magnitude of the velocities
    double vmag = sqrt(vx_samp * vx_samp + vy_samp * vy_samp);

    //compute the number of steps we must take along this trajectory to be "safe"
    int num_steps = int(max((vmag * sim_time_) / sim_granularity_, abs(vtheta_samp) / sim_granularity_) + 0.5);

    double dt = sim_time_ / num_steps;
    double time = 0.0;

    //create a potential trajectory
    traj.resetPoints();
    traj.xv_ = vx_samp; 
    traj.yv_ = vy_samp; 
    traj.thetav_ = vtheta_samp;
    traj.cost_ = -1.0;

    if(num_steps == 0)
      return false;

    for(int i = 0; i < num_steps; ++i)
    {
      //get map coordinates of a point
      unsigned int cell_x, cell_y;

      //we don't want a path that goes off the known map
      if(!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)){
        traj.cost_ = -1.0;
        return false;
      }

      //check the point on the trajectory for legality
      double footprint_cost = footprintCost(x_i, y_i, theta_i);

      //if the footprint hits an obstacle this trajectory is invalid
      if(footprint_cost < 0){
        traj.cost_ = -1.0;
        return false;
      }

      time += dt;

      //the point is legal... add it to the trajectory
      traj.addPoint(x_i, y_i, theta_i);

      //calculate velocities
      vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
      vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
      vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

      //calculate positions
      x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
      y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
      theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

    }
    double cost = -1.0;
    traj.cost_ = cost;
    return true;
  }


  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double AntiCollisionBaseController::footprintCost(double x_i, double y_i, double theta_i){
    //if we have no footprint... do nothing
    if(footprint_spec_.size() < 3)
      return -1.0;

    //build the oriented footprint
    double cos_th = cos(theta_i);
    double sin_th = sin(theta_i);
    vector<Point> oriented_footprint;
    for(unsigned int i = 0; i < footprint_spec_.size(); ++i){
      Point new_pt;
      new_pt.x = x_i + (footprint_spec_[i].x * cos_th - footprint_spec_[i].y * sin_th);
      new_pt.y = y_i + (footprint_spec_[i].x * sin_th + footprint_spec_[i].y * cos_th);
      oriented_footprint.push_back(new_pt);
    }

    Point robot_position;
    robot_position.x = x_i;
    robot_position.y = y_i;

    //check if the footprint is legal
    double footprint_cost = world_model_->footprintCost(robot_position, oriented_footprint, inscribed_radius_, circumscribed_radius_);
    return footprint_cost;
  }


  //Scale the desired trajectory until you find one that does the right thing
  void AntiCollisionBaseController::computeSafeVelocity(double x, double y, double theta, double vx_current, double vy_current, double vtheta_current, double vx_desired, double vy_desired, double vtheta_desired, double &vx_result, double &vy_result, double &vtheta_result)
  { 
    bool not_done = true;
    double scale = 1.0;
    double dScale = 0.1;
    double vx_tmp, vy_tmp, vtheta_tmp;

    Trajectory traj;

    while(not_done)
    {
      vx_tmp = vx_desired * scale;
      vy_tmp = vy_desired * scale;
      vtheta_tmp = vtheta_desired * scale;
      if(generateTrajectory(x,y,theta,vx_current,vy_current,vtheta_current,vx_tmp,vy_tmp,vtheta_tmp,acc_lim_x_,acc_lim_y_,acc_lim_theta_,traj))
      {
        not_done = false;
      }
      scale -= dScale;
      if(scale < 0)
      {
        not_done = false;
      }
    }
    vx_result = vx_tmp;
    vy_result = vy_tmp;
    vtheta_result = vtheta_tmp;
  }

  void AntiCollisionBaseController::joyCallBack(const robot_msgs::PoseDotConstPtr& msg)
  {
    last_cmd_received_ = ros::Time::now();
    vel_desired_.lock();
    vel_desired_ = *msg;
    vel_desired_.unlock();
  }

  void AntiCollisionBaseController::odomCallback(const deprecated_msgs::RobotBase2DOdomConstPtr& msg){
    base_odom_.lock();
    try
    {
      tf::Stamped<btVector3> v_in(btVector3(msg->vel.x, msg->vel.y, 0), ros::Time(), msg->header.frame_id), v_out;
      tf_.transformVector(robot_base_frame_, ros::Time(), v_in, msg->header.frame_id, v_out);
      base_odom_.vel.vx = v_in.x();
      base_odom_.vel.vy = v_in.y();
      base_odom_.ang_vel.vz = msg->vel.th;
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


  bool AntiCollisionBaseController::getRobotPose(double &x, double &y, double &theta)
  {
    tf::Stamped<tf::Pose> global_pose;
    if(!costmap_ros_->getRobotPose(global_pose))
      return false;

    double uselessPitch, uselessRoll, yaw;
    global_pose.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);

    x = global_pose.getOrigin().x();
    y = global_pose.getOrigin().y();
    theta = yaw;
    return true;
  }

  void AntiCollisionBaseController::spin()
  {
    ros::Rate r(controller_frequency_);
    while(ros_node_.ok())
    {
      ros::spinOnce();
      double vx_desired, vy_desired, vt_desired;
      vel_desired_.lock();
      vx_desired = vel_desired_.vel.vx;
      vy_desired = vel_desired_.vel.vy;
      vt_desired = vel_desired_.ang_vel.vz;
      vel_desired_.unlock();

      double x,y,theta;
      if(!getRobotPose(x,y,theta))
      {
        ROS_ERROR("Robot pose not returned by tf");
        continue;
      }

      double vx_current,vy_current,vt_current;
      base_odom_.lock();
      vx_current = base_odom_.vel.vx;
      vy_current = base_odom_.vel.vy;
      vt_current = base_odom_.ang_vel.vz;
      base_odom_.unlock();

      //we also want to clear the robot footprint from the costmap we're using
      costmap_ros_->clearRobotFootprint();
      //reinitialize the copy of the costmap the controller will use
      costmap_ros_->getCostmapCopy(costmap_);

      double vx_result, vy_result, vt_result;
      computeSafeVelocity(x,y,theta,vx_current,vy_current,vt_current,vx_desired,vy_desired,vt_desired,vx_result,vy_result,vt_result);

      if(ros::Time::now() - last_cmd_received_ > ros::Duration(timeout_))
      {
        ROS_WARN("Last command was received too far back, setting current velocity to 0 for safety");
        vx_result = 0.0;
        vy_result = 0.0;
        vt_result = 0.0;
      }

      PoseDot cmd;
      cmd.vel.vx = vx_result;
      cmd.vel.vy = vy_result;
      cmd.ang_vel.vz = vt_result;
      base_cmd_pub_.publish(cmd);

      r.sleep();
    }
    return;
  }
}

using namespace anti_collision_base_controller;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "anti_collision_base_controller");
  ros::spinOnce();
  AntiCollisionBaseController anti_collision;
  anti_collision.spin();
  return(0);
}

/*
int main(int argc, char** argv){
  ros::init(argc, argv);
  ros::Node ros_node("anti_collision_base_controller");
  AntiCollisionBaseController anti_collision;
  anti_collision.spin();
  return(0);
}
*/