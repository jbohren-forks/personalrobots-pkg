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
#include <nav/move_base_local.h>

using namespace base_local_planner;
using namespace costmap_2d;
using namespace robot_actions;
using namespace nav_robot_actions;

namespace nav {
  MoveBaseLocal::MoveBaseLocal(ros::Node& ros_node, tf::TransformListener& tf) : 
    Action<robot_msgs::PoseStamped, robot_msgs::PoseStamped>(ros_node.getName()), ros_node_(ros_node), tf_(tf),
    tc_(NULL), controller_costmap_ros_(NULL), action_name_("move_base_local"), laser_controller_("laser_tilt_controller") {

    //get the laser controller name
    ros_node_.param(action_name_ + "/laser_tilt_controller", laser_controller_, laser_controller_);
    if(laser_controller_ == "")
    {
      ROS_ERROR("%s: tilt_laser_controller param was not set.",action_name_.c_str());
      terminate();
      return;
    }
    req_laser_.command.profile = "linear";
    req_laser_.command.period = 2;
    req_laser_.command.amplitude = 0.65;
    req_laser_.command.offset = 0.25;

    //get some parameters that will be global to the move base node
    ros_node_.param("~base_local_planner/robot_base_frame", robot_base_frame_, std::string("base_link"));
    ros_node_.param("~controller_frequency", controller_frequency_, 20.0);

    //for comanding the base
    ros_node_.advertise<robot_msgs::PoseDot>("cmd_vel", 1);

    double inscribed_radius, circumscribed_radius;
    //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    ros_node_.param("~base_local_planner/costmap/inscribed_radius", inscribed_radius, 0.325);
    ros_node_.param("~base_local_planner/costmap/circumscribed_radius", circumscribed_radius, 0.46);

    //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    controller_costmap_ros_ = new Costmap2DROS(ros_node_, tf_, std::string("base_local_planner"));
    controller_costmap_ros_->getCostmapCopy(controller_costmap_);

    robot_msgs::Point pt;
    //create a square footprint
    pt.x = inscribed_radius + .01;
    pt.y = -1 * (inscribed_radius + .01);
    footprint_.push_back(pt);
    pt.x = -1 * (inscribed_radius + .01);
    pt.y = -1 * (inscribed_radius + .01);
    footprint_.push_back(pt);
    pt.x = -1 * (inscribed_radius + .01);
    pt.y = inscribed_radius + .01;
    footprint_.push_back(pt);
    pt.x = inscribed_radius + .01;
    pt.y = inscribed_radius + .01;
    footprint_.push_back(pt);

    //give the robot a nose
    pt.x = circumscribed_radius;
    pt.y = 0;
    footprint_.push_back(pt);

    //create a trajectory controller
    tc_ = new TrajectoryPlannerROS(ros_node_, tf_, controller_costmap_, footprint_, &controller_costmap_);
  }

  MoveBaseLocal::~MoveBaseLocal(){
    if(tc_ != NULL)
      delete tc_;

    if(controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;
  }

  void MoveBaseLocal::getRobotPose(std::string frame, tf::Stamped<tf::Pose>& pose){
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = robot_base_frame_;
    robot_pose.stamp_ = ros::Time();

    try{
      tf_.transformPose(frame, robot_pose, pose);
    }
    catch(tf::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return; // kind of pointless unless there's more code added below this try catch block
    }
    catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    }
  }

  robot_actions::ResultStatus MoveBaseLocal::execute(const robot_msgs::PoseStamped& goal, robot_msgs::PoseStamped& feedback){
    if (!ros::service::call(laser_controller_ + "/set_periodic_cmd", req_laser_, res_laser_))
    {
      ROS_ERROR("%s: Failed to start laser.", action_name_.c_str());
      return robot_actions::ABORTED;
    }    
    ros::Duration cycle_time = ros::Duration(1.0 / controller_frequency_);
    while(!isPreemptRequested()){
      struct timeval start, end;
      double start_t, end_t, t_diff;
      gettimeofday(&start, NULL);

      //get the start time of the loop
      ros::Time start_time = ros::Time::now();

      //update feedback to correspond to our current position
      tf::Stamped<tf::Pose> global_pose;
      getRobotPose(goal.header.frame_id, global_pose);
      tf::PoseStampedTFToMsg(global_pose, feedback);

      //push the feedback out
      update(feedback);

      //make sure to update the costmap we'll use for this cycle
      controller_costmap_ros_->getCostmapCopy(controller_costmap_);

      //check that the observation buffers for the costmap are current
      if(!controller_costmap_ros_->isCurrent()){
        ROS_WARN("Sensor data is out of date, we're not going to allow commanding of the base for safety");
        continue;
      }


      bool valid_control = false;
      robot_msgs::PoseDot cmd_vel;
      //pass plan to controller
      std::vector<robot_msgs::PoseStamped> global_plan;
      robot_msgs::PoseStamped robot_start;
      tf::PoseStampedTFToMsg(global_pose, robot_start);
      global_plan.push_back(robot_start);
      global_plan.push_back(goal);
      tc_->updatePlan(global_plan);
      //get observations for the non-costmap controllers
      std::vector<Observation> observations;
      controller_costmap_ros_->getMarkingObservations(observations);
      valid_control = tc_->computeVelocityCommands(cmd_vel, observations);

      //give the base the velocity command
      ros_node_.publish("cmd_vel", cmd_vel);

      //check for success
      if(tc_->goalReached())
        return robot_actions::SUCCESS;


      //if we don't have a valid control... we'll abort
      if(!valid_control){
        return robot_actions::ABORTED;
      }

      gettimeofday(&end, NULL);
      start_t = start.tv_sec + double(start.tv_usec) / 1e6;
      end_t = end.tv_sec + double(end.tv_usec) / 1e6;
      t_diff = end_t - start_t;
      ROS_DEBUG("Full control cycle: %.9f Valid control: %d, Vel Cmd (%.2f, %.2f, %.2f)", t_diff, valid_control, cmd_vel.vel.vx, cmd_vel.vel.vy, cmd_vel.ang_vel.vz);

      ros::Duration actual;
      //sleep the remainder of the cycle
      if(!sleepLeftover(start_time, cycle_time, actual))
        ROS_WARN("Controll loop missed its desired cycle time of %.4f... the loop actually took %.4f seconds", cycle_time.toSec(), actual.toSec());
    }
    return robot_actions::PREEMPTED;
  }

  bool MoveBaseLocal::sleepLeftover(ros::Time start, ros::Duration cycle_time, ros::Duration& actual){
    ros::Time expected_end = start + cycle_time;
    ros::Time actual_end = ros::Time::now();
    ///@todo: because durations don't handle subtraction properly right now
    ros::Duration sleep_time = ros::Duration((expected_end - actual_end).toSec()); 

    //set the actual amount of time the loop took
    actual = actual_end - start;

    if(sleep_time < ros::Duration(0.0)){
      return false;
    }

    sleep_time.sleep();
    return true;
  }

  void MoveBaseLocal::resetCostmaps(){
    controller_costmap_ros_->resetMapOutsideWindow(5.0, 5.0);
  }

};

int main(int argc, char** argv){
  ros::init(argc, argv);
  ros::Node ros_node("move_base_local");
  tf::TransformListener tf(ros_node, true, ros::Duration(10));
  
  nav::MoveBaseLocal move_base(ros_node, tf);
  robot_actions::ActionRunner runner(20.0);
  runner.connect<robot_msgs::PoseStamped, nav_robot_actions::MoveBaseState, robot_msgs::PoseStamped>(move_base);
  runner.run();

  ros_node.spin();

  return(0);

}
