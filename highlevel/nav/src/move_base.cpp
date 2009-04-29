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
#include <nav/move_base.h>

using namespace base_local_planner;
using namespace costmap_2d;
using namespace navfn;
using namespace robot_actions;

namespace nav {
  MoveBase::MoveBase(ros::Node& ros_node, tf::TransformListener& tf) : 
    Action<robot_msgs::PoseStamped, robot_msgs::PoseStamped>(ros_node.getName()), ros_node_(ros_node), tf_(tf),
    run_planner_(true), tc_(NULL), planner_costmap_ros_(NULL), controller_costmap_ros_(NULL), 
    planner_(NULL), valid_plan_(false), new_plan_(false) {

    //get some parameters that will be global to the move base node
    ros_node_.param("~navfn/robot_base_frame", robot_base_frame_, std::string("base_link"));
    ros_node_.param("~controller_frequency", controller_frequency_, 20.0);

    //for comanding the base
    ros_node_.advertise<robot_msgs::PoseDot>("cmd_vel", 1);

    double inscribed_radius, circumscribed_radius;
    //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    ros_node_.param("~navfn/costmap/inscribed_radius", inscribed_radius, 0.325);
    ros_node_.param("~navfn/costmap/circumscribed_radius", circumscribed_radius, 0.46);

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ros_ = new Costmap2DROS(ros_node_, tf_, std::string("navfn"));
    planner_costmap_ros_->getCostmapCopy(planner_costmap_);

    //initialize the NavFn planner
    planner_ = new NavfnROS(ros_node_, tf_, planner_costmap_);
    ROS_INFO("MAP SIZE: %d, %d", planner_costmap_.cellSizeX(), planner_costmap_.cellSizeY());

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
    tc_ = new TrajectoryPlannerROS(ros_node_, tf_, controller_costmap_, footprint_, &planner_costmap_);

    //TODO:spawn planning thread here?
  }

  MoveBase::~MoveBase(){
    if(planner_ != NULL)
      delete planner_;

    if(tc_ != NULL)
      delete tc_;

    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;

    if(controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;
  }


  void MoveBase::makePlan(const robot_msgs::PoseStamped& goal){
    //since this gets called on handle activate
    if(planner_costmap_ros_ == NULL)
      return;

    //update the copy of the costmap the planner uses
    planner_costmap_ros_->getCostmapCopy(planner_costmap_);

    //since we have a controller that knows the full footprint of the robot... we may as well clear it
    tc_->clearRobotFootprint(planner_costmap_);

    std::vector<robot_msgs::PoseStamped> global_plan;
    bool valid_plan = planner_->makePlan(goal, global_plan);

    //we'll also push the goal point onto the end of the plan to make sure orientation is taken into account
    if(valid_plan)
      global_plan.push_back(goal);

    lock_.lock();
    //copy over the new global plan
    valid_plan_ = valid_plan;
    new_plan_ = true;
    global_plan_ = global_plan;
    lock_.unlock();
  }

  void MoveBase::getRobotPose(std::string frame, tf::Stamped<tf::Pose>& pose){
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = robot_base_frame_;
    robot_pose.stamp_ = ros::Time();

    try{
      tf_.transformPose(frame, robot_pose, pose);
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
    }
  }

  robot_actions::ResultStatus MoveBase::execute(const robot_msgs::PoseStamped& goal, robot_msgs::PoseStamped& feedback){
    //update the goal
    goal_ = goal;

    //first... make a plan to the goal
    makePlan(goal_);

    costmap_2d::Rate r(controller_frequency_);
    while(!isPreemptRequested()){
      struct timeval start, end;
      double start_t, end_t, t_diff;
      gettimeofday(&start, NULL);

      //update feedback to correspond to our current position
      tf::Stamped<tf::Pose> global_pose;
      getRobotPose(goal_.header.frame_id, global_pose);
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
      lock_.lock();
      if(valid_plan_){
        //if we have a new plan... we'll update the plan for the controller
        if(new_plan_){
          tc_->updatePlan(global_plan_);
          new_plan_ = false;
        }
        //get observations for the non-costmap controllers
        std::vector<Observation> observations;
        controller_costmap_ros_->getMarkingObservations(observations);
        valid_control = tc_->computeVelocityCommands(cmd_vel, observations);
      }
      else{
        //we don't have a valid plan... so we want to stop
        cmd_vel.vel.vx = 0.0;
        cmd_vel.vel.vy = 0.0;
        cmd_vel.ang_vel.vz = 0.0;
      }

      //give the base the velocity command
      ros_node_.publish("cmd_vel", cmd_vel);

      lock_.unlock();

      //check for success
      if(tc_->goalReached())
        return robot_actions::SUCCESS;


      //if we don't have a valid control... we need to re-plan explicitly
      if(!valid_control){
        makePlan(goal_);

        //if planning fails here... try to revert to the static map outside a given area
        if(!valid_plan_){
          resetCostmaps();
        }

      }

      gettimeofday(&end, NULL);
      start_t = start.tv_sec + double(start.tv_usec) / 1e6;
      end_t = end.tv_sec + double(end.tv_usec) / 1e6;
      t_diff = end_t - start_t;
      ROS_DEBUG("Full control cycle: %.9f Valid control: %d, Vel Cmd (%.2f, %.2f, %.2f)", t_diff, valid_control, cmd_vel.vel.vx, cmd_vel.vel.vy, cmd_vel.ang_vel.vz);

      //sleep the remainder of the cycle
      if(!r.sleep())
        ROS_WARN("Controll loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }
    return robot_actions::PREEMPTED;
  }

  void MoveBase::resetCostmaps(){
    planner_costmap_ros_->resetMapOutsideWindow(5.0, 5.0);
    controller_costmap_ros_->resetMapOutsideWindow(5.0, 5.0);
  }

};

int main(int argc, char** argv){
  ros::init(argc, argv);
  ros::Node ros_node("move_base");
  tf::TransformListener tf(ros_node, true, ros::Duration(10));
  
  nav::MoveBase move_base(ros_node, tf);
  robot_actions::ActionRunner runner(20.0);
  runner.connect<robot_msgs::PoseStamped, nav_robot_actions::MoveBaseState, robot_msgs::PoseStamped>(move_base);
  runner.run();

  ros_node.spin();

  return(0);

}
