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
    planner_(NULL), valid_plan_(false), new_plan_(false), attempted_rotation_(false), attempted_costmap_reset_(false),
    done_half_rotation_(false), done_full_rotation_(false) {

    //get some parameters that will be global to the move base node
    ros_node_.param("~navfn/robot_base_frame", robot_base_frame_, std::string("base_link"));
    ros_node_.param("~controller_frequency", controller_frequency_, 20.0);
    ros_node_.param("~planner_patience", planner_patience_, 10.0);
    ros_node_.param("~controller_patience", controller_patience_, 10.0);

    //for comanding the base
    ros_node_.advertise<robot_msgs::PoseDot>("cmd_vel", 1);

    //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    ros_node_.param("~navfn/costmap/inscribed_radius", inscribed_radius_, 0.325);
    ros_node_.param("~navfn/costmap/circumscribed_radius", circumscribed_radius_, 0.46);

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
    double padding;
    ros_node_.param("~footprint_padding", padding, 0.01);
    //create a square footprint
    pt.x = inscribed_radius_ + padding;
    pt.y = -1 * (inscribed_radius_ + padding);
    footprint_.push_back(pt);
    pt.x = -1 * (inscribed_radius_ + padding);
    pt.y = -1 * (inscribed_radius_ + padding);
    footprint_.push_back(pt);
    pt.x = -1 * (inscribed_radius_ + padding);
    pt.y = inscribed_radius_ + padding;
    footprint_.push_back(pt);
    pt.x = inscribed_radius_ + padding;
    pt.y = inscribed_radius_ + padding;
    footprint_.push_back(pt);

    //give the robot a nose
    pt.x = circumscribed_radius_;
    pt.y = 0;
    footprint_.push_back(pt);

    //create a trajectory controller
    tc_ = new TrajectoryPlannerROS(ros_node_, tf_, controller_costmap_, footprint_, &planner_costmap_);

    //advertise a service for getting a plan
    ros_node_.advertiseService("~make_plan", &MoveBase::planService, this);

    //initially clear any unknown space around the robot
    planner_costmap_ros_->clearNonLethalWindow(circumscribed_radius_ * 2, circumscribed_radius_ * 2);
    controller_costmap_ros_->clearNonLethalWindow(circumscribed_radius_ * 2, circumscribed_radius_ * 2);

    //TODO:spawn planning thread here?
  }

  bool MoveBase::planService(nav_srvs::Plan::Request &req, nav_srvs::Plan::Response &resp){
    if(isActive()){
      ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
      return false;
    }

    //make sure we have a costmap for our planner
    if(planner_costmap_ros_ == NULL){
      ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }

    //update the copy of the costmap the planner uses
    planner_costmap_ros_->getCostmapCopy(planner_costmap_);

    //since we have a controller that knows the full footprint of the robot... we may as well clear it
    //tc_->clearRobotFootprint(planner_costmap_); //now done in sensors

    //if we have a tolerance on the goal point that is greater 
    //than the resolution of the map... compute the full potential function
    double resolution = planner_costmap_ros_->resolution();
    std::vector<robot_msgs::PoseStamped> global_plan;
    if(req.tolerance > resolution){
      planner_->computePotential(req.goal.pose.position);
      robot_msgs::PoseStamped p;
      p = req.goal;
      p.pose.position.x = req.goal.pose.position.x - req.tolerance;
      p.pose.position.y = req.goal.pose.position.y - req.tolerance; 
      bool found_legal = false;
      while(!found_legal && p.pose.position.y < req.goal.pose.position.y + req.tolerance){
        while(!found_legal && p.pose.position.x < req.goal.pose.position.x + req.tolerance){
          if(planner_->makePlan(p, global_plan)){
            global_plan.push_back(p);
            found_legal = true;
          }
          p.pose.position.x += resolution;
        }
        p.pose.position.y += resolution;
      }
    }

    //copy the plan into a message to send out
    resp.plan.set_poses_size(global_plan.size());
    for(unsigned int i = 0; i < global_plan.size(); ++i){
      resp.plan.poses[i] = global_plan[i];
    }

    return true;
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
    //tc_->clearRobotFootprint(planner_costmap_); //now done in sensors

    std::vector<robot_msgs::PoseStamped> global_plan;
    bool valid_plan = planner_->makePlan(goal, global_plan);

    //we'll also push the goal point onto the end of the plan to make sure orientation is taken into account
    if(valid_plan){
      robot_msgs::PoseStamped goal_copy = goal;
      goal_copy.header.stamp = ros::Time::now();
      global_plan.push_back(goal_copy);

      //reset our flags for attempts to help create a valid plan
      attempted_rotation_ = false;
      attempted_costmap_reset_ = false;
      new_plan_ = true;
    }

    lock_.lock();
    //copy over the new global plan
    valid_plan_ = valid_plan;
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
    while(!isPreemptRequested() && ros_node_.ok()){
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

      robot_msgs::PoseDot cmd_vel;

      //check that the observation buffers for the costmap are current
      if(!controller_costmap_ros_->isCurrent()){
        ROS_WARN("Sensor data is out of date, we're not going to allow commanding of the base for safety");
        cmd_vel.vel.vx = 0.0;
        cmd_vel.vel.vy = 0.0;
        cmd_vel.ang_vel.vz = 0.0;
        //give the base the velocity command
        ros_node_.publish("cmd_vel", cmd_vel);
        r.sleep();
        continue;
      }


      bool valid_control = false;
      //pass plan to controller
      
      if(valid_plan_){
        //if we have a new plan... we'll update the plan for the controller
        if(new_plan_){
          new_plan_ = false;
          if(!tc_->updatePlan(global_plan_)){
            resetState();
            ROS_WARN("move_base aborted because it failed to pass the plan from the planner to the controller");
            return robot_actions::ABORTED;
          }
        }

        //get observations for the non-costmap controllers
        std::vector<Observation> observations;
        controller_costmap_ros_->getMarkingObservations(observations);
        valid_control = tc_->computeVelocityCommands(cmd_vel, observations);

        if(valid_control)
          last_valid_control_ = ros::Time::now();

        //check for success
        if(tc_->goalReached()){
          if(attempted_rotation_){
            valid_control = false;
            if(done_half_rotation_){
              done_full_rotation_ = true;
            }
            else{
              done_half_rotation_ = true;
            }
          }
          else
            return robot_actions::SUCCESS;
        }

        //if we can't rotate to clear out space... just say we've done them and try to reset to the static map
        if(!valid_control && attempted_rotation_){
          done_full_rotation_ = true;
          done_half_rotation_ = true;
        }

      }
      else{
        //we don't have a valid plan... so we want to stop
        cmd_vel.vel.vx = 0.0;
        cmd_vel.vel.vy = 0.0;
        cmd_vel.ang_vel.vz = 0.0;
      }

      //give the base the velocity command
      ros_node_.publish("cmd_vel", cmd_vel);

      //if we don't have a valid control... we need to re-plan explicitly
      if(!valid_control){
        ros::Duration patience = ros::Duration(controller_patience_);

        //if we have a valid plan, but can't find a valid control for a certain time... abort
        if(last_valid_control_ + patience < ros::Time::now()){
          resetState();
          ROS_WARN("move_base aborting because the controller could not find valid velocity commands for over %.4f seconds", patience.toSec());
          return robot_actions::ABORTED;
        }

        //try to make a plan
        if(done_half_rotation_ && !done_full_rotation_ || !tryPlan(goal_)){
          //if we've tried to reset our map and to rotate in place, to no avail, we'll abort the goal
          if(attempted_costmap_reset_ && done_full_rotation_){
            resetState();
            ROS_WARN("move_base aborting because the planner could not find a valid plan, even after reseting the map and attempting in place rotation");
            return robot_actions::ABORTED;
          }

          if(done_full_rotation_){
            resetCostmaps(circumscribed_radius_ * 2, circumscribed_radius_ * 2);
            attempted_rotation_ = false;
            done_half_rotation_ = false;
            done_full_rotation_ = false;
            attempted_costmap_reset_ = true;
          }
          else{
            //clear things in the static map that are really far away
            resetCostmaps(3.0, 3.0);
            //if planning fails... we'll try rotating in place to clear things out
            double angle = M_PI; //rotate 180 degrees
            tf::Stamped<tf::Pose> rotate_goal = tf::Stamped<tf::Pose>(tf::Pose(tf::Quaternion(angle, 0.0, 0.0), tf::Point(0.0, 0.0, 0.0)), ros::Time::now(), robot_base_frame_);
            robot_msgs::PoseStamped rotate_goal_msg;
            PoseStampedTFToMsg(rotate_goal, rotate_goal_msg);
            global_plan_.clear();
            global_plan_.push_back(rotate_goal_msg);
            valid_plan_ = true;
            new_plan_ = true;
            attempted_rotation_ = true;
          }
        }

        r.sleep();
        continue;
      }

      gettimeofday(&end, NULL);
      start_t = start.tv_sec + double(start.tv_usec) / 1e6;
      end_t = end.tv_sec + double(end.tv_usec) / 1e6;
      t_diff = end_t - start_t;
      ROS_DEBUG("Full control cycle: %.9f Valid control: %d, Vel Cmd (%.2f, %.2f, %.2f)", t_diff, valid_control, cmd_vel.vel.vx, cmd_vel.vel.vy, cmd_vel.ang_vel.vz);

      //sleep the remainder of the cycle
      if(!r.sleep())
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }
    return robot_actions::PREEMPTED;
  }

  void MoveBase::resetState(){
    attempted_rotation_ = false;
    done_half_rotation_ = false;
    done_full_rotation_ = false;
    attempted_costmap_reset_ = false;
  }

  bool MoveBase::tryPlan(robot_msgs::PoseStamped goal){
    ros::Duration patience = ros::Duration(planner_patience_);
    ros::Time attempt_end = ros::Time::now() + patience;
    costmap_2d::Rate r(controller_frequency_);
    while(ros::Time::now() < attempt_end && !isPreemptRequested() && ros_node_.ok()){
      makePlan(goal);

      //check if we've got a valid plan
      if(valid_plan_)
        return true;

      //for now... we'll publish zero velocity
      robot_msgs::PoseDot cmd_vel;

      last_valid_control_ = ros::Time::now();
      cmd_vel.vel.vx = 0.0;
      cmd_vel.vel.vy = 0.0;
      cmd_vel.ang_vel.vz = 0.0;
      //give the base the velocity command
      ros_node_.publish("cmd_vel", cmd_vel);

      r.sleep();
    }

    //if we still don't have a valid plan... then our planning attempt has failed
    return false;
  }

  void MoveBase::resetCostmaps(double size_x, double size_y){
    planner_costmap_ros_->resetMapOutsideWindow(size_x, size_y);
    controller_costmap_ros_->resetMapOutsideWindow(size_x, size_y);
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

  //ros::MultiThreadedSpinner s;
  //ros::spin(s);
  
  ros_node.spin();

  return(0);

}
