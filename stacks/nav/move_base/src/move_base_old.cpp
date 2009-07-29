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
#include <move_base/move_base_old.h>
#include <cstdlib>
#include <ctime>

using namespace costmap_2d;
using namespace robot_actions;

namespace move_base {

  MoveBase::MoveBase(std::string name, tf::TransformListener& tf) :
    Action<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped>(name), tf_(tf),
    tc_(NULL), planner_costmap_ros_(NULL), controller_costmap_ros_(NULL), 
    planner_(NULL){

    //get some parameters that will be global to the move base node
    std::string global_planner, local_planner;
    ros_node_.param("~base_global_planner", global_planner, std::string("NavfnROS"));
    ros_node_.param("~base_local_planner", local_planner, std::string("TrajectoryPlannerROS"));
    ros_node_.param("~global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    ros_node_.param("~global_costmap/global_frame", global_frame_, std::string("/map"));
    ros_node_.param("~controller_frequency", controller_frequency_, 20.0);
    ros_node_.param("~planner_patience", planner_patience_, 5.0);
    ros_node_.param("~controller_patience", controller_patience_, 15.0);

    //for comanding the base
    vel_pub_ = ros_node_.advertise<robot_msgs::PoseDot>("cmd_vel", 1);
    vis_pub_ = ros_node_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    ros_node_.param("~local_costmap/inscribed_radius", inscribed_radius_, 0.325);
    ros_node_.param("~local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
    ros_node_.param("~clearing_radius", clearing_radius_, circumscribed_radius_);
    ros_node_.param("~conservative_reset_dist", conservative_reset_dist_, 3.0);

    ros_node_.param("~shutdown_costmaps", shutdown_costmaps_, false);

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ros_ = new Costmap2DROS("global_costmap", tf_);

    //initialize the global planner
    try {
      planner_ = nav_robot_actions::BGPFactory::Instance().CreateObject(global_planner, global_planner, *planner_costmap_ros_);
    } catch (Loki::DefaultFactoryError<std::string, nav_robot_actions::BaseGlobalPlanner>::Exception)
    {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered?", global_planner.c_str());
      exit(0);
    }

    ROS_INFO("MAP SIZE: %d, %d", planner_costmap_ros_->cellSizeX(), planner_costmap_ros_->cellSizeY());

    //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    controller_costmap_ros_ = new Costmap2DROS("local_costmap", tf_);

    //create a local planner
    try {
      tc_ = nav_robot_actions::BLPFactory::Instance().CreateObject(local_planner, 
          local_planner, tf_, *controller_costmap_ros_);
    } catch (Loki::DefaultFactoryError<std::string, nav_robot_actions::BaseLocalPlanner>::Exception)
    {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered?", local_planner.c_str());
      exit(0);
    }

    //advertise a service for getting a plan
    make_plan_srv_ = ros_node_.advertiseService("~make_plan", &MoveBase::planService, this);

    //initially clear any unknown space around the robot
    planner_costmap_ros_->clearNonLethalWindow(circumscribed_radius_ * 2, circumscribed_radius_ * 2);
    controller_costmap_ros_->clearNonLethalWindow(circumscribed_radius_ * 2, circumscribed_radius_ * 2);

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }

    //initially, we'll need to make a plan
    state_ = PLANNING;

    //the initial clearing state will be to conservatively clear the costmaps
    clearing_state_ = CONSERVATIVE_RESET;

  }

  void MoveBase::clearCostmapWindows(double size_x, double size_y){
    tf::Stamped<tf::Pose> global_pose;

    //clear the planner's costmap
    planner_costmap_ros_->getRobotPose(global_pose);

    std::vector<geometry_msgs::Point> clear_poly;
    double x = global_pose.getOrigin().x();
    double y = global_pose.getOrigin().y();
    geometry_msgs::Point pt;

    pt.x = x - size_x / 2;
    pt.y = y - size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_x / 2;
    clear_poly.push_back(pt);

    planner_costmap_ros_->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

    //clear the controller's costmap
    controller_costmap_ros_->getRobotPose(global_pose);

    clear_poly.clear();
    x = global_pose.getOrigin().x();
    y = global_pose.getOrigin().y();

    pt.x = x - size_x / 2;
    pt.y = y - size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_x / 2;
    clear_poly.push_back(pt);

    controller_costmap_ros_->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
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

    tf::Stamped<tf::Pose> global_pose;
    if(!planner_costmap_ros_->getRobotPose(global_pose)){
      ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
      return false;
    }

    geometry_msgs::PoseStamped start;
    tf::poseStampedTFToMsg(global_pose, start);

    //update the copy of the costmap the planner uses
    clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);

    //if we have a tolerance on the goal point that is greater 
    //than the resolution of the map... compute the full potential function
    double resolution = planner_costmap_ros_->resolution();
    std::vector<geometry_msgs::PoseStamped> global_plan;
    geometry_msgs::PoseStamped p;
    p = req.goal;
    p.pose.position.y = req.goal.pose.position.y - req.tolerance; 
    bool found_legal = false;
    while(!found_legal && p.pose.position.y <= req.goal.pose.position.y + req.tolerance){
      p.pose.position.x = req.goal.pose.position.x - req.tolerance;
      while(!found_legal && p.pose.position.x <= req.goal.pose.position.x + req.tolerance){
        if(planner_->makePlan(start, p, global_plan)){
          if(!global_plan.empty()){
            global_plan.push_back(p);
            found_legal = true;
          }
          else
            ROS_DEBUG("Failed to find a  plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
        }
        p.pose.position.x += resolution*3.0;
      }
      p.pose.position.y += resolution*3.0;
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

  void MoveBase::publishGoal(const geometry_msgs::PoseStamped& goal){
    visualization_msgs::Marker marker;
    marker.header = goal.header;
    marker.ns = "move_base";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.pose = goal.pose;
    marker.scale.x = 0.5;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    vis_pub_.publish(marker);
  }

  bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    //make sure to set the plan to be empty initially
    plan.clear();

    //since this gets called on handle activate
    if(planner_costmap_ros_ == NULL)
      return false;

    //get the starting pose of the robot
    tf::Stamped<tf::Pose> global_pose;
    if(!planner_costmap_ros_->getRobotPose(global_pose))
      return false;

    geometry_msgs::PoseStamped start;
    tf::poseStampedTFToMsg(global_pose, start);

    //if the planner fails or returns a zero length plan, planning failed
    if(!planner_->makePlan(start, goal, plan) || plan.empty()){
      ROS_DEBUG("Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    //we'll also push the goal point onto the end of the plan to make sure orientation is taken into account
    geometry_msgs::PoseStamped goal_copy = goal;
    goal_copy.header.stamp = ros::Time::now();
    plan.push_back(goal_copy);

    return true;
  }

  void MoveBase::rotateRobot(){
    ros::Rate r(controller_frequency_);
    //we'll perform two 180 degree in-place rotations
    for(unsigned int i = 0; i < 2; ++i){
      ROS_DEBUG("180 rotation %d", i);
      double angle = M_PI; //rotate 180 degrees
      tf::Stamped<tf::Pose> rotate_goal = tf::Stamped<tf::Pose>(tf::Pose(tf::Quaternion(angle, 0.0, 0.0), tf::Point(0.0, 0.0, 0.0)), ros::Time(), robot_base_frame_);
      geometry_msgs::PoseStamped rotate_goal_msg;

      try{
        tf_.transformPose(global_frame_, rotate_goal, rotate_goal);
      }
      catch(tf::TransformException& ex){
        ROS_ERROR("This tf error should never happen: %s", ex.what());
        return;

      }

      poseStampedTFToMsg(rotate_goal, rotate_goal_msg);
      std::vector<geometry_msgs::PoseStamped> rotate_plan;
      rotate_plan.push_back(rotate_goal_msg);

      //pass the rotation goal to the controller
      if(!tc_->updatePlan(rotate_plan)){
        ROS_ERROR("Failed to pass global plan to the controller, aborting in place rotation attempt.");
        return;
      }

      robot_msgs::PoseDot cmd_vel;
      while(!isPreemptRequested() && ros_node_.ok() && !tc_->goalReached()){
        if(tc_->computeVelocityCommands(cmd_vel)){
          //make sure that we send the velocity command to the base
          vel_pub_.publish(cmd_vel);
          ROS_DEBUG("Velocity commands produced by controller: vx: %.2f, vy: %.2f, vth: %.2f", cmd_vel.vel.vx, cmd_vel.vel.vy, cmd_vel.ang_vel.vz);
        }
        else{
          //if we can't perform an in-place rotation then we'll just return 
          return;
        }
        //make sure to sleep in the meantime
        r.sleep();

      }
    }
  }

  void MoveBase::publishZeroVelocity(){
    robot_msgs::PoseDot cmd_vel;
    cmd_vel.vel.vx = 0.0;
    cmd_vel.vel.vy = 0.0;
    cmd_vel.ang_vel.vz = 0.0;
    vel_pub_.publish(cmd_vel);

  }

  robot_actions::ResultStatus MoveBase::execute(const geometry_msgs::PoseStamped& goal, geometry_msgs::PoseStamped& feedback){
    //if we shutdown our costmaps when we're deactivated... we need to start them back up now
    if(shutdown_costmaps_){
      planner_costmap_ros_->start();
      controller_costmap_ros_->start();
    }

    //on activation... we'll reset our costmaps
    clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);

    //publish the goal point to the visualizer
    publishGoal(goal);

    std::vector<geometry_msgs::PoseStamped> global_plan;
    robot_msgs::PoseDot cmd_vel;
    ros::Time last_valid_plan, last_valid_control;

    last_valid_control = ros::Time::now();
    last_valid_plan = ros::Time::now();

    costmap_2d::Rate r(controller_frequency_);

    ROS_DEBUG("move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);

    //we'll loop while the node is up and while the action is not pre-empted
    while(!isPreemptRequested() && ros_node_.ok()){
      //for timing that gives real time even in simulation
      struct timeval start, end;
      double start_t, end_t, t_diff;
      gettimeofday(&start, NULL);

      //update feedback to correspond to our curent position
      tf::Stamped<tf::Pose> global_pose;
      planner_costmap_ros_->getRobotPose(global_pose);
      tf::poseStampedTFToMsg(global_pose, feedback);

      //push the feedback out
      update(feedback);

      //check that the observation buffers for the costmap are current, we don't want to drive blind 
      if(!controller_costmap_ros_->isCurrent()){
        ROS_WARN("Sensor data is out of date, we're not going to allow commanding of the base for safety");
        publishZeroVelocity();
        r.sleep();
        continue;
      }

      //the move_base state machine, handles the control logic for navigation
      switch(state_){
        //if we are in a planning state, then we'll attempt to make a plan
        case PLANNING:
          ROS_DEBUG("In planning state");
          if(makePlan(goal, global_plan)){
            if(!tc_->updatePlan(global_plan)){
              //ABORT and SHUTDOWN COSTMAPS
              ROS_ERROR("Failed to pass global plan to the controller, aborting.");
              resetState();
              return robot_actions::ABORTED;
            }
            last_valid_plan = ros::Time::now();
            state_ = CONTROLLING;

            //make sure to reset clearing state since we were able to find a valid plan
            clearing_state_ = CONSERVATIVE_RESET;

            publishZeroVelocity();
          }
          else{
            ros::Time attempt_end = last_valid_plan + ros::Duration(planner_patience_);

            //check if we've tried to make a plan for over our time limit 
            if(ros::Time::now() > attempt_end){
              //we'll move into our obstacle clearing mode
              state_ = CLEARING;
              publishZeroVelocity();
            }
          }
          break;
            
        //if we're controlling, we'll attempt to find valid velocity commands
        case CONTROLLING:
          ROS_DEBUG("In controlling state");

          //check to see if we've reached our goal
          if(tc_->goalReached()){
            ROS_DEBUG("Goal reached!");
            resetState();
            return robot_actions::SUCCESS;
          } 

          if(tc_->computeVelocityCommands(cmd_vel)){
            last_valid_control = ros::Time::now();
            //make sure that we send the velocity command to the base
            vel_pub_.publish(cmd_vel);
          }
          else {
            ros::Time attempt_end = last_valid_control + ros::Duration(controller_patience_);

            //check if we've tried to find a valid control for longer than our time limit
            if(ros::Time::now() > attempt_end){
              ROS_ERROR("Aborting because of failure to find a valid control for %.2f seconds", controller_patience_);
              resetState();
              return robot_actions::ABORTED;
            } 

            //otherwise, if we can't find a valid control, we'll go back to planning
            last_valid_plan = ros::Time::now();
            state_ = PLANNING;
            publishZeroVelocity();
          }

          break;

        //we'll try to clear out space with the following actions
        case CLEARING:
          switch(clearing_state_){
            //first, we'll try resetting the costmaps conservatively to see if we find a plan
            case CONSERVATIVE_RESET:
              ROS_DEBUG("In conservative reset state");
              resetCostmaps(conservative_reset_dist_, conservative_reset_dist_);
              clearing_state_ = IN_PLACE_ROTATION;
              state_ = PLANNING;
              break;
            //next, we'll try an in-place rotation to try to clear out space
            case IN_PLACE_ROTATION:
              ROS_DEBUG("In in-place rotation state");
              rotateRobot();
              clearing_state_ = AGGRESSIVE_RESET;
              state_ = PLANNING;
              break;
            //finally, we'll try resetting the costmaps aggresively to clear out space
            case AGGRESSIVE_RESET:
              ROS_DEBUG("In aggressive reset state");
              resetCostmaps(circumscribed_radius_ * 2, circumscribed_radius_ * 2);
              clearing_state_ = ABORT;
              state_ = PLANNING;
              break;
            //if all of the above fail, we can't drive safely, so we'll abort
            case ABORT:
              ROS_ERROR("Aborting because a valid plan could not be found. Even after attempting to reset costmaps and rotating in place");
              resetState();
              return robot_actions::ABORTED;
              break;
            default:
              ROS_ERROR("This case should never be reached, something is wrong, aborting");
              resetState();
              return robot_actions::ABORTED;
              break;
          }
          break;
        default:
          ROS_ERROR("This case should never be reached, something is wrong, aborting");
          resetState();
          return robot_actions::ABORTED;
          break;
      }

      gettimeofday(&end, NULL);
      start_t = start.tv_sec + double(start.tv_usec) / 1e6;
      end_t = end.tv_sec + double(end.tv_usec) / 1e6;
      t_diff = end_t - start_t;
      ROS_DEBUG("Full control cycle time: %.9f\n", t_diff);

      //make sure to sleep for the remainder of our cycle time
      if(!r.sleep() && state_ == CONTROLLING)
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }

    //make sure to reset state and stop on pre-emption
    resetState();
    return robot_actions::PREEMPTED;

  } 

  void MoveBase::resetState(){
    state_ = PLANNING;
    clearing_state_ = CONSERVATIVE_RESET;
    publishZeroVelocity();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }
  }

  void MoveBase::resetCostmaps(double size_x, double size_y){
    planner_costmap_ros_->resetMapOutsideWindow(size_x, size_y);
    controller_costmap_ros_->resetMapOutsideWindow(size_x, size_y);
  }

};

int main(int argc, char** argv){
  ros::init(argc, argv, "move_base");
  tf::TransformListener tf(ros::Duration(10));
  ros::NodeHandle n;
  
  move_base::MoveBase move_base(n.getName(), tf);
  robot_actions::ActionRunner runner(20.0);
  runner.connect<geometry_msgs::PoseStamped, nav_robot_actions::MoveBaseState, geometry_msgs::PoseStamped>(move_base);
  runner.run();

  //ros::MultiThreadedSpinner s;
  ros::spin();
  
  return(0);

}
