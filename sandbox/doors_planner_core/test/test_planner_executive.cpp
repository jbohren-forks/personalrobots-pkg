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
 *   * Neither the name of Willow Garage nor the names of its
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
 * $Id: test_executive.cpp 15054 2009-05-07 21:21:18Z meeussen $
 *
 *********************************************************************/

/* Author: Sachin Chitta*/


#include <boost/thread/thread.hpp>
#include <boost/scoped_ptr.hpp>

#include <door_msgs/Door.h>
#include <door_msgs/DoorCmd.h>
#include <ros/node.h>
#include <ros/ros.h>
#include <robot_actions/action_client.h>
#include <geometry_msgs/Pose2D.h>
#include <pr2_robot_actions/DoorActionState.h>
#include <pr2_robot_actions/DoorCmdActionState.h>
#include <robot_actions/NoArgumentsActionState.h>
#include <pr2_robot_actions/SwitchControllersState.h>
#include <nav_robot_actions/MoveBaseState.h>
#include <door_functions/door_functions.h>

#include <algorithm>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_arm/MoveArmAction.h>

#include <pr2_msgs/SetPeriodicCmd.h>
#include <pr2_msgs/SetLaserTrajCmd.h>

using namespace ros;
using namespace std;
using namespace door_functions;


static const ros::Duration timeout_short = Duration().fromSec(2.0);
static const ros::Duration timeout_medium = Duration().fromSec(10.0);
static const ros::Duration timeout_long = Duration().fromSec(1000.0);

static const std::string robot_base_frame_ = "base_link";
static const std::string global_frame_ = "odom_combined";

using namespace move_base_msgs;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<move_arm::MoveArmAction> MoveArmClient;

class DoorPlannerExecutive
{
  public:

    tf::TransformListener tf_;

    pr2_robot_actions::SwitchControllers switchlist_;

    std_msgs::Empty empty;

    boost::scoped_ptr<robot_actions::ActionClient<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty> > tuck_arm_;
    boost::scoped_ptr<robot_actions::ActionClient<pr2_robot_actions::SwitchControllers, pr2_robot_actions::SwitchControllersState,  std_msgs::Empty> > switch_controllers_;
    boost::scoped_ptr<robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> > detect_door_;
    boost::scoped_ptr<robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> > detect_handle_;
    boost::scoped_ptr<robot_actions::ActionClient<door_msgs::DoorCmd, pr2_robot_actions::DoorCmdActionState, door_msgs::Door> > grasp_handle_door_;
    boost::scoped_ptr<robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> > touch_door_;
    boost::scoped_ptr<robot_actions::ActionClient<door_msgs::DoorCmd, pr2_robot_actions::DoorCmdActionState, door_msgs::Door> > unlatch_handle_;
    boost::scoped_ptr<robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> > open_door_;
    boost::scoped_ptr<robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> > push_door_;
    boost::scoped_ptr<robot_actions::ActionClient<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty> > release_handle_;
    boost::scoped_ptr<robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> > move_base_door_;
    boost::scoped_ptr<robot_actions::ActionClient<door_msgs::DoorCmd, pr2_robot_actions::DoorCmdActionState, door_msgs::Door> > sbpl_door_planner_;

    boost::scoped_ptr<MoveBaseClient> move_base_client_;
    boost::scoped_ptr<MoveArmClient> move_arm_client_;

    std::map<std::string, std::string> controller_targets_;
    std::vector<std::string> current_controllers_;

    void registerController(const std::string name, const std::string group)
    {
      controller_targets_[name] = group;
    }

    double dist(geometry_msgs::Point a, geometry_msgs::Point b)
    {
      return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + + (a.z-b.z)*(a.z-b.z));
    }

    geometry_msgs::Point pose2DToPoint(geometry_msgs::Pose2D p)
    {
      geometry_msgs::Point result;
      result.x = p.x;
      result.y = p.y;
      result.z = 0.0;
      return result;
    }

    geometry_msgs::Pose2D poseStampedTFToPose2D(tf::Stamped<tf::Pose> pose)
    {
      double useless_pitch, useless_roll, yaw;
      pose.getBasis().getEulerZYX(yaw, useless_pitch, useless_roll);

      geometry_msgs::Pose2D result;
      result.x = pose.getOrigin().x();
      result.y = pose.getOrigin().y();
      result.theta = yaw;
      return result;
    }

    tf::Stamped<tf::Pose> getGraspHandleApproachPoint(const door_msgs::Door& door, double dist, int side)
    {
      KDL::Vector x_axis(1,0,0);
      // get vector to center of frame
      KDL::Vector frame_1(door.frame_p1.x, door.frame_p1.y, door.frame_p1.z);
      KDL::Vector frame_2(door.frame_p2.x, door.frame_p2.y, door.frame_p2.z);
      KDL::Vector frame_center = (frame_2 + frame_1)/2.0;
    
      // get robot pose
      KDL::Vector frame_normal = getFrameNormal(door);
      KDL::Vector robot_pos = frame_center + (frame_normal * dist);
      if(side == door_msgs::DoorCmd::PULL)
      {
        robot_pos = frame_center + frame_normal * fabs(dist);
      }
      else
      {
        robot_pos = frame_center + frame_normal * -fabs(dist);
      }
      tf::Stamped<tf::Pose> robot_pose;
      robot_pose.frame_id_ = door.header.frame_id;
      robot_pose.stamp_ = door.header.stamp;
      robot_pose.setOrigin( tf::Vector3(robot_pos(0), robot_pos(1), robot_pos(2)));
      if(side == door_msgs::DoorCmd::PULL)
        robot_pose.setRotation( tf::Quaternion(getVectorAngle(-x_axis, frame_normal), 0, 0) );     
      else
        robot_pose.setRotation( tf::Quaternion(getVectorAngle(x_axis, frame_normal), 0, 0) );     

      return robot_pose;  
    }

    bool startControllers(const std::vector<std::string> &new_controllers, std::vector<std::string> &current_controllers)
    {
      std::vector<std::string> start_controllers, stop_controllers, tmp; 
      addControllers(new_controllers,current_controllers,start_controllers,stop_controllers);
      updateCurrentControllers(start_controllers,stop_controllers,current_controllers,tmp);

      switchlist_.start_controllers.clear();  switchlist_.stop_controllers.clear();
      switchlist_.start_controllers = start_controllers;
      switchlist_.stop_controllers = stop_controllers;
      if (switch_controllers_->execute(switchlist_, empty, timeout_medium) != robot_actions::SUCCESS) 
        return false;

      current_controllers.clear();
      current_controllers = tmp;
      return true;
    }

    void addControllers(const std::vector<std::string> &new_controllers, const std::vector<std::string> &current_controllers, std::vector<std::string> &start_controllers, std::vector<std::string> &stop_controllers)
    {
      std::vector<std::string> cc = current_controllers;
      std::vector<std::string> start_c;
      std::vector<std::string> stop_c;
      start_controllers.clear();
      stop_controllers.clear();

      for(unsigned int i=0; i< new_controllers.size(); i++)
      {
        start_c.push_back(new_controllers[i]);
        for(std::vector<std::string>::iterator it=cc.begin(); it != cc.end(); it++)
        {
          std::map<std::string,std::string>::iterator new_it,map_it;
          map_it = controller_targets_.find(new_controllers[i]);
          if (map_it == controller_targets_.end())
            continue;
          new_it = controller_targets_.find(*it);
          if (new_it == controller_targets_.end())
            continue;
          if(map_it->second == new_it->second)
          {
            stop_c.push_back(*it);
            cc.erase(it);
          }
        }
      }
      // Find the unique set of controllers that need to be started = unique(start_controllers) - current_controllers
      // Find the unique set of controllers that need to be stopped = unique(stop_controllers) - start_controllers 
      std::set_difference(start_c.begin(), start_c.end(), current_controllers.begin(), current_controllers.end(), start_controllers.begin()) ;
      std::set_difference(stop_c.begin(), stop_c.end(), start_controllers.begin(), start_controllers.end(), stop_controllers.begin());
    }

    void updateCurrentControllers(const std::vector<std::string> &start_controllers, const std::vector<std::string> &stop_controllers, const std::vector<std::string> &current_controllers, std::vector<std::string> &updated_controllers)
    {
      updated_controllers.clear();
      std::vector<std::string> tmp;
      std::set_difference(current_controllers.begin(), current_controllers.end(), stop_controllers.begin(), stop_controllers.end(),tmp.begin());
      std::set_union(tmp.begin(), tmp.end(), start_controllers.begin(), start_controllers.end(), updated_controllers.begin());
    }

    bool doorOpenDirection(door_msgs::Door door, geometry_msgs::PoseStamped robot_pose, int &door_open_direction)
    {
      geometry_msgs::PoseStamped robot_pose_door;
      tf::Stamped<tf::Pose> robot_pose_tf, robot_pose_door_tf;
      poseStampedMsgToTF(robot_pose,robot_pose_tf);
      //get the global pose of the robot in the door frame
      try
      {
        tf_.transformPose(door.header.frame_id, robot_pose_tf, robot_pose_door_tf);
      }
      catch(tf::LookupException& ex) 
      {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
        return false;
      }
      catch(tf::ConnectivityException& ex) 
      {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
        return false;
      }
      catch(tf::ExtrapolationException& ex) 
      {
        ROS_ERROR("Extrapolation Error: %s\n", ex.what());
        return false;
      }

      tf::poseStampedTFToMsg(robot_pose_door_tf,robot_pose_door);
      if(door.header.frame_id != robot_pose_door.header.frame_id)
      {
        ROS_ERROR("Frame id in door message: %s does not match frame id in robot pose: %s",door.header.frame_id.c_str(),robot_pose_door.header.frame_id.c_str());
        return false;
      } 
      geometry_msgs::Pose2D push_pose = poseStampedTFToPose2D(getGraspHandleApproachPoint(door,0.5,door_msgs::DoorCmd::PUSH));
      geometry_msgs::Pose2D pull_pose = poseStampedTFToPose2D(getGraspHandleApproachPoint(door,0.5,door_msgs::DoorCmd::PULL));
      if(dist(pose2DToPoint(push_pose),robot_pose_door.pose.position) < dist(pose2DToPoint(pull_pose),robot_pose_door.pose.position))
      {
        door_open_direction = door_msgs::DoorCmd::PUSH;
        ROS_INFO("Opening the door by pushing");
      }
      else
      {
        door_open_direction = door_msgs::DoorCmd::PULL;
        ROS_INFO("Opening the door by pulling");
      }
      return true;
    }

    bool init()
    {
      tuck_arm_.reset(new robot_actions::ActionClient<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty>("safety_tuck_arms"));
      switch_controllers_.reset(new robot_actions::ActionClient<pr2_robot_actions::SwitchControllers, pr2_robot_actions::SwitchControllersState,  std_msgs::Empty>("switch_controllers"));
      detect_door_.reset(new robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door>("detect_door"));
      detect_handle_.reset(new robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door>("detect_handle_no_camera"));
      grasp_handle_door_.reset(new robot_actions::ActionClient<door_msgs::DoorCmd, pr2_robot_actions::DoorCmdActionState, door_msgs::Door>("grasp_handle"));
      touch_door_.reset(new robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door>("touch_door"));
      unlatch_handle_.reset(new robot_actions::ActionClient<door_msgs::DoorCmd, pr2_robot_actions::DoorCmdActionState, door_msgs::Door>("unlatch_handle"));
      open_door_.reset(new robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door>("open_door"));
      push_door_.reset(new robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door>("push_door"));
      release_handle_.reset(new robot_actions::ActionClient<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty>("release_handle"));
      move_base_door_.reset(new robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door>("move_base_door"));
      sbpl_door_planner_.reset(new robot_actions::ActionClient<door_msgs::DoorCmd, pr2_robot_actions::DoorCmdActionState, door_msgs::Door>("sbpl_door_planner"));

      move_base_client_.reset(new MoveBaseClient("move_base_local"));
      move_arm_client_.reset(new MoveArmClient("move_arm_client"));

      current_controllers_.clear();

      registerController("r_arm_cartesian_tff_controller","right_arm");
      registerController("r_arm_joint_trajectory_controller","right_arm");
      registerController("r_arm_constraint_cartesian_pose_controller","right_arm");
      registerController("r_arm_constraint_cartesian_twist_controller","right_arm");
      registerController("r_arm_constraint_cartesian_wrench_controller","right_arm");
      registerController("r_arm_constraint_cartesian_trajectory_controller","right_arm");

      registerController("r_gripper_effort_controller","right_gripper");

      registerController("laser_tilt_controller","laser_tilt");

      registerController("head_controller","head");

      return true;      
    }

    bool detectDoor(const door_msgs::Door &door_prior, door_msgs::Door &door)
    {
      ROS_INFO("Detecting door");
      int num_attempts = 0;
      int max_num_attempts = 5;
      switchlist_.start_controllers.clear();  switchlist_.stop_controllers.clear();
      switchlist_.start_controllers.push_back("laser_tilt_controller");
      if (switch_controllers_->execute(switchlist_, empty, timeout_short) != robot_actions::SUCCESS) return false;
      while (detect_door_->execute(door_prior, door, timeout_long) != robot_actions::SUCCESS && num_attempts < max_num_attempts)
      {
        ROS_INFO("Attempt %d failed",num_attempts);
        num_attempts++;
      };
      if(num_attempts >= max_num_attempts)
      {
        ROS_INFO("Detect door failed");
        return false;
      }
      else
      {
        ROS_INFO("Frame id in returned message is %s",door.header.frame_id.c_str());
        cout << "Detected door is " << endl << door << endl;
        ROS_INFO("Frame id for detected door is %s",door.header.frame_id.c_str());
        return true;
      }
      return true;
    }

    bool tuckArm()
    {
      // tuck arm
      ROS_INFO("Tucking arm");
      switchlist_.start_controllers.clear();  switchlist_.stop_controllers.clear();
      switchlist_.start_controllers.push_back("r_arm_joint_trajectory_controller");
      int status = switch_controllers_->execute(switchlist_, empty, timeout_medium);
      if (status != robot_actions::SUCCESS) 
      {
        ROS_ERROR("Switch controllers failed");
        return false;
      }
      if (tuck_arm_->execute(empty, empty, timeout_medium) != robot_actions::SUCCESS) 
        return false;

      ROS_INFO("Tuck arm success");
      return true;
    }

    bool detectHandle(const door_msgs::Door &door_prior, door_msgs::Door &door, int &door_open_direction)
    {
      ROS_INFO("Detecting handle");
      door = door_prior;
      int num_attempts = 0;
      int max_num_attempts = 5;
      switchlist_.start_controllers.clear();  
      switchlist_.stop_controllers.clear();
/*      if(door_open_direction == door_msgs::DoorCmd::PULL)
      {
        switchlist_.start_controllers.push_back("laser_tilt_controller");
      }
*/
      switchlist_.start_controllers.push_back("head_controller");
      switchlist_.start_controllers.push_back("head_pan_joint_position_controller");
      switchlist_.start_controllers.push_back("head_tilt_joint_position_controller");
      if (switch_controllers_->execute(switchlist_, empty, timeout_short) != robot_actions::SUCCESS) 
        return false;
      while (detect_handle_->execute(door_prior, door, timeout_long) != robot_actions::SUCCESS && num_attempts < max_num_attempts)
      {
        num_attempts++;
      };
      if(num_attempts >= max_num_attempts)
      {
        ROS_INFO("Detect handle failed");
        return false;
      }
      else
      {
        cout << "Detected handle " << door << endl;
        return true;
      }
      return true;
    }

    bool getRobotPose(tf::Stamped<tf::Pose>& global_pose)
    {
      double transform_tolerance = 0.2;
      global_pose.setIdentity();
      tf::Stamped<tf::Pose> robot_pose;
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
      if (current_time.toSec() - global_pose.stamp_.toSec() > transform_tolerance) {
        ROS_WARN("DoorPlannerExecutive timeout: Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
                 current_time.toSec() ,global_pose.stamp_.toSec() ,transform_tolerance);
        return false;
      }
      return true;
    }

    bool setLaserParams(const pr2_msgs::LaserTrajCmd &nav_cmd)
    {
      ros::NodeHandle node_handle;
      pr2_msgs::SetLaserTrajCmd srv;
      ros::ServiceClient laser_client = node_handle.serviceClient<pr2_msgs::SetLaserTrajCmd>("/laser_tilt_controller/set_traj_cmd",true);
      srv.request.command = nav_cmd;
      if(laser_client.call(srv))
        return true;
      else
        return false;
    }

    bool setLaserParams(const std::string &profile, const double &period, const double &amplitude, const double &offset)
    {
      ros::NodeHandle node_handle;
      ros::ServiceClient laser_client = node_handle.serviceClient<pr2_msgs::SetPeriodicCmd>("/laser_tilt_controller/set_periodic_cmd",true);
      pr2_msgs::SetPeriodicCmd srv;
      srv.request.command.profile = profile;
      srv.request.command.period = period;
      srv.request.command.amplitude = amplitude;
      srv.request.command.offset = offset;
      if(laser_client.call(srv))
        return true;
      else
        return false;
    }
      
    bool approachDoor(const door_msgs::Door &door, int door_open_direction)
    {
      MoveBaseGoal goal;

      ROS_INFO("Approaching door");
      double door_approach_distance = 0.6;
      geometry_msgs::PoseStamped goal_msg;

      pr2_msgs::LaserTrajCmd nav_cmd;
      nav_cmd.profile = "blended_linear";
      nav_cmd.pos.resize(3);
      nav_cmd.time.resize(3);
      nav_cmd.max_rate = 5.0;
      nav_cmd.max_accel = 5.0;
      nav_cmd.pos[0] = 1.2;
      nav_cmd.pos[1] = -0.7;
      nav_cmd.pos[2] = 1.2;
      nav_cmd.time[0] = 0.0;
      nav_cmd.time[1] = 1.8;
      nav_cmd.time[2] = 2.025;

      if(!setLaserParams(nav_cmd))
      {
        ROS_ERROR("Could not set laser params");
        return false;
      }

      tf::poseStampedTFToMsg(getGraspHandleApproachPoint(door,door_approach_distance,door_open_direction), goal_msg);
      ROS_INFO("Moving to pose: %f, %f, %f in frame: %s",goal_msg.pose.position.x,goal_msg.pose.position.y,goal_msg.pose.position.z,goal_msg.header.frame_id.c_str());
      goal.target_pose = goal_msg;
      goal.target_pose.header.stamp = ros::Time();
      move_base_client_->sendGoal(goal);
      bool finished_before_timeout = move_base_client_->waitForGoalToFinish(timeout_medium);
      if (!finished_before_timeout) 
      {
        ROS_ERROR("Move base failed.");
        return false;
      }
      return true;
    }

    bool touchDoor(const door_msgs::Door &door)
    {
      ROS_INFO("Touch door");
      door_msgs::Door tmp_door;
      switchlist_.start_controllers.clear();  switchlist_.stop_controllers.clear();
      switchlist_.stop_controllers.push_back("r_arm_joint_trajectory_controller");
      switchlist_.start_controllers.push_back("r_gripper_effort_controller");
      switchlist_.start_controllers.push_back("r_arm_constraint_cartesian_trajectory_controller");
      switchlist_.start_controllers.push_back("r_arm_constraint_cartesian_pose_controller");
      switchlist_.start_controllers.push_back("r_arm_constraint_cartesian_twist_controller");
      switchlist_.start_controllers.push_back("r_arm_constraint_cartesian_wrench_controller");
      if (switch_controllers_->execute(switchlist_, empty, timeout_short) != robot_actions::SUCCESS) 
      {
        ROS_ERROR("Switch controllers failed");
        return false;
      }
      if (touch_door_->execute(door, tmp_door, timeout_long) != robot_actions::SUCCESS) 
      {
        ROS_ERROR("Touch door failed");
        return false;
      }
      return true;
    }

/*    bool refineDoorModel(const door_msgs::Door &door)
    {
      door_msgs::Door tmp_door;
      ROS_INFO("Refining door model");
      boost::thread* thread;
      thread = new boost::thread(boost::bind(&robot_actions::ActionClient<door_msgs::Door, 
                                             pr2_robot_actions::DoorActionState, door_msgs::Door>::execute,
                                             *refine_door_, door, tmp_door, timeout_long));
    }
*/
    bool pushThroughDoor(const door_msgs::Door &door)
    {
      ROS_INFO("Pushing through door");
      boost::thread* thread;
      bool done = false;
      door_msgs::Door tmp_door;
      // push door in separate thread
      switchlist_.start_controllers.clear();  switchlist_.stop_controllers.clear();
      switchlist_.stop_controllers.push_back("r_arm_constraint_cartesian_trajectory_controller");
      if (switch_controllers_->execute(switchlist_, empty, timeout_short) != robot_actions::SUCCESS)
      {
        ROS_ERROR("Switch controllers failed");
        return false;
      }
      thread = new boost::thread(boost::bind(&robot_actions::ActionClient<door_msgs::Door, 
                                             pr2_robot_actions::DoorActionState, door_msgs::Door>::execute,
                                             *push_door_, door, tmp_door, timeout_long));
      // move through door
      switchlist_.start_controllers.clear();  
      switchlist_.stop_controllers.clear();
      if (switch_controllers_->execute(switchlist_, empty, timeout_short) != robot_actions::SUCCESS)
      {
        ROS_ERROR("Switch controllers failed");
        return false;
      }

      if(move_base_door_->execute(door, tmp_door) != robot_actions::SUCCESS)
      {
        ROS_ERROR("Move base door failed");
        done = false;
      }
      else
      {
        done = true;
      }
      push_door_->preempt();

      thread->join();
      delete thread;
      return done;
    }

    bool graspHandle(const door_msgs::DoorCmd &door_cmd)
    {
      ROS_INFO("Grasping handle");
      door_msgs::Door tmp_door;
      // grasp handle
      switchlist_.start_controllers.clear();  
      switchlist_.stop_controllers.clear();
      switchlist_.stop_controllers.push_back("r_arm_joint_trajectory_controller");
      switchlist_.start_controllers.push_back("r_gripper_effort_controller");

      switchlist_.start_controllers.push_back("r_arm_constraint_cartesian_trajectory_controller");
      switchlist_.start_controllers.push_back("r_arm_constraint_cartesian_pose_controller");
      switchlist_.start_controllers.push_back("r_arm_constraint_cartesian_twist_controller");
      switchlist_.start_controllers.push_back("r_arm_constraint_cartesian_wrench_controller");
      if (switch_controllers_->execute(switchlist_, empty, timeout_short) != robot_actions::SUCCESS) 
        return false;
      if (grasp_handle_door_->execute(door_cmd, tmp_door, timeout_long) != robot_actions::SUCCESS) 
        return false;
      return true;
    }

    bool unlatchHandle(const door_msgs::Door &door, int door_open_direction)
    {
      door_msgs::DoorCmd door_cmd;
      door_cmd.door = door;
      door_cmd.side = door_open_direction;
      ROS_INFO("Unlatch handle");
      door_msgs::Door tmp_door;
      // unlatch handle
      switchlist_.start_controllers.clear();  
      switchlist_.stop_controllers.clear();
      switchlist_.stop_controllers.push_back("r_arm_constraint_cartesian_trajectory_controller");
      switchlist_.stop_controllers.push_back("r_arm_constraint_cartesian_pose_controller");
      switchlist_.stop_controllers.push_back("r_arm_constraint_cartesian_twist_controller");
      switchlist_.start_controllers.push_back("r_arm_cartesian_tff_controller");
      ROS_INFO("Switching controllers for unlatch handle");
      if (switch_controllers_->execute(switchlist_, empty, timeout_short) != robot_actions::SUCCESS) 
        return false;
      ROS_INFO("Switching controllers finished");
      if (unlatch_handle_->execute(door_cmd, tmp_door, timeout_long) != robot_actions::SUCCESS) 
        return false;
      ROS_INFO("Unlatch handle finished");
      return true;
    }

    bool openDoorUsingPlanner(const door_msgs::Door &door, int door_open_direction)
    {
      ROS_INFO("Open door using the planner");
      door_msgs::Door tmp_door;
      // open door in separate thread
      switchlist_.start_controllers.clear();  
      switchlist_.stop_controllers.clear();
      switchlist_.stop_controllers.push_back("r_arm_cartesian_tff_controller");
      switchlist_.stop_controllers.push_back("r_arm_constraint_cartesian_wrench_controller");
      switchlist_.start_controllers.push_back("r_arm_joint_trajectory_controller");
      if (switch_controllers_->execute(switchlist_, empty, timeout_short) != robot_actions::SUCCESS) 
        return false;

      ROS_INFO("Controllers for door planner ready");
      door_msgs::DoorCmd door_cmd;
      door_cmd.door = door;
      door_cmd.side = door_open_direction;
      // move through door
      if (sbpl_door_planner_->execute(door_cmd, tmp_door) != robot_actions::SUCCESS) 
      {
        ROS_INFO("Door planner failed");
        return false;
      }
      ROS_INFO("Door planner done");
      return true;
    }

    bool releaseHandle(const door_msgs::Door &door)
    {
      ROS_INFO("Release handle");
      door_msgs::Door tmp_door;
      switchlist_.start_controllers.clear();  
      switchlist_.stop_controllers.clear();
      switchlist_.stop_controllers.push_back("r_arm_joint_trajectory_controller");
      switchlist_.start_controllers.push_back("r_arm_constraint_cartesian_trajectory_controller");
      switchlist_.start_controllers.push_back("r_arm_constraint_cartesian_pose_controller");
      switchlist_.start_controllers.push_back("r_arm_constraint_cartesian_twist_controller");
      switchlist_.start_controllers.push_back("r_arm_constraint_cartesian_wrench_controller");

      if (switch_controllers_->execute(switchlist_, empty, timeout_short) != robot_actions::SUCCESS) 
        return false;
      if (release_handle_->execute(empty, empty, timeout_long) != robot_actions::SUCCESS) 
        return false;

      return true;
    }

    bool openDoor(door_msgs::Door &door)
    {
      ROS_INFO("Open door");
      tf::Stamped<tf::Pose> robot_pose;
      getRobotPose(robot_pose);
      geometry_msgs::PoseStamped robot_position;
      tf::poseStampedTFToMsg(robot_pose,robot_position);

      int door_open_direction;
      if(!doorOpenDirection(door, robot_position, door_open_direction))
        return false;
      door_msgs::Door tmp_door;
      if(door_open_direction == door_msgs::DoorCmd::PUSH || door_open_direction == door_msgs::DoorCmd::PULL)
      {
        if(!detectDoor(door,tmp_door))
        {
          ROS_INFO("Could not detect door");
          return false;
        }
        else
        {
          ROS_INFO("Detected door");
          door = tmp_door;
        }
      }

      bool open_by_pushing = (door.latch_state == door_msgs::Door::UNLATCHED);
      if(door_open_direction == door_msgs::DoorCmd::PULL)
      {
        open_by_pushing = false;
      }

      if (!open_by_pushing)
      {
        if(detectHandle(door,tmp_door,door_open_direction))
        {
          door = tmp_door;
        }
        else
        {
          return false;
        }
      }

      if(!approachDoor(door,door_open_direction))
      {
        ROS_INFO("Could not approach door using move base");
        return false;
      }

      if (open_by_pushing)  // touch door
      {
        if(!touchDoor(door))
        {
          return false;
        }
        if(!pushThroughDoor(door))
        {
          return false;
        }
      }
      else
      {
        door_msgs::DoorCmd door_cmd;
        door_cmd.door = door;
        door_cmd.side = door_open_direction;
        if(!graspHandle(door_cmd))
        {
          return false;
        }
        if(!unlatchHandle(door,door_open_direction))
        {
//          return false;
        }
        if(!openDoorUsingPlanner(door,door_open_direction))
        {
          return false;
        }
      }    
      if (!open_by_pushing)
      {
        if(!releaseHandle(door))
        {
          return false;
        }
      }
      return true;
    }

    bool graspHandleUsingPlanner(const door_msgs::Door &door, int side)
    {
      ROS_INFO("Grasp handle using planner");
      // Use move arm to grasp the other handle
      // Then use the door opening code to open the door
      switchlist_.start_controllers.push_back("r_arm_joint_trajectory_controller");
      switchlist_.stop_controllers.push_back("r_arm_constraint_cartesian_trajectory_controller");
      switchlist_.stop_controllers.push_back("r_arm_constraint_cartesian_pose_controller");
      switchlist_.stop_controllers.push_back("r_arm_constraint_cartesian_twist_controller");
      switchlist_.stop_controllers.push_back("r_arm_constraint_cartesian_wrench_controller");
      if (switch_controllers_->execute(switchlist_, empty, timeout_short) != robot_actions::SUCCESS) 
        return false;

      tf::Stamped<tf::Pose> handle_pose = getHandlePose(door,side);
      geometry_msgs::PoseStamped handle_msg;
      handle_pose.stamp_ = ros::Time::now();
      poseStampedTFToMsg(handle_pose, handle_msg);

      if(!setLaserParams("linear",10.0,0.75,0.25))
      {
        ROS_ERROR("Could not set laser periodic command for move arm");
        return false;
      }

      move_arm::MoveArmGoal move_arm_goal;
      move_arm_goal.goal_constraints.set_pose_constraint_size(1);

      move_arm_goal.goal_constraints.pose_constraint[0].pose = handle_msg;
      move_arm_goal.goal_constraints.pose_constraint[0].pose.header.stamp = ros::Time::now();
      move_arm_goal.goal_constraints.pose_constraint[0].pose.header.frame_id = "torso_lift_link";

      move_arm_goal.goal_constraints.pose_constraint[0].link_name = "r_wrist_roll_link";
      move_arm_goal.goal_constraints.pose_constraint[0].position_tolerance_above.x = 0.005;
      move_arm_goal.goal_constraints.pose_constraint[0].position_tolerance_below.x = 0.005;
      move_arm_goal.goal_constraints.pose_constraint[0].position_tolerance_above.y = 0.005;
      move_arm_goal.goal_constraints.pose_constraint[0].position_tolerance_below.y = 0.005;
      move_arm_goal.goal_constraints.pose_constraint[0].position_tolerance_above.z = 0.005;
      move_arm_goal.goal_constraints.pose_constraint[0].position_tolerance_below.z = 0.005;

      move_arm_goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.x = 0.01;
      move_arm_goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.x = 0.01;
      move_arm_goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.y = 0.01;
      move_arm_goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.y = 0.01;
      move_arm_goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.z = 0.01;
      move_arm_goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.z = 0.01;

      move_arm_goal.goal_constraints.pose_constraint[0].orientation_importance = 0.1;
      move_arm_goal.goal_constraints.pose_constraint[0].type = motion_planning_msgs::PoseConstraint::POSITION_X + motion_planning_msgs::PoseConstraint::POSITION_Y + motion_planning_msgs::PoseConstraint::POSITION_Z + motion_planning_msgs::PoseConstraint::ORIENTATION_R + motion_planning_msgs::PoseConstraint::ORIENTATION_P + motion_planning_msgs::PoseConstraint::ORIENTATION_Y;   
      move_arm_client_->sendGoal(move_arm_goal);
      return true;
    }
};

/* Door location in Gazebo 
 - frame (1.20233 -0.365404 0) -- (1.20183 0.65571 0)
 - door (1.20237 -0.365404 0) -- (1.20183 0.65571 0)
 - handle (1.1538 -0.0847961 0.804179)
 - travel_dir (1 0.000495378 0)
 - latch_state 2
 - hinge side 2
 - rot_dir 2
 - angle [deg] 0.00208026
*/

int main (int argc, char **argv)
{
  ros::init(argc, argv);	
  door_msgs::Door tmp_door;
  ros::Node node("test_planner");

  door_msgs::Door door;
  door.frame_p1.x = 1.20233;
  door.frame_p1.y = -0.365404;
  door.frame_p2.x = 1.20183;
  door.frame_p2.y = 0.65571;
  door.door_p1.x = 1.20233;
  door.door_p1.y = -0.365404;
  door.door_p2.x = 1.20233;
  door.door_p2.y = 0.65571;
  door.travel_dir.x = -1.0;
  door.travel_dir.y = 0.0;
  door.travel_dir.z = 0.0;
  door.rot_dir = door_msgs::Door::ROT_DIR_COUNTERCLOCKWISE;
//  door.rot_dir = door_msgs::Door::ROT_DIR_CLOCKWISE;
  door.hinge = door_msgs::Door::HINGE_P1;
  door.header.frame_id = "base_footprint";

  DoorPlannerExecutive dpe;
  dpe.init();
  std::cout << "before " << door << std::endl;
  ROS_INFO("Frame id %s",door.header.frame_id.c_str());
  if(!dpe.tuckArm())
  {
    return(-1);
  }
  if(!dpe.openDoor(door))
  {
    return(-1);
  }

  ROS_INFO("Door angle is now : %f",getDoorAngle(tmp_door));

  if(fabs(getDoorAngle(tmp_door)) < (M_PI/2.0-0.3))
  {
    if(!dpe.graspHandleUsingPlanner(tmp_door,door_msgs::DoorCmd::PUSH))
    {
      return(-1);
    }
    if(!dpe.openDoorUsingPlanner(door,door_msgs::DoorCmd::PUSH))
    {
      return(-1);
    }
  }
    
  if(!dpe.tuckArm())
  {
    return(-1);
  }

  return (0);
}
