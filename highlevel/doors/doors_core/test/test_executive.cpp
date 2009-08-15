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
 * $Id$
 *
 *********************************************************************/

/* Author: Wim Meeussen */


#include <boost/thread/thread.hpp>
#include <door_msgs/Door.h>
#include <ros/node.h>
#include <robot_actions/action_client.h>
#include <pr2_robot_actions/Pose2D.h>
#include <pr2_robot_actions/DoorActionState.h>
#include <robot_actions/NoArgumentsActionState.h>
#include <pr2_robot_actions/SwitchControllersState.h>
#include <nav_robot_actions/MoveBaseState.h>
#include <door_functions/door_functions.h>


using namespace ros;
using namespace std;
using namespace door_functions;




// -----------------------------------
//              MAIN
// -----------------------------------

int
  main (int argc, char **argv)
{
  ros::init(argc, argv);

  ros::Node node("test_executive");
  boost::thread* thread;

  ROS_INFO("Test executive started");

  door_msgs::Door prior_door;
  prior_door.frame_p1.x = 1.0;
  prior_door.frame_p1.y = -0.5;
  prior_door.frame_p2.x = 1.0;
  prior_door.frame_p2.y = 0.5;
  prior_door.door_p1.x = 1.0;
  prior_door.door_p1.y = -0.5;
  prior_door.door_p2.x = 1.0;
  prior_door.door_p2.y = 0.5;
  prior_door.travel_dir.x = 1.0;
  prior_door.travel_dir.y = 0.0;
  prior_door.travel_dir.z = 0.0;
  prior_door.rot_dir = door_msgs::Door::ROT_DIR_COUNTERCLOCKWISE;
  //prior_door.rot_dir = door_msgs::Door::ROT_DIR_CLOCKWISE;
  prior_door.hinge = door_msgs::Door::HINGE_P2;
  prior_door.header.frame_id = "base_footprint";
  /*
  prior_door.frame_p1.x = 13.720980644226074;
  prior_door.frame_p1.y = 21.296588897705078;
  prior_door.frame_p2.x = 13.784474372863770;
  prior_door.frame_p2.y = 22.195737838745117;
  prior_door.door_p1.x = 13.720980644226074;
  prior_door.door_p1.y = 21.296588897705078;
  prior_door.door_p2.x = 13.784474372863770;
  prior_door.door_p2.y = 22.195737838745117;
  prior_door.travel_dir.x = -1.936123440473645;
  prior_door.travel_dir.y = 3.251793805466352;
  prior_door.rot_dir = door_msgs::Door::ROT_DIR_COUNTERCLOCKWISE;
  prior_door.hinge = door_msgs::Door::HINGE_P1;
  prior_door.header.frame_id = "map";
  */    
  pr2_robot_actions::SwitchControllers switchlist;
  std_msgs::Empty empty;

  Duration timeout_short = Duration().fromSec(2.0);
  Duration timeout_medium = Duration().fromSec(10.0);
  Duration timeout_long = Duration().fromSec(40.0);

  ROS_INFO("Starting acion clients");
  robot_actions::ActionClient<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty> tuck_arm("safety_tuck_arms");
  robot_actions::ActionClient<pr2_robot_actions::SwitchControllers, pr2_robot_actions::SwitchControllersState,  std_msgs::Empty> switch_controllers("switch_controllers");
  robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> detect_door("detect_door");
  robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> detect_handle("detect_handle");
  robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> grasp_handle("grasp_handle");
  robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> touch_door("touch_door");
  robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> unlatch_handle("unlatch_handle");
  robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> open_door("open_door");
  robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> push_door("push_door");
  robot_actions::ActionClient<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty> release_handle("release_handle");
  robot_actions::ActionClient<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door> move_base_door("move_base_door");
  robot_actions::ActionClient<geometry_msgs::PoseStamped, nav_robot_actions::MoveBaseState, geometry_msgs::PoseStamped> move_base_local("move_base_local");

  door_msgs::Door tmp_door;
  door_msgs::Door backup_door;

  std::ostringstream os; os << prior_door;
  ROS_INFO("before %s", os.str().c_str());

  // tuck arm
  ROS_INFO("begining tuck arms");
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.start_controllers.push_back("r_arm_joint_trajectory_controller");
  if (switch_controllers.execute(switchlist, empty, timeout_medium) != robot_actions::SUCCESS) return -1;
  if (tuck_arm.execute(empty, empty, timeout_medium) != robot_actions::SUCCESS) return -1;
  ROS_INFO("done tuck arms");

  // detect door
  ROS_INFO("begining detect door");
  door_msgs::Door res_detect_door;
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.start_controllers.push_back("laser_tilt_controller");
  if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
  ROS_INFO("door detect");
  while (detect_door.execute(prior_door, tmp_door, timeout_long) != robot_actions::SUCCESS);
  res_detect_door = tmp_door;

  std::ostringstream os2; os2 << res_detect_door;
  ROS_INFO("detect door %s", os2.str().c_str());
  
  // detect handle if door is latched
  ROS_INFO("begining detect handle");
  door_msgs::Door res_detect_handle;
  bool open_by_pushing = false;
  if (res_detect_door.latch_state == door_msgs::Door::UNLATCHED)
    open_by_pushing = true;
  if (!open_by_pushing){
    ROS_INFO("Not opening by pushing - pointing head at handle and detecting");
    switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
    switchlist.start_controllers.push_back("head_controller");
    switchlist.start_controllers.push_back("head_pan_joint_position_controller");
    switchlist.start_controllers.push_back("head_tilt_joint_position_controller");
    if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
    while (detect_handle.execute(res_detect_door, tmp_door, timeout_long) != robot_actions::SUCCESS);
    res_detect_handle = tmp_door;
    
    std::ostringstream os3; os3 << res_detect_handle;
    ROS_INFO("detect handle %s", os3.str().c_str());
  }

  // approach door
  geometry_msgs::PoseStamped goal_msg;
  tf::poseStampedTFToMsg(getRobotPose(res_detect_door, -0.6), goal_msg);
  ROS_INFO("move to pose %f, %f, %f", goal_msg.pose.position.x, goal_msg.pose.position.y, goal_msg.pose.position.z);
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
  while (move_base_local.execute(goal_msg, goal_msg) != robot_actions::SUCCESS) {cout << "re-trying move base local" << endl;};
  ROS_INFO("door approach finished");

  // touch door
  if (open_by_pushing){
    ROS_INFO("Open by pushing");
    switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
    switchlist.stop_controllers.push_back("r_arm_joint_trajectory_controller");
    switchlist.start_controllers.push_back("r_gripper_effort_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_trajectory_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_pose_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_twist_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_wrench_controller");
    if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
    if (touch_door.execute(res_detect_door, tmp_door, timeout_long) != robot_actions::SUCCESS) return -1;
    ROS_INFO("Door touched");

    // push door in separate thread
    ROS_INFO("Open by pushing in seperate thread");
    switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
    switchlist.stop_controllers.push_back("r_arm_constraint_cartesian_trajectory_controller");
    if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
    thread = new boost::thread(boost::bind(&robot_actions::ActionClient<door_msgs::Door, 
					   pr2_robot_actions::DoorActionState, door_msgs::Door>::execute, 
					   &push_door, res_detect_door, tmp_door, timeout_long));
  }
  else{
    ROS_INFO("Open by grasping and unlatching");
    // grasp handle
    switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
    switchlist.stop_controllers.push_back("r_arm_joint_trajectory_controller");
    switchlist.start_controllers.push_back("r_gripper_effort_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_trajectory_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_pose_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_twist_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_wrench_controller");
    if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
    if (grasp_handle.execute(res_detect_handle, tmp_door, timeout_long) != robot_actions::SUCCESS) return -1;

    ROS_INFO("Unlatching");
    // unlatch handle
    switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
    switchlist.stop_controllers.push_back("r_arm_constraint_cartesian_trajectory_controller");
    switchlist.stop_controllers.push_back("r_arm_constraint_cartesian_pose_controller");
    switchlist.stop_controllers.push_back("r_arm_constraint_cartesian_twist_controller");
    switchlist.start_controllers.push_back("r_arm_cartesian_tff_controller");
    if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
    if (unlatch_handle.execute(res_detect_handle, tmp_door, timeout_long) != robot_actions::SUCCESS) return -1;

    ROS_INFO("Open goor in seprate thread");
    // open door in separate thread
    switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
    if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
    thread = new boost::thread(boost::bind(&robot_actions::ActionClient<door_msgs::Door, 
					   pr2_robot_actions::DoorActionState, door_msgs::Door>::execute, 
					   &open_door, res_detect_handle, tmp_door, timeout_long));
  }    

  bool move_thru_success = true;
  // move throught door
  pr2_robot_actions::Pose2D pose2d;
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;

  std::ostringstream os4; os4 << res_detect_handle;
  ROS_INFO("Moving through door with door message %s", os4.str().c_str());
  if (move_base_door.execute(res_detect_door, tmp_door) != robot_actions::SUCCESS) 
    {
      move_thru_success = false;
      backup_door = res_detect_door;
    };

  ROS_INFO("Preempt openning thread and join");
  // preempt open/push door
  if (open_by_pushing)
    push_door.preempt();
  else
    open_door.preempt();
  thread->join();
  delete thread;


  // release handle
  if (!open_by_pushing){
    ROS_INFO("Releasing handle");
    switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
    switchlist.stop_controllers.push_back("r_arm_cartesian_tff_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_trajectory_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_pose_controller");
    switchlist.start_controllers.push_back("r_arm_constraint_cartesian_twist_controller");
    if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
    if (release_handle.execute(empty, empty, timeout_long) != robot_actions::SUCCESS) return -1;
  }

  // tuck arm
  ROS_INFO("Tucking arm");
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.stop_controllers.push_back("r_arm_constraint_cartesian_trajectory_controller");
  switchlist.stop_controllers.push_back("r_arm_constraint_cartesian_pose_controller");
  switchlist.stop_controllers.push_back("r_arm_constraint_cartesian_twist_controller");
  switchlist.stop_controllers.push_back("r_arm_constraint_cartesian_wrench_controller");
  switchlist.start_controllers.push_back("r_arm_joint_trajectory_controller");
  if (switch_controllers.execute(switchlist, empty, timeout_medium) != robot_actions::SUCCESS) return -1;
  if (tuck_arm.execute(empty, empty, timeout_medium) != robot_actions::SUCCESS) return -1;

  if(!move_thru_success)
  {
    std::ostringstream os5; os5 << backup_door;
    ROS_INFO("Move through failed, using backup_door %s", os5.str().c_str());
    backup_door.travel_dir.x = -1.0;
    backup_door.travel_dir.y = 0.0;
    if (move_base_door.execute(backup_door, tmp_door) != robot_actions::SUCCESS)
      return -1;
  }

  // stop remaining controllers
  ROS_INFO("Stop controllers");
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.stop_controllers.push_back("laser_tilt_controller");
  switchlist.stop_controllers.push_back("head_controller");
  switchlist.stop_controllers.push_back("r_gripper_effort_controller");
  switchlist.stop_controllers.push_back("r_arm_joint_trajectory_controller");
  if (switch_controllers.execute(switchlist, empty, timeout_medium) != robot_actions::SUCCESS) return -1;

  ROS_INFO("Done");
  return (0);
}
