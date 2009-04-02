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
 *********************************************************************/

#include <ros/node.h>
#include <robot_msgs/Door.h>
#include <robot_msgs/TaskFrameFormalism.h>
#include <robot_actions/Pose2D.h>
#include <robot_actions/MoveBaseState.h>
#include <robot_srvs/SwitchController.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <door_handle_detector/DoorDetector.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <robot_srvs/MoveToPose.h>
#include <kdl/frames.hpp>
#include <sbpl_arm_executive/pr2_arm_node.h>

using namespace tf;
using namespace KDL;
using namespace ros;
using namespace std;
using namespace pr2_arm_node;

class OpenDoorExecutiveTest : public PR2ArmNode
{
private:
  tf::TransformListener tf_; 
  string fixed_frame_, robot_frame_;
  Duration pause_, polling_;
  std_msgs::String joy_msg_;

  enum {INITIALIZED, WAITING, DETECTING, GRASPING, OPENDOOR, SUCCESS, FAILED};
  int state_;
  bool planner_running_, planner_finished_, joy_command_;

  robot_msgs::Door my_door_;
  robot_msgs::TaskFrameFormalism tff_msg_;
  robot_actions::MoveBaseState planner_state_;

  door_handle_detector::DoorDetector::Request  req_doordetect;
  door_handle_detector::DoorDetector::Response res_doordetect;

  robot_srvs::MoveToPose::Request  req_moveto;
  robot_srvs::MoveToPose::Response res_moveto;

  robot_srvs::SwitchController::Request req_switch;
  robot_srvs::SwitchController::Response res_switch;

public:
  OpenDoorExecutiveTest(std::string node_name):
    PR2ArmNode(node_name, "right_arm", "right_gripper"),
    tf_(*this),
    state_(INITIALIZED)
  {
    // initialize my door
    double tmp; int tmp2;
    param("~/door_frame_p1_x", tmp, 1.5); my_door_.frame_p1.x = tmp;
    param("~/door_frame_p1_y", tmp, -0.5); my_door_.frame_p1.y = tmp;
    param("~/door_frame_p2_x", tmp, 1.5); my_door_.frame_p2.x = tmp;
    param("~/door_frame_p2_y", tmp, 0.5); my_door_.frame_p2.y = tmp;
    param("~/door_hinge" , tmp2, -1); my_door_.hinge = tmp2;
    param("~/door_rot_dir" , tmp2, -1); my_door_.rot_dir = tmp2;
    my_door_.header.frame_id = "base_footprint";

    advertise<robot_msgs::TaskFrameFormalism>("cartesian_tff_right/command",1);
    advertise<std_msgs::Float64>("gripper_effort/set_command",1);
    advertise<robot_actions::Pose2D>("/move_base_node/activate", 10);
    subscribe("/move_base_node/feedback", planner_state_,  &OpenDoorExecutiveTest::plannerCallback, 1);
    subscribe("/joy_annotator/annotation_msg", joy_msg_, &OpenDoorExecutiveTest::joyCallback,1);

    // frames
    fixed_frame_ = "odom_combined";
    robot_frame_ = "base_link";

    pause_ = Duration().fromSec(4.0);
    polling_ = Duration().fromSec(0.01);

    // start arm trajectory controller
    cout << "turn on moveto controller..." << flush;
    req_switch.stop_controllers.clear(); 
    req_switch.start_controllers.clear();     
    req_switch.start_controllers.push_back("cartesian_trajectory_right");  
    ros::service::call("switch_controller", req_switch, res_switch);
    if (!res_switch.ok)
      cout << "failed" << endl;
    else
      cout << "successful" << endl;
  }
  
  
  ~OpenDoorExecutiveTest()
  {
    unsubscribe("state");
    unsubscribe("annotation_msg");

    cout << "stop moveto and tff controllers..." << flush;
    req_switch.start_controllers.clear();     
    req_switch.stop_controllers.clear();      
    req_switch.stop_controllers.push_back("cartesian_tff_right");
    req_switch.stop_controllers.push_back("cartesian_trajectory_right");
    ros::service::call("switch_controller", req_switch, res_switch);
    if (!res_switch.ok)
      cout << "failed" << endl;
    else
      cout << "successful" << endl;
  }
  

  // -------------------------
  bool tuck_arm()
  // -------------------------
  {
    pause_.sleep();

    robot_msgs::PoseStamped init_pose;
    init_pose.header.frame_id = robot_frame_;
    init_pose.pose.position.x = 0.4;
    init_pose.pose.position.y = 0.0;
    init_pose.pose.position.z = 0.6;
    init_pose.pose.orientation.x = 0;
    init_pose.pose.orientation.y = 0;
    init_pose.pose.orientation.z = 0;
    init_pose.pose.orientation.w = 1;	  
    if (!moveTo(init_pose))
      return false;

    return true;
  }

  

  // -------------------------
  bool detectDoor(const robot_msgs::Door& door_estimate,  robot_msgs::Door& door_detection)
  // -------------------------
  {
    // start detection
    req_doordetect.door = door_estimate;
    if (ros::service::call("door_handle_detector", req_doordetect, res_doordetect)){
      door_detection = res_doordetect.door;
      return true;
    }
    else
      return false;
  }


  // -------------------------
  bool graspDoor(const robot_msgs::Door& door)
  // -------------------------
  {
    // get orientation
    Vector normal = getNormalOnDoor(door);
    normal = transformVectorToFrame(door.header.frame_id, fixed_frame_, normal, door.header.stamp);
    Vector x_axis(1,0,0);
    double dot      = normal(0) * x_axis(0) + normal(1) * x_axis(1);
    double perp_dot = normal(1) * x_axis(0) - normal(0) * x_axis(1);
    double z_angle = atan2(perp_dot, dot);
    cout << "z_angle in " << fixed_frame_ << " = " << z_angle << endl;

    // get robot position
    Vector robot_pos((door.door_p1.x + door.door_p2.x)/2.0, 
		     (door.door_p1.y + door.door_p2.y)/2.0,
		     (door.door_p1.z + door.door_p2.z)/2.0);
    robot_pos = transformPointToFrame(door.header.frame_id, fixed_frame_, robot_pos, door.header.stamp);
    robot_pos = robot_pos - (normal * 0.7);

    cout << "get door in " << door.header.frame_id << " with center " << robot_pos(0) << " " << robot_pos(1) << " " << robot_pos(2) << endl;

    // get gripper position
    robot_msgs::PoseStamped gripper_pose_msg;
    Stamped<Pose> gripper_pose;
    gripper_pose.frame_id_ = fixed_frame_;
    Vector point(door.handle.x, door.handle.y, door.handle.z);
    point = transformPointToFrame(door.header.frame_id, fixed_frame_, point, door.header.stamp);
    gripper_pose.setOrigin( Vector3(point[0], point[1], point[2]) );
    gripper_pose.setRotation( Quaternion(z_angle, 0, M_PI/2.0) ); 
    PoseStampedTFToMsg(gripper_pose, gripper_pose_msg);

    // move the robot in front of the door
    robot_actions::Pose2D robot_pose_msg;
    robot_pose_msg.header.frame_id = fixed_frame_;
    robot_pose_msg.x = robot_pos(0);
    robot_pose_msg.y = robot_pos(1);
    robot_pose_msg.th = z_angle;
    planner_finished_ = false;
    publish("/move_base_node/activate", robot_pose_msg);
    cout << "moving in front of door...  " << flush;
    while (!planner_finished_)
      polling_.sleep();
    cout << "arrived in front of door" << endl;

    // open the gripper
    std_msgs::Float64 gripper_msg;
    gripper_msg.data = 2.0;
    publish("gripper_effort/set_command", gripper_msg);

    // move gripper in front of door
    Vector offset = normal * -0.15;
    gripper_pose_msg.pose.position.x = gripper_pose_msg.pose.position.x + offset[0];
    gripper_pose_msg.pose.position.y = gripper_pose_msg.pose.position.y + offset[1];
    gripper_pose_msg.pose.position.z = gripper_pose_msg.pose.position.z + offset[2];
    moveTo(gripper_pose_msg);
    
    // move gripper over door handle
    gripper_pose_msg.pose.position.x = gripper_pose_msg.pose.position.x - 1.5*offset[0];
    gripper_pose_msg.pose.position.y = gripper_pose_msg.pose.position.y - 1.5*offset[1];
    gripper_pose_msg.pose.position.z = gripper_pose_msg.pose.position.z - 1.5*offset[2];
    moveTo(gripper_pose_msg);

    // close the gripper
    gripper_msg.data = -2.0;
    publish("gripper_effort/set_command", gripper_msg);
    pause_.sleep();

    return true;
  }



  // -------------------------
  bool openDoor()
  // -------------------------
  {
    cout << "switch from moveto to tff controller..." << flush;
    req_switch.stop_controllers.clear();      
    req_switch.start_controllers.clear();     
    req_switch.stop_controllers.push_back("cartesian_trajectory_right");
    req_switch.start_controllers.push_back("cartesian_tff_right");
    ros::service::call("switch_controller", req_switch, res_switch);
    if (!res_switch.ok)
      return false;
    cout << "successful" << endl;

    // turn handle
    tff_msg_.mode.vel.x = tff_msg_.FORCE;
    tff_msg_.mode.vel.y = tff_msg_.FORCE;
    tff_msg_.mode.vel.z = tff_msg_.FORCE;
    tff_msg_.mode.rot.x = tff_msg_.FORCE;
    tff_msg_.mode.rot.y = tff_msg_.FORCE;
    tff_msg_.mode.rot.z = tff_msg_.POSITION;

    tff_msg_.value.vel.x = 10.0;
    tff_msg_.value.vel.y = 0.0;
    tff_msg_.value.vel.z = 0.0;
    tff_msg_.value.rot.x = -1.5;
    tff_msg_.value.rot.y = 0.0;
    tff_msg_.value.rot.z = 0.0;

    publish("cartesian_tff_right/command", tff_msg_);
    pause_.sleep();


    // open door
    tff_msg_.mode.vel.x = tff_msg_.VELOCITY;
    tff_msg_.mode.vel.y = tff_msg_.FORCE;
    tff_msg_.mode.vel.z = tff_msg_.FORCE;
    tff_msg_.mode.rot.x = tff_msg_.FORCE;
    tff_msg_.mode.rot.y = tff_msg_.FORCE;
    tff_msg_.mode.rot.z = tff_msg_.POSITION;

    tff_msg_.value.vel.x = 0.25;
    tff_msg_.value.vel.y = 0.0;
    tff_msg_.value.vel.z = 0.0;
    tff_msg_.value.rot.x = 0.0;
    tff_msg_.value.rot.y = 0.0;
    tff_msg_.value.rot.z = 0.0;

    publish("cartesian_tff_right/command", tff_msg_);
    pause_.sleep();
    pause_.sleep();
    pause_.sleep();
    pause_.sleep();

    // finish
    tff_msg_.mode.vel.x = tff_msg_.FORCE;
    tff_msg_.mode.vel.y = tff_msg_.FORCE;
    tff_msg_.mode.vel.z = tff_msg_.FORCE;
    tff_msg_.mode.rot.x = tff_msg_.FORCE;
    tff_msg_.mode.rot.y = tff_msg_.FORCE;
    tff_msg_.mode.rot.z = tff_msg_.FORCE;

    tff_msg_.value.vel.x = 0.0;
    tff_msg_.value.vel.y = 0.0;
    tff_msg_.value.vel.z = 0.0;
    tff_msg_.value.rot.x = 0.0;
    tff_msg_.value.rot.y = 0.0;
    tff_msg_.value.rot.z = 0.0;

    publish("cartesian_tff_right/command", tff_msg_);


    // open/close the gripper
    std_msgs::Float64 gripper_msg;
    gripper_msg.data = 2.0;
    publish("gripper_effort/set_command", gripper_msg);
    pause_.sleep();
    gripper_msg.data = -2.0;
    publish("gripper_effort/set_command", gripper_msg);


    cout << "switch from tff to moveto controller..." << flush;
    req_switch.stop_controllers.clear();      
    req_switch.start_controllers.clear();     
    req_switch.stop_controllers.push_back("cartesian_tff_right");
    req_switch.start_controllers.push_back("cartesian_trajectory_right");
    ros::service::call("switch_controller", req_switch, res_switch);
    if (!res_switch.ok)
      return false;
    cout << "successful" << endl;

    return true;
  }



  

  void spin()
  {
    while (ok()){
      switch (state_){
	
      case INITIALIZED:{
        cout << "Tucking away arm... " << endl;
        if (tuck_arm()){
          state_ = WAITING;
	  cout << "Waiting for joystick command... " << endl;
          joy_command_ = false;
        }
        else
          state_ = FAILED;
        break;
      }
      case WAITING:{
        if (joy_command_){
          nodHead(1);
          state_  = DETECTING;
        }
        break;
      }
      case DETECTING:{
        cout << "Detecting door... " << endl;
        if (detectDoor(my_door_, my_door_)){
          nodHead(4);
          state_ = GRASPING;
        }
        else
          state_ = FAILED;
        break;
      }
      case GRASPING:{
        cout << "Grasping door... " << endl;
        if (graspDoor(my_door_))
          state_ = OPENDOOR;
        else
          state_ = FAILED;
        break;
      }
      case OPENDOOR:{
        cout << "Opening door... " << endl;
        if (openDoor())
          state_ = SUCCESS;
        else
          state_ = FAILED;
        break;
      }
      case FAILED:{
        shakeHead();
        cout << "FAILED" << endl;
        state_ =INITIALIZED;
        break;
      }
      case SUCCESS:{
        cout << "success" << endl;
        nodHead(4);
        state_ = INITIALIZED;
        break;
      }
        pause_.sleep();
      }
    }
  }
    
  bool moveTo(robot_msgs::PoseStamped& pose)
  {
    pose.header.stamp = Time().now();
    
    cout << "giving moveto command for time " 
         << pose.header.stamp.toSec() << " and frame " 
         << pose.header.frame_id << " to position " 
	 << pose.pose.position.x << " " << pose.pose.position.y << " "<< pose.pose.position.z << endl;
    req_moveto.pose = pose;
    if (!ros::service::call("cartesian_trajectory_right/move_to", req_moveto, res_moveto))
      return false;
    cout << "moveto command finished" << endl; 
    
    return true;
  }
  

  
  Vector getNormalOnDoor(const robot_msgs::Door& door)
  {
    string door_frame = door.header.frame_id;

    Vector door1, door2, tmp, normal;
    cout << "door p1 in " << door_frame << " = " 
         << door.door_p1.x << " " << door.door_p1.y << " "<< door.door_p1.z << endl;
    cout << "door p2 in " << door_frame << " = " 
         << door.door_p2.x << " " << door.door_p2.y << " "<< door.door_p2.z << endl;
    door1[0] = door.door_p1.x;
    door1[1] = door.door_p1.y;
    door1[2] = 0;
    door2[0] = door.door_p2.x;
    door2[1] = door.door_p2.y;
    door2[2] = 0;
    
    // calculate normal in base_link_frame
    door1 = transformPointToFrame(door_frame, robot_frame_, door1, door.header.stamp);
    door2 = transformPointToFrame(door_frame, robot_frame_, door2, door.header.stamp);
    tmp = (door1 - door2); tmp.Normalize();
    normal = tmp * Vector(0,0,1);
    
    // if normal points towards robot, invert normal
    if (dot(normal, door1) < 0)
      normal = normal * -1;
    
    // convert normal to door frame
    normal = transformVectorToFrame(robot_frame_, door_frame, normal, door.header.stamp);
    cout << "normal on door in " << my_door_.header.frame_id << " = " 
         <<  normal[0] << " " << normal[1] << " " << normal[2] << endl;
    
    return normal;
  }
  
  

  Vector transformPointToFrame(const string& frame_start, const string& frame_goal, const Vector& vec, const Time& time)
  {
    Stamped<tf::Point> pnt(Point(vec(0), vec(1), vec(2)), time, frame_start);
    tf_.transformPoint(frame_goal, pnt, pnt);
    return Vector(pnt[0], pnt[1], pnt[2]);
  }
  
  
  Vector transformVectorToFrame(const string& frame_start, const string& frame_goal, const Vector& vec, const Time& time)
  {
    Stamped<tf::Point> pnt(Point(vec(0), vec(1), vec(2)), time, frame_start);
    tf_.transformVector(frame_goal, pnt, pnt);
    return Vector(pnt[0], pnt[1], pnt[2]);
  }
  
  void plannerCallback()
  {
    if (planner_state_.status.value == planner_state_.status.ACTIVE)
      planner_running_ = true;

    if (planner_running_ && planner_state_.status.value != planner_state_.status.ACTIVE){
      planner_running_ = false;
      planner_finished_ = true;
    }
  }

  void joyCallback()
  {
    cout << "got joystick message " << joy_msg_.data << endl;

    std::string status_string;
    ROS_INFO("Joystick message: %s",joy_msg_.data.c_str());
    if (joy_msg_.data == "detect_door")
      joy_command_ = true;
  }


}; // class





// -----------------------------------
//              MAIN
// -----------------------------------

int main(int argc, char** argv)
{
  ros::init(argc,argv); 

  OpenDoorExecutiveTest executive("open_door_executive_test");

  executive.spin();

  return 0;
}
