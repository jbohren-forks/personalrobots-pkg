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
#include <robot_msgs/Planner2DGoal.h>
#include <robot_srvs/SwitchController.h>
#include <std_msgs/Float64.h>
#include <door_handle_detector/DoorDetector.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <robot_srvs/MoveToPose.h>
#include <kdl/frames.hpp>

using namespace tf;
using namespace KDL;
using namespace ros;
using namespace std;

static const string fixed_frame = "odom_combined";


class OpenDoorExecutiveTest : public ros::Node
{
private:
  tf::TransformListener tf_; 

  enum {INITIALIZED, DETECTING, GRASPING, OPENDOOR, SUCCESS, FAILED, FINISHED };
  int state_;
  bool goal_acchieved_;

  robot_msgs::Door my_door_;
  robot_msgs::TaskFrameFormalism tff_msg_;

  door_handle_detector::DoorDetector::Request  req_doordetect;
  door_handle_detector::DoorDetector::Response res_doordetect;

  robot_srvs::MoveToPose::Request  req_moveto;
  robot_srvs::MoveToPose::Response res_moveto;

  robot_srvs::SwitchController::Request req_switch;
  robot_srvs::SwitchController::Response res_switch;

public:
  OpenDoorExecutiveTest(std::string node_name):
    ros::Node(node_name),
    tf_(*this),
    state_(INITIALIZED)
    //state_(DETECTING)
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
    advertise<robot_msgs::Planner2DGoal>("goal", 10);

    //    subscribe();
  }
  
  
  ~OpenDoorExecutiveTest()
  {}
  

  // -------------------------
  bool initialize()
  // -------------------------
  {
    // start arm trajectory controller
    cout << "turn on moveto controller..." << flush;
    req_switch.stop_controllers.clear(); 
    req_switch.start_controllers.clear();     req_switch.start_controllers.push_back("cartesian_trajectory_right");  
    ros::service::call("switch_controller", req_switch, res_switch);
    if (!res_switch.ok)
      return false;
    cout << "successful" << endl;
    
    robot_msgs::PoseStamped init_pose;
    init_pose.header.frame_id = "base_link";
    init_pose.pose.position.x = 0.2;
    init_pose.pose.position.y = 0.0;
    init_pose.pose.position.z = 0.4;
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
    // get orientation in fixed_frame, with X-axis alligned with door normal, and Z-axis up
    Vector normal = getNormalOnDoor(door);
    normal = transformVectorToFrame(door.header.frame_id, fixed_frame, normal);
    double z_angle = acos(dot(normal, Vector(1,0,0)));
    Quaternion orientation(z_angle, 0, M_PI/2.0); 

    // move the robot in front of the door
    Vector robot_pos(door.door_p1.x - door.door_p2.x, door.door_p1.y - door.door_p2.y, door.door_p1.z - door.door_p2.z);
    robot_pos = transformPointToFrame(door.header.frame_id, fixed_frame, robot_pos);
    robot_pos = robot_pos - 1.0*normal;

    robot_msgs::Planner2DGoal robot_pose_msg;
    robot_pose_msg.header.frame_id = fixed_frame;
    robot_pose_msg.enable = true;
    robot_pose_msg.timeout = 1000000;
    robot_pose_msg.goal.x = robot_pos(0);
    robot_pose_msg.goal.y = robot_pos(1);
    robot_pose_msg.goal.th = z_angle;
    goal_acchieved_ = false;
    publish("goal", robot_pose_msg);
    while (!goal_acchieved_)
      Duration().fromSec(0.1).sleep();

    // move gripper in front of door
    robot_msgs::PoseStamped pose_msg;
    Vector handle_pos(door.handle.x, door.handle.y, door.handle.z);
    handle_pos = transformPointToFrame(door.header.frame_id, fixed_frame, handle_pos);
    Stamped<Pose> pose(Pose(orientation, Vector3(handle_pos(0), handle_pos(1), handle_pos(2))), Time(), fixed_frame);
    PoseStampedTFToMsg(pose, pose_msg);
    pose_msg.pose.position.x = pose_msg.pose.position.x - 0.15 * normal(0);
    pose_msg.pose.position.y = pose_msg.pose.position.y - 0.15 * normal(1);
    pose_msg.pose.position.z = pose_msg.pose.position.z - 0.15 * normal(2);
    moveTo(pose_msg);

    // open the gripper
    std_msgs::Float64 gripper_msg;
    gripper_msg.data = 2.0;
    publish("gripper_effort/set_command", gripper_msg);
    usleep(1e6 * 4);
    
    // move gripper over door handle
    pose_msg.pose.position.x = pose_msg.pose.position.x + 0.2 * normal(0);
    pose_msg.pose.position.y = pose_msg.pose.position.y + 0.2 * normal(1);
    pose_msg.pose.position.z = pose_msg.pose.position.z + 0.2 * normal(2);
    moveTo(pose_msg);

    // close the gripper
    gripper_msg.data = -2.0;
    publish("gripper_effort/set_command", gripper_msg);
    usleep(1e6 * 7);

    return true;
  }



  // -------------------------
  bool openDoor()
  // -------------------------
  {
    cout << "switch from moveto to tff controller..." << flush;
    req_switch.stop_controllers.clear();      req_switch.stop_controllers.push_back("cartesian_trajectory_right");
    req_switch.start_controllers.clear();     req_switch.start_controllers.push_back("cartesian_tff_right");
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

    tff_msg_.value.vel.x = 0.0;
    tff_msg_.value.vel.y = -20.0;
    tff_msg_.value.vel.z = 0.0;
    tff_msg_.value.rot.x = -0.5;
    tff_msg_.value.rot.y = 0.0;
    tff_msg_.value.rot.z = 0.0;

    publish("cartesian_tff_right/command", tff_msg_);
    usleep(1e6*6);


    // open door
    tff_msg_.mode.vel.x = tff_msg_.VELOCITY;
    tff_msg_.mode.vel.y = tff_msg_.FORCE;
    tff_msg_.mode.vel.z = tff_msg_.FORCE;
    tff_msg_.mode.rot.x = tff_msg_.FORCE;
    tff_msg_.mode.rot.y = tff_msg_.FORCE;
    tff_msg_.mode.rot.z = tff_msg_.POSITION;

    tff_msg_.value.vel.x = 0.05;
    tff_msg_.value.vel.y = 0.0;
    tff_msg_.value.vel.z = 0.0;
    tff_msg_.value.rot.x = 0.0;
    tff_msg_.value.rot.y = 0.0;
    tff_msg_.value.rot.z = 0.0;

    publish("cartesian_tff_right/command", tff_msg_);
    usleep(1e6*15);

    cout << "trun off tff controller..." << flush;
    req_switch.stop_controllers.clear();      req_switch.stop_controllers.push_back("cartesian_tff_right");
    req_switch.start_controllers.clear();  
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
          cout << "Initializing door opening... " << endl;
          usleep(1e6*2);
          if (initialize())
	    state_ = DETECTING;
	  else
	    state_ = FAILED;
	  break;
	}
	case DETECTING:{
          cout << "Detecting door... " << endl;
	  if (detectDoor(my_door_, my_door_))
	    state_ = GRASPING;
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
          cout << "FAILED" << endl;
          state_ = FINISHED;
          break;
	}
        case SUCCESS:{
          cout << "success" << endl;
          state_ = FINISHED;
          break;
	}
        case FINISHED: return;
        }
        usleep(1e3*100);
      }
  }


  bool moveTo(robot_msgs::PoseStamped& pose)
  {
    pose.header.stamp = Time().now() - Duration().fromSec(1); 

    cout << "giving moveto command for time " 
         << pose.header.stamp.toSec() << " and frame " 
         << pose.header.frame_id << endl;
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
    cout << "door p1 " << door.door_p1.x << " " << door.door_p1.y << " "<< door.door_p1.z << endl;
    cout << "door p2 " << door.door_p2.x << " " << door.door_p2.y << " "<< door.door_p2.z << endl;
    door1[0] = door.door_p1.x;
    door1[1] = door.door_p1.y;
    door1[2] = 0;
    door2[0] = door.door_p2.x;
    door2[1] = door.door_p2.y;
    door2[2] = 0;

    // calculate normal in base_link_frame
    door1 = transformPointToFrame(door_frame, "base_link", door1);
    door2 = transformPointToFrame(door_frame, "base_link", door2);
    tmp = (door1 - door2); tmp.Normalize();
    normal = tmp * Vector(0,0,1);

    // if normal points towards robot, invert normal
    if (dot(normal, door1) < 0)
      normal = normal * -1;

    // convert normal to door frame
    normal = transformVectorToFrame("base_link", door_frame, normal);
    cout << "normal on door in " << my_door_.header.frame_id << " = " 
         <<  normal[0] << " " << normal[1] << " " << normal[2] << endl;

    return normal;
  }



  Vector transformPointToFrame(const string& frame_start, const string& frame_goal, const Vector& vec)
  {
    Stamped<tf::Point> pnt(Point(vec(0), vec(1), vec(2)), Time(), frame_start);
    tf_.transformPoint(frame_goal, pnt, pnt);
    return Vector(pnt[0], pnt[1], pnt[2]);
  }


  Vector transformVectorToFrame(const string& frame_start, const string& frame_goal, const Vector& vec)
  {
    Stamped<tf::Point> pnt(Point(vec(0), vec(1), vec(2)), Time(), frame_start);
    tf_.transformVector(frame_goal, pnt, pnt);
    return Vector(pnt[0], pnt[1], pnt[2]);
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
