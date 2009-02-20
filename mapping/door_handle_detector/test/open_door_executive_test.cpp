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
#include <door_handle_detector/DoorDetector.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <robot_mechanism_controllers/cartesian_trajectory_controller.h>
#include <robot_mechanism_controllers/cartesian_tff_controller.h>
#include <robot_mechanism_controllers/MoveToPose.h>
#include <kdl/frames.hpp>

using namespace tf;
using namespace KDL;
using namespace ros;

class OpenDoorExecutiveTest : public ros::Node
{
private:
  tf::TransformListener tf_; 

  enum {INITIALIZED, DETECTING, GRASPING, OPENDOOR, SUCCESS, FAILED };
  int state_;
  robot_msgs::Door my_door_;
  robot_msgs::TaskFrameFormalism tff_msg_;

  std_srvs::Empty::Request  req_empty;
  std_srvs::Empty::Response res_empty;

  door_handle_detector::DoorDetector::Request  req_doordetect;
  door_handle_detector::DoorDetector::Response res_doordetect;

  robot_mechanism_controllers::MoveToPose::Request  req_moveto;
  robot_mechanism_controllers::MoveToPose::Response res_moveto;


public:
  OpenDoorExecutiveTest(std::string node_name):
    ros::Node(node_name),
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
    my_door_.header.frame_id = "odom_combined";
  }
  
  
  ~OpenDoorExecutiveTest()
  {}
  
  
  bool detectDoor(const robot_msgs::Door& door_estimate,  robot_msgs::Door& door_detection)
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


  bool graspDoor(const robot_msgs::Door& door)
  {
    robot_msgs::PoseStamped pose_msg;
    Stamped<Pose> pose;
    pose.frame_id_ = door.header.frame_id;

    Vector normal = getNormalOnDoor(door);
    Vector point(door.handle.x, door.handle.y, door.handle.z);

    pose.setOrigin( Vector3(point[0], point[1], point[2]) );
    Vector x_axis(1,0,0);
    double z_angle = acos(dot(-normal, x_axis));
    cout << "z_angle " << z_angle << endl;
    pose.setRotation( Quaternion(z_angle, 0, M_PI/2.0) ); 
    PoseStampedTFToMsg(pose, pose_msg);

    // give some time for manual positioning of robot base
    usleep(1e6 * 10);

    // move in front of door
    Vector offset = normal * 0.1;
    pose_msg.pose.position.x = pose_msg.pose.position.x + offset[0];
    pose_msg.pose.position.y = pose_msg.pose.position.y + offset[1];
    pose_msg.pose.position.z = pose_msg.pose.position.z + offset[2];
    moveTo(pose_msg);
    
    // move over door handle
    pose_msg.pose.position.x = pose_msg.pose.position.x - offset[0];
    pose_msg.pose.position.y = pose_msg.pose.position.y - offset[1];
    pose_msg.pose.position.z = pose_msg.pose.position.z - offset[2];
    moveTo(pose_msg);

    // close the gripper


    // stop arm trajectory controller
    cout << "stopping moveto controller" << endl;
    if (!ros::service::call("cartesian_trajectory_right/stop", req_empty, res_empty))
      return false;

    return true;
  }


  bool openDoor()
  {
    // turn on tff controller
    cout << "starting tff controller" << endl;
    if (!ros::service::call("cartesian_tff_right/start", req_empty, res_empty))
      return false;

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
    usleep(1e6*3);


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

    // turn off tff controller
    cout << "stopping tff controller" << endl;
    if (!ros::service::call("cartesian_tff_right/stop", req_empty, res_empty))
      return false;

    return true;
  }


  bool initialize()
  {
    // start arm trajectory controller
    cout << "starting moveto controller" << endl;
    if (!ros::service::call("cartesian_trajectory_right/start", req_empty, res_empty))
      return false;
    cout << "starting moveto controller successful" << endl;
    
    robot_msgs::PoseStamped init_pose;
    init_pose.header.frame_id = "base_link";
    // dirty hack because service sets time 0 to time now
    init_pose.header.stamp = Time().now() - Duration().fromSec(1); 
    init_pose.pose.position.x = 0.5;
    init_pose.pose.position.y = 0.0;
    init_pose.pose.position.y = 0.4;
    init_pose.pose.orientation.x = 0;
    init_pose.pose.orientation.y = 0;
    init_pose.pose.orientation.z = 0;
    init_pose.pose.orientation.w = 1;	  
    moveTo(init_pose);
  }



  void spin()
  {
    while (ok()){
	switch (state_){
	  
	case INITIALIZED:{
          if (initialize())
	    state_ = SUCCESS;
	  else
	    state_ = FAILED;
	  break;
	}
	case DETECTING:{
	  if (detectDoor(my_door_, my_door_))
	    state_ = GRASPING;
	  else
	    state_ = FAILED;
	  break;
	}
	case GRASPING:{
	  if (graspDoor(my_door_))
	    state_ = OPENDOOR;
	  else
	    state_ = FAILED;
	  break;
	}
        case OPENDOOR:{
          if (openDoor())
	    state_ = SUCCESS;
	  else
	    state_ = FAILED;
          break;
        }
	}
	usleep(1e3*100);
      }
  }


  bool moveTo(const robot_msgs::PoseStamped& pose)
  {
    cout << "giving moveto command for time " 
         << pose.header.stamp.toSec() << " and frame " 
         << pose.header.frame_id << endl;
    req_moveto.pose = pose;
    if (!ros::service::call("cartesian_trajectory_right/move_to", req_moveto, res_moveto))
      return false;

    return true;
  }



  Vector getNormalOnDoor(const robot_msgs::Door& door)
  {
    Vector door1, door2, tmp, normal;
    cout << "door p1 " << door.door_p1.x << " " << door.door_p1.y << " "<< door.door_p1.z << endl;
    cout << "door p2 " << door.door_p2.x << " " << door.door_p2.y << " "<< door.door_p2.z << endl;
    door1[0] = door.door_p1.x;
    door1[1] = door.door_p1.y;
    door1[2] = 0;
    door2[0] = door.door_p2.x;
    door2[1] = door.door_p2.y;
    door2[2] = 0;
    tmp = (door1 - door2); tmp.Normalize();
    normal = tmp * Vector(0,0,1);

    cout << "normal on door = " <<  normal[0] << " " << normal[1] << " " << normal[2] << endl;

    return normal;
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
