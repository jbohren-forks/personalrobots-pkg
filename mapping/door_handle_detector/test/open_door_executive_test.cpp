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
#include <door_handle_detector/DoorDetector.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <robot_mechanism_controllers/cartesian_trajectory_controller.h>
#include <kdl/frames.hpp>

using namespace tf;
using namespace KDL;

class OpenDoorExecutiveTest : public ros::Node
{
private:
  tf::TransformListener tf_; 

  enum {INITIALIZED, DETECTING, GRASPING, FINISHED };
  int state_;
  robot_msgs::Door my_door_;


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

    advertise<robot_msgs::PoseStamped>("cartesian_trajectory/command",1);
  }
  
  
  ~OpenDoorExecutiveTest()
  {}
  
  
  bool DetectDoor(const robot_msgs::Door& door_estimate,  robot_msgs::Door& door_detection)
  {
    door_handle_detector::DoorDetector::Request  req;
    door_handle_detector::DoorDetector::Response res;
    req.door = door_estimate;
    if (ros::service::call("door_handle_detector", req, res)){
      door_detection = res.door;
      return true;
    }
    else
      return false;
  }


  bool GraspDoor(const robot_msgs::Door& door)
  {
    robot_msgs::PoseStamped pose_msg;
    Stamped<Pose> pose;
    pose.frame_id_ = door.header.frame_id;

    Vector normal = getNormalOnDoor(door);
    Vector point(door.handle.x, door.handle.y, door.handle.z);

    pose.setOrigin( Vector3(point[0], point[1], point[2]) );
    Vector z_axis(0,0,1);
    double z_angle = dot(normal, z_axis);
    cout << "z_angle " << z_angle << endl;
    pose.setRotation( Quaternion(z_angle, 0, 0) ); 
    PoseStampedTFToMsg(pose, pose_msg);

    // move in front of door
    pose_msg.pose.position.x = pose_msg.pose.position.x - 0.1;
    publish("cartesian_trajectory/command", pose_msg);
    usleep(1e6 * 10);
    
    // move over door handle
    pose_msg.pose.position.x = pose_msg.pose.position.x + 0.1;
    publish("cartesian_trajectory/command", pose_msg);
    usleep(1e6 * 10);

    return true;
  }



  void spin()
  {
    while (ok()){
	switch (state_){
	  
	case INITIALIZED:{
	  state_ = DETECTING;
	  break;
	}
	case DETECTING:{
	  DetectDoor(my_door_, my_door_);
	  state_ = GRASPING;
	  break;
	}
	case GRASPING:{
	  GraspDoor(my_door_);
	  
	  state_ = FINISHED;
	  break;
	}
	}
	usleep(1e3*100);
      }
  }



  Vector getNormalOnDoor(const robot_msgs::Door& door)
  {
    Vector door1, door2, tmp, normal;
    door1[0] = door.door_p1.x;
    door1[1] = door.door_p1.y;
    door2[0] = door.door_p2.x;
    door2[1] = door.door_p2.y;
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
