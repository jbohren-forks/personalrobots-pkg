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
#include <robot_msgs/Planner2DState.h>
#include <robot_srvs/SwitchController.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <door_handle_detector/DetectDoorActionStatus.h>
#include <door_handle_detector/DoorDetector.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <robot_srvs/MoveToPose.h>
#include <kdl/frames.hpp>
#include <sbpl_arm_executive/pr2_arm_node.h>
#include <robot_actions/action.h>
#include <robot_actions/action_runner.h>

using namespace tf;
using namespace KDL;
using namespace ros;
using namespace std;
using namespace pr2_arm_node;


static const string fixed_frame = "odom_combined";
static const string robot_frame = "base_link";






  

Vector transformPointTo(const string& frame_start, const string& frame_goal, 
                        const Vector& vec, const Time& time, const tf::Transformer& tf)
{
  Stamped<tf::Point> pnt(Point(vec(0), vec(1), vec(2)), time, frame_start);
  tf.transformPoint(frame_goal, pnt, pnt);
  return Vector(pnt[0], pnt[1], pnt[2]);
}

Vector transformVectorTo(const string& frame_start, const string& frame_goal, 
                         const Vector& vec, const Time& time, const tf::Transformer& tf)
{
  Stamped<tf::Point> pnt(Point(vec(0), vec(1), vec(2)), time, frame_start);
  tf.transformVector(frame_goal, pnt, pnt);
  return Vector(pnt[0], pnt[1], pnt[2]);
}

Vector getNormalOnDoor(const robot_msgs::Door& door, const tf::Transformer& tf)
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
  door1 = transformPointTo(door_frame, robot_frame, door1, door.header.stamp, tf);
  door2 = transformPointTo(door_frame, robot_frame, door2, door.header.stamp, tf);
  tmp = (door1 - door2); tmp.Normalize();
  normal = tmp * Vector(0,0,1);
  
  // if normal points towards robot, invert normal
  if (dot(normal, door1) < 0)
    normal = normal * -1;
  
  // convert normal to door frame
  normal = transformVectorTo(robot_frame, door_frame, normal, door.header.stamp, tf);
  cout << "normal on door in " << door_frame << " = " 
       <<  normal[0] << " " << normal[1] << " " << normal[2] << endl;
  
  return normal;
}




class DetectDoorAction: public robot_actions::Action<robot_msgs::Door, robot_msgs::Door>
{
public:
  DetectDoorAction(): robot_actions::Action<robot_msgs::Door, robot_msgs::Door>("detect_door_action") {};
  ~DetectDoorAction(){};

  virtual void handleActivate(const robot_msgs::Door& door)
  {
    notifyActivated();

    req_doordetect.door = door;
    if (ros::service::call("door_handle_detector", req_doordetect, res_doordetect)){
      notifySucceeded(res_doordetect.door);
    }
    notifyAborted(door);
  }

  virtual void handlePreempt(){};



private:
  door_handle_detector::DoorDetector::Request  req_doordetect;
  door_handle_detector::DoorDetector::Response res_doordetect;
};





class GraspDoorAction: public robot_actions::Action<robot_msgs::Door, std_msgs::Empty>
{
public:
  GraspDoorAction() : 
    robot_actions::Action<robot_msgs::Door, std_msgs::Empty>("grasp_door_action"), 
    node_(ros::Node::instance()),
    request_preempt_(false),
    tf_(*node_)
  {
    node_->advertise<std_msgs::Float64>("gripper_effort/set_command",10);
  };
  ~GraspDoorAction(){};

  virtual void handleActivate(const robot_msgs::Door& door)
  {
    notifyActivated();

    // door needs to be in fixed frame
    if (door.header.frame_id != fixed_frame)
      notifyAborted(empty_);

    // get orientation
    Vector normal = getNormalOnDoor(door, tf_);
    Vector handle(door.handle.x, door.handle.y, door.handle.z);
    Vector x_axis(1,0,0);
    double dot      = normal(0) * x_axis(0) + normal(1) * x_axis(1);
    double perp_dot = normal(1) * x_axis(0) - normal(0) * x_axis(1);
    double z_angle = atan2(perp_dot, dot);
    cout << "z_angle in " << fixed_frame << " = " << z_angle << endl;

    // open the gripper
    if (request_preempt_) {notifyPreempted(empty_); return;}
    std_msgs::Float64 gripper_msg;
    gripper_msg.data = 2.0;
    node_->publish("gripper_effort/set_command", gripper_msg);

    // move gripper in front of door
    if (request_preempt_) {notifyPreempted(empty_); return;}
    Stamped<Pose> gripper_pose;
    gripper_pose.frame_id_ = fixed_frame;
    gripper_pose.setOrigin( Vector3(handle(0) + (normal(0) * -0.15), handle(1) + (normal(1) * -0.15), handle(2) + (normal(2) * -0.15)));
    gripper_pose.setRotation( Quaternion(z_angle, 0, M_PI/2.0) ); 
    PoseStampedTFToMsg(gripper_pose, req_moveto.pose);
    if (!ros::service::call("cartesian_trajectory_right/move_to", req_moveto, res_moveto)){
      if (request_preempt_)
        notifyPreempted(empty_);
      else
        notifyAborted(empty_);
      return;
    }

    // move gripper over door handle
    if (request_preempt_) {notifyPreempted(empty_); return;}
    gripper_pose.frame_id_ = fixed_frame;
    gripper_pose.setOrigin( Vector3(handle(0) + (normal(0) * 0.2 ), handle(1) + (normal(1) * 0.2),  handle(2) + (normal(2) * 0.2)));
    gripper_pose.setRotation( Quaternion(z_angle, 0, M_PI/2.0) ); 
    PoseStampedTFToMsg(gripper_pose, req_moveto.pose);
    if (!ros::service::call("cartesian_trajectory_right/move_to", req_moveto, res_moveto)){
      if (request_preempt_)
        notifyPreempted(empty_);
      else
        notifyAborted(empty_);
      return;
    }

    // close the gripper
    if (request_preempt_) {notifyPreempted(empty_); return;}
    gripper_msg.data = -2.0;
    node_->publish("gripper_effort/set_command", gripper_msg);
    for (unsigned int i=0; i<100; i++){
      Duration().fromSec(4.0/100.0).sleep();
      if (request_preempt_) {
        gripper_msg.data = 0.0;
        node_->publish("gripper_effort/set_command", gripper_msg);
        notifyPreempted(empty_); 
        return;
      }
    }

    notifySucceeded(empty_);
  }


  virtual void handlePreempt()
  {
    request_preempt_ = true;
    ros::service::call("cartesian_trajectory_right/preempt", req_empty, res_empty);
  };


private:
  std_msgs::Empty empty_;

  ros::Node* node_;

  bool request_preempt_;
  tf::TransformListener tf_; 

  std_srvs::Empty::Request  req_empty;
  std_srvs::Empty::Response res_empty;
  robot_srvs::MoveToPose::Request  req_moveto;
  robot_srvs::MoveToPose::Response res_moveto;
};







class OpenDoorAction: public robot_actions::Action<robot_msgs::Door, robot_msgs::Door>
{
public:
  OpenDoorAction() : 
    robot_actions::Action<robot_msgs::Door, robot_msgs::Door>("open_door_action"), 
    node_(ros::Node::instance()),    
    request_preempt_(false)
  {};

  ~OpenDoorAction(){};

  virtual void handleActivate(const robot_msgs::Door& door)
  {
    notifyActivated();

    // stop
    tff_stop_.mode.vel.x = tff_stop_.FORCE;
    tff_stop_.mode.vel.y = tff_stop_.FORCE;
    tff_stop_.mode.vel.z = tff_stop_.FORCE;
    tff_stop_.mode.rot.x = tff_stop_.FORCE;
    tff_stop_.mode.rot.y = tff_stop_.FORCE;
    tff_stop_.mode.rot.z = tff_stop_.FORCE;

    tff_stop_.value.vel.x = 0.0;
    tff_stop_.value.vel.y = 0.0;
    tff_stop_.value.vel.z = 0.0;
    tff_stop_.value.rot.x = 0.0;
    tff_stop_.value.rot.y = 0.0;
    tff_stop_.value.rot.z = 0.0;

    // turn handle
    tff_handle_.mode.vel.x = tff_handle_.FORCE;
    tff_handle_.mode.vel.y = tff_handle_.FORCE;
    tff_handle_.mode.vel.z = tff_handle_.FORCE;
    tff_handle_.mode.rot.x = tff_handle_.FORCE;
    tff_handle_.mode.rot.y = tff_handle_.FORCE;
    tff_handle_.mode.rot.z = tff_handle_.POSITION;

    tff_handle_.value.vel.x = 10.0;
    tff_handle_.value.vel.y = 0.0;
    tff_handle_.value.vel.z = 0.0;
    tff_handle_.value.rot.x = 0.0;
    tff_handle_.value.rot.y = 0.0;
    tff_handle_.value.rot.z = 0.0;

    // open door
    tff_door_.mode.vel.x = tff_door_.VELOCITY;
    tff_door_.mode.vel.y = tff_door_.FORCE;
    tff_door_.mode.vel.z = tff_door_.FORCE;
    tff_door_.mode.rot.x = tff_door_.FORCE;
    tff_door_.mode.rot.y = tff_door_.FORCE;
    tff_door_.mode.rot.z = tff_door_.POSITION;

    tff_door_.value.vel.x = 0.25;
    tff_door_.value.vel.y = 0.0;
    tff_door_.value.vel.z = 0.0;
    tff_door_.value.rot.x = 0.0;
    tff_door_.value.rot.y = 0.0;
    tff_door_.value.rot.z = 0.0;



    // turn handle
    for (unsigned int i=0; i<100; i++){
      tff_handle_.value.rot.x += -1.5/100.0;
      node_->publish("cartesian_tff_right/command", tff_handle_);
      Duration().fromSec(4.0/100.0).sleep();
      if (request_preempt_) {
        node_->publish("cartesian_tff_right/command", tff_stop_);
        notifyPreempted(door); 
        return;
      }
    }

    // open door
    node_->publish("cartesian_tff_right/command", tff_door_);
    for (unsigned int i=0; i<500; i++){
      Duration().fromSec(10.0/500.0).sleep();
      if (request_preempt_) {
        node_->publish("cartesian_tff_right/command", tff_stop_);
        notifyPreempted(door); 
        return;
      }
    }

    // finish
    node_->publish("cartesian_tff_right/command", tff_stop_);

    notifySucceeded(door);
  }


  virtual void handlePreempt()
  {
    request_preempt_ = true;
  };



private:
  ros::Node* node_;

  bool request_preempt_;

  robot_msgs::TaskFrameFormalism tff_stop_, tff_handle_, tff_door_;


};



  







// -----------------------------------
//              MAIN
// -----------------------------------

int main(int argc, char** argv)
{
  ros::init(argc,argv); 

  DetectDoorAction detect;
  GraspDoorAction grasp;
  OpenDoorAction open;

  robot_actions::ActionRunner runner(10.0);
  runner.connect<robot_msgs::Door, door_handle_detector::DetectDoorActionStatus, robot_msgs::Door>(detect);

  runner.run();

  return 0;
}
