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
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <door_handle_detector/GraspDoorActionStatus.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <robot_srvs/MoveToPose.h>
#include <kdl/frames.hpp>
#include <robot_actions/action.h>
#include <robot_actions/action_runner.h>

using namespace tf;
using namespace KDL;
using namespace ros;
using namespace std;


static const string fixed_frame = "odom_combined";









//-------------------------------------------------------
class GraspDoorAction: public robot_actions::Action<robot_msgs::Door, robot_msgs::Door>
//-------------------------------------------------------
{
public:
  GraspDoorAction() : 
    robot_actions::Action<robot_msgs::Door, robot_msgs::Door>("grasp_door_action"), 
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

    // door needs to be in time fixed frame
    if (door.header.frame_id != fixed_frame)
      notifyAborted(door);
    Vector normal(door.normal.x, door.normal.y, door.normal.z);
    Vector handle(door.handle.x, door.handle.y, door.handle.z);
    Stamped<Pose> gripper_pose;
    gripper_pose.frame_id_ = fixed_frame;

    // open the gripper
    if (request_preempt_) {notifyPreempted(door); return;}
    std_msgs::Float64 gripper_msg;
    gripper_msg.data = 2.0;
    node_->publish("gripper_effort/set_command", gripper_msg);

    // move gripper in front of door
    if (request_preempt_) {notifyPreempted(door); return;}
    gripper_pose.setOrigin( Vector3(handle(0) + (normal(0) * -0.15), handle(1) + (normal(1) * -0.15), handle(2) + (normal(2) * -0.15)));
    gripper_pose.setRotation( Quaternion(getDoorAngle(door), 0, M_PI/2.0) ); 
    gripper_pose.stamp_ = Time::now();
    PoseStampedTFToMsg(gripper_pose, req_moveto.pose);
    if (!ros::service::call("cartesian_trajectory_right/move_to", req_moveto, res_moveto)){
      if (request_preempt_)
        notifyPreempted(door);
      else
        notifyAborted(door);
      return;
    }

    // move gripper over door handle
    if (request_preempt_) {notifyPreempted(door); return;}
    gripper_pose.frame_id_ = fixed_frame;
    gripper_pose.setOrigin( Vector3(handle(0) + (normal(0) * 0.2 ), handle(1) + (normal(1) * 0.2),  handle(2) + (normal(2) * 0.2)));
    gripper_pose.setRotation( Quaternion(getDoorAngle(door), 0, M_PI/2.0) ); 
    gripper_pose.stamp_ = Time::now();
    PoseStampedTFToMsg(gripper_pose, req_moveto.pose);
    if (!ros::service::call("cartesian_trajectory_right/move_to", req_moveto, res_moveto)){
      if (request_preempt_)
        notifyPreempted(door);
      else
        notifyAborted(door);
      return;
    }

    // close the gripper
    if (request_preempt_) {notifyPreempted(door); return;}
    gripper_msg.data = -2.0;
    node_->publish("gripper_effort/set_command", gripper_msg);
    for (unsigned int i=0; i<100; i++){
      Duration().fromSec(4.0/100.0).sleep();
      if (request_preempt_) {
        gripper_msg.data = 0.0;
        node_->publish("gripper_effort/set_command", gripper_msg);
        notifyPreempted(door); 
        return;
      }
    }

    notifySucceeded(door);
  }


  virtual void handlePreempt()
  {
    request_preempt_ = true;
    ros::service::call("cartesian_trajectory_right/preempt", req_empty, res_empty);
  };


private:
  // get angle between the door normal and the x-axis
  double getDoorAngle(const robot_msgs::Door& door)
  {
    Vector normal(door.normal.x, door.normal.y, door.normal.z);
    Vector x_axis(1,0,0);
    double dot      = normal(0) * x_axis(0) + normal(1) * x_axis(1);
    double perp_dot = normal(1) * x_axis(0) - normal(0) * x_axis(1);
    return atan2(perp_dot, dot);
  }

  ros::Node* node_;

  bool request_preempt_;
  tf::TransformListener tf_; 

  std_srvs::Empty::Request  req_empty;
  std_srvs::Empty::Response res_empty;
  robot_srvs::MoveToPose::Request  req_moveto;
  robot_srvs::MoveToPose::Response res_moveto;
};








// -----------------------------------
//              MAIN
// -----------------------------------

int main(int argc, char** argv)
{
  ros::init(argc,argv); 

  GraspDoorAction grasp;

  robot_actions::ActionRunner runner(10.0);
  runner.connect<robot_msgs::Door, door_handle_detector::GraspDoorActionStatus, robot_msgs::Door>(grasp);

  runner.run();

  return 0;
}
