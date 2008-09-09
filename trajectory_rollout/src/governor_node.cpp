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
*********************************************************************/
#include <trajectory_rollout/governor_node.h>

using namespace std;
using namespace trajectory_rollout;

GovernorNode::GovernorNode() : 
  ros::node("trajectory_rollout"), map_(MAP_ROWS, MAP_COLS), 
  tf_(*this, true, 1 * 1000000000ULL, 100000000ULL), 
  tc_(map_, MAX_ACC_X, MAX_ACC_Y, MAX_ACC_THETA, SIM_TIME, SIM_STEPS, VEL_SAMPLES, &tf_),
  cycle_time_(0.1)
{
  robot_vel_.x = 0.0;
  robot_vel_.y = 0.0;
  robot_vel_.yaw = 0.0;
  robot_vel_.frame = "FRAMEID_ROBOT";
  robot_vel_.time = 0.0;

  advertise<std_msgs::Polyline2D>("gui_path");
  advertise<std_msgs::BaseVel>("cmd_vel");
  subscribe("score_map", map_msg_, &GovernorNode::mapReceived);
  subscribe("odom", odom_msg_, &GovernorNode::odomReceived);
}

void GovernorNode::odomReceived(){
  //we want to make sure that processPlan isn't using robot_vel_
  vel_lock.lock();
  robot_vel_.x = odom_msg_.vel.x;
  robot_vel_.y = odom_msg_.vel.y;
  robot_vel_.yaw = odom_msg_.vel.th;
  //give robot_vel_ back
  vel_lock.unlock();

}

void GovernorNode::mapReceived(){
  //make sure we don't process the map while we update it
  map_lock.lock();
  
  //update the map from the message
  map_.update(map_msg_);

  map_lock.unlock();
}

void GovernorNode::processPlan(){

  libTF::TFPose2D robot_pose;
  robot_pose.x = 0.0;
  robot_pose.y = 0.0;
  robot_pose.yaw = 0.0;
  robot_pose.frame = "FRAMEID_ROBOT";
  robot_pose.time = 0;

  //we want to make sure that the odomReceived callback isn't updating velocities
  vel_lock.lock();
  libTF::TFPose2D robot_vel = robot_vel_;
  //give robot_vel_ back
  vel_lock.unlock();

  //we need to lock the map while we process it
  map_lock.lock();
  Trajectory path = tc_.findBestPath(robot_pose, robot_vel);
  //give map_ back
  map_lock.unlock();

  libTF::TFPose2D drive_cmds = tc_.getDriveVelocities(path);

  //drive the robot!
  cmd_vel_msg_.vx = drive_cmds.x;
  cmd_vel_msg_.vy = drive_cmds.y;
  cmd_vel_msg_.vw = drive_cmds.yaw;
  publish("cmd_vel", cmd_vel_msg_);
}

//wait out remaining time of cycle
void GovernorNode::sleep(double loopstart){
  struct timeval curr;
  double curr_t, t_diff;
  gettimeofday(&curr, NULL);
  curr_t = curr.tv_sec + curr.tv_usec / 1e6;
  t_diff = cycle_time_ - (curr_t - loopstart);
  if(t_diff <= 0.0)
    printf("Governor Node missed deadline and is not sleeping\n");
  else
    usleep((unsigned int) rint(t_diff * 1e6));
}

int main(int argc, char** argv){
  ros::init(argc, argv);
  GovernorNode gn;

  struct timeval curr;

  while(gn.ok()){
    gettimeofday(&curr,NULL);
    gn.processPlan();
    gn.sleep(curr.tv_sec + curr.tv_usec / 1e6);
  }

  ros::fini();
  return(0);
}
