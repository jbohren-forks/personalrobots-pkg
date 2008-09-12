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
  ros::node("governor_node"), map_(MAP_SIZE_X, MAP_SIZE_Y), 
  tf_(*this, true, 1 * 1000000000ULL, 100000000ULL), 
  tc_(map_, SIM_TIME, SIM_STEPS, VEL_SAMPLES, &tf_),
  cycle_time_(0.1)
{
  robot_vel_.x = 0.0;
  robot_vel_.y = 0.0;
  robot_vel_.yaw = 0.0;
  robot_vel_.frame = "FRAMEID_ROBOT";
  robot_vel_.time = 0.0;

  advertise<std_msgs::Polyline2D>("local_path");
  //advertise<std_msgs::BaseVel>("cmd_vel");
  subscribe("score_map", map_msg_, &GovernorNode::mapReceived, 1);
  subscribe("odom", odom_msg_, &GovernorNode::odomReceived, 1);
}

void GovernorNode::odomReceived(){
  //we want to make sure that processPlan isn't using robot_vel_
  vel_lock.lock();
  robot_vel_.x = odom_msg_.vel.x;
  robot_vel_.y = odom_msg_.vel.y;
  robot_vel_.yaw = odom_msg_.vel.th;
  robot_vel_.frame = "FRAMEID_ROBOT";
  robot_vel_.time = 0;
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

  libTF::TFPose2D robot_acc;
  robot_acc.x = MAX_ACC_X;
  robot_acc.y = MAX_ACC_Y;
  robot_acc.yaw = MAX_ACC_THETA;
  robot_acc.frame = "FRAMEID_ROBOT";
  robot_acc.time = 0;



  libTF::TFPose2D robot_vel;
  //we need robot_vel_ to compute global_vel so we'll lock
  vel_lock.lock();
  robot_vel = robot_vel_;
  //give robot_vel_ back
  vel_lock.unlock();

  /*uncomment if controller wants to operate in world space
  libTF::TFPose2D global_pose;
  libTF::TFPose2D global_vel;
  libTF::TFPose2D global_acc;

  //transform
  try
  {
    global_pose = tf_.transformPose2D("FRAMEID_MAP", robot_pose);
    global_vel = tf_.transformPose2D("FRAMEID_MAP", robot_vel);
    global_acc = tf_.transformPose2D("FRAMEID_MAP", robot_acc);
  }
  catch(libTF::TransformReference::LookupException& ex)
  {
    puts("no global->local Tx yet");
    printf("%s\n", ex.what());
    return;
  }
  catch(libTF::TransformReference::ConnectivityException& ex)
  {
    puts("no global->local Tx yet");
    printf("%s\n", ex.what());
    return;
  }
  catch(libTF::TransformReference::ExtrapolateException& ex)
  {
    //      puts("extrapolation required");
    //      printf("%s\n", ex.what());
    return;
  }
  printf("Transforms done\n");
  */


  //we need to lock the map while we process it
  map_lock.lock();
  //Trajectory path = tc_.findBestPath(global_pose, global_vel, global_acc);
  Trajectory path = tc_.findBestPath(robot_pose, robot_vel, robot_acc);
  //give map_ back
  map_lock.unlock();

  libTF::TFPose2D drive_cmds = tc_.getDriveVelocities(path);

  //let's print debug output to the screen
  path_msg.set_points_size(path.points_.size());
  path_msg.color.r = 1.0;
  path_msg.color.g = 0;
  path_msg.color.b = 0;
  path_msg.color.a = 0;
  for(unsigned int i = 0; i < path.points_.size(); ++i){
    //path_msg.points[i].x = ((int)((path.points_[i].x_ - map_.origin_x) / map_.scale));
    //path_msg.points[i].y = ((int)((path.points_[i].y_ - map_.origin_y) / map_.scale));
    path_msg.points[i].x = path.points_[i].x_;
    path_msg.points[i].y = path.points_[i].y_;
    //printf("(%.2f, %.2f), ", path_msg.points[i].x, path_msg.points[i].y);
  }
  //printf("(%.2f, %.2f)\n ", path_msg.points[0].x, path_msg.points[0].y);
  //printf("(%.2f, %.2f)\n ", path.points_[0].x_, path.points_[0].y_);
  //printf("(%.2f, %.2f)\n ", map_.origin_x, map_.origin_y);
  publish("local_path", path_msg);
  printf("path msg\n");

  //drive the robot!
  cmd_vel_msg_.vx = drive_cmds.x;
  cmd_vel_msg_.vy = drive_cmds.y;
  cmd_vel_msg_.vw = drive_cmds.yaw;
  //publish("cmd_vel", cmd_vel_msg_);
}

//wait out remaining time of cycle
void GovernorNode::sleep(double loopstart){
  struct timeval curr;
  double curr_t, t_diff;
  gettimeofday(&curr, NULL);
  curr_t = curr.tv_sec + curr.tv_usec / 1e6;
  t_diff = cycle_time_ - (curr_t - loopstart);
  if(t_diff <= 0.0)
    //printf("Governor Node missed deadline and is not sleeping\n");
    return;
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
