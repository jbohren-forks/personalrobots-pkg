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
  tc_(map_, SIM_TIME, SIM_STEPS, VEL_SAMPLES, ROBOT_FRONT_RADIUS, ROBOT_SIDE_RADIUS, SAFE_DIST, 
      PDIST_SCALE, GDIST_SCALE, OCCDIST_SCALE, DFAST_SCALE, &tf_),
  cycle_time_(0.1)
{
  robot_vel_.x = 0.0;
  robot_vel_.y = 0.0;
  robot_vel_.yaw = 0.0;
  robot_vel_.frame = "base";
  robot_vel_.time = 0.0;

  advertise<std_msgs::Polyline2D>("local_path");
  advertise<std_msgs::BaseVel>("cmd_vel");
  subscribe("wavefront_plan", plan_msg_, &GovernorNode::planReceived, 1);
  subscribe("odom", odom_msg_, &GovernorNode::odomReceived, 1);
}

void GovernorNode::odomReceived(){
  //we want to make sure that processPlan isn't using robot_vel_
  vel_lock.lock();
  robot_vel_.x = odom_msg_.vel.x;
  robot_vel_.y = odom_msg_.vel.y;
  robot_vel_.yaw = odom_msg_.vel.th;
  robot_vel_.frame = "base";
  robot_vel_.time = 0;
  //give robot_vel_ back
  vel_lock.unlock();
}

void GovernorNode::planReceived(){
  //make sure we don't process the map while we update it
  map_lock.lock();
  
  //update the map from the message
  map_.update(plan_msg_.map);

  //update the global plan from the message
  vector<std_msgs::Point2DFloat32> plan;

  for(unsigned int i = 0; i < plan_msg_.plan.path_size; ++i){
    plan.push_back(plan_msg_.plan.path[i]);
  }
  tc_.updatePlan(plan);

  map_lock.unlock();
}

void GovernorNode::processPlan(){
  libTF::TFPose2D robot_pose;
  robot_pose.x = 0.0;
  robot_pose.y = 0.0;
  robot_pose.yaw = 0.0;
  robot_pose.frame = "base";
  robot_pose.time = 0;

  libTF::TFPose2D robot_acc;
  robot_acc.x = MAX_ACC_X;
  robot_acc.y = MAX_ACC_Y;
  robot_acc.yaw = MAX_ACC_THETA;
  robot_acc.frame = "base";
  robot_acc.time = 0;



  libTF::TFPose2D robot_vel;
  //we need robot_vel_ to compute global_vel so we'll lock
  vel_lock.lock();
  robot_vel = robot_vel_;
  //give robot_vel_ back
  vel_lock.unlock();

  struct timeval start;
  struct timeval end;
  double start_t, end_t, t_diff;
  //we need to lock the map while we process it
  map_lock.lock();
  //Trajectory path = tc_.findBestPath(global_pose, global_vel, global_acc);
  gettimeofday(&start,NULL);
  int path_index = tc_.findBestPath(robot_pose, robot_vel, robot_acc);
  gettimeofday(&end,NULL);
  start_t = start.tv_sec + double(start.tv_usec) / 1e6;
  end_t = end.tv_sec + double(end.tv_usec) / 1e6;
  t_diff = end_t - start_t;
  fprintf(stderr, "Cycle Time: %.3f\n", t_diff);
  //give map_ back
  map_lock.unlock();

  printf("Robot Vel - vx: %.2f, vy: %.2f, vth: %.2f\n", robot_vel.x, robot_vel.y, robot_vel.yaw);

  if(path_index == -1){
    cmd_vel_msg_.vx = 0.0;
    cmd_vel_msg_.vy = 0.0;
    cmd_vel_msg_.vw = 0.0;
  }
  else{

    libTF::TFPose2D drive_cmds = tc_.getDriveVelocities(tc_.trajectories_[path_index]);

    //let's print debug output to the screen
    path_msg.set_points_size(tc_.num_steps_);
    path_msg.color.r = 0;
    path_msg.color.g = 0;
    path_msg.color.b = 1.0;
    path_msg.color.a = 0;
    for(int i = 0; i < tc_.num_steps_; ++i){
      path_msg.points[i].x = tc_.trajectory_pts_(0, path_index * tc_.num_steps_ + i); 
      path_msg.points[i].y = tc_.trajectory_pts_(1, path_index * tc_.num_steps_ + i);
    }
    publish("local_path", path_msg);
    printf("path msg\n");

    //drive the robot!
    cmd_vel_msg_.vx = drive_cmds.x;
    cmd_vel_msg_.vy = drive_cmds.y;
    cmd_vel_msg_.vw = drive_cmds.yaw;

    /*
       cmd_vel_msg_.vx = 0.0;
       cmd_vel_msg_.vy = 0.0;
       cmd_vel_msg_.vw = 0.0;
       */
  }

  printf("Vel CMD - vx: %.2f, vy: %.2f, vt: %.2f\n", cmd_vel_msg_.vx, cmd_vel_msg_.vy, cmd_vel_msg_.vw);
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

  struct timeval start;
  //struct timeval end;
  //double start_t, end_t, t_diff;

  while(gn.ok()){
    gettimeofday(&start,NULL);
    gn.processPlan();
    gn.sleep(start.tv_sec + start.tv_usec / 1e6);
    //gettimeofday(&end,NULL);
    //start_t = start.tv_sec + start.tv_usec / 1e6;
    //end_t = end.tv_sec + end.tv_usec / 1e6;
    //t_diff = end_t - start_t;
    //printf("Cycle Time: %.2f\n", t_diff);
  }

  ros::fini();
  return(0);
}
