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

#include <ros/node.h>
#include <pr2_mechanism_controllers/GraspPointSrv.h>
#include <pr2_mechanism_controllers/TrajectoryStart.h>

static int done = 0;

void finalize(int donecare)
{
  done = 1;
}

int main( int argc, char** argv )
{

  /*********** Initialize ROS  ****************/
  ros::init(argc,argv);
  ros::node *node = new ros::node("grasp_point_client"); 

  signal(SIGINT,  finalize);
  signal(SIGQUIT, finalize);
  signal(SIGTERM, finalize);

  pr2_mechanism_controllers::GraspPointSrv::request  req;
  pr2_mechanism_controllers::GraspPointSrv::response res;

  req.transform.header.stamp = ros::Time::now();
  req.transform.header.frame_id = "r_shoulder_pan_link";
  req.transform.pose.position.x = 0.5;
  req.transform.pose.position.y = 0.0;
  req.transform.pose.position.z = +0.2;

  req.transform.pose.orientation.x = 0.0;
  req.transform.pose.orientation.y = 0.0;
  req.transform.pose.orientation.z = 0.0;
  req.transform.pose.orientation.w = 1.0;

  if (ros::service::call("/grasp_point_node/SetGraspPoint", req, res))
  {
    ROS_INFO("Done");

    for(int i=0; i<2;i++)
    {
      for(int j=0; j< (int) res.traj.points[0].get_positions_size(); j++)
      {
        std::cout << res.traj.points[i].positions[j] << " ";
      }
      std::cout << std::endl;
    }
  }
  else
  {
    ROS_INFO("IK failed");
    return 0;
  }

  sleep(1);

  pr2_mechanism_controllers::TrajectoryStart::request  reqt;
  pr2_mechanism_controllers::TrajectoryStart::response rest;

  int num_points = (int) res.traj.get_points_size();
  int num_joints = (int) res.traj.points[0].get_positions_size();

  reqt.traj.set_points_size(num_points);
  reqt.requesttiming = 1;

  for(int i=0; i<num_points; i++)
  {
    reqt.traj.points[i].set_positions_size(num_joints);
    for(int j=0; j<num_joints; j++)
    {
      reqt.traj.points[i].positions[j] = res.traj.points[i].positions[j];
    }
  }
  reqt.traj.points[1].time = 4.0;

  if (ros::service::call("/right_arm_trajectory_controller/TrajectoryStart", reqt, rest))
  {
    ROS_INFO("Done");
  }
  else
  {
    ROS_INFO("Trajectory failed");
  }
//  sleep(20);
  ros::fini();
  return 0;
}
