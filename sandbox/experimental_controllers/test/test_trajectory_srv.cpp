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
#include <manipulation_srvs/SetSplineTraj.h>
#include <manipulation_srvs/QuerySplineTraj.h>
#include <manipulation_srvs/CancelSplineTraj.h>

static int done = 0;

void finalize(int donecare)
{
  done = 1;
}

int main( int argc, char** argv )
{

  /*********** Initialize ROS  ****************/
  ros::init(argc,argv);
  ros::Node *node = new ros::Node("arm_trajectory_controller_client"); 

  signal(SIGINT,  finalize);
  signal(SIGQUIT, finalize);
  signal(SIGTERM, finalize);

  manipulation_srvs::SetSplineTraj::Request  req;
  manipulation_srvs::SetSplineTraj::Response res;

  manipulation_srvs::QuerySplineTraj::Request  req_q;
  manipulation_srvs::QuerySplineTraj::Response res_q;

  manipulation_srvs::CancelSplineTraj::Request  rec_req;
  manipulation_srvs::CancelSplineTraj::Response rec_res;

  int num_segments = 2;
  int num_joints = 7;

  req.spline.set_segments_size(num_segments);

  req.spline.segments[0].set_a_size(7);
  req.spline.segments[0].set_b_size(7);
  req.spline.segments[0].set_c_size(7);
  req.spline.segments[0].set_d_size(7);
  req.spline.segments[0].set_e_size(7);
  req.spline.segments[0].set_f_size(7);

  req.spline.segments[0].a[0] = 0.0;
  req.spline.segments[0].a[1] = 0.0;
  req.spline.segments[0].a[2] = 0.0;
  req.spline.segments[0].a[3] = 0.0;
  req.spline.segments[0].a[4] = 0.0;
  req.spline.segments[0].a[5] = 0.1;
  req.spline.segments[0].a[6] = 0.0;

  req.spline.segments[0].b[0] = 0.125;
  req.spline.segments[0].b[1] = 0.0;
  req.spline.segments[0].b[2] = 0.0;
  req.spline.segments[0].b[3] = 0.0;
  req.spline.segments[0].b[4] = 0.0;
  req.spline.segments[0].b[5] = 0.0;
  req.spline.segments[0].b[6] = 0.0;

  req.spline.segments[0].duration = ros::Time(4.0);



  req.spline.segments[1].set_a_size(7);
  req.spline.segments[1].set_b_size(7);
  req.spline.segments[1].set_c_size(7);
  req.spline.segments[1].set_d_size(7);
  req.spline.segments[1].set_e_size(7);
  req.spline.segments[1].set_f_size(7);

  req.spline.segments[1].a[0] = 0.5;
  req.spline.segments[1].a[1] = 0.0;
  req.spline.segments[1].a[2] = 0.0;
  req.spline.segments[1].a[3] = 0.0;
  req.spline.segments[1].a[4] = 0.0;
  req.spline.segments[1].a[5] = 0.1;
  req.spline.segments[1].a[6] = 0.0;

  req.spline.segments[1].b[0] = -0.25;
  req.spline.segments[1].b[1] = 0.0;
  req.spline.segments[1].b[2] = 0.0;
  req.spline.segments[1].b[3] = 0.0;
  req.spline.segments[1].b[4] = 0.0;
  req.spline.segments[1].b[5] = 0.0;
  req.spline.segments[1].b[6] = 0.0;

  req.spline.segments[1].duration = ros::Time(4.0);


  if (ros::service::call("/trajectory_controller/SetSplineTrajectory", req, res))
  {
    ROS_INFO("Done calling service, response id: %d",res.trajectory_id);
  }

  req_q.trajectory_id = res.trajectory_id;
  rec_req.trajectory_id = res.trajectory_id;

  usleep(1e4);  
  if(ros::service::call("/trajectory_controller/CancelSplineTrajectory", rec_req, rec_res))  
  {
    ROS_INFO("Trying to cancel the trajectory");
  }
  else
  {
    ROS_INFO("service call failed");
  }

  usleep(1e4);

  if (ros::service::call("/trajectory_controller/SetSplineTrajectory", req, res))
  {
    ROS_INFO("Done calling service, response id: %d",res.trajectory_id);
  }

  req_q.trajectory_id = res.trajectory_id;

  if(ros::service::call("/trajectory_controller/QuerySplineTrajectory", req_q, res_q))  
  {
    while(res_q.trajectory_status != res_q.State_Done)
    {
      ros::service::call("/trajectory_controller/QuerySplineTrajectory", req_q, res_q);
      usleep(1e5);
    }  
    ROS_INFO("response 3:: id:%d, response:%d",req_q.trajectory_id,res_q.trajectory_status);
  }
  else
  {
    ROS_INFO("service call failed");
  }


/*  if(ros::service::call("r_arm_joint_trajectory_controller/TrajectoryQuery", req_q, res_q))  
  {
    while(res_q.done != res_q.State_Deleted && res_q.done != res_q.State_Canceled)
    {
      ros::service::call("r_arm_joint_trajectory_controller/TrajectoryQuery", req_q, res_q);
      usleep(1e5);
    }
  
    ROS_INFO("response 1:: id:%d, time:%f, response:%d",req_q.trajectoryid,res_q.trajectorytime,res_q.done);
  }
  else
  {
    ROS_INFO("service call failed");
  }

  usleep(1e4);
  req_q.trajectoryid =  pr2_mechanism_controllers::TrajectoryQuery::Request::Query_Joint_Names;

  if(ros::service::call("r_arm_joint_trajectory_controller/TrajectoryQuery", req_q, res_q))  
  {
    ROS_INFO("response 2:: %f, %d",res_q.trajectorytime,res_q.done);
  }
  else
  {
    ROS_INFO("service call failed");
    for(int i=0; i < (int) res_q.jointnames.size(); i++)
    {
      ROS_INFO("Joint name: %s", res_q.jointnames[i].c_str());
    }      
  }

  if (ros::service::call("r_arm_joint_trajectory_controller/TrajectoryStart", req, res))
  {
    ROS_INFO("Done");
  }

  req_q.trajectoryid = res.trajectoryid;
  rec_req.trajectoryid = res.trajectoryid;

  sleep(4.0);
  
  if(ros::service::call("r_arm_joint_trajectory_controller/TrajectoryCancel", rec_req, rec_res))  
  {
  }
  else
  {
    ROS_INFO("service call failed");
  }

  if(ros::service::call("r_arm_joint_trajectory_controller/TrajectoryQuery", req_q, res_q))  
  {
    while(res_q.done != res_q.State_Deleted && res_q.done != res_q.State_Canceled)
    {
      ros::service::call("r_arm_joint_trajectory_controller/TrajectoryQuery", req_q, res_q);
      usleep(1e5);
    }  
    ROS_INFO("response 3:: id:%d, time:%f, response:%d",req_q.trajectoryid,res_q.trajectorytime,res_q.done);
  }
  else
  {
    ROS_INFO("service call failed");
  }
*/
}
