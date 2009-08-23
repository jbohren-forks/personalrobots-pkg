/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_calibration_actions/CaptureRobotPixelsAction.h>
#include <pr2_calibration_actions/robot_pixels_capture.h>
#include <pr2_calibration_actions/GetRobotPixelsConfig.h>

#include <pr2_mechanism_controllers/TrajectoryStart.h>
#include <pr2_mechanism_controllers/TrajectoryQuery.h>

#include <move_arm/MoveArmAction.h>

using namespace pr2_calibration_actions;

void activeCaptureCallback()
{
  ROS_INFO("Capture Request transitioned to ACTIVE!");
}

void feedbackCaptureCallback(const CaptureRobotPixelsFeedbackConstPtr& feedback)
{
  printf("%s:\n", feedback->joint_states_channel.channel_name.c_str());
  for (unsigned int i=0; i<feedback->joint_states_channel.ranges.size(); i++)
  {
    printf("  ranges[%2u]: %5.3f\n", i, feedback->joint_states_channel.ranges[i]);
  }

  for (unsigned int i=0; i<feedback->pixel_channels.size(); i++)
  {
    printf("%s:\n", feedback->pixel_channels[i].channel_name.c_str());
    for (unsigned int j=0; j<feedback->pixel_channels[i].ranges.size(); j++)
    {
      printf("  ranges[%2u]: %5.3f\n", i, feedback->pixel_channels[i].ranges[j]);
    }
  }
}

void doneCaptureCallback(const actionlib::TerminalState& terminal_state, const CaptureRobotPixelsResultConstPtr& result)
{
  ROS_INFO("Capture DONE! With terminal_state [%s]", terminal_state.toString().c_str());
}

void activeArmCallback()
{
  ROS_INFO("Arm Request transitioned to ACTIVE!");
}


void feedbackArmCallback(const move_arm::MoveArmFeedbackConstPtr& feedback)
{
  ROS_INFO("Got move arm feedback");
}

void doneArmCallback(const actionlib::TerminalState& terminal_state, const move_arm::MoveArmResultConstPtr& result)
{
  ROS_INFO("Arm DONE! With terminal_state [%s]", terminal_state.toString().c_str());
}

void execThread()
{
  ros::NodeHandle nh;

  actionlib::SimpleActionClient<move_arm::MoveArmAction> arm_client(nh, "move_right_arm");
  actionlib::SimpleActionClient<CaptureRobotPixelsAction> capture_client(nh, "capture_robot_pixels");
  ros::ServiceClient client = nh.serviceClient<GetRobotPixelsConfig>("get_robot_pixels_config");
  ros::ServiceClient traj_client  = nh.serviceClient<pr2_mechanism_controllers::TrajectoryStart>("right_arm_trajectory_controller/TrajectoryStart");
  ros::ServiceClient query_client = nh.serviceClient<pr2_mechanism_controllers::TrajectoryQuery>("right_arm_trajectory_controller/TrajectoryQuery");

  for (unsigned int cur_command_num=0; cur_command_num<3; cur_command_num++)
  {
    GetRobotPixelsConfig srv;
    srv.request.command_num = cur_command_num;
    if (client.call(srv))
      ROS_INFO("Successfully called config server");
    else
      ROS_FATAL("Error calling config server");

    const unsigned int N = srv.response.config.joint_states_config.joint_names.size();
    /*move_arm::MoveArmGoal arm_goal;
    arm_goal.goal_constraints.joint_constraint.resize( N );
    ROS_ERROR_COND(srv.response.command.size() != N, "N=%u, but command.size()=%u", N, srv.response.command.size());
    ROS_ERROR_COND(srv.response.config.joint_states_config.joint_names.size() != N, "N=%u, but joint_names.size()=%u",
                   N, srv.response.config.joint_states_config.joint_names.size());

    for (unsigned int i=0; i<N; i++)
    {
      arm_goal.goal_constraints.joint_constraint[i].value.resize(1);
      arm_goal.goal_constraints.joint_constraint[i].tolerance_above.resize(1);
      arm_goal.goal_constraints.joint_constraint[i].tolerance_below.resize(1);
      arm_goal.goal_constraints.joint_constraint[i].tolerance_above[0] = 0.001;
      arm_goal.goal_constraints.joint_constraint[i].tolerance_below[0] = 0.001;

      arm_goal.goal_constraints.joint_constraint[i].joint_name = srv.response.config.joint_states_config.joint_names[i];
      arm_goal.goal_constraints.joint_constraint[i].value[0]   = srv.response.command[i];
    }
    */

    pr2_mechanism_controllers::TrajectoryStart traj_srv;
    traj_srv.request.traj.names = srv.response.config.joint_states_config.joint_names;
    traj_srv.request.traj.points.resize(1);
    traj_srv.request.traj.points[0].positions.resize(N);
    for (unsigned int i=0; i<N; i++)
      traj_srv.request.traj.points[0].positions[i] = srv.response.command[i];
    traj_srv.request.hastiming = 0;
    traj_srv.request.requesttiming = 1;
    if (traj_client.call(traj_srv))
      ROS_INFO("Successfully called arm traj server");
    else
      ROS_FATAL("Error calling arm traj");

    bool waiting = true;
    pr2_mechanism_controllers::TrajectoryQuery query_srv;
    while (waiting)
    {
      query_srv.request.trajectoryid = traj_srv.response.trajectoryid;

      if (!query_client.call(query_srv))
        ROS_ERROR("Error querying trajectory");

      waiting = (query_srv.response.done == pr2_mechanism_controllers::TrajectoryQuery::Response::State_Active ||
                 query_srv.response.done == pr2_mechanism_controllers::TrajectoryQuery::Response::State_Queued);
      ros::Duration(.25).sleep();
    }
    ROS_INFO("Done waiting for arm. Status: %u", query_srv.response.done);

    CaptureRobotPixelsGoal capture_goal;

    /*ROS_INFO("Waiting 10s for [move_rarm] to come up");
    sleep(1);
    if (arm_client.waitForActionServerToStart(ros::Duration(10,0)))
      ROS_INFO("[move_rarm] started");
    else
    {
      ROS_ERROR("[move_rarm] is not started");
      return;
    }*/

    //arm_client.sendGoal(arm_goal, &doneArmCallback, &activeArmCallback, &feedbackArmCallback);

    /*ROS_INFO("Waiting 20s for arm command to finish");
    if (arm_client.waitForGoalToFinish(ros::Duration(20, 0)))
      ROS_INFO("Arm finished with status [%s]", arm_client.getTerminalState().toString().c_str());
    else
    {
      ROS_ERROR("Arm timed out");
      return;
    }*/


    /*
    ROS_INFO("Waiting Capture action to come up");
    if (capture_client.waitForActionServerToStart(ros::Duration(10,0)))
      ROS_INFO("Capture action came up!");
    else
    {
      ROS_INFO("Capture action is not started");
      return;
    }


    capture_client.sendGoal(srv.response.config, &doneCaptureCallback, &activeCaptureCallback, &feedbackCaptureCallback);
    if (capture_client.waitForGoalToFinish(ros::Duration(20, 0)))
      ROS_INFO("Capture finished with status [%s]", capture_client.getTerminalState().toString().c_str());
    else
      ROS_ERROR("Capture timed out");
      */
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_pixel_capture_node");

  ros::NodeHandle nh;

  boost::thread exec_thread(&execThread);

  ros::spin();

  //exec_thread.join();

  return 0;
}


