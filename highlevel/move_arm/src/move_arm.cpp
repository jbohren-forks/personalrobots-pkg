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
*  POSSIBILITY OF SUCH DAMAGE.    return robot_actions::PREEMPTED;

*
* Authors: Sachin Chitta, Ioan Sucan
*********************************************************************/
#include <move_arm/move_arm.h>
#include <robot_msgs/JointTraj.h>
#include <pr2_mechanism_controllers/TrajectoryStart.h>
#include <pr2_mechanism_controllers/TrajectoryQuery.h>


using namespace robot_actions;

namespace move_arm 
{
  MoveArm::MoveArm() : Action<pr2_robot_actions::MoveArmGoal, int32_t>("move_arm")
  {
    node_handle_.param<int>("~arm_number_joints", arm_number_joints_,7);
    node_handle_.param<std::string>("~ik_service_name", ik_service_name_,"pr2_ik");
    node_handle_.param<std::string>("~ik_query_name", ik_query_name_, "pr2_query");
    node_handle_.param<std::string>("~control_topic_name",control_service_name_,"r_arm_joint_trajectory_controller/TrajectoryStart");
    node_handle_.param<std::string>("~control_query_name",control_query_name_,"r_arm_joint_trajectory_controller/TrajectoryQuery");
  }

  MoveArm::~MoveArm()
  {
  }

  robot_actions::ResultStatus MoveArm::execute(const pr2_robot_actions::MoveArmGoal& goal, int32_t& feedback)
  { 
    robot_msgs::PoseStamped arm_cartesian_goal = goal.goal_constraints.pose_constraint[0].pose;
    std::vector<std::pair<std::string, double> > solution;
    if(!computeIK(arm_cartesian_goal,solution))
    {
      ROS_ERROR("MoveArm:: IK failed");
      return robot_actions::ABORTED;
    }
    if(!sendControl(solution))
    {
      ROS_ERROR("MoveArm:: Control failed");
      return robot_actions::ABORTED;
    }
       
    return robot_actions::SUCCESS;
  }

  bool MoveArm::sendControl(const std::vector<std::pair<std::string, double> > &solution)
  {
    robot_msgs::JointTraj joint_traj;
    joint_traj.set_points_size(1);
    joint_traj.points[0].set_positions_size(solution.size());

    for(int j =0; j < (int) solution.size(); j++)
    {
      joint_traj.points[0].positions[j] = solution[j].second;
    }

    ros::ServiceClient client = node_handle_.serviceClient<pr2_mechanism_controllers::TrajectoryStart>(control_service_name_);
    pr2_mechanism_controllers::TrajectoryStart::Request req;
    pr2_mechanism_controllers::TrajectoryStart::Response res;
    req.traj = joint_traj;
    req.requesttiming = 1;

    ROS_DEBUG("MoveArm:: Sending out control");
    if(!client.call(req,res))
      return false;
    return true;
  }

  bool MoveArm::getControlJointNames(std::vector<std::string> &joint_names)
  {
    ros::ServiceClient client_query = node_handle_.serviceClient<pr2_mechanism_controllers::TrajectoryQuery>(control_query_name_);
    pr2_mechanism_controllers::TrajectoryQuery::Request req_query;
    pr2_mechanism_controllers::TrajectoryQuery::Response res_query;
    req_query.trajectoryid  = -1;

    if(!client_query.call(req_query,res_query))
      return false;
    joint_names.resize(res_query.get_jointnames_size());

    for(int i=0; i < (int)res_query.get_jointnames_size(); i++)
    {
      joint_names[i] = res_query.jointnames[i];
    }
    return true;
  }

  bool MoveArm::computeIK(const robot_msgs::PoseStamped &pose_stamped_msg, std::vector<std::pair<std::string, double> > &solution)
  {
    // define the service messages
    manipulation_srvs::IKService::Request request;
    manipulation_srvs::IKService::Response response;

    request.data.pose_stamped = pose_stamped_msg;
    request.data.set_positions_size(arm_number_joints_);
    request.data.set_joint_names_size(arm_number_joints_);

    for(int i=0; i < arm_number_joints_; i++)
    {
      request.data.positions[i] = 0.0; // TODO - FILL THIS UP
    } 
    // BIG TODO - FILL THESE UP 
    request.data.joint_names[0] = "r_shoulder_pan_joint";
    request.data.joint_names[1] = "r_shoulder_lift_joint";
    request.data.joint_names[2] = "r_upper_arm_roll_joint";

    request.data.joint_names[3] = "r_elbow_flex_joint";
    request.data.joint_names[4] = "r_forearm_roll_joint";
    request.data.joint_names[5] = "r_wrist_flex_joint";
    request.data.joint_names[6] = "r_wrist_roll_joint";

    ros::ServiceClient client = node_handle_.serviceClient<manipulation_srvs::IKService>(ik_service_name_);

    if (client.call(request, response))
    { 
      ROS_DEBUG("MoveArm:: Got IK solution");  
      solution.resize(arm_number_joints_);
      for(int i=0; i< arm_number_joints_; i++)
      {
        solution[i].first = request.data.joint_names[i];
        solution[i].second = response.solution[i];
        ROS_DEBUG("Joint %s: %f",solution[i].first.c_str(),solution[i].second);
      }
      return true;      
    }
    else
    {
      ROS_ERROR("MoveArm:: Service %s failed",ik_service_name_.c_str());
      return false;
    }
  }
};

int main(int argc, char** argv){
//  ros::init(argc, argv, "move_arm");  
  ros::init(argc,argv);
  ros::Node ros_node("move_arm");

  move_arm::MoveArm move_arm;
  robot_actions::ActionRunner runner(20.0);
  runner.connect<pr2_robot_actions::MoveArmGoal, pr2_robot_actions::MoveArmState, int32_t>(move_arm);
  runner.run();
  ros::spin();
  return(0);

}
