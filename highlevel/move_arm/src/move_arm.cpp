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

using namespace robot_actions;

namespace move_arm {

  MoveArm::MoveArm() : Action<pr2_robot_actions::MoveArmGoal, int32_t>("move_arm")
  {

  }
  MoveArm::~MoveArm()
  {

  }

  robot_actions::ResultStatus MoveArm::execute(const pr2_robot_actions::MoveArmGoal& goal, int32_t& feedback)
  {
    
    return robot_actions::SUCCESS;
  }

  bool MoveArm::computeIK(const robot_msgs::PoseStamped &pose_stamped_msg, std::vector<std::pair<std::string, double> > &solution)
  {
    // define the service messages
  pr2_ik::IKService::Request request;
  pr2_ik::IKService::Response response;

  ros::ServiceClient client = node_handle_.serviceClient<pr2_ik::IKService>("pr2_ik");
  if (client.call(req, res))
      plan_id_ = res.id;
  else
      ROS_ERROR("Service 'plan_kinematic_path' failed");

    tf::Stamped<tf::Pose> pose_stamped_tf;
    KDL::Frame pose_kdl;
    PoseStampedMsgToTF(pose_stamped_msg, pose_stamped_tf);
    ROS_DEBUG("Converted pose command to tf");

    // convert to reference frame of root link of the chain
    tf_.transformPose(root_name_ik_, pose_stamped_tf, pose_stamped_tf);
    ROS_DEBUG("Converted tf command to root name");

    poseToFrame(pose_stamped, pose_kdl);
    ROS_DEBUG("Converted tf command to KDL");

    //Do the IK
    KDL::JntArray jnt_pos_in;
    KDL::JntArray jnt_pos_out;
    jnt_pos_in.resize(arm_number_joints_);
    jnt_pos_in(pr2_ik_solver_.pr2_ik_->free_angle_) = 0.0; //TODO - specify this in a better way
    bool ik_valid = (pr2_ik_solver_.CartToJntSearch(jnt_pos_in,pose_kdl_,jnt_pos_out,10) >= 0);

    if(ik_valid)
    {
      solution.resize(arm_number_joints_);
      for(int i=0; i< arm_number_joints_; i++)
	{
	  solution[i].first = arm_joint_names_[i];
	  solution[i].second = jnt_pos_out(i);
	}
      return true;
    }

      ROS_ERROR("IK invalid");   
      return false;
  }


};

int main(int argc, char** argv){
  ros::init(argc, argv, "move_arm");
  
  move_arm::MoveArm move_arm;
  robot_actions::ActionRunner runner(20.0);
  runner.connect<pr2_robot_actions::MoveArmGoal, pr2_robot_actions::MoveArmState, int32_t>(move_arm);
  runner.run();

  ros::spin();

  return(0);

}
