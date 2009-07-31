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


#include "mturk_grab_object/grasp_table_object_node.h"
#include "motion_planning_msgs/PoseConstraint.h"

using namespace std;
using namespace robot_msgs;
using namespace mturk_grab_object;

GraspObjectNode::GraspObjectNode():move_arm("move_right_arm"){}

void GraspObjectNode::setup()
{
  detect_object_client_=n_.serviceClient<tabletop_srvs::FindTable>("table_object_detector");

  image_sub_=n_.subscribe<sensor_msgs::Image>("image",1,boost::bind(&GraspObjectNode::imageCallback,this,_1));

  n_.param("~is_test",is_test_,false);

  num_skip_images_=10;
  num_already_skipped_images_=0;  

  default_arm_pose.position.x =  0.109557;
  default_arm_pose.position.y = -0.978815;
  default_arm_pose.position.z =  0.892993;

  default_arm_pose.orientation.x =  -0.0770335;
  default_arm_pose.orientation.y =   0.235906;
  default_arm_pose.orientation.z =  -0.570976;
  default_arm_pose.orientation.w =   0.78256;

  num_attempts=0;
  num_success=0;
}


void setupGoal(const std::string &link, const robot_msgs::Pose &pz, pr2_robot_actions::MoveArmGoal &goal)
{
  goal.goal_constraints.pose_constraint.resize(1);
  goal.goal_constraints.pose_constraint[0].type = motion_planning_msgs::PoseConstraint::POSITION_X + motion_planning_msgs::PoseConstraint::POSITION_Y + motion_planning_msgs::PoseConstraint::POSITION_Z +
    + motion_planning_msgs::PoseConstraint::ORIENTATION_R + motion_planning_msgs::PoseConstraint::ORIENTATION_P + motion_planning_msgs::PoseConstraint::ORIENTATION_Y;
  goal.goal_constraints.pose_constraint[0].link_name = link;
  goal.goal_constraints.pose_constraint[0].pose.header.stamp = ros::Time::now();
  goal.goal_constraints.pose_constraint[0].pose.header.frame_id = "/base_link";
  goal.goal_constraints.pose_constraint[0].pose.pose = pz;
  ROS_INFO_STREAM("pose: " << pz.position.x << pz.position.y << pz.position.z);
  goal.goal_constraints.pose_constraint[0].position_tolerance_above.x = 0.01;
  goal.goal_constraints.pose_constraint[0].position_tolerance_above.y = 0.01;
  goal.goal_constraints.pose_constraint[0].position_tolerance_above.z = 0.015;
  goal.goal_constraints.pose_constraint[0].position_tolerance_below.x = 0.01;
  goal.goal_constraints.pose_constraint[0].position_tolerance_below.y = 0.01;
  goal.goal_constraints.pose_constraint[0].position_tolerance_below.z = 0.015;

  goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.x = 0.15;
  goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.y = 0.15;
  goal.goal_constraints.pose_constraint[0].orientation_tolerance_above.z = 0.15;
  goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.x = 0.15;
  goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.y = 0.15;
  goal.goal_constraints.pose_constraint[0].orientation_tolerance_below.z = 0.15;

  goal.goal_constraints.pose_constraint[0].orientation_importance = 0.01;
}

void GraspObjectNode::imageCallback(const sensor_msgs::ImageConstPtr& the_image)
{
  ROS_INFO("image");

  if(num_skip_images_<num_already_skipped_images_)
    {
      num_already_skipped_images_=0;
      if(active_task.get())
	{
	  if(active_task->isActive())
	    {
	      ROS_INFO("Pursuing previous task");
	      return;
	    }
	}

      tabletop_srvs::FindTable::Request req;
      tabletop_srvs::FindTable::Response resp;

      if(!detect_object_client_.call(req,resp))
	{
	  ROS_WARN("detect object service failed");
	  return;
	}      

      ROS_INFO_STREAM("Objects detected " << resp.table.objects.size());
      if(resp.table.objects.size()==0)
	{
	  ROS_WARN("No objects");
	  return;
	}
      active_task=boost::shared_ptr<GraspTask>(new GraspTask);
      active_task->image_=the_image;
      active_task->state = GraspTask::CREATED; 
      active_task->active_object_=resp.table.objects[0];

      Pose grasp_pose;
      computeGraspPose(active_task->active_object_,grasp_pose);

      Point32 minBnd=active_task->active_object_.min_bound;
      Point32 maxBnd=active_task->active_object_.max_bound;
      ROS_INFO_STREAM("dX: "<< maxBnd.x-minBnd.x << ", dY: " << maxBnd.y-minBnd.y <<", dZ: "<<maxBnd.z-minBnd.z);
      std::string link_name("r_wrist_roll_link");


      num_attempts++;

      pr2_robot_actions::MoveArmGoal g;
      double allowed_time=30.0;
      int32_t                         feedback;

      bool bOK=true;
      setupGoal(link_name, grasp_pose, g);
      if (move_arm.execute(g, feedback, ros::Duration(allowed_time)) != robot_actions::SUCCESS)
	{
	  std::cerr << "Failed achieving goal" << std::endl;
	  bOK=false;
	}
      else
	{
	  std::cout << "Success!" << std::endl;
	  ros::Duration d(5.0);
	  d.sleep();
	}


      setupGoal(link_name, default_arm_pose, g);
      if (move_arm.execute(g, feedback, ros::Duration(allowed_time)) != robot_actions::SUCCESS)      
	{
	  bOK=false;
	  std::cerr << "Failed achieving DEFAULT GOAL" << std::endl;
	}
      else
	std::cout << "Success for default goal." << std::endl;

      if(bOK)
	  num_success++;

      ROS_INFO_STREAM(" Success rate = " << ((double)num_success)/((double)num_attempts) << "(" << num_success << " out of " << num_attempts << ")");

    }
  else
    {
      num_already_skipped_images_++;
    }
}



void GraspObjectNode::computeGraspPose(const tabletop_msgs::ObjectOnTable& object_on_table, robot_msgs::Pose& grasp_pose)
{
  Point32 center=object_on_table.center;
  Point32 minBnd=object_on_table.min_bound;
  Point32 maxBnd=object_on_table.max_bound;

  grasp_pose.position.x =  center.x;
  grasp_pose.position.y =  minBnd.y-0.30;
  grasp_pose.position.z =  maxBnd.z-0.05;

  grasp_pose.orientation.x =  -0.00409821;
  grasp_pose.orientation.y =  0.123473;
  grasp_pose.orientation.z =  0.853525;
  grasp_pose.orientation.w =  0.506194;

  grasp_pose.orientation.x =  0.680547;
  grasp_pose.orientation.y =  0.696198;
  grasp_pose.orientation.z =  -0.195794;
  grasp_pose.orientation.w =  0.117598;
  
}


int main(int argc, char **argv)
{
  ros::init(argc, argv,"novelty_estimator_node");

  try
  {
    GraspObjectNode gn;
    gn.setup();

    ros::MultiThreadedSpinner spinner;
    ros::spin(spinner);
  }
  catch(std::runtime_error& e)
  {
    fprintf(stderr, "%s\n", e.what());
  }
  
  return 0;
}
