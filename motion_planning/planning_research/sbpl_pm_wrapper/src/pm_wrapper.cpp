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

/** \Author: Benjamin Cohen  **/

#include <sbpl_pm_wrapper/pm_wrapper.h>

using namespace sbpl_arm_planner_node;

pm_wrapper::pm_wrapper()
{
  node_.param<std::string>("~planning_frame_id", frame_, "base_link");
  node_.param<std::string>("~robot_description", robot_description_,"robotdesc/pr2");
  node_.param<std::string>("~arm", arm_name_,"right_arm");
};

pm_wrapper::~pm_wrapper()
{
  delete collision_model_;
  delete planning_monitor_;
};

/** \brief initialize the planning monitor - choose the links to be checked for collision, set the groupID (must be called first) */
bool pm_wrapper::initPlanningMonitor(const std::vector<std::string> &links, tf::TransformListener * tfl)
{
  collision_check_links_ = links;
  collision_model_ = new planning_environment::CollisionModels(robot_description_, collision_check_links_);
  planning_monitor_ = new planning_environment::PlanningMonitor(collision_model_, tfl);

  groupID_ = planning_monitor_->getKinematicModel()->getGroupID(arm_name_);

  std::vector<std::string> joint_names;
  planning_monitor_->getKinematicModel()->getJointsInGroup(joint_names, groupID_);
  if(joint_names.empty())
  {
    ROS_WARN("[pm_wrapper] joint group %i is empty", groupID_);
    return false;
  }
  ROS_INFO("[pm_wrapper] joint names in group #%i",groupID_);
  for(unsigned int i = 0; i < joint_names.size(); i++)
    ROS_INFO("[pm_wrapper]%i: %s", i, joint_names[i].c_str());

  return true;
}

/** \brief fill the start state of the planning monitor with the current state of the robot in the request message */
planning_models::StateParams* pm_wrapper::fillStartState(const std::vector<motion_planning_msgs::KinematicJoint> &given)
{
  planning_models::StateParams *s = planning_monitor_->getKinematicModel()->newStateParams();
  for (unsigned int i = 0 ; i < given.size() ; ++i)
  {	
    if (!planning_monitor_->getTransformListener()->frameExists(given[i].header.frame_id))
    {
      ROS_ERROR("Frame '%s' for joint '%s' in starting state is unknown", given[i].header.frame_id.c_str(), given[i].joint_name.c_str());
      continue;
    }
    motion_planning_msgs::KinematicJoint kj = given[i];
    if (planning_monitor_->transformJointToFrame(kj, planning_monitor_->getFrameId()))
      s->setParamsJoint(kj.value, kj.joint_name);
  }

  if (s->seenAll())
    return s;
  else
  {
    if (planning_monitor_->haveState())
    {
      ROS_INFO("Using the current state to fill in the starting state for the motion plan");
      std::vector<planning_models::KinematicModel::Joint*> joints;
      planning_monitor_->getKinematicModel()->getJoints(joints);
      for (unsigned int i = 0 ; i < joints.size() ; ++i)
	if (!s->seenJoint(joints[i]->name))
	  s->setParamsJoint(planning_monitor_->getRobotState()->getParamsJoint(joints[i]->name), joints[i]->name);
      return s;
    }
  }
  delete s;
  ROS_WARN("fillstartState is returning NULL");
  return NULL;
}

/** \brief update the planning monitor with the current robot state and lock it (call right before planning) */
void pm_wrapper::updatePM(const motion_planning_srvs::MotionPlan::Request &req)
{
  start_state_ = fillStartState(req.start_state);

  if(start_state_ == NULL)
    ROS_WARN("start_state_ == NULL");

  planning_monitor_->getKinematicModel()->computeTransforms(start_state_->getParams());
  planning_monitor_->getEnvironmentModel()->updateRobotModel();

  planning_monitor_->getEnvironmentModel()->lock();
  planning_monitor_->getKinematicModel()->lock();

  ROS_INFO("updatedPM()");
}

/** \brief check if links in collision_check_links_ are free of collision */
bool pm_wrapper::areLinksValid(const double * angles)
{
//   ROS_INFO("checking %.3f %.3f %.3f %.3f %.3f %.3f %.3f",angles[0],angles[1],angles[2],angles[3],angles[4],angles[5],angles[6]);

  planning_monitor_->getKinematicModel()->computeTransformsGroup(angles, groupID_);
  planning_monitor_->getEnvironmentModel()->updateRobotModel();

  return !(planning_monitor_->getEnvironmentModel()->isCollision());
}

/** \brief unlock the planning monitor mutexes (call after planning completes) */
void pm_wrapper::unlockPM()
{
  // after done planning
  planning_monitor_->getEnvironmentModel()->unlock();
  planning_monitor_->getKinematicModel()->unlock();
}
