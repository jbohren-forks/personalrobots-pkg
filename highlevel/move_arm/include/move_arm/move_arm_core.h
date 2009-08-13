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

 *
 * Authors: Sachin Chitta, Ioan Sucan
 *********************************************************************/

#ifndef MOVE_ARM_MOVE_ARM_CORE_H_
#define MOVE_ARM_MOVE_ARM_CORE_H_

#include <ros/ros.h>

#include <pr2_mechanism_controllers/TrajectoryStart.h>
#include <pr2_mechanism_controllers/TrajectoryQuery.h>
#include <pr2_mechanism_controllers/TrajectoryCancel.h>

#include <planning_environment/monitors/planning_monitor.h>

namespace move_arm
{

/// the string used internally to access control starting service; this should be remaped in the launch file
static const std::string CONTROL_START_NAME      = "controller_start";

/// the string used internally to access control querying service; this should be remaped in the launch file
static const std::string CONTROL_QUERY_NAME      = "controller_query";

/// the string used internally to access control canceling service; this should be remaped in the launch file
static const std::string CONTROL_CANCEL_NAME     = "controller_cancel";

/// the string used internally to access the long range motion planning service; this should be remaped in the launch file
static const std::string LR_MOTION_PLAN_NAME     = "get_motion_plan_lr";

/// the string used internally to access the short range motion planning service; this should be remaped in the launch file
static const std::string SR_MOTION_PLAN_NAME     = "get_motion_plan_sr";

/// the string used internally to access valid state searching service; this should be remaped in the launch file
static const std::string SEARCH_VALID_STATE_NAME = "get_valid_state";

/// the string used internally to access inverse kinematics service; this should be remaped in the launch file
static const std::string ARM_IK_NAME             = "arm_ik";


/** \brief Configuration of actions that need to actuate parts of the robot */
class MoveArmCore
{
  friend class MoveArm;

public:

  MoveArmCore(void)
  {
    collisionModels_ = NULL;
    planningMonitor_ = NULL;
  }

  virtual ~MoveArmCore(void)
  {
    if (planningMonitor_)
      delete planningMonitor_;
    if (collisionModels_)
      delete collisionModels_;
  }


  bool configure(void)
  {
    nodeHandle_.param<std::string>("~group", group_, std::string());

    if (group_.empty())
    {
      ROS_ERROR("No '~group' parameter specified. Without the name of the group of joints to plan for, action cannot start");
      return false;
    }

    // monitor robot
    collisionModels_ = new planning_environment::CollisionModels("robot_description");
    planningMonitor_ = new planning_environment::PlanningMonitor(collisionModels_, &tf_);

    if (!collisionModels_->loadedModels())
      return false;

    nodeHandle_.param<bool>("~perform_ik", perform_ik_, true);

    if (collisionModels_->getKinematicModel()->getGroupID(group_) < 0)
    {
      ROS_ERROR("Group '%s' is not known", group_.c_str());
      return false;
    }
    else
      ROS_INFO("Configuring action core for '%s' (IK is %senabled)", group_.c_str(), perform_ik_ ? "" : "not ");

    planningMonitor_->waitForState();
    planningMonitor_->waitForMap();

    if (!getControlJointNames(groupJointNames_))
      return false;

    nodeHandle_.param<bool>("~show_collisions", show_collisions_, false);

    if (show_collisions_)
      ROS_INFO("Found collisions will be displayed as visualization markers");

    return true;
  }

protected:

  bool getControlJointNames(std::vector<std::string> &joint_names)
  {
    ros::ServiceClient client_query = nodeHandle_.serviceClient<pr2_mechanism_controllers::TrajectoryQuery>(CONTROL_QUERY_NAME);
    pr2_mechanism_controllers::TrajectoryQuery::Request  req_query;
    pr2_mechanism_controllers::TrajectoryQuery::Response res_query;
    req_query.trajectoryid = pr2_mechanism_controllers::TrajectoryQuery::Request::Query_Joint_Names;

    bool result = client_query.call(req_query, res_query);

    if (!result)
    {
      ROS_INFO("Querying controller for joint names ...");
      ros::Duration(5.0).sleep();
      result = client_query.call(req_query, res_query);
      if (result)
        ROS_INFO("Joint names received");
    }

    if (!result)
    {
      ROS_ERROR("Unable to retrieve controller joint names from control query service");
      return false;
    }

    joint_names = res_query.jointnames;

    // make sure we have the right joint names
    for(unsigned int i = 0; i < joint_names.size() ; ++i)
    {
      if (planning_models::KinematicModel::Joint *j = planningMonitor_->getKinematicModel()->getJoint(joint_names[i]))
      {
        ROS_DEBUG("Using joing '%s' with %u parameters", j->name.c_str(), j->usedParams);
        if (planningMonitor_->getKinematicModel()->getJointIndexInGroup(j->name, group_) < 0)
          return false;
      }
      else
      {
        ROS_ERROR("Joint '%s' is not known", joint_names[i].c_str());
        return false;
      }
    }

    std::vector<std::string> groupNames;
    planningMonitor_->getKinematicModel()->getJointsInGroup(groupNames, group_);
    if (groupNames.size() != joint_names.size())
    {
      ROS_ERROR("The group '%s' does not have the same number of joints as the controller can handle", group_.c_str());
      return false;
    }

    return true;
  }

  ros::NodeHandle                        nodeHandle_;
  tf::TransformListener                  tf_;
  planning_environment::CollisionModels *collisionModels_;
  planning_environment::PlanningMonitor *planningMonitor_;

  std::string                            group_;
  std::vector<std::string>               groupJointNames_;
  bool                                   perform_ik_;
  bool                                   show_collisions_;

};

}

#endif
