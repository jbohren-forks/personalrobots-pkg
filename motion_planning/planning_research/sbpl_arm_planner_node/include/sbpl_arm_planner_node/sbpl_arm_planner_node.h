/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __SBPL_ARM_PLANNER_NODE_H_
#define __SBPL_ARM_PLANNER_NODE_H_

#include <iostream> 

/** ROS **/
#include <ros/node.h>

/** TF **/
#include <tf/tf.h>

/** Messages needed for trajectory control and collision map**/
#include <robot_msgs/Pose.h>
#include <robot_msgs/JointTraj.h>
#include <robot_msgs/CollisionMap.h>
#include <robot_msgs/JointTrajPoint.h>

/** sbpl planner include files **/
#include <sbpl_arm_planner/headers.h>

/** services **/
#include <sbpl_arm_planner_node/PlanPathSrv.h>

namespace sbpl_arm_planner_node
{
#define VERBOSE 1
#define MAX_RUNTIME 60.0

   class SBPLArmPlannerNode : public ros::Node
   {
     public:

      SBPLArmPlannerNode(std::string node_name);

      ~SBPLArmPlannerNode();

      bool planPath(sbpl_arm_planner_node::PlanPathSrv::Request &req, sbpl_arm_planner_node::PlanPathSrv::Response &resp);

     private:

      double torso_arm_offset_x_;

      double torso_arm_offset_y_;

      double torso_arm_offset_z_;

      double allocated_time_;

      bool forward_search_;

      bool search_mode_;

      int num_joints_;
    
      std::string collision_map_topic_;

      std::string node_name_;

      robot_msgs::CollisionMap collision_map_;

      MDPConfig mdp_cfg_;

      EnvironmentROBARM3D pr2_arm_env_;

      ARAPlanner *planner_;

      FILE *env_config_fp_;

      FILE *planner_config_fp_;

      bool initializePlannerAndEnvironment();

      bool setStart(const robot_msgs::JointTrajPoint &start);

      bool setGoals(const std::vector<robot_msgs::Pose> &goals);

      bool replan(robot_msgs::JointTraj &arm_path);

      void collisionMapCallback();

   };
}

#endif
