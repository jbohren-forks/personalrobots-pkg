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

 /** \author Benjamin Cohen, Sachin Chitta  */

#ifndef __SBPL_ARM_PLANNER_NODE_H_
#define __SBPL_ARM_PLANNER_NODE_H_

#include <iostream>

/** ROS **/
#include <ros/ros.h>
#include "ros/node_handle.h"

/** TF **/
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_notifier.h>
#include <tf/transform_datatypes.h>

/** Messages needed for trajectory control and collision map**/
#include <robot_msgs/Pose.h>
#include <manipulation_msgs/JointTraj.h>
#include <mapping_msgs/CollisionMap.h>
#include <manipulation_msgs/JointTrajPoint.h>
#include <motion_planning_msgs/KinematicPath.h>
#include <motion_planning_msgs/KinematicState.h>
#include <motion_planning_msgs/PoseConstraint.h>
#include <mechanism_msgs/MechanismState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/** sbpl planner include files **/
#include <sbpl_arm_planner/headers.h>

/** services **/
#include <sbpl_arm_planner_node/PlanPathSrv.h>
#include <motion_planning_srvs/MotionPlan.h>

#include <boost/thread/mutex.hpp>
#include "boost/shared_ptr.hpp"

// #include <sbpl_pm_wrapper/pm_wrapper.h>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <robot_msgs/Wrench.h>
#include <motion_planning_srvs/MotionPlan.h>
#include <manipulation_msgs/IKRequest.h>
#include <manipulation_srvs/IKService.h>
#include <manipulation_srvs/IKQuery.h>

namespace sbpl_arm_planner_node
{
#define VERBOSE 1
#define MAX_RUNTIME 60.0

class SBPLArmPlannerNode
  {
    public:

      SBPLArmPlannerNode();

      ~SBPLArmPlannerNode();

      bool init();

      int run();

      bool planKinematicPath(motion_planning_srvs::MotionPlan::Request &req, motion_planning_srvs::MotionPlan::Response &res);

      void getCurrentJointAngles(const std::vector <std::string> &joint_names, std::vector <double> *joint_angles);

    private:

      ros::NodeHandle node_;
      ros::Publisher marker_publisher_;
      ros::Publisher sbpl_map_publisher_;
      ros::ServiceServer planning_service_;
      ros::Subscriber mechanism_subscriber_;
      ros::Subscriber col_map_subscriber_;
      ros::Subscriber point_cloud_subscriber_;

      std::string planner_type_;

      std::string pr2_desc_;

      double torso_arm_offset_x_;

      double torso_arm_offset_y_;

      double torso_arm_offset_z_;

      double allocated_time_;

      bool lowres_cc_;

      bool dijkstra_heuristic_;

      bool use_cm_for_voxel_;

      int env_width_;

      int env_height_;

      int env_depth_;

      double env_resolution_;

      bool forward_search_;

      bool search_mode_;

      bool use_voxel3d_grid_;

      bool use_collision_map_;

      bool visualize_goal_;

      bool planner_initialized_;

      bool use_multires_primitives_;

      bool enable_pm_;

      bool bCartesianPlanner_;

      int num_joints_;

      std::string collision_map_topic_;

      std::string point_cloud_topic_;

      std::string node_name_;

      std::string planning_frame_;

      std::string arm_name_;

      std::vector<std::string> joint_names_;

      mapping_msgs::CollisionMap collision_map_;

      mapping_msgs::CollisionMap sbpl_collision_map_;

      mechanism_msgs::MechanismState mechanism_state_;

      std::vector<motion_planning_msgs::PoseConstraint> goal_pose_constraint_; //in planning frame

      std::vector<motion_planning_msgs::JointConstraint> goal_joint_constraint_; //in planning frame

      MDPConfig mdp_cfg_;

      EnvironmentROBARM3D sbpl_arm_env_;

      ARAPlanner *planner_;

      FILE *env_config_fp_;

      FILE *planner_config_fp_;

      boost::mutex mPlanning_;

      bool bPlanning_;

      sbpl_arm_planner_node::pm_wrapper *pm_;

      boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

      boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_pose_solver_;

      tf::TransformListener tf_;

      boost::shared_ptr<Voxel3d> env_grid_;

      boost::shared_ptr<Voxel3d> planning_grid_;

      boost::shared_ptr<Voxel3d> lowres_env_grid_;

      boost::shared_ptr<Voxel3d> lowres_planning_grid_;

      robot_msgs::PointCloud point_cloud_;

      robot_msgs::PointCloud tf_point_cloud_;

      boost::mutex mCopyingVoxel_;

      KDL::Chain arm_chain_;

      KDL::JntArray jnt_pos_;

      visualization_msgs::Marker goal_marker_;

      visualization_msgs::MarkerArray goal_marker_array_;


      bool initializePlannerAndEnvironment();

      bool setStart(const motion_planning_msgs::KinematicState &start_state);

      bool setGoalPosition(const std::vector<motion_planning_msgs::PoseConstraint, std::allocator<motion_planning_msgs::PoseConstraint> > &goals);

      bool setGoalState(const std::vector<motion_planning_msgs::JointConstraint> &joint_constraint);

      bool planToState(motion_planning_srvs::MotionPlan::Request &req, motion_planning_srvs::MotionPlan::Response &res);

      bool planToPosition(motion_planning_srvs::MotionPlan::Request &req, motion_planning_srvs::MotionPlan::Response &res);

      bool plan(motion_planning_msgs::KinematicPath &arm_path);

      void getSBPLCollisionMap();

      void collisionMapCallback(const mapping_msgs::CollisionMapConstPtr &collision_map);

      void pointCloudCallback(const robot_msgs::PointCloudConstPtr &point_cloud);

      void dummyCallback(const mechanism_msgs::MechanismStateConstPtr &mechanism_state);

      void createOccupancyGrid();

      void visualizeGoalPosition(robot_msgs::PoseStamped pose);

      void initializePM();

      bool updateOccupancyGrid();

      void updatePMWrapper(motion_planning_srvs::MotionPlan::Request &req);

      void finishPath(motion_planning_msgs::KinematicPath &arm_path, motion_planning_msgs::PoseConstraint &goal_pose);

      bool computeIK(const robot_msgs::PoseStamped &pose_stamped_msg, std::vector<double> jnt_pos, std::vector<double> &solution);

      bool initChain(std::string robot_description);

			bool interpolatePathToGoal(std::vector<std::vector<double> > &path, double inc);
  };
}

#endif
