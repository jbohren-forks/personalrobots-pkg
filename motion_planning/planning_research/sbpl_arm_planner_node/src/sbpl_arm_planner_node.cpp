
/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *W
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
 /** \author Sachin Chitta, Benjamin Cohen */

#include <sbpl_arm_planner_node/sbpl_arm_planner_node.h>


using namespace std;
using namespace sbpl_arm_planner_node;

SBPLArmPlannerNode::SBPLArmPlannerNode(std::string node_name):ros::Node(node_name),node_name_(node_name)
{
  param<std::string>("~arm_name", arm_name_, "right_arm");
  param ("~allocated_time", allocated_time_, 1.0);
  param ("~forward_search", forward_search_, true);
  param ("~use_collision_map",use_collision_map_,false);
  param ("~search_mode", search_mode_, true);
  param ("~num_joints", num_joints_, 7);
  param<std::string>("~planner_type", planner_type_, "cartesian"); //"cartesian" or "joint_space"
  param ("~torso_arm_offset_x", torso_arm_offset_x_, 0.0);
  param ("~torso_arm_offset_y", torso_arm_offset_y_, 0.0);
  param ("~torso_arm_offset_z", torso_arm_offset_z_, 0.0);
  param<std::string>("~collision_map_topic", collision_map_topic_, "collision_map");

  advertiseService("plan_kinematic_path", &SBPLArmPlannerNode::planKinematicPath, this);

  // collision map
  subscribe(collision_map_topic_, collision_map_, &SBPLArmPlannerNode::collisionMapCallback,1);
  advertise<robot_msgs::CollisionMap> ("sbpl_collision_map", 1);

  //initialize planner
  planner_ = new ARAPlanner(&pr2_arm_env_, forward_search_);
  initializePlannerAndEnvironment();
};

SBPLArmPlannerNode::~SBPLArmPlannerNode()
{
  delete planner_;
  unsubscribe(collision_map_topic_);
  unadvertise("sbpl_collision_map");
  unadvertiseService("plan_kinematic_path");
};

void PrintUsage(char *argv[])
{
  printf("USAGE: %s <cfg file>\n", argv[0]);
}

bool SBPLArmPlannerNode::initializePlannerAndEnvironment()
{
  std::string env_config_string;
  std::string planner_config_string;

  this->param<std::string>("~env_config", env_config_string, " ");
  env_config_fp_ = fopen("sbpl_env_cfg_tmp.txt","wt");
  fprintf(env_config_fp_,"%s",env_config_string.c_str());

  ROS_INFO("\n\nEnvironment");
  ROS_INFO("%s",env_config_string.c_str());
  ROS_INFO("Environment");

  this->param<std::string>("~planner_config", planner_config_string, " ");
  planner_config_fp_ = fopen("sbpl_planner_cfg_tmp.txt","wt");
  fprintf(planner_config_fp_,"%s",planner_config_string.c_str());

  ROS_INFO("\n\nPlanner");
  ROS_INFO("%s",planner_config_string.c_str());
  ROS_INFO("Planner\n");

  fclose(env_config_fp_);
  fclose(planner_config_fp_);

  env_config_fp_ = fopen("sbpl_env_cfg_tmp.txt","rt");
  planner_config_fp_ = fopen("sbpl_planner_cfg_tmp.txt","rt");

  if(!pr2_arm_env_.InitEnvFromFilePtr(env_config_fp_, planner_config_fp_))
  {
    printf("ERROR: InitEnvFromFilePtr failed\n");
    return false;
  }

  fclose(env_config_fp_);
  fclose(planner_config_fp_);

  if(!pr2_arm_env_.InitializeMDPCfg(&mdp_cfg_))  //Initialize MDP Info
  {
    printf("ERROR: InitializeMDPCfg failed\n");
    return false;
  }

  //set epsilon
  planner_->set_initialsolution_eps(pr2_arm_env_.GetEpsilon());

  //set search mode (true - settle with first solution)
  planner_->set_search_mode(search_mode_);

  return true;
}

void SBPLArmPlannerNode::collisionMapCallback()
{
  if(!use_collision_map_)
    return;

  printf("[collisionMapCallback] collision_map returned %i boxes.\n",collision_map_.boxes.size());

  std::vector<std::vector<double> > sbpl_boxes(collision_map_.boxes.size());
  for(unsigned int i=0; i < collision_map_.boxes.size(); i++)
  {
    sbpl_boxes[i].resize(6);
    sbpl_boxes[i][0] = collision_map_.boxes[i].center.x - torso_arm_offset_x_;
    sbpl_boxes[i][1] = collision_map_.boxes[i].center.y - torso_arm_offset_y_;
    sbpl_boxes[i][2] = collision_map_.boxes[i].center.z - torso_arm_offset_z_;

    sbpl_boxes[i][3] = collision_map_.boxes[i].extents.x;
    sbpl_boxes[i][4] = collision_map_.boxes[i].extents.y;
    sbpl_boxes[i][5] = collision_map_.boxes[i].extents.z;

//     printf("[SBPLArmPlannerNode] obstacle %i: %.3f %.3f %.3f %.3f %.3f %.3f\n",i,sbpl_boxes[i][0],sbpl_boxes[i][1],
//            sbpl_boxes[i][2],sbpl_boxes[i][3],sbpl_boxes[i][4],sbpl_boxes[i][5]);
  }

  //NOTE: clear map for dynamic obstacle support
  pr2_arm_env_.ClearEnv();
  pr2_arm_env_.AddObstacles(sbpl_boxes);

  //get sbpl collision map and publish it
  getSBPLCollisionMap();
}

void SBPLArmPlannerNode::getSBPLCollisionMap()
{
    std::vector<std::vector<double> > sbpl_cubes = (*(pr2_arm_env_.getCollisionMap()));

//     for(unsigned int i=0; i < sbpl_cubes.size(); i++)
//     {
//         printf("[getSBPLCollisionMap] cube %i:, ",i);
//         for(unsigned int k=0; k < sbpl_cubes[i].size(); k++)
//             printf("%.3f ", sbpl_cubes[i][k]);
//         printf("\n");
//     }

    sbpl_collision_map_.header.frame_id = "torso_lift_link";
    sbpl_collision_map_.header.stamp = ros::Time::now();

    sbpl_collision_map_.set_boxes_size(sbpl_cubes.size());
    for(unsigned int i=0; i < sbpl_cubes.size(); i++)
    {
        sbpl_collision_map_.boxes[i].center.x = sbpl_cubes[i][0] + torso_arm_offset_x_;
        sbpl_collision_map_.boxes[i].center.y = sbpl_cubes[i][1] + torso_arm_offset_y_;
        sbpl_collision_map_.boxes[i].center.z = sbpl_cubes[i][2] + torso_arm_offset_z_;

        sbpl_collision_map_.boxes[i].extents.x = sbpl_cubes[i][3];
        sbpl_collision_map_.boxes[i].extents.y = sbpl_cubes[i][4];
        sbpl_collision_map_.boxes[i].extents.z = sbpl_cubes[i][5];

        sbpl_collision_map_.boxes[i].axis.x = 0.0;
        sbpl_collision_map_.boxes[i].axis.y = 0.0;
        sbpl_collision_map_.boxes[i].axis.z = 0.0;

        sbpl_collision_map_.boxes[i].angle = 0.0;
    }

    ROS_DEBUG("[getSBPLCollisionMap] publishing %i obstacles.\n",sbpl_cubes.size());
    publish ("sbpl_collision_map", sbpl_collision_map_);
}

/** set start configuration */
bool SBPLArmPlannerNode::setStart(const motion_planning_msgs::KinematicJoint &start_state)
{
  double sbpl_start[num_joints_];

  for(int i=0; i< num_joints_; i++)
  {
    sbpl_start[i] = start_state.value[i];
  }

  ROS_INFO("[setStart] start: %1.2f %1.2f %1.2f %1.2f %1.2f %1.2f %1.2f", sbpl_start[0],sbpl_start[1],sbpl_start[2],sbpl_start[3],sbpl_start[4],sbpl_start[5],sbpl_start[6]);

  if(pr2_arm_env_.SetStartJointConfig(sbpl_start, true) == 0)
  {
    ROS_ERROR("[setStart] Environment failed to set start state\n");
    return false;
  }

  if(planner_->set_start(mdp_cfg_.startstateid) == 0)
  {
    ROS_ERROR("[setStart] Failed to set start state\n");
    return false;
  }

  return true;
}

/** set cartesian goals */
bool SBPLArmPlannerNode::setGoalPosition(const std::vector<motion_planning_msgs::PoseConstraint, std::allocator<motion_planning_msgs::PoseConstraint> > &goals)
{
  double roll,pitch,yaw;
  tf::Pose tf_pose;
  vector <int> sbpl_type(1,0);
  vector <vector <double> > sbpl_goal(goals.size());
  vector <vector <double> > sbpl_tolerance(goals.size());

  // NOTE roll and yaw might be reversed!
  for(unsigned int i = 0; i < goals.size(); i++)
  {
    sbpl_goal[i].resize(6,0);
    sbpl_tolerance[i].resize(2,0);

    sbpl_goal[i][0] = goals[i].pose.pose.position.x;
    sbpl_goal[i][1] = goals[i].pose.pose.position.y;
    sbpl_goal[i][2] = goals[i].pose.pose.position.z;

    tf::PoseMsgToTF(goals[i].pose.pose, tf_pose);
    btMatrix3x3 mat = tf_pose.getBasis();

    mat.getEulerZYX(yaw,pitch,roll);

    sbpl_goal[i][3] = roll;
    sbpl_goal[i][4] = pitch;
    sbpl_goal[i][5] = yaw;

    sbpl_tolerance[i][0]  = goals[i].position_distance;
    sbpl_tolerance[i][1]  = goals[i].orientation_distance;

    sbpl_type[i] = goals[i].type;
  }

  pr2_arm_env_.SetGoalPosition(sbpl_goal, sbpl_tolerance, sbpl_type);

  if(planner_->set_goal(mdp_cfg_.goalstateid) == 0)
  {
    ROS_ERROR("Failed to set goal state\n");
    return false;
  }
  return true;
}

/** set joint space goals */
bool SBPLArmPlannerNode::setGoalState(const std::vector<motion_planning_msgs::JointConstraint> &joint_constraint)
{
  printf("[setGoalState] setting goal state...\n");

  unsigned int i =0;
  std::vector<std::vector<double> > sbpl_goal(joint_constraint.size());
  std::vector<std::vector<double> > sbpl_tolerance_above;
  std::vector<std::vector<double> > sbpl_tolerance_below;

  for(i = 0; i < sbpl_goal.size(); i++)
  {
    sbpl_goal[i].resize(joint_constraint[i].get_value_size());
    sbpl_goal[i] = joint_constraint[i].value;

    sbpl_tolerance_above[i].resize(joint_constraint[i].get_toleranceAbove_size());
    sbpl_tolerance_above[i] = joint_constraint[i].toleranceAbove;

    sbpl_tolerance_below[i].resize(joint_constraint[i].get_toleranceBelow_size());
    sbpl_tolerance_below[i] = joint_constraint[i].toleranceBelow;
  }

  pr2_arm_env_.SetGoalConfiguration(sbpl_goal,sbpl_tolerance_above,sbpl_tolerance_below);

  if(planner_->set_goal(mdp_cfg_.goalstateid) == 0)
  {
      ROS_ERROR("Failed to set goal state\n");
      return false;
  }

  printf("[setGoalState] successfully set goal state...\n");
  return true;
}

/** plan to joint space goal */
bool SBPLArmPlannerNode::planToState(motion_planning_srvs::KinematicPlan::Request &req, motion_planning_srvs::KinematicPlan::Response &res)
{
  motion_planning_msgs::KinematicPath arm_path;

  ROS_INFO("[planToState] Planning...");

  ROS_INFO("[planToState] start state: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
         req.start_state[0].value[0],req.start_state[0].value[1],req.start_state[0].value[2],req.start_state[0].value[3],req.start_state[0].value[4],req.start_state[0].value[5],req.start_state[0].value[6]);

  ROS_INFO("[planToState] goal state: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
         req.goal_constraints.joint_constraint[0].value[0],req.goal_constraints.joint_constraint[0].value[1],req.goal_constraints.joint_constraint[0].value[2],req.goal_constraints.joint_constraint[0].value[3],req.goal_constraints.joint_constraint[0].value[4],req.goal_constraints.joint_constraint[0].value[5],req.goal_constraints.joint_constraint[0].value[6]);

  if(setStart(req.start_state[0]))
  {
    if(setGoalState(req.goal_constraints.joint_constraint))
    {
      ROS_INFO("[planToState] Goal configuration has been set.");

        if(plan(arm_path))
        {
          ROS_INFO("[planToState] Planning successful.");
          res.path = arm_path;
          res.path.model_id = "right_arm";
          res.path.header.stamp = ros::Time::now();
          return true;
        }
        else
        {
          ROS_INFO("[planToState] Planning unsuccessful.");
        }
    }
    else
    {
      ROS_INFO("[planToState] Set goal unsuccessful.");
    }
  }
  else
  {
    ROS_INFO("[planToState] Set start unsuccessful.");
  }
  return false;
}

/** plan to cartesian goal(s) */
bool SBPLArmPlannerNode::planToPosition(motion_planning_srvs::KinematicPlan::Request &req, motion_planning_srvs::KinematicPlan::Response &res)
{
//   for(unsigned int i=0; i < req.get_start_state_size(); i++)
//   {
//       for(unsigned int j=0; j<req.start_state[i].get_value_size(); j++)
//         printf("%s: %.2f\n",req.start_state[i].joint_name.c_str(), req.start_state[i].value[j]);
//   }

//   for(int i=0; i < req.get_start_state_size(); i++)
//   {
//     printf("start_state %d: %d values\n",req.get_start_state_size(), req.start_state[i].get_value_size());
//   }

  motion_planning_msgs::KinematicPath arm_path;
  tf::TransformListener tf_pose;
  std::string torso_frame = "torso_lift_link";
  std::vector<robot_msgs::Pose> pose_torso_frame(req.goal_constraints.get_pose_constraint_size());
  std::vector<motion_planning_msgs::PoseConstraint> pose_constraint_torso_link(req.goal_constraints.get_pose_constraint_size());

  for(unsigned int i=0; i<req.goal_constraints.get_pose_constraint_size(); i++)
  {
    pose_constraint_torso_link[i] = req.goal_constraints.pose_constraint[i];
    tf_pose.transformPose(torso_frame,pose_constraint_torso_link[i].pose,pose_constraint_torso_link[i].pose);
  }

  ROS_INFO("[planToPosition] start state: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
           req.start_state[30].value[0],req.start_state[31].value[0],req.start_state[32].value[0],
           req.start_state[33].value[0],req.start_state[34].value[0],req.start_state[35].value[0],req.start_state[36].value[0]);
  ROS_INFO("[planToPosition] goal position: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
           req.goal_constraints.pose_constraint[0].pose.pose.position.x, req.goal_constraints.pose_constraint[0].pose.pose.position.y,req.goal_constraints.pose_constraint[0].pose.pose.position.z,
           req.goal_constraints.pose_constraint[0].pose.pose.orientation.x,req.goal_constraints.pose_constraint[0].pose.pose.orientation.y,req.goal_constraints.pose_constraint[0].pose.pose.orientation.z,req.goal_constraints.pose_constraint[0].pose.pose.orientation.w);

  if(setStart(req.start_state[0]))
  {
    if(setGoalPosition(pose_constraint_torso_link))
    {
      if(plan(arm_path))
      {
        ROS_INFO("[planToPosition] Planning successful.");
        res.path = arm_path;
        res.path.model_id = "right_arm";
        res.path.header.stamp = ros::Time::now();
        res.path.set_names_size(7);
        res.path.names[0] = "r_shoulder_pan_joint";
        res.path.names[1] = "r_shoulder_lift_joint";
        res.path.names[2] = "r_upper_arm_roll_joint";
        res.path.names[3] = "r_elbow_flex_joint";
        res.path.names[4] = "r_forearm_roll_joint";
        res.path.names[5] = "r_wrist_flex_joint";
        res.path.names[6] = "r_wrist_roll_joint";
        res.path.header.frame_id = "torso_lift_link";
        res.path.start_state.set_vals_size(req.get_start_state_size());
        for(unsigned int j = 0; j < req.get_start_state_size(); j++)
          res.path.start_state.vals[j] = req.start_state[j].value[0];

        return true;
      }
      else
      {
        ROS_INFO("[planToPosition] Planning unsuccessful.");
      }
    }
    else
    {
      ROS_INFO("[planToPosition] Set goal unsuccessful.");
    }
  }
  else
  {
    ROS_INFO("[planToPosition] Set start unsuccessful.");
  }

  res.unsafe = 0;
  res.approximate = 0;
  res.distance = 0;
  return false;
}

/** call back function */
bool SBPLArmPlannerNode::planKinematicPath(motion_planning_srvs::KinematicPlan::Request &req, motion_planning_srvs::KinematicPlan::Response &res)
{
  if(planner_type_ == "joint_space")
  {
    if(!planToState(req, res))
      return false;
  }
  else
  {
    ROS_INFO("[planKinematicPath] plan to Position called...");
    if(!planToPosition(req, res))
      return false;
  }

  return true;
}

/** retrieve plan from sbpl */
bool SBPLArmPlannerNode::plan(motion_planning_msgs::KinematicPath &arm_path)
{
  ROS_INFO("[plan] requesting plan from planner...");
  bool b_ret(false);
  int i;
  double angles_r[num_joints_];
  vector<int> solution_state_ids_v;
  clock_t start_time = clock();

  planner_->costs_changed();

  // plan
  b_ret = planner_->replan(allocated_time_, &solution_state_ids_v);

  ROS_INFO("[plan] retrieving of the plan completed in %.4f seconds", double(clock()-start_time) / CLOCKS_PER_SEC);
  ROS_INFO("[plan] size of solution = %d",solution_state_ids_v.size());

  // if a path is returned, then pack it into msg form
  if(b_ret)
  {
    arm_path.set_states_size(solution_state_ids_v.size());
    for(i = 0; i < (int) solution_state_ids_v.size(); i++)
      arm_path.states[i].set_vals_size(num_joints_);

    
    ROS_INFO("Path:");

    
    for(i = 0; i < (int) solution_state_ids_v.size(); i++) 
    {
      pr2_arm_env_.StateID2Angles(solution_state_ids_v[i], angles_r);
      for (unsigned int p = 0; p < (unsigned int) num_joints_; p++)
      {
        arm_path.states[i].vals[p] = angles_r[p];
      }
      ROS_INFO("state %d: %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
               i,arm_path.states[i].vals[0],arm_path.states[i].vals[1],arm_path.states[i].vals[2],arm_path.states[i].vals[3],
               arm_path.states[i].vals[4],arm_path.states[i].vals[5],arm_path.states[i].vals[6]);
    }
  }

  return b_ret;
}

int main(int argc, char *argv[])
{
  ros::init(argc,argv); 
  SBPLArmPlannerNode node("plan_path_node");

  try 
  {
    node.spin();
  }
  catch(char const* e)
  {
    std::cout << e << std::endl;
  }
  return(0);
}

