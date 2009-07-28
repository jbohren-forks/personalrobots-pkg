/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 /** \author Sachin Chitta, Benjamin Cohen */

#include <sbpl_arm_planner_node/sbpl_arm_planner_node.h>
clock_t starttime;


using namespace std;
using namespace sbpl_arm_planner_node;

SBPLArmPlannerNode::SBPLArmPlannerNode()
{
  planner_initialized_ = false;
};

SBPLArmPlannerNode::~SBPLArmPlannerNode()
{
  delete planner_;
//   node_.unsubscribe(collision_map_topic_);
//   node_.unadvertiseService("plan_kinematic_path");
// 
//   if(visualize_goal_)
//     node_.unadvertise("visualization_marker");
//   if(!use_voxel3d_grid_)
//     node_.unadvertise("sbpl_collision_map");
};

bool SBPLArmPlannerNode::init()
{
  // planner parameters
  node_.param ("~search_mode", search_mode_, true); //true: stop after first solution
  node_.param ("~allocated_time", allocated_time_, 20.0);
  node_.param ("~forward_search", forward_search_, true);
  node_.param ("~use_dijkstra_heuristic", dijkstra_heuristic_, true);
	node_.param ("~print_out_path", print_path_, false);
  node_.param<std::string>("~planner_type", planner_type_, "cartesian"); //"cartesian" or "joint_space"
  node_.param<std::string>("~planning_frame", planning_frame_, std::string("torso_lift_link"));

  // robot parameters
  node_.param<std::string>("~arm_name", arm_name_, "right_arm");

  std::string pr2_urdf_param;
  node_.searchParam("robot_description",pr2_urdf_param);
	node_.param<std::string>(pr2_urdf_param, pr2_desc_, "robot_description");

  node_.param ("~num_joints", num_joints_, 7);
  node_.param ("~torso_arm_offset_x", torso_arm_offset_x_, 0.0);
  node_.param ("~torso_arm_offset_y", torso_arm_offset_y_, 0.0);
  node_.param ("~torso_arm_offset_z", torso_arm_offset_z_, 0.0);
  joint_names_.resize(num_joints_);
  node_.param<std::string>("~joint_name_0", joint_names_[0], "r_shoulder_pan_joint");
  node_.param<std::string>("~joint_name_1", joint_names_[1], "r_shoulder_lift_joint");
  node_.param<std::string>("~joint_name_2", joint_names_[2], "r_upper_arm_roll_joint");
  node_.param<std::string>("~joint_name_3", joint_names_[3], "r_elbow_flex_joint");
  node_.param<std::string>("~joint_name_4", joint_names_[4], "r_forearm_roll_joint");
  node_.param<std::string>("~joint_name_5", joint_names_[5], "r_wrist_flex_joint");
  node_.param<std::string>("~joint_name_6", joint_names_[6], "r_wrist_roll_joint");

  // planning environment parameters
  node_.param ("~use_voxel3d_grid", use_voxel3d_grid_, true);
  node_.param ("~lowres_cc", lowres_cc_, true);
  node_.param ("~use_collision_map_occ_to_update_voxel", use_cm_for_voxel_, true);
  node_.param ("~use_collision_map",use_collision_map_,false);
  node_.param ("~use_exact_gripper_collision_checking",enable_pm_, true);
  node_.param ("~use multiresolution_motion_primitives", use_multires_primitives_, true);
  node_.param ("~voxel_grid_width_", env_width_, 120);
  node_.param ("~voxel_grid_height_", env_height_, 220);
  node_.param ("~voxel_grid_depth_", env_depth_, 200);
  node_.param ("~voxel_grid_resolution_", env_resolution_, 0.01);

  // topic names
  node_.param<std::string>("~point_cloud", point_cloud_topic_, "full_cloud_filtered");
  node_.param<std::string>("~collision_map_topic", collision_map_topic_, "collision_map_occ");

  // visualization parameters
  node_.param ("~visualize_goal", visualize_goal_, true);

  // main planning service
  planning_service_ = node_.advertiseService("plan_kinematic_path", &SBPLArmPlannerNode::planKinematicPath,this);

  mechanism_subscriber_ = node_.subscribe("/mechanism_state", 1, &SBPLArmPlannerNode::dummyCallback, this);

  if(visualize_goal_)
    marker_publisher_ = node_.advertise<visualization_msgs::Marker>("visualization_marker", 3);

  //initialize voxel grid  & subscribe to correct collision map topic
  if(use_voxel3d_grid_)
  {
    createOccupancyGrid();

    if(use_cm_for_voxel_)
      col_map_subscriber_ = node_.subscribe(collision_map_topic_, 1, &SBPLArmPlannerNode::collisionMapCallback, this);
    else
      point_cloud_subscriber_ = node_.subscribe(point_cloud_topic_, 1, &SBPLArmPlannerNode::pointCloudCallback, this);
  }
  else
  {
    col_map_subscriber_ = node_.subscribe(collision_map_topic_, 1, &SBPLArmPlannerNode::collisionMapCallback, this);
    sbpl_map_publisher_ = node_.advertise<mapping_msgs::CollisionMap> ("sbpl_collision_map", 1);
  }


  //initialize planner
  planner_ = new ARAPlanner(&sbpl_arm_env_, forward_search_);
  if(!initializePlannerAndEnvironment())
    return false;
	
  //initialize the planning monitor
  initializePM();

  //initialize the planning environment
  sbpl_arm_env_.initPlanningMonitor(pm_);
  ROS_INFO("Initialized planning monitor");

  ROS_INFO("Low-Resolution collision checking is %senabled.",lowres_cc_ ? "" : "not ");
  ROS_INFO("Is %susing collision_map_occ for planning.",use_cm_for_voxel_ ? "" : "not ");
  ROS_INFO("Is %susing a voxel3d grid for planning.",use_voxel3d_grid_ ? "" : "not ");
  planner_initialized_ = true;
  bPlanning_ = false;

  ROS_INFO("sbpl_arm_planner_node is initialized.");
  return true;
}

int SBPLArmPlannerNode::run()
{
  ros::spin();
  return 0;
}

void PrintUsage(char *argv[])
{
  printf("USAGE: %s <cfg file>\n", argv[0]);
}

/** \brief Initialize the SBPL planner and the sbpl_arm_planner environment */
bool SBPLArmPlannerNode::initializePlannerAndEnvironment()
{
  std::string env_config_string;
  std::string planner_config_string;

  this->node_.param<std::string>("~env_config", env_config_string, " ");
  env_config_fp_ = fopen("sbpl_env_cfg_tmp.txt","wt");
  fprintf(env_config_fp_,"%s",env_config_string.c_str());

  ROS_INFO("\n\nEnvironment");
  ROS_INFO("%s",env_config_string.c_str());
  ROS_INFO("Environment");

  this->node_.param<std::string>("~planner_config", planner_config_string, " ");
  planner_config_fp_ = fopen("sbpl_planner_cfg_tmp.txt","wt");
  fprintf(planner_config_fp_,"%s",planner_config_string.c_str());

  ROS_INFO("\n\nPlanner");
  ROS_INFO("%s",planner_config_string.c_str());
  ROS_INFO("Planner\n");

  fclose(env_config_fp_);
  fclose(planner_config_fp_);

  env_config_fp_ = fopen("sbpl_env_cfg_tmp.txt","rt");
  planner_config_fp_ = fopen("sbpl_planner_cfg_tmp.txt","rt");

  // check if the robot
  if(pr2_desc_.empty())
  {
    ROS_ERROR("Robot description file is empty.");
    return false;
  }

  if(!sbpl_arm_env_.InitEnvFromFilePtr(env_config_fp_, planner_config_fp_, pr2_desc_))
  {
    ROS_ERROR("ERROR: InitEnvFromFilePtr failed\n");
    return false;
  }

  fclose(env_config_fp_);
  fclose(planner_config_fp_);

  if(!sbpl_arm_env_.InitializeMDPCfg(&mdp_cfg_))  //Initialize MDP Info
  {
    ROS_ERROR("ERROR: InitializeMDPCfg failed\n");
    return false;
  }

  // set environment parameters
  sbpl_arm_env_.SetEnvParameter("useDijkstraHeuristic", dijkstra_heuristic_);
  sbpl_arm_env_.SetEnvParameter("useFastCollisionChecking", lowres_cc_);
  sbpl_arm_env_.SetEnvParameter("exactGripperCollisionChecking", enable_pm_);
  sbpl_arm_env_.SetEnvParameter("useVoxel3dForOccupancyGrid", use_voxel3d_grid_);
  sbpl_arm_env_.SetEnvParameter("useMultiResolutionMotionPrimitives", use_multires_primitives_);
	sbpl_arm_env_.SetEnvParameter("saveExpandedStateIDs", true);

  //set epsilon
  planner_->set_initialsolution_eps(sbpl_arm_env_.GetEpsilon());

  //set search mode (true - settle with first solution)
  search_mode_ = false; // it needs to be set to false to be able to set a max planning time (allocated_secs)
  planner_->set_search_mode(search_mode_);

  if(!initChain(pr2_desc_))
  {
    ROS_INFO("unable to initialize chain for finishPath");
    return false;
  }

  ROS_INFO("Initialized planning environment.");
  return true;
}

/** \brief Callback function that's called by the collision map topic. It reformats the collision map for the sbpl grid */
void SBPLArmPlannerNode::collisionMapCallback(const mapping_msgs::CollisionMapConstPtr &collision_map)
{
//   ROS_INFO("collisionMapCallback called.");
  if(mPlanning_.try_lock())
  {
    if(bPlanning_)
    {
      mPlanning_.unlock();
      return;
    }
    mPlanning_.unlock();
  }
  else
    return;

  ros::Time start = ros::Time::now();
  ros::Duration d;

  if(!use_collision_map_)
    return;

  if(!use_voxel3d_grid_)
  {
    ROS_INFO("[collisionMapCallback] collision_map returned %i boxes",collision_map->boxes.size());
    std::vector<std::vector<double> > sbpl_boxes(collision_map->boxes.size());
    for(unsigned int i=0; i < collision_map->boxes.size(); i++)
    {
      sbpl_boxes[i].resize(6);
      sbpl_boxes[i][0] = collision_map->boxes[i].center.x;
      sbpl_boxes[i][1] = collision_map->boxes[i].center.y;
      sbpl_boxes[i][2] = collision_map->boxes[i].center.z;
      sbpl_boxes[i][3] = collision_map->boxes[i].extents.x;
      sbpl_boxes[i][4] = collision_map->boxes[i].extents.y;
      sbpl_boxes[i][5] = collision_map->boxes[i].extents.z;
      ROS_DEBUG("[SBPLArmPlannerNode] obstacle %i: %.3f %.3f %.3f %.3f %.3f %.3f",i,sbpl_boxes[i][0],sbpl_boxes[i][1],sbpl_boxes[i][2],sbpl_boxes[i][3],sbpl_boxes[i][4],sbpl_boxes[i][5]);
    }

    //a quick hack to send the robot base as a big obstacle to the planner
    std::vector<double> robot_base(6);
    robot_base[0] = .2;
    robot_base[1] = -.188;
    robot_base[2] = -.45;
    robot_base[3] = .04;
    robot_base[4] = .04;
    robot_base[5] = .04;

    for(double x = .2; x < .3; x += .01)
      for(double y = -.3; y < .3; y += .02)
	for(double z = -.5; z > -.75; z -= .04)
	{
	  robot_base[0] = x;
	  robot_base[1] = y;
	  robot_base[2] = z;
	  sbpl_boxes.push_back(robot_base);
	}

    // clear old map
    sbpl_arm_env_.ClearEnv();

    // add new obstacles
    sbpl_arm_env_.AddObstacles(sbpl_boxes);

    //get sbpl collision map and publish it for display purposes
    getSBPLCollisionMap();

    return;
  }

  if(use_cm_for_voxel_)
  {
    //clear voxel grid
    env_grid_->reset();
    if(lowres_cc_)
      lowres_env_grid_->reset();

    if(mCopyingVoxel_.try_lock())
    {
			double x,y,z;
      // add base laser as a cubic obstacle (temporary)
      for(y = -.04; y < .04; y=y+.01)
      {
				for(x = 0.29; x < 0.37; x=x+.01)
				{
					for(z = -0.52; z < -.46; z=z+.01)
					{
						env_grid_->putWorldObstacle(x,y,z);
						if(lowres_cc_)
							env_grid_->putWorldObstacle(x,y,z);
					}
				}
      }

      // add robot base as a cubic obstacle (temporary)
      for(y = -.36; y < .36; y=y+.02)
      {
				for(x = 0.18; x < 0.42; x=x+.02)
				{
					for(z = -0.6; z < -0.5; z=z+0.1)
					{
						env_grid_->putWorldObstacle(x,y,z);
						if(lowres_cc_)
							env_grid_->putWorldObstacle(x,y,z);
					}
				}
      }

      ROS_DEBUG("collision_map_occ is in %s", collision_map->header.frame_id.c_str());
      for(unsigned int i=0; i < collision_map->boxes.size(); i++)
      {
				env_grid_->putWorldObstacle(collision_map->boxes[i].center.x,collision_map->boxes[i].center.y,collision_map->boxes[i].center.z);
				if(lowres_cc_)
					lowres_env_grid_->putWorldObstacle(collision_map->boxes[i].center.x,collision_map->boxes[i].center.y,collision_map->boxes[i].center.z);
			}
//       if(visualize_)
      env_grid_->updateVisualizations();

      mCopyingVoxel_.unlock();
    }
  }

  ROS_DEBUG("collisionMapCallback took %lf seconds", (ros::Time::now() - start).toSec());
}

/** \brief Callback function that updates the voxel when a new point cloud is published */
void SBPLArmPlannerNode::pointCloudCallback(const sensor_msgs::PointCloudConstPtr &point_cloud)
{
  if(mPlanning_.try_lock())
  {
    if(bPlanning_)
    {
      mPlanning_.unlock();
      return;
    }
    mPlanning_.unlock();
  }
  else
    return;

  ROS_INFO("pointCloudCallback");
  ros::Time start;
  ros::Duration d;

  if(!use_collision_map_)
    return;

  if(mCopyingVoxel_.try_lock())
  {
    //clear voxel grid
    ROS_DEBUG("Resetting voxel grid");
    env_grid_->reset();
    if(lowres_cc_)
      lowres_env_grid_->reset();

    //for now add robot base as a cubic obstacle
    for(double y = -.3; y < .3; y = y + .02)
    {
      for(double x = 0.18; x < 0.38; x = x + .02)
      {
	env_grid_->putWorldObstacle(x,y,-0.5);
	if(lowres_cc_)
	  env_grid_->putWorldObstacle(x,y,-0.5);
      }
    }

    //update voxel grid
    if(!point_cloud->header.frame_id.empty())
    {
      ROS_DEBUG("About to transform point cloud from %s frame into %s frame",point_cloud->header.frame_id.c_str(), planning_frame_.c_str());
// 	start = ros::Time::now();
      tf_.transformPointCloud(planning_frame_, (*point_cloud), tf_point_cloud_);
      env_grid_->updateWorld(tf_point_cloud_);
      if(lowres_cc_)
	lowres_env_grid_->updateWorld(tf_point_cloud_);
// 	d = ros::Time::now() - start;
// 	ROS_INFO("updateVoxelGrid: %lf sec",d.toSec());
    }

    mCopyingVoxel_.unlock();
  }
}

void SBPLArmPlannerNode::dummyCallback(const mechanism_msgs::MechanismStateConstPtr &mechanism_state)
{
  if(mPlanning_.try_lock())
  {
    if(bPlanning_)
    {
      mPlanning_.unlock();
      return;
    }
    mPlanning_.unlock();
  }
  else
    return;
//   double xyz_m[3], rpy_r[3], angles_r[7];
//   std::vector <double> joint_angles;
//   getCurrentJointAngles(joint_names_, &joint_angles);
  // 	
//   ROS_INFO("Current: %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
// 	   joint_angles.at(0), joint_angles.at(1), joint_angles.at(2), joint_angles.at(3), joint_angles.at(4), joint_angles.at(5),joint_angles.at(6));
  // 
//   for(unsigned int a = 0; a < joint_angles.size(); a++)
//     angles_r[a] = joint_angles.at(a);
  // 
//   sbpl_arm_env_.ComputeEndEffectorPos(angles_r, xyz_m, rpy_r);
//   ROS_INFO("        xyz: %.3f %.3f %.3f   rpy: %.3f %.3f %.3f",
// 	   xyz_m[0],xyz_m[1],xyz_m[2],rpy_r[0],rpy_r[1],rpy_r[2]);
}

/** \brief Fetch the SBPL collision map and publish it for debugging purposes in rviz */
void SBPLArmPlannerNode::getSBPLCollisionMap()
{
  std::vector<std::vector<double> > sbpl_cubes = (*(sbpl_arm_env_.getCollisionMap()));

//     for(unsigned int i=0; i < sbpl_cubes.size(); i++)
//     {
//         ROS_DEBUG("[getSBPLCollisionMap] cube %i:, ",i);
//         for(unsigned int k=0; k < sbpl_cubes[i].size(); k++)
// 	  ROS_DEBUG("%.3f ", sbpl_cubes[i][k]);
// 	ROS_DEBUG("\n");
//     }

  sbpl_collision_map_.header.frame_id = "torso_lift_link";
  sbpl_collision_map_.header.stamp = ros::Time::now();

  sbpl_collision_map_.set_boxes_size(sbpl_cubes.size());
  for(unsigned int i=0; i < sbpl_cubes.size(); i++)
  {
    sbpl_collision_map_.boxes[i].center.x = sbpl_cubes[i][0];
    sbpl_collision_map_.boxes[i].center.y = sbpl_cubes[i][1];
    sbpl_collision_map_.boxes[i].center.z = sbpl_cubes[i][2];

    sbpl_collision_map_.boxes[i].extents.x = sbpl_cubes[i][3];
    sbpl_collision_map_.boxes[i].extents.y = sbpl_cubes[i][4];
    sbpl_collision_map_.boxes[i].extents.z = sbpl_cubes[i][5];

    sbpl_collision_map_.boxes[i].axis.x = 0.0;
    sbpl_collision_map_.boxes[i].axis.y = 0.0;
    sbpl_collision_map_.boxes[i].axis.z = 0.0;

    sbpl_collision_map_.boxes[i].angle = 0.0;
  }

  ROS_DEBUG("[getSBPLCollisionMap] publishing %i obstacles.\n",sbpl_cubes.size());
  sbpl_map_publisher_.publish(sbpl_collision_map_);
}

/** \brief Set start configuration */
bool SBPLArmPlannerNode::setStart(const motion_planning_msgs::KinematicState &start_state)
{
  double* sbpl_start;
  sbpl_start = new double[start_state.get_vals_size()];

  for(unsigned int i=0; i< start_state.get_vals_size(); i++)
  {
    sbpl_start[i] = (double)(start_state.vals[i]);
  }

  ROS_INFO("[setStart] start: %1.2f %1.2f %1.2f %1.2f %1.2f %1.2f %1.2f", sbpl_start[0],sbpl_start[1],sbpl_start[2],sbpl_start[3],sbpl_start[4],sbpl_start[5],sbpl_start[6]);

  if(sbpl_arm_env_.SetStartJointConfig(sbpl_start, true) == 0)
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

/** \brief Set cartesian goal(s) */
bool SBPLArmPlannerNode::setGoalPosition(const std::vector<motion_planning_msgs::PoseConstraint, std::allocator<motion_planning_msgs::PoseConstraint> > &goals)
{
  double roll,pitch,yaw;
  tf::Pose tf_pose;
	std::vector <int> sbpl_type(1,0);
	std::vector <std::vector <double> > sbpl_goal(goals.size(), std::vector<double> (6,0));
	std::vector <std::vector <double> > sbpl_tolerance(goals.size(), std::vector<double> (12,0));

  for(unsigned int i = 0; i < goals.size(); i++)
  {
//     sbpl_goal[i].resize(6,0);
//     sbpl_tolerance[i].resize(2,0);

    sbpl_goal[i][0] = goals[i].pose.pose.position.x;
    sbpl_goal[i][1] = goals[i].pose.pose.position.y;
    sbpl_goal[i][2] = goals[i].pose.pose.position.z;

    tf::poseMsgToTF(goals[i].pose.pose, tf_pose);
    btMatrix3x3 mat = tf_pose.getBasis();

    mat.getEulerZYX(yaw,pitch,roll);

    sbpl_goal[i][3] = roll;
    sbpl_goal[i][4] = pitch;
    sbpl_goal[i][5] = yaw;

// # The acceptable tolerance
// geometry_msgs/Point position_tolerance_above
// geometry_msgs/Point position_tolerance_below
// 
// # The acceptable tolerance (roll pitch yaw)
// geometry_msgs/Point orientation_tolerance_above
// geometry_msgs/Point orientation_tolerance_below

    sbpl_tolerance[i][0]  = goals[i].position_tolerance_above.x;
    sbpl_tolerance[i][1]  = goals[i].orientation_tolerance_above.x;
    sbpl_type[i] = goals[i].type;

    ROS_INFO("goal %d: xyz: %.2f %.2f %.2f  (tol: %.3f) rpy: %.2f %.2f %.2f  (tol: %.3f)",
	     i, sbpl_goal[i][0],sbpl_goal[i][1],sbpl_goal[i][2],sbpl_tolerance[i][0],sbpl_goal[i][3],sbpl_goal[i][4],sbpl_goal[i][5], sbpl_tolerance[i][1]);
  }

  if(!sbpl_arm_env_.SetGoalPosition(sbpl_goal, sbpl_tolerance, sbpl_type))
  {
    ROS_ERROR("Failed to set goal state. Perhaps goal position is out of reach...");
    return false;
  }

  if(planner_->set_goal(mdp_cfg_.goalstateid) == 0)
  {
    ROS_ERROR("Failed to set goal state\n");
    return false;
  }
  return true;
}

/** \brief Set joint space goal(s) */
bool SBPLArmPlannerNode::setGoalState(const std::vector<motion_planning_msgs::JointConstraint> &joint_constraint)
{
  int nind = 0;
  unsigned int i =0;
  double * angles = new double [num_joints_];
  std::vector<std::vector<double> > sbpl_goal(1);
  std::vector<std::vector<double> > sbpl_tolerance_above(1);
  std::vector<std::vector<double> > sbpl_tolerance_below(1);


  if((int)(joint_constraint.size()) < num_joints_)
  {
    ROS_INFO("The goal state does not describe all of the planning joints");
    return false; 
  }

  //NOTE temporary - fix this
  sbpl_goal[0].resize(num_joints_, 0.0);
  sbpl_tolerance_above[0].resize(num_joints_);
  sbpl_tolerance_below[0].resize(num_joints_);

//   ROS_INFO("goal state:");
//   for(i = 0; i < joint_constraint.size(); i++)
//   {
//     for(unsigned int k = 0; k < joint_constraint[i].value.size(); k++)
//       ROS_INFO("%s: %f", joint_constraint[i].joint_name.c_str(), joint_constraint[i].value[k]);
//   }

  goal_joint_constraint_.resize(num_joints_);

  for(i = 0; i < joint_constraint.size(); i++)
  {
    if(joint_names_[nind].compare(joint_constraint[i].joint_name) == 0)
    {
//       ROS_INFO("%i: %s",i,joint_constraint[i].joint_name.c_str());
      sbpl_goal[0][nind] = joint_constraint[i].value[0];
      sbpl_tolerance_above[0][nind] = joint_constraint[i].tolerance_above[0];
      sbpl_tolerance_below[0][nind] = joint_constraint[i].tolerance_below[0];

      angles[nind] =  joint_constraint[i].value[0];

      goal_joint_constraint_[nind].set_value_size(1);
      goal_joint_constraint_[nind].joint_name = joint_constraint[i].joint_name;
      goal_joint_constraint_[nind].value[0] = joint_constraint[i].value[0];

      nind++;
    }
    if(nind == num_joints_)
      break;
  }

  // temporary
  double xyz_m[3],rpy_r[3];
  sbpl_arm_env_.ComputeEndEffectorPos(angles, xyz_m, rpy_r);
  ROS_INFO("[setGoalState] %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
	   angles[0],angles[1],angles[2],angles[3],angles[4],angles[5],angles[6]);
  ROS_INFO("[setGoalState]      xyz: %.3f %.3f %.3f   rpy: %.3f %.3f %.3f",
	   xyz_m[0],xyz_m[1],xyz_m[2],rpy_r[0],rpy_r[1],rpy_r[2]);

  delete angles;


  if(!sbpl_arm_env_.SetGoalConfiguration(sbpl_goal,sbpl_tolerance_above,sbpl_tolerance_below))
  {
    ROS_INFO("Planner failed to set goal configuration");
    return false;
  }

  if(planner_->set_goal(mdp_cfg_.goalstateid) == 0)
  {
    ROS_ERROR("Failed to set goal state\n");
    return false;
  }

//   printf("[setGoalState] Successfully set goal state.\n");
  return true;
}

/** \brief Plan a path to a joint space goal(s) */
bool SBPLArmPlannerNode::planToState(motion_planning_srvs::MotionPlan::Request &req, motion_planning_srvs::MotionPlan::Response &res)
{
  unsigned int i;
  int nind = 0;
  motion_planning_msgs::KinematicState start;
  motion_planning_msgs::KinematicPath arm_path;

  bCartesianPlanner_ = false;
//   ROS_INFO("[planToState] Planning...");

  //check for an empty start state
  if(req.start_state.empty())
  {
    ROS_INFO("No start state given. Unable to plan.");
    return false;
  }

//   for(i = 0; i < req.get_start_state_size(); i++)
//     ROS_INFO("%s: %.3f",req.start_state[i].joint_name.c_str(),req.start_state[i].value[0]);

  // update the planning monitor's starting position
  updatePMWrapper(req);

  // get the starting position of the arm joints the planner will be planning for
  start.set_vals_size(num_joints_);
  for(i = 0; i < req.get_start_state_size(); i++)
  {
    ROS_DEBUG("%s: %.3f",req.start_state[i].joint_name.c_str(),req.start_state[i].value[0]);
    if(joint_names_[nind].compare(req.start_state[i].joint_name) == 0)
    {
      start.vals[nind] = req.start_state[i].value[0];
      nind++;
    }
    if(nind == num_joints_)
      break;
  }

  starttime = clock();
  
  if(setStart(start))
  {
    ROS_INFO("[planToState] successfully set starting configuration");

    updateOccupancyGrid();

    if(setGoalState(req.goal_constraints.joint_constraint))
    {
      ROS_INFO("[planToState] Goal configuration has been set.");

      if(plan(arm_path))
      {
	ROS_INFO("Planning took %lf seconds",(clock() - starttime) / (double)CLOCKS_PER_SEC);
	bPlanning_ = false;
	mPlanning_.unlock();
	pm_->unlockPM();
	ROS_INFO("[planToPosition] Planning successful.");
		
	res.path = arm_path;
	res.path.model_id = req.params.model_id;
	res.path.header.stamp = ros::Time::now();

	if(!req.start_state[0].header.frame_id.empty())
	  res.path.header.frame_id = req.start_state[0].header.frame_id;
	else
	  res.path.header.frame_id = planning_frame_;

	res.path.set_times_size(res.path.get_states_size());
	res.path.times[0] = 0;
	for(i = 1; i < res.path.get_states_size(); i++)
	  res.path.times[i] = res.path.times[i-1] + 0.001;

	res.path.set_names_size(num_joints_);
	for(i = 0; i < (unsigned int)num_joints_; i++)
	  res.path.names[i] = joint_names_[i];

	res.path.start_state = req.start_state;
	res.approximate = 0;
	res.distance = 0;

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

  ROS_INFO("Planning took %lf seconds",(clock() - starttime) / (double)CLOCKS_PER_SEC);
  pm_->unlockPM();
  bPlanning_ = false;
  mPlanning_.unlock();

  return false;
}

/** \brief Plan a path to a cartesian goal(s) */
bool SBPLArmPlannerNode::planToPosition(motion_planning_srvs::MotionPlan::Request &req, motion_planning_srvs::MotionPlan::Response &res)
{
  unsigned int i;
  int nind = 0;
  motion_planning_msgs::KinematicPath arm_path;
  motion_planning_msgs::KinematicState start;
  tf::Stamped<tf::Pose> pose_stamped;
//   std::vector<motion_planning_msgs::PoseConstraint> pose_constraint_torso_link(req.goal_constraints.get_pose_constraint_size());

  bCartesianPlanner_ = true;

  //empty old goals
  goal_pose_constraint_.resize(0);

  goal_pose_constraint_.resize(req.goal_constraints.get_pose_constraint_size());

//   ROS_INFO("goal pose has %i poses",goal_pose_constraint_.size());

  //check for an empty start state
  if(req.get_start_state_size() <= 0)
  {
    ROS_INFO("[planToPosition] No start state given. Unable to plan.");
    return false;
  }

  //check if goal constraint is empty
  if(req.goal_constraints.get_pose_constraint_size() <= 0)
  {
    ROS_INFO("[planToPosition] No pose constraints given. Unable to plan.");
    return false;
  }

  // update the planning monitor's starting position
  updatePMWrapper(req);

  //transform goal into planning_frame_
  for(i = 0; i < req.goal_constraints.get_pose_constraint_size(); i++)
  {
    goal_pose_constraint_[i] = req.goal_constraints.pose_constraint[i];
    poseStampedMsgToTF( goal_pose_constraint_[i].pose, pose_stamped);
    ROS_DEBUG("[planToPosition] converted pose_constraint to tf");

    if (!tf_.canTransform(planning_frame_, pose_stamped.frame_id_, pose_stamped.stamp_, ros::Duration(3.0)))
    {
      ROS_ERROR("[sbpl_arm_planner_node] Cannot transform from %s to %s",pose_stamped.frame_id_.c_str(), planning_frame_.c_str());
      return false;
    }

    ROS_DEBUG("[planToPosition] attempting to call transformPose()...");
    tf_.transformPose(planning_frame_, goal_pose_constraint_[i].pose, goal_pose_constraint_[i].pose);
//     goal_pose_constraint_.push_back(goal_pose_constraint_[i]);
    ROS_DEBUG("[planToPosition] successfully transformed pose from %s to %s",pose_stamped.frame_id_.c_str(), planning_frame_.c_str());
  }

  // get the starting position of the specific arm joints the planner will be planning for
  start.set_vals_size(num_joints_);
  for(i = 0; i < req.get_start_state_size(); i++)
  {
    ROS_DEBUG("%s: %.3f",req.start_state[i].joint_name.c_str(),req.start_state[i].value[0]);
    if(joint_names_[nind].compare(req.start_state[i].joint_name) == 0)
    {
      start.vals[nind] = req.start_state[i].value[0];
      nind++;
    }
    if(nind == num_joints_)
      break;
  }
  if(nind != num_joints_)
    ROS_INFO("[planToPosition] Not all joints were assigned a starting position.");

  starttime = clock();

  ROS_DEBUG("About to set start configuration\n");
  if(setStart(start))
  {
    ROS_INFO("[planToPosition] successfully set starting configuration");

    updateOccupancyGrid();

    if(visualize_goal_)
      visualizeGoalPosition(goal_pose_constraint_[0].pose);

    if(setGoalPosition(goal_pose_constraint_))
    {
      ROS_INFO("[planToPosition] successfully sent goal constraints");
      if(plan(arm_path))
      {
				if(print_path_)
					printPath(arm_path);

				ROS_INFO("Planning took %lf seconds",(clock() - starttime) / (double)CLOCKS_PER_SEC);
				bPlanning_ = false;
				mPlanning_.unlock();
				pm_->unlockPM();
				ROS_INFO("[planToPosition] Planning successful.");

				res.path = arm_path;
				res.path.model_id = req.params.model_id;
				res.path.header.stamp = ros::Time::now();

				if(!req.start_state[0].header.frame_id.empty())
					res.path.header.frame_id = req.start_state[0].header.frame_id;
				else
					res.path.header.frame_id = planning_frame_;

				ROS_INFO("path is in %s",res.path.header.frame_id.c_str());

				res.path.set_times_size(res.path.get_states_size());
				res.path.times[0] = 0;
				for(i = 1; i < res.path.get_states_size(); i++)
					res.path.times[i] = res.path.times[i-1] + 0.001;

				res.path.set_names_size(num_joints_);
				for(i = 0; i < (unsigned int)num_joints_; i++)
					res.path.names[i] = joint_names_[i];

				res.path.start_state = req.start_state;
				res.approximate = 0;
				res.distance = 0;

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

  ROS_INFO("Planning took %lf seconds",(clock() - starttime) / (double)CLOCKS_PER_SEC);
  pm_->unlockPM();
  bPlanning_ = false;
  mPlanning_.unlock();

  return false;
}

/** \brief Planning service call back function */
bool SBPLArmPlannerNode::planKinematicPath(motion_planning_srvs::MotionPlan::Request &req, motion_planning_srvs::MotionPlan::Response &res)
{
  if(!planner_initialized_)
  {
    ROS_ERROR("Hold up a second...the planner isn't initialized yet. Try again in a second or two.");
    return false;
  }

  if(planner_type_ == "joint_space" || (req.goal_constraints.get_pose_constraint_size() <= 0 && req.goal_constraints.get_joint_constraint_size() > 0))
  {
    if(req.goal_constraints.get_joint_constraint_size() <= 0)
    {
      ROS_INFO("There are no joint_constraints in the request message");
      return false;
    }
    if(!planToState(req, res))
      return false;
  }
  else
  {
    if(req.goal_constraints.get_pose_constraint_size() <= 0)
    {
      ROS_INFO("There are no pose_constraints in the request message");
      return false;
    }
    if(!planToPosition(req, res))
      return false;
  }

  return true;
}

/** \brief Retrieve plan from sbpl */
bool SBPLArmPlannerNode::plan(motion_planning_msgs::KinematicPath &arm_path)
{
  ROS_INFO("[plan] requesting plan from planner...");
  bool b_ret(false);
  unsigned int i;
  double angles_r[num_joints_], error_m, error_r, yaw, pitch, roll, xyz_m[3], rpy_r[3];
	std::vector<std::vector <double> > path(2, std::vector<double> (num_joints_));
  vector<int> solution_state_ids_v;
  std::vector<double> final_waypoint;
  clock_t start_time = clock();
  tf::Pose tf_pose;

  //reinitialize the search space - search from scratch
  planner_->costs_changed();

  //plan
  b_ret = planner_->replan(allocated_time_, &solution_state_ids_v);

  ROS_INFO("[plan] retrieving of the plan completed in %.4f seconds", double(clock()-start_time) / CLOCKS_PER_SEC);
  ROS_DEBUG("[plan] size of solution = %d", solution_state_ids_v.size());

	ROS_DEBUG("displaying expanded states....");
	displayExpandedStates();

  // if a path is returned, then pack it into msg form
  if(b_ret)
  {
		ROS_INFO("*** a path was planned ***");

    arm_path.set_states_size(solution_state_ids_v.size());
    for(i = 0; i < solution_state_ids_v.size(); i++)
      arm_path.states[i].set_vals_size(num_joints_);

    for(i = 0; i < arm_path.get_states_size(); i++)
    {
      sbpl_arm_env_.StateID2Angles(solution_state_ids_v[i], angles_r);
      for (unsigned int p = 0; p < (unsigned int) num_joints_; ++p)
				arm_path.states[i].vals[p] = angles_r[p];
    }

    if(bCartesianPlanner_)
    {
      std::vector<double> jnt_pos(num_joints_,0);
      for(int j=0; j<num_joints_; ++j)
				jnt_pos[j] = arm_path.states[arm_path.get_states_size()-1].vals[j];

			bool invalid_ik = true;
			unsigned int ik_attempts = 0;
      while(invalid_ik && ik_attempts < 5)
      {
				if(computeIK(goal_pose_constraint_[0].pose,jnt_pos,final_waypoint))
				{
					for(int b = 0; b < num_joints_; ++b)
						angles_r[b] = final_waypoint[b];

					if(sbpl_arm_env_.isValidJointConfiguration(angles_r))
					{
						invalid_ik = false;
						ROS_INFO("IK solution is free of collision");
					}
					else
						ROS_INFO("Computed an IK solution but it was in collision");
				}

				if(invalid_ik)
				  ROS_INFO("IK solution is invalid.trying again...");
				else
				{
					for(int b = 0; b < num_joints_; ++b)
					{
						path[0][b] = arm_path.states[arm_path.get_states_size()-1].vals[b];
						path[1][b] = final_waypoint[b];
					}

					if(!interpolatePathToGoal(path, 0.1))
						ROS_INFO("ik solution %i: a joint configuration on the way to the final waypoint is in collision", ik_attempts);
					else
						invalid_ik = false;
				}
				++ik_attempts;
      }

			if(invalid_ik)
				ROS_INFO("IK was unable to come up with a valid joint configuration. The path is approximate.");
			else
			{
				// append final waypoint to path
				arm_path.set_states_size(arm_path.get_states_size()+1);
				arm_path.states[arm_path.get_states_size()-1].set_vals_size(num_joints_);
				for(int j = 0; j < num_joints_; j++)
				{
					arm_path.states[arm_path.get_states_size()-1].vals[j] = final_waypoint[j];
					angles_r[j] = arm_path.states[arm_path.get_states_size()-1].vals[j];
				}

				i = arm_path.get_states_size() - 1;
				ROS_INFO("IK Solution:");
				ROS_INFO("state %d: %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
								 i,arm_path.states[i].vals[0],arm_path.states[i].vals[1],arm_path.states[i].vals[2],arm_path.states[i].vals[3],
        				 arm_path.states[i].vals[4],arm_path.states[i].vals[5],arm_path.states[i].vals[6]);
				ROS_INFO("    diff: %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
									angles::shortest_angular_distance(arm_path.states[i].vals[0], arm_path.states[i-1].vals[0]),
									angles::shortest_angular_distance(arm_path.states[i].vals[1], arm_path.states[i-1].vals[1]),
									angles::shortest_angular_distance(arm_path.states[i].vals[2], arm_path.states[i-1].vals[2]),
									angles::shortest_angular_distance(arm_path.states[i].vals[3], arm_path.states[i-1].vals[3]),
									angles::shortest_angular_distance(arm_path.states[i].vals[4], arm_path.states[i-1].vals[4]),
									angles::shortest_angular_distance(arm_path.states[i].vals[5], arm_path.states[i-1].vals[5]),
									angles::shortest_angular_distance(arm_path.states[i].vals[6], arm_path.states[i-1].vals[6]));

				for(int j = 0; j < num_joints_; j++)
					angles_r[j] = arm_path.states[i].vals[j];

				sbpl_arm_env_.ComputeEndEffectorPos(angles_r, xyz_m, rpy_r);
				ROS_INFO("        xyz: %.3f %.3f %.3f   rpy: %.3f %.3f %.3f",
								 xyz_m[0],xyz_m[1],xyz_m[2],rpy_r[0],rpy_r[1],rpy_r[2]);
			}
			for(unsigned int j = 0; j < goal_pose_constraint_.size(); ++j)
			{
				tf::poseMsgToTF(goal_pose_constraint_[j].pose.pose, tf_pose);
				btMatrix3x3 mat = tf_pose.getBasis();
				mat.getEulerZYX(yaw,pitch,roll);

				error_m = sqrt((xyz_m[0]-goal_pose_constraint_[j].pose.pose.position.x)*(xyz_m[0]-goal_pose_constraint_[j].pose.pose.position.x) +
						(xyz_m[1]-goal_pose_constraint_[j].pose.pose.position.y)*(xyz_m[1]-goal_pose_constraint_[j].pose.pose.position.y) +
						(xyz_m[2]-goal_pose_constraint_[j].pose.pose.position.z)*(xyz_m[2]-goal_pose_constraint_[j].pose.pose.position.z));

				error_r = sqrt((rpy_r[0] - yaw)*(rpy_r[0] -yaw) + (rpy_r[1] - pitch)*(rpy_r[1] - pitch) + (rpy_r[2] - roll)*(rpy_r[2] - roll));

				ROS_INFO("error: %.4fm, %.4f rad (%.4f deg)\n\n", error_m, error_r, (error_r*180.0)/PI_CONST);
			}
    }
    else //joint space planner
    {
      if(goal_joint_constraint_.empty())
      {
				ROS_INFO("goal joint constraint is empty");
        return false;
      }
      final_waypoint.resize(num_joints_,0);
      for(int j=0; j<num_joints_; ++j)
				final_waypoint[j] = goal_joint_constraint_[j].value[0];

			// append actual goal to path
			arm_path.set_states_size(arm_path.get_states_size()+1);
			arm_path.states[arm_path.get_states_size()-1].set_vals_size(num_joints_);
			for(int j = 0; j < num_joints_; j++)
			{
				arm_path.states[arm_path.get_states_size()-1].vals[j] = final_waypoint[j];
				angles_r[j] = arm_path.states[arm_path.get_states_size()-1].vals[j];
			}

			for(int j = 0; j < num_joints_; j++)
				angles_r[j] = arm_path.states[i].vals[j];

// 			sbpl_arm_env_.ComputeEndEffectorPos(angles_r, xyz_m, rpy_r);
// 			ROS_INFO("        xyz: %.3f %.3f %.3f   rpy: %.3f %.3f %.3f",
// 				xyz_m[0],xyz_m[1],xyz_m[2],rpy_r[0],rpy_r[1],rpy_r[2]);

			for(int b=0; b < num_joints_; ++b)
			{
				path[0][b] = arm_path.states[arm_path.get_states_size()-2].vals[b];
				path[1][b] = arm_path.states[arm_path.get_states_size()-1].vals[b];
			}

			if(!interpolatePathToGoal(path, 0.1))
			{
				ROS_INFO("a joint configuration on the way to the final waypoint is in collision");
				return false;
			}
		}
  }

  return b_ret;
}

/** \brief Initialize the occupancy grid used by the SBPL planner. */
void SBPLArmPlannerNode::createOccupancyGrid()
{
  bool visualize = true;
  mCopyingVoxel_.lock();
  env_grid_.reset(new Voxel3d(env_width_,env_height_,env_depth_, env_resolution_, tf::Vector3(-.1, -1, -1), visualize));
  planning_grid_.reset(new Voxel3d(env_width_,env_height_,env_depth_, env_resolution_, tf::Vector3(-.1, -1, -1), visualize));

  if(lowres_cc_)
  {
    lowres_env_grid_.reset(new Voxel3d(env_width_/2,env_height_/2,env_depth_/2, env_resolution_*2, tf::Vector3(-.1, -1, -1), false));
    lowres_planning_grid_.reset(new Voxel3d(env_width_/2,env_height_/2,env_depth_/2, env_resolution_*2, tf::Vector3(-.1, -1, -1), false));
  }

  if(!point_cloud_.header.frame_id.empty())
  {
    ROS_DEBUG("About to transform point cloud from %s frame into %s frame",point_cloud_.header.frame_id.c_str(), planning_frame_.c_str());
    tf_.transformPointCloud(planning_frame_, point_cloud_, tf_point_cloud_);
    ROS_DEBUG("Transformed point cloud successfully");
    env_grid_->updateWorld(point_cloud_);

    if(lowres_cc_)
      lowres_env_grid_->updateWorld(point_cloud_);

    //just for display purposes
    planning_grid_.swap(env_grid_);
  }
  mCopyingVoxel_.unlock();
}

/** \brief Check mechanism_state for current joint angles */
void SBPLArmPlannerNode::getCurrentJointAngles(const std::vector <std::string> &joint_names, std::vector <double> *joint_angles)
{
  int nind = 0;
  joint_angles->resize(num_joints_);
  for(unsigned int i = 0; i < mechanism_state_.get_joint_states_size(); i++)
  {
    if(joint_names_[nind].compare(mechanism_state_.joint_states[i].name) == 0)
    {
      ROS_DEBUG("%s: %.3f",mechanism_state_.joint_states[i].name.c_str(), mechanism_state_.joint_states[i].position);
      joint_angles->at(nind) = mechanism_state_.joint_states[i].position;
      nind++;
    }
    if(nind == num_joints_)
      break;
  }
}

/** \brief Publish a visualization marker to display the goal end effector position in rviz */
void SBPLArmPlannerNode::visualizeGoalPosition(geometry_msgs::PoseStamped pose)
{
  goal_marker_.header.stamp = pose.header.stamp;
  goal_marker_.header.frame_id = pose.header.frame_id;
  goal_marker_.ns = "end_effector_goal";
  goal_marker_.type = visualization_msgs::Marker::ARROW;
  goal_marker_.action = 0;  // 0 add/modify an object, 1 (deprecated), 2 deletes an object
  goal_marker_.pose = pose.pose;
  goal_marker_.scale.x = env_resolution_*10; // Scale of the object 1,1,1 means default (usually 1 meter square)
  goal_marker_.scale.y = env_resolution_*10;
  goal_marker_.scale.z = env_resolution_*10;
  goal_marker_.color.r = 0.0;
  goal_marker_.color.g = 0.7;
  goal_marker_.color.b = 0.6;
  goal_marker_.color.a = 0.5;
  goal_marker_.lifetime = ros::Duration(40.0);

  marker_publisher_.publish(goal_marker_);

  goal_marker_.header.stamp = pose.header.stamp;
  goal_marker_.header.frame_id = pose.header.frame_id;
  goal_marker_.ns = "end_effector_goal_sphere";
  goal_marker_.type = visualization_msgs::Marker::SPHERE;
  goal_marker_.action = 0;  // 0 add/modify an object, 1 (deprecated), 2 deletes an object
  goal_marker_.pose = pose.pose;
  goal_marker_.scale.x = 0.07; // Scale of the object 1,1,1 means default (usually 1 meter square)
  goal_marker_.scale.y = 0.07;
  goal_marker_.scale.z = 0.07;
  goal_marker_.color.r = 1.0;
  goal_marker_.color.g = 0.0;
  goal_marker_.color.b = 0.6;
  goal_marker_.color.a = 0.5;
  goal_marker_.lifetime = ros::Duration(40.0);

  marker_publisher_.publish(goal_marker_);
}

bool SBPLArmPlannerNode::updateOccupancyGrid()
{
  // update the planning grid
  mCopyingVoxel_.lock();
  planning_grid_.swap(env_grid_);
  if(lowres_cc_)
    lowres_planning_grid_.swap(lowres_env_grid_);

  if(!sbpl_arm_env_.updateVoxelGrid(planning_grid_, lowres_planning_grid_))
  {
    ROS_INFO("Unable to update the planner's occupancy grid");
    return false;
  }
  mCopyingVoxel_.unlock();

  // lock the mutex around the bPlanning_ bool
  mPlanning_.lock();
  bPlanning_ = true;

  return true;
}

/** \brief Initialize the planning monitor used for collision checking */
void SBPLArmPlannerNode::initializePM()
{
  pm_ = new sbpl_arm_planner_node::pm_wrapper();

  std::vector<std::string> links;
  links.push_back("r_gripper_palm_link");
  links.push_back("r_gripper_l_finger_link");
  links.push_back("r_gripper_r_finger_link");
  links.push_back("r_gripper_l_finger_tip_link");
  links.push_back("r_gripper_r_finger_tip_link");

  pm_->initPlanningMonitor(links, &tf_);

  ROS_INFO("initialized PM()");
}

/** \brief Update the planning monitor with the current goal request message */
void SBPLArmPlannerNode::updatePMWrapper(motion_planning_srvs::MotionPlan::Request &req)
{
  pm_->updatePM(req);
}

void SBPLArmPlannerNode::finishPath(motion_planning_msgs::KinematicPath &arm_path, motion_planning_msgs::PoseConstraint &goal)
{
  KDL::Chain kdl_chain_;
  KDL::Twist twist;
  KDL::JntArray jnt_pos, jnt_eff;
  KDL::Jacobian jacobian;
  KDL::Frame goal_frame, current_frame;
  tf::Pose current_pose;
  tf::Pose goal_pose;
  std::vector<double> last_waypoint(7,0);

  jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(arm_chain_));
  jnt_pos.resize(arm_chain_.getNrOfJoints());
  jnt_eff.resize(arm_chain_.getNrOfJoints());
  jacobian.resize(arm_chain_.getNrOfJoints());

  for (unsigned int p = 0; p < (unsigned int) num_joints_; p++)
  {
    jnt_pos(p) = arm_path.states[arm_path.get_states_size()-1].vals[p];
//     ROS_INFO("%i: %f", p, jnt_pos(p));
  }

  // get the chain jacobian
  jnt_to_jac_solver_->JntToJac(jnt_pos, jacobian);

  ROS_DEBUG("arm_chain_ has %i joints and %i segments", arm_chain_.getNrOfJoints(),arm_chain_.getNrOfSegments());
  ROS_DEBUG("jnt_pos has %i rows",jnt_pos.rows());
 
  //get pose of final waypoint in path
  if(jnt_to_pose_solver_->JntToCart(jnt_pos, current_frame, arm_chain_.getNrOfSegments()) < 0)
  {
    printf("JntToCart returned < 0. Will not finish the path.\n");
    return;
  }

  ROS_INFO("goal in planning frame: %f %f %f  %f %f %f %f",goal.pose.pose.position.x, goal.pose.pose.position.y,goal.pose.pose.position.z,
	   goal.pose.pose.orientation.x,goal.pose.pose.orientation.y,goal.pose.pose.orientation.z,goal.pose.pose.orientation.w);

  //convert goal (PoseConstraint) to a KDL::Frame
  goal_frame.p(0) = goal.pose.pose.position.x;
  goal_frame.p(1) = goal.pose.pose.position.y;
  goal_frame.p(2) = goal.pose.pose.position.z;
  goal_frame.M = KDL::Rotation::Quaternion(goal.pose.pose.orientation.x,goal.pose.pose.orientation.y,goal.pose.pose.orientation.z,goal.pose.pose.orientation.w);

  // convert the difference in the poses into a twist
  twist = diff(goal_frame, current_frame);

  ROS_INFO("vel: %f %f %f    rot: %f %f %f", twist.vel(0),twist.vel(1),twist.vel(2),twist.rot(0),twist.rot(1),twist.rot(2));

  arm_path.set_states_size(arm_path.get_states_size()+1);
  arm_path.states[arm_path.get_states_size()-1].set_vals_size(num_joints_);

    // Determines the desired wrench from the pose error.
//   KDL::Wrench wrench_desi;
//   for (unsigned int i = 0; i < pids_.size(); ++i)
//     wrench_desi(i) = pids_[i].updatePid(pose_error_(i), twist_meas_filtered_(i), dt);

  // convert the twist into joint efforts
  for (unsigned int i = 0; i < arm_chain_.getNrOfJoints(); i++)
  {
    jnt_eff(i) = 0;
    for (unsigned int j=0; j<6; j++)
      jnt_eff(i) += (jacobian(j,i) * twist(j));

    ROS_INFO("jnt_eff[%i] = %f",i,jnt_eff(i));
    arm_path.states[arm_path.get_states_size()-1].vals[i] = jnt_eff(i) + jnt_pos(i);
  }


}

/** \brief Compute IK using pr2_ik service */
bool SBPLArmPlannerNode::computeIK(const geometry_msgs::PoseStamped &pose_stamped_msg, std::vector<double> jnt_pos, std::vector<double> &solution)
{
  manipulation_srvs::IKService::Request request;
  manipulation_srvs::IKService::Response response;

  request.data.pose_stamped = pose_stamped_msg;
  request.data.joint_names = joint_names_;
  request.data.positions.clear();
  for(int j = 0 ; j < num_joints_; ++j)
    request.data.positions.push_back(jnt_pos[j]);

	ros::ServiceClient client = node_.serviceClient<manipulation_srvs::IKService>("pr2_ik_right_arm/ik_service", true);

  if(client.call(request, response))
  {
    ROS_DEBUG("Obtained IK solution");
    solution = response.solution;
    if(solution.size() != request.data.positions.size())
    {
      ROS_ERROR("Incorrect number of elements in IK output");
      return false;
    }
    ROS_DEBUG("IK Solution");
    for(unsigned int i = 0; i < solution.size() ; ++i)
      ROS_DEBUG("%i: %f", i, solution[i]);
  }
  else
  {
    ROS_ERROR("IK service failed");
    return false;
  }

  return true;
}

/** \brief Initialize KDL Chain for using FK */
bool SBPLArmPlannerNode::initChain(std::string robot_description)
{
  KDL::Tree my_tree;
  if (!KDL::treeFromString(robot_description, my_tree))
  {
    printf("Failed to parse tree from manipulator description file.\n");
    return false;;
  }

  if (!my_tree.getChain("torso_lift_link", "r_wrist_roll_link", arm_chain_))
  {
    printf("Error: could not fetch the KDL chain for the desired manipulator. Exiting.\n"); 
    return false;
  }

  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(arm_chain_));

  return true;
}

bool SBPLArmPlannerNode::interpolatePathToGoal(std::vector<std::vector<double> > &path, double inc)
{
	unsigned int i = 0, completed_joints = 0, pind = path.size() - 2;
	double diff, angles[num_joints_];
	std::vector<double> waypoint(path[pind].size(), 0);
	while(path[pind].size() != completed_joints)
	{
		ROS_DEBUG("%i: %.3f %.3f %.3f %.3f %.3f %.3f %.3f",pind,path[pind][0],path[pind][1],path[pind][2],path[pind][3],path[pind][4],path[pind][5],path[pind][6]);
		completed_joints = 0;
		for(i = 0; i < path[pind].size(); ++i)
		{
			if(path[pind][i] == path.back()[i])
			{
				waypoint[i] = path[pind][i];
				++completed_joints;
			}
			else
			{
				diff = path.back()[i] - path[pind][i];
				if(diff < 0) // the current goal is higher than the goal angle
				{
					if(min(fabs(diff), inc) == inc)
						waypoint[i]= path[pind][i] - inc;
					else
						waypoint[i]= path[pind][i] + diff;
				}
				else //the current angle is lower than the goal angle
				{
					if(min(fabs(diff), inc) == inc)
						waypoint[i]= path[pind][i] + inc;
					else
						waypoint[i]= path[pind][i] + diff;
				}
			}
		}
		++pind;
		
		for(int j = 0; j < num_joints_; ++j)
			angles[j] = path[pind][j];
		
		if(!sbpl_arm_env_.isValidJointConfiguration(angles))
		{
			ROS_INFO("invalid configuration: %.3f %.3f %.3f %.3f %.3f %.3f %.3f",angles[0],angles[1],angles[2],angles[3],angles[4],angles[5],angles[6]);
			return false;
		}

		path.insert(path.begin()+pind, waypoint);
	}

	return true;
}

void SBPLArmPlannerNode::displayExpandedStates()
{
  std::vector<int> states;
	int last = 0;
	double angles[num_joints_], xyz_m[3], rpy_r[3];
	visualization_msgs::Marker obs_marker;

	//get expanded state IDs
	states = sbpl_arm_env_.debugExpandedStates();

	std::string frame_id = "torso_lift_link";	
	obs_marker.header.frame_id = planning_frame_;
	obs_marker.header.stamp = ros::Time::now();
	obs_marker.ns = "sbpl_arm_planner";
	obs_marker.id = 0;
	obs_marker.type = visualization_msgs::Marker::CUBE_LIST;
	obs_marker.action = 0;
	obs_marker.scale.x = env_resolution_;
	obs_marker.scale.y = env_resolution_;
	obs_marker.scale.z = env_resolution_;
	obs_marker.color.r = 0.0;
	obs_marker.color.g = 1.0;
	obs_marker.color.b = 0.5;
	obs_marker.color.a = 0.5;
	obs_marker.lifetime = ros::Duration(50.0);

	obs_marker.points.reserve(50000);
	for (unsigned int k = 0; k < states.size(); ++k) 
	{
    sbpl_arm_env_.StateID2Angles(states[k], angles);
		sbpl_arm_env_.ComputeEndEffectorPos(angles, xyz_m, rpy_r);
		
		last = obs_marker.points.size();
		obs_marker.points.resize(last + 1);		
		obs_marker.points[last].x = xyz_m[0];
		obs_marker.points[last].y = xyz_m[1];
		obs_marker.points[last].z = xyz_m[2];
	}

	ROS_DEBUG("Published markers for %d expanded nodes",obs_marker.points.size());
	marker_publisher_.publish(obs_marker);
}

void SBPLArmPlannerNode::printPath(const motion_planning_msgs::KinematicPath &arm_path)
{
	double xyz_m[3], rpy_r[3], angles_r[num_joints_];
	tf::Pose tf_pose;

	ROS_INFO("Path:");
	for(unsigned int i = 0; i < arm_path.get_states_size(); i++)
	{
		ROS_INFO("state %d: %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
						 i,arm_path.states[i].vals[0],arm_path.states[i].vals[1],arm_path.states[i].vals[2],arm_path.states[i].vals[3],
			 arm_path.states[i].vals[4],arm_path.states[i].vals[5],arm_path.states[i].vals[6]);

		sbpl_arm_env_.ComputeEndEffectorPos(angles_r, xyz_m, rpy_r);
		ROS_INFO("        xyz: %.3f %.3f %.3f   rpy: %.3f %.3f %.3f", xyz_m[0],xyz_m[1],xyz_m[2],rpy_r[0],rpy_r[1],rpy_r[2]);
	}

	// prints out the error between the final waypoint of the path and the goal pose
	if(bCartesianPlanner_)
	{
		double error_m, error_r, roll, pitch, yaw;
		for(unsigned int j = 0; j < goal_pose_constraint_.size(); ++j)
		{
			tf::poseMsgToTF(goal_pose_constraint_[j].pose.pose, tf_pose);
			btMatrix3x3 mat = tf_pose.getBasis();
			mat.getEulerZYX(yaw,pitch,roll);

			error_m = sqrt((xyz_m[0]-goal_pose_constraint_[j].pose.pose.position.x)*(xyz_m[0]-goal_pose_constraint_[j].pose.pose.position.x) +
					(xyz_m[1]-goal_pose_constraint_[j].pose.pose.position.y)*(xyz_m[1]-goal_pose_constraint_[j].pose.pose.position.y) +
					(xyz_m[2]-goal_pose_constraint_[j].pose.pose.position.z)*(xyz_m[2]-goal_pose_constraint_[j].pose.pose.position.z));

			error_r = sqrt((rpy_r[0] - yaw)*(rpy_r[0] -yaw) + (rpy_r[1] - pitch)*(rpy_r[1] - pitch) + (rpy_r[2] - roll)*(rpy_r[2] - roll));

			ROS_INFO("error: %.4f m, %.4f rad (%.4f deg)\n\n", error_m, error_r, (error_r*180.0)/PI_CONST);
		}
	}
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "plan_path_node");
  SBPLArmPlannerNode arm_planner;
  if(!arm_planner.init())
    return 1;

  ros::spin();
  return 0;
}

