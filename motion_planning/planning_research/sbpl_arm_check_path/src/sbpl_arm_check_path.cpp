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

#include <sbpl_arm_check_path/sbpl_arm_check_path.h>

using namespace std;
using namespace sbpl_arm_check_path;

SBPLArmCheckPath::SBPLArmCheckPath(std::string node_name):ros::Node(node_name),node_name_(node_name)
{
  param ("~num_joints", num_joints_, 7);
  param ("~torso_arm_offset_x", torso_arm_offset_x_, 0.0);
  param ("~torso_arm_offset_y", torso_arm_offset_y_, 0.0);
  param ("~torso_arm_offset_z", torso_arm_offset_z_, 0.0);
  advertiseService(node_name + "/CheckPath", &SBPLArmCheckPath::checkPath, this);
  subscribe(collision_map_topic_, collision_map_, &SBPLArmCheckPath::collisionMapCallback,1);

};

SBPLArmCheckPath::~SBPLArmCheckPath()
{
  unsubscribe(collision_map_topic_);
  unadvertiseService(node_name_ + "/CheckPath");
};

bool SBPLArmCheckPath::initializeEnvironment()
{
  std::string env_config_string;
  std::string planner_config_string;

  this->param<std::string>("~sbpl_env_config", env_config_string, " ");
  env_config_fp_ = fopen("sbpl_env_cfg_tmp.txt","w");
  fprintf(env_config_fp_,"%s",env_config_string.c_str());

  this->param<std::string>("~sbpl_planner_config", planner_config_string, " ");
  planner_config_fp_ = fopen("sbpl_planner_cfg_tmp.txt","w");
  fprintf(planner_config_fp_,"%s",planner_config_string.c_str());

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

  return true;
}

void SBPLArmCheckPath::collisionMapCallback()
{
  int num_boxes = (int) collision_map_.boxes.size();
  double **sbpl_boxes = new double*[num_boxes];

  for(int i=0; i < num_boxes; i++)
  {
    sbpl_boxes[i] = new double[6];

    sbpl_boxes[i][0] = collision_map_.boxes[i].center.x - torso_arm_offset_x_;
    sbpl_boxes[i][1] = collision_map_.boxes[i].center.y - torso_arm_offset_y_;
    sbpl_boxes[i][2] = collision_map_.boxes[i].center.z - torso_arm_offset_z_;

    sbpl_boxes[i][0] = collision_map_.boxes[i].extents.x;
    sbpl_boxes[i][1] = collision_map_.boxes[i].extents.y;
    sbpl_boxes[i][2] = collision_map_.boxes[i].extents.z;
  }
  pr2_arm_env_.AddObstaclesToEnv(sbpl_boxes,num_boxes);

  for(int i=0; i < num_boxes; i++)
  {
    delete sbpl_boxes[i];
  }
  delete sbpl_boxes;
}

bool SBPLArmCheckPath::checkPath(sbpl_arm_check_path::CheckPathSrv::Request &req, sbpl_arm_check_path::CheckPathSrv::Response &resp)
{
  int num_points = req.traj.points.size();
  double **sbpl_joint_traj = new double*[num_points];

  for(int i=0; i<num_points; i++)
  {
    sbpl_joint_traj[i] = new double[num_joints_];

    for(int j=0; j<num_joints_; j++)
    {
      sbpl_joint_traj[i][j] = req.traj.points[i].positions[j];
    }
  }
  return pr2_arm_env_.isPathValid(sbpl_joint_traj,num_points);//returns false if in collision and true if not in collision
}

int main(int argc, char *argv[])
{
  ros::init(argc,argv); 
  SBPLArmCheckPath node("sbpl_arm_check_path_node");

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
