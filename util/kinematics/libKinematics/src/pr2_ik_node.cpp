//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.


#include <libKinematics/pr2_ik_node.h>

LibKinematicsNode::LibKinematicsNode(std::string node_name,std::string arm_name):ros::Node(node_name),arm_name_(arm_name)
{};

LibKinematicsNode::~LibKinematicsNode(){};

bool LibKinematicsNode::initializeKinematicModel()
{
  std::vector<NEWMAT::Matrix> axis;
  std::vector<NEWMAT::Matrix> anchor;
  std::vector<std::string> joint_type;
  std::vector<double> min_joint_limits;
  std::vector<double> max_joint_limits;
  std::vector<double> angle_multipliers;

  NEWMAT::Matrix aj(3,1);
  NEWMAT::Matrix an(3,1);

  std::string xml_content;
  std::vector<robot_desc::URDF::Group*> groups;


  (ros::g_node)->getParam("robotdesc/pr2",xml_content);

  // wait for robotdesc/pr2 on param server
  while(!urdf_model_.loadString(xml_content.c_str()))
  {
    ROS_INFO("WARNING: grasp point node is waiting for robotdesc/pr2 in param server.  run roslaunch send.xml or similar.");
    (ros::g_node)->getParam("robotdesc/pr2",xml_content);
    usleep(100000);
  }

  urdf_model_.getGroups(groups);

  int group_index = -1;

  for(int i=0; i < (int) groups.size(); i++)
  {
    if(groups[i]->name == arm_name_)
    {
      group_index = i;
      break;
    }
  }

  if(!group_index) 
    return false;

  if((int) groups[group_index]->linkRoots.size() != 1)
  {
    fprintf(stderr,"robot_kinematics.cpp::Too many roots in serial chain!\n");
    return -1;
  }

  robot_desc::URDF::Link *link_current = groups[group_index]->linkRoots[0];
  root_link_name_ = link_current->name;
  joint_type.resize(NUM_JOINTS);

  min_joint_limits.resize(NUM_JOINTS);
  max_joint_limits.resize(NUM_JOINTS);
  angle_multipliers.resize(NUM_JOINTS);
  for(int i=0; i<NUM_JOINTS; i++)
  {
    aj << fabs(link_current->joint->axis[0]) << fabs(link_current->joint->axis[1]) << fabs(link_current->joint->axis[2]);
    axis.push_back(aj);
    if(i > 0)
    {
      an(1,1) = an(1,1) + link_current->xyz[0];
      an(2,1) = an(2,1) + link_current->xyz[1];
      an(3,1) = an(3,1) + link_current->xyz[2];
    }
    else
    {
      an(1,1) = 0.0;
      an(2,1) = 0.0;
      an(3,1) = 0.0;
    }
    anchor.push_back(an);   
    min_joint_limits[i] = link_current->joint->limit[0];
    max_joint_limits[i] = link_current->joint->limit[1];
    angle_multipliers[i] = link_current->joint->axis[0]*fabs(link_current->joint->axis[0]) +  link_current->joint->axis[1]*fabs(link_current->joint->axis[1]) +  link_current->joint->axis[2]*fabs(link_current->joint->axis[2]);
    ROS_INFO("Adding joint %s\naxis: %f %f %f\nanchor: %f %f %f",link_current->joint->name.c_str(),link_current->joint->axis[0],link_current->joint->axis[1],link_current->joint->axis[2],an(1,1),an(2,1),an(3,1));
    if(min_joint_limits[i] == 0.0 && max_joint_limits[i] == 0.0)
    {
      ROS_INFO("Continuous joint");
      min_joint_limits[i] = -M_PI;
      max_joint_limits[i] = M_PI;
    }
    if(i==2)
    {
      init_solution_theta3_ = (min_joint_limits[i]+max_joint_limits[i])/2.0;
      ROS_INFO("Initial guess for inverse kinematics: %f",init_solution_theta3_);
    }
    ROS_INFO("Joint limits %f, %f\n",min_joint_limits[i],max_joint_limits[i]);

    link_current = findNextLinkInGroup(link_current, groups[group_index]);
  }

  for(int i=0; i < 7; i++)
    joint_type[i] = std::string("ROTARY");

  arm_kinematics_ = new kinematics::arm7DOF(anchor,axis,joint_type);
  arm_kinematics_->SetJointLimits(min_joint_limits,max_joint_limits);
  arm_kinematics_->increment_ = increment_;
  arm_kinematics_->setAngleMultipliers(angle_multipliers);
  return true;
}


robot_desc::URDF::Link* LibKinematicsNode::findNextLinkInGroup(robot_desc::URDF::Link *link_current, robot_desc::URDF::Group* group)
{
  std::vector<robot_desc::URDF::Link*>::iterator link_iter;

#ifdef DEBUG
  cout << "Current link:: " << link_current->name << endl; 
#endif
  for(link_iter = link_current->children.begin(); link_iter != link_current->children.end(); link_iter++)
  {
#ifdef DEBUG
    cout << (*link_iter)->name;
#endif
    if((*link_iter)->insideGroup(group))
      return *link_iter;
  }
  return NULL;
}

bool LibKinematicsNode::init()
{
  if(!initializeKinematicModel())
    return false;

  return true;
}


using namespace kinematics;

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  LibKinematicsNode kn("lib_kinematic_node","right_arm");
  kn.init();

  NEWMAT::Matrix g(4,4); 
  g(1,1) = 1.0;
  g(1,2) = 0.0;
  g(1,3) = 0.0;
  g(2,1) = 0.0;
  g(2,2) = 1.0;
  g(2,3) = 0.0;
  g(3,1) = 0.0;
  g(3,2) = 0.0;
  g(3,3) = 1.0;

  g(1,4) = 0.75;
  g(2,4) = 0.0;
  g(3,4) = 0.0;
  g(4,4) = 1.0;

 kn.arm_kinematics_->computeIKFast(g,2,kn.init_solution_theta3_);

 for(int i=0; i < kn.arm_kinematics_->solution_ik_.size(); i++)
 { 
   for(int j=0; j< 7; j++)
   {
     printf("%f ",kn.arm_kinematics_->solution_ik_[i][j]);
   }
   printf("\n");
 }

  ros::fini();
  return 0;
}

