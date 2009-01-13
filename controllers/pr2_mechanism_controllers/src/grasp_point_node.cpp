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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#include <pr2_mechanism_controllers/grasp_point_node.h>

using namespace grasp_point_node;

GraspPointNode::GraspPointNode(std::string node_name):ros::node(node_name),tf_(*this, true, 10000000000ULL)
{
  service_prefix_ = node_name;
}

GraspPointNode::~GraspPointNode()
{
  this->unadvertise_service(service_prefix_ + "/SetGraspPoint");
}

void GraspPointNode::init()
{
  this->param<std::string>(service_prefix_ + "/robot_description",robot_description_model_,"robotdesc/pr2");
  this->param<double>(service_prefix_ + "/grasp_standoff_distance",grasp_standoff_distance_,0.05);
  this->param<std::string>(service_prefix_ + "/arm_name",arm_name_,"right_arm");
  this->param<double>(service_prefix_ + "/increment",increment_,0.01);
  initializeKinematicModel();

  this->advertise_service(service_prefix_ + "/SetGraspPoint", &GraspPointNode::processGraspPointService, this);
}

robot_desc::URDF::Link* GraspPointNode::findNextLinkInGroup(robot_desc::URDF::Link *link_current, robot_desc::URDF::Group* group)
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


int GraspPointNode::initializeKinematicModel()
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


  (ros::g_node)->get_param(robot_description_model_,xml_content);

  // wait for robotdesc/pr2 on param server
  while(!urdf_model_.loadString(xml_content.c_str()))
  {
    ROS_INFO("WARNING: grasp point node is waiting for robotdesc/pr2 in param server.  run roslaunch send.xml or similar.");
    (ros::g_node)->get_param("robotdesc/pr2",xml_content);
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
    return -1;

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
  return 1;
}

bool GraspPointNode::computeIKSolution(const tf::Pose &pose,std::vector<double> &soln)
{
  NEWMAT::Matrix g0(4,4);
  btScalar m[16];
  pose.getOpenGLMatrix(m);

  ROS_INFO("computeIKSolution: Input transform");
  for(int i=0; i< 4; i++)
  {
    for(int j=0; j<4; j++)
    {
      g0(j+1,i+1) = m[i*4+j];
    }
  }
  std::cout << g0 << endl;

  if(arm_kinematics_->computeIKFast(g0,2,init_solution_theta3_))
  {
    chooseSoln(arm_kinematics_->solution_ik_,soln);
    return true;
  }

  return false;
}


bool GraspPointNode::chooseSoln(const std::vector<std::vector<double> > &ik_solns, std::vector<double> &solution)
{

  for(int i=0; i< (int) ik_solns.size(); i++)
  {
    for(int j=0; j<NUM_JOINTS; j++)
    {
      std::cout << ik_solns[i][j] << " ";
    }
    std::cout << std::endl;
  }

  for(int i=0; i< (int) ik_solns.size(); i++)
  {
    if(ik_solns[i][1] < 0)
    {
      for(int j=0; j<NUM_JOINTS; j++)
      {
        solution[j] = ik_solns[i][j];
      }
      init_solution_theta3_ = solution[2];
      return true;
    }
  }
  return false;
}

tf::Transform GraspPointNode::calculateIntermediatePoint(tf::Transform grasp_point)  //Assumption is that the grasp point is located along the X axis of the grasp point transform
{
  btMatrix3x3 rot;
  rot.setIdentity();
  tf::Transform x(rot,btVector3(-grasp_standoff_distance_,0.0,0.0));
  tf::Transform result = grasp_point*x;
  return result;
}

bool GraspPointNode::processGraspPointService(pr2_mechanism_controllers::GraspPointSrv::request &req, pr2_mechanism_controllers::GraspPointSrv::response &resp)
{
  std_msgs::PoseStamped grasp_point_transformed;
  std_msgs::PoseStamped grasp_point_requested = req.transform;
  tf::Pose grasp_point;

  grasp_point_requested.header.stamp = ros::Time::now();

  ROS_INFO("Joint::%s %s", req.transform.header.frame_id.c_str(),root_link_name_.c_str());
  if(std::string(req.transform.header.frame_id) == root_link_name_)
  {
    grasp_point_transformed = req.transform;
  }
  else
  {
    try{
      tf_.transformPose(root_link_name_,grasp_point_requested,grasp_point_transformed);
    }
    catch(tf::LookupException& ex) {
      ROS_INFO("No Transform available Error\n");
      return false;
    }
    catch(tf::ConnectivityException& ex) {
      ROS_INFO("TF:: Connectivity Error\n");
      return false;
    }
    catch(tf::ExtrapolationException& ex) {
      ROS_INFO("Extrapolation Error\n");
      return false;
    }
    catch(tf::TransformException e) {
      return false;
    }
  }
  tf::PoseMsgToTF(grasp_point_transformed.pose,grasp_point);
  tf::Transform intermediate_point = calculateIntermediatePoint(grasp_point);

  std::vector<double> soln[2];

  soln[0].resize(NUM_JOINTS);
  soln[1].resize(NUM_JOINTS);

  if(!computeIKSolution(intermediate_point,soln[0]))
    return false;
  if(!computeIKSolution(grasp_point,soln[1]))
    return false;

  resp.traj.set_points_size(2);

  for(int i=0; i<2;i++)
  {
    resp.traj.points[i].set_positions_size(NUM_JOINTS);
    for(int j=0; j<NUM_JOINTS; j++)
    {
      resp.traj.points[i].positions[j] = soln[i][j];
    }
  }

  return true;
}

using namespace grasp_point_node;

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  GraspPointNode graspnode("grasp_point_node");
  graspnode.init();
  graspnode.spin();
  ros::fini();
  return 0;
}

