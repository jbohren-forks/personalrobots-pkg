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

#include <ros/node.h>
#include <Eigen/Array>
#include <pr2_ik/pr2_ik_solver.h>

using namespace Eigen;
using namespace pr2_ik;

PR2IKSolver::PR2IKSolver():ChainIkSolverPos()
{
  active_ = false;
  int free_angle;
  std::string urdf_xml;
  ros::Node::instance()->param("~urdf_xml", urdf_xml,std::string("/robotdesc/pr2"));
  ros::Node::instance()->param("~free_angle",free_angle,2);
  // Load robot description
  TiXmlDocument xml;
  ROS_INFO("Reading xml file from parameter server\n");
  assert(ros::Node::instance());
  std::string result;
  if (ros::Node::instance()->getParam(urdf_xml, result))
    xml.Parse(result.c_str());
  else
  {
    ROS_FATAL("Could not load the xml from parameter server: %s\n", urdf_xml.c_str());
    exit(1);
  }
  TiXmlElement *root_element = xml.RootElement();
  TiXmlElement *root = xml.FirstChildElement("robot");
  if (!root || !root_element)
  {
    ROS_FATAL("Could not parse the xml from %s\n", urdf_xml.c_str());
    exit(1);
  }
  urdf::normalizeXml(root_element);
  robot_model_.initXml(root);

  // get name of root and tip from the parameter server
  std::string root_name;
  std::string tip_name;
  if (!ros::Node::instance()->getParam("~root_name", root_name)){
    ROS_FATAL("PR2IK: No root name found on parameter server");
    exit(1);
  }
  if (!ros::Node::instance()->getParam("~tip_name", tip_name)){
    ROS_FATAL("PR2IK: No tip name found on parameter server");
    exit(1);
  }

  std::vector<tf::Transform> link_offset;

  double torso_shoulder_offset_x;

  double torso_shoulder_offset_y;

  double torso_shoulder_offset_z;

  double shoulder_upperarm_offset;

  double upperarm_elbow_offset;

  double elbow_wrist_offset;

  std::vector<double> angle_multipliers;

  std::vector<double> min_angles;

  std::vector<double> max_angles;


  int num_joints = 0;
  mechanism::Link *link = robot_model_.getLink(tip_name);

  while(link && num_joints < 7)
  {
    mechanism::Joint *joint = robot_model_.getJoint(link->joint_name_);
    if(!joint)
    {
      ROS_ERROR("Could not find joint: %s",link->joint_name_.c_str());
      exit(1);
    }
    if(joint->type_ != mechanism::JOINT_NONE && joint->type_ != mechanism::JOINT_FIXED)
    {
      link_offset.push_back(link->getOffset());
      angle_multipliers.push_back(joint->axis_[0]*fabs(joint->axis_[0]) +  joint->axis_[1]*fabs(joint->axis_[1]) +  joint->axis_[2]*fabs(joint->axis_[2]));
      ROS_INFO("Joint axis: %d, %f, %f, %f",6-num_joints,joint->axis_[0],joint->axis_[1],joint->axis_[2]);
      if(joint->type_ != mechanism::JOINT_CONTINUOUS)
      {
        min_angles.push_back(joint->joint_limit_min_);
        max_angles.push_back(joint->joint_limit_max_);
      }
      else
      {
        min_angles.push_back(-M_PI);
        max_angles.push_back(M_PI);
      }
      num_joints++;
    }
    link = robot_model_.getLink(link->parent_name_);
  } 
  std::reverse(angle_multipliers.begin(),angle_multipliers.end());
  std::reverse(min_angles.begin(),min_angles.end());
  std::reverse(max_angles.begin(),max_angles.end());
  std::reverse(link_offset.begin(),link_offset.end());

  if(num_joints != 7)
  {
    ROS_FATAL("Chain does not have 7 joints");
    exit(1);
  }
    
  torso_shoulder_offset_x = link_offset[0].getOrigin().x();
  torso_shoulder_offset_y = link_offset[0].getOrigin().y();
  torso_shoulder_offset_z = link_offset[0].getOrigin().z();

  shoulder_upperarm_offset = distance(link_offset[1]);
  upperarm_elbow_offset = distance(link_offset[3]);
  elbow_wrist_offset = distance(link_offset[5]);

  ROS_INFO("Torso shoulder offset: %f %f %f",torso_shoulder_offset_x,torso_shoulder_offset_y,torso_shoulder_offset_z);
  ROS_INFO("Shoulder upper arm offset: %f",shoulder_upperarm_offset);
  ROS_INFO("Upper arm elbow offset: %f",upperarm_elbow_offset);
  ROS_INFO("Elbow wrist offset: %f",elbow_wrist_offset);

  pr2_ik_ = new PR2IK(shoulder_upperarm_offset,
                      upperarm_elbow_offset,
                      elbow_wrist_offset,
                      torso_shoulder_offset_x,
                      torso_shoulder_offset_y,
                      torso_shoulder_offset_z,free_angle);
  pr2_ik_->setAngleMultipliers(angle_multipliers);
  pr2_ik_->setJointLimits(min_angles,max_angles);


  for(int i=0; i<7; i++)
  {
    ROS_INFO("Joint: %d, angle limits: (%f,%f)",i,min_angles[i],max_angles[i]);
  }

  ros::Node::instance()->param<double>("~search_discretization_angle",search_discretization_angle_,0.001);

  // create robot chain from root to tip
  if (!chain_.init(&robot_model_, root_name, tip_name))
    ROS_ERROR("Could not initialize chain object");

  active_ = true;
}


PR2IKSolver::~PR2IKSolver()
{
  delete pr2_ik_;
}

double PR2IKSolver::distance(const tf::Transform &transform)
{
  return sqrt(transform.getOrigin().x()*transform.getOrigin().x()+transform.getOrigin().y()*transform.getOrigin().y()+transform.getOrigin().z()*transform.getOrigin().z());
}

Eigen::Matrix4f PR2IKSolver::KDLToEigenMatrix(const KDL::Frame &p)
{
  Eigen::Matrix4f b = Eigen::Matrix4f::Identity();
  for(int i=0; i < 3; i++)
  {
    for(int j=0; j<3; j++)
    {
      b(i,j) = p.M(i,j);
    }
    b(i,3) = p.p(i);
  }
  return b;
}

double PR2IKSolver::computeEuclideanDistance(const std::vector<double> &array_1, const KDL::JntArray &array_2)
{
  double distance = 0.0;
  for(int i=0; i< (int) array_1.size(); i++)
  {
    distance += (array_1[i] - array_2(i))*(array_1[i] - array_2(i));
  }
  return sqrt(distance);
}

int PR2IKSolver::CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray &q_out)
{
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  if(pr2_ik_->free_angle_ == 0)
  {
    pr2_ik_->computeIKEfficient(b,q_init(0));
  }
  else
  {
    pr2_ik_->computeIKEfficientTheta3(b,q_init(2));
  }
  
  if(pr2_ik_->solution_ik_.size() < 1)
    return -1;

  double min_distance = 1e6;
  int min_index = -1;

  for(int i=0; i< (int) pr2_ik_->solution_ik_.size(); i++)
  {     
    double tmp_distance = computeEuclideanDistance(pr2_ik_->solution_ik_[i],q_init);
    if(tmp_distance < min_distance)
    {
      min_distance = tmp_distance;
      min_index = i;
    }
  }

  if(min_index > -1)
  {
    for(int i=0; i < (int)pr2_ik_->solution_ik_[min_index].size(); i++)
    {   
      q_out(i) = pr2_ik_->solution_ik_[min_index][i];
    }
    return 1;
  }
  else
    return -1;
}

int PR2IKSolver::CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, std::vector<KDL::JntArray> &q_out)
{
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  KDL::JntArray q;

  if(pr2_ik_->free_angle_ == 0)
  {
    pr2_ik_->computeIKEfficient(b,q_init(0));
  }
  else
  {
    pr2_ik_->computeIKEfficientTheta3(b,q_init(2));
  }
  
  if(pr2_ik_->solution_ik_.size() < 1)
    return -1;

  q.resize(7);
  q_out.clear();
  for(int i=0; i< (int) pr2_ik_->solution_ik_.size(); i++)
  {     
    for(int j=0; j < 7; j++)
    {   
      q(j) = pr2_ik_->solution_ik_[i][j];
    }
    q_out.push_back(q);
  }
  return 1;
}

bool PR2IKSolver::getCount(int &count, int max_count, int min_count)
{
  if(count > 0)
  {
    if(-count >= min_count)
    {   
      count = -count;
      return true;
    }
    else if(count+1 <= max_count)
    {
      count = count+1;
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    if(1-count <= max_count)
    {
      count = 1-count;
      return true;
    }
    else if(count-1 >= min_count)
    {
      count = count -1;
      return true;
    }
    else
      return false;
  }
}

int PR2IKSolver::CartToJntSearch(const KDL::JntArray& q_in, const KDL::Frame& p_in, std::vector<KDL::JntArray> &q_out, const double &timeout)
{
  KDL::JntArray q_init = q_in;
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  double initial_guess = q_init(pr2_ik_->free_angle_);

  ros::Time start_time = ros::Time::now();
  double loop_time = 0;
  int count = 0;

  int num_positive_increments = (int)(pr2_ik_->max_angles_[pr2_ik_->free_angle_]-initial_guess)/search_discretization_angle_;
  int num_negative_increments = (int)(initial_guess-pr2_ik_->min_angles_[pr2_ik_->free_angle_])/search_discretization_angle_;
  ROS_DEBUG("%f %f %f %d %d \n\n",initial_guess,pr2_ik_->max_angles_[pr2_ik_->free_angle_],pr2_ik_->min_angles_[pr2_ik_->free_angle_],num_positive_increments,num_negative_increments);
  while(loop_time < timeout)
  {
    if(CartToJnt(q_init,p_in,q_out) > 0)
      return 1;
    if(!getCount(count,num_positive_increments,-num_negative_increments))
      return -1;
    q_init(pr2_ik_->free_angle_) = initial_guess + search_discretization_angle_ * count;
    ROS_DEBUG("%d, %f",count,q_init(pr2_ik_->free_angle_));
    loop_time = (ros::Time::now()-start_time).toSec();
  }
  return -1;
}


