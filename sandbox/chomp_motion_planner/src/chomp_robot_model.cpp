/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Mrinal Kalakrishnan */

#include <chomp_motion_planner/chomp_robot_model.h>
#include <kdl_parser/tree_parser.hpp>

using namespace std;

namespace chomp
{

ChompRobotModel::ChompRobotModel():
  robot_models_(NULL)
{
}

ChompRobotModel::~ChompRobotModel()
{
  if (robot_models_)
    delete robot_models_;
}

bool ChompRobotModel::init()
{
  // create the robot model
  robot_models_ = new planning_environment::RobotModels("robot_description");

  if (!robot_models_->loadedModels())
  {
    ROS_ERROR("Could not load robot models.");
    return false;
  }

  // get the urdf as a string:
  string urdf_string;
  if (!node_handle_.getParam(robot_models_->getDescription(), urdf_string))
  {
    return false;
  }

  // Construct the KDL tree
  if (!treeFromString(urdf_string, kdl_tree_, joint_segment_mapping_))
  {
    ROS_ERROR("Failed to construct KDL tree from URDF.");
    return false;
  }
  num_kdl_joints_ = kdl_tree_.getNrOfJoints();

  kdl_number_to_urdf_name_.resize(num_kdl_joints_);
  // Create the inverse mapping - KDL segment to joint name
  // (at the same time) Create a mapping from KDL numbers to URDF joint names and vice versa
  for (map<string, string>::iterator it = joint_segment_mapping_.begin(); it!= joint_segment_mapping_.end(); ++it)
  {
    std::string joint_name = it->first;
    std::string segment_name = it->second;
    segment_joint_mapping_.insert(make_pair(segment_name, joint_name));
    int kdl_number = kdl_tree_.getSegment(segment_name)->second.q_nr;
    kdl_number_to_urdf_name_[kdl_number] = joint_name;
    urdf_name_to_kdl_number_.insert(make_pair(joint_name, kdl_number));
  }

  // initialize the planning groups
  std::map<std::string, std::vector<std::string> > groups = robot_models_->getPlanningGroups();
  for (std::map< std::string, std::vector<std::string> >::iterator it = groups.begin(); it != groups.end() ; ++it)
  {
    ChompPlanningGroup group;
    group.num_links_ = it->second.size();
    group.num_joints_ = 0;
    group.link_names_.resize(group.num_links_);
    for (int i=0; i<group.num_links_; i++)
    {
      std::string link_name = it->second[i];
      group.link_names_[i] = link_name;
      const KDL::Segment* segment = &(kdl_tree_.getSegment(link_name)->second.segment);
      KDL::Joint::JointType joint_type =  segment->getJoint().getType();
      if (joint_type != KDL::Joint::None)
      {
        ChompJoint joint;
        joint.chomp_joint_index_ = group.num_joints_;
        joint.kdl_joint_index_ = kdl_tree_.getSegment(link_name)->second.q_nr;
        joint.kdl_joint_ = &(segment->getJoint());
        joint.link_name_ = link_name;
        joint.joint_name_ = segment_joint_mapping_[link_name];

        planning_models::KinematicModel::Joint* kin_model_joint = robot_models_->getKinematicModel()->getJoint(joint.joint_name_);
        if (planning_models::KinematicModel::RevoluteJoint* revolute_joint = dynamic_cast<planning_models::KinematicModel::RevoluteJoint*>(kin_model_joint))
        {
          joint.wrap_around = revolute_joint->continuous;
          joint.has_joint_limits = !(joint.wrap_around);
          joint.joint_limit_min = revolute_joint->limit[0];
          joint.joint_limit_max = revolute_joint->limit[1];
        }
        else if (planning_models::KinematicModel::PrismaticJoint* prismatic_joint = dynamic_cast<planning_models::KinematicModel::PrismaticJoint*>(kin_model_joint))
        {
          joint.wrap_around = false;
          joint.has_joint_limits = true;
          joint.joint_limit_min = prismatic_joint->limit[0];
          joint.joint_limit_max = prismatic_joint->limit[1];
        }
        else
        {
          ROS_WARN("CHOMP cannot handle floating or planar joints yet. Please redefine them as combinations of prismatic and revolute joints.");
        }

        group.num_joints_++;
        group.chomp_joints_.push_back(joint);
      }
    }
    planning_groups_.insert(make_pair(it->first, group));
  }

  return true;
}

} // namespace chomp
