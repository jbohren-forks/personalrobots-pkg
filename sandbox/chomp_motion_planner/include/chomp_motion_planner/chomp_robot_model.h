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

#ifndef CHOMP_ROBOT_MODEL_H_
#define CHOMP_ROBOT_MODEL_H_

#include <ros/ros.h>
#include <planning_environment/robot_models.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

#include <map>
#include <vector>
#include <string>

namespace chomp
{

/**
 * \brief Contains all the information needed for CHOMP planning.
 *
 * Initializes the robot models for CHOMP
 * from the "robot_description" parameter, in the same way that
 * planning_environment::RobotModels does it.
 */
class ChompRobotModel
{
public:

  /**
   * \brief Contains information about a single joint for CHOMP planning
   */
  struct ChompJoint
  {
    const KDL::Joint* kdl_joint_;                               /** Pointer to the KDL joint in the tree */
    int kdl_joint_index_;                                       /** Index for use in a KDL joint array */
    int chomp_joint_index_;                                     /** Joint index for CHOMP */
    std::string joint_name_;                                    /** Name of the joint */
    std::string link_name_;                                     /** Name of the corresponding link (from planning.yaml) */
  };

  /**
   * \brief Contains information about a planning group
   */
  struct ChompPlanningGroup
  {

    int num_joints_;                                            /**< Number of joints used in planning */
    std::vector<ChompJoint> chomp_joints_;                  /**< Joints used in planning */
    int num_links_;                                             /**< Number of links used for collision checking */
    std::vector<std::string> link_names_;                       /**< Links used for collision checking */
  };

  ChompRobotModel();
  virtual ~ChompRobotModel();

  /**
   * \brief Initializes the robot models for CHOMP
   *
   * \return true if successful, false if not
   */
  bool init();

  /**
   * \brief Gets the planning group corresponding to the group name
   */
  const ChompPlanningGroup* getPlanningGroup(std::string group_name) const;

  /**
   * \brief Gets the planning_environment::RobotModels class
   */
  const planning_environment::RobotModels* getRobotModels() const;

private:
  ros::NodeHandle node_handle_;                                 /**< ROS Node handle */
  planning_environment::RobotModels *robot_models_;             /**< Robot model */

  KDL::Tree kdl_tree_;                                          /**< The KDL tree of the entire robot */
  std::map<std::string, std::string> joint_segment_mapping_;    /**< Joint -> Segment mapping for KDL tree */
  std::map<std::string, std::string> segment_joint_mapping_;    /**< Segment -> Joint mapping for KDL tree */
  std::map<std::string, ChompPlanningGroup> planning_groups_;   /**< Planning group information */
};

/////////////////////////////// inline functions follow ///////////////////////////////////

inline const ChompRobotModel::ChompPlanningGroup* ChompRobotModel::getPlanningGroup(std::string group_name) const
{
  std::map<std::string, ChompRobotModel::ChompPlanningGroup>::const_iterator it = planning_groups_.find(group_name);
  if (it == planning_groups_.end())
    return NULL;
  else
    return &(it->second);
}

inline const planning_environment::RobotModels* ChompRobotModel::getRobotModels() const
{
  return robot_models_;
}

} // namespace chomp
#endif /* CHOMP_ROBOT_MODEL_H_ */
