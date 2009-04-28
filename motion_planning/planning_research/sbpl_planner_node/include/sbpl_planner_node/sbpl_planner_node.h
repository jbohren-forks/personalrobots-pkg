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

#include <iostream> 

/** ROS **/
#include <ros/node.h>
#include <boost/thread.hpp>

/** TF **/
#include <tf/tf.h>

/** Messages needed for trajectory control and collision map**/
#include <robot_msgs/Pose.h>
#include <robot_msgs/JointTraj.h>
#include <robot_msgs/JointTrajPoint.h>
#include <robot_msgs/CollisionMap.h>

// Costmap used for the map representation
#include <old_costmap_2d/costmap_2d.h>
#include <old_costmap_2d/costmap_node.h>

// MPGlue and sbpl headers
#include <mpglue/sbpl_planner.h>
#include <mpglue/sbpl_environment.h>
#include <mpglue/plan.h>
#include <sfl/util/strutil.hpp>
#include <sbpl/headers.h>

// Planner messages
#include <sbpl_planner_node/PlanPathSrv.h>

class SBPLPlannerNode 
{
  public:
  SBPLPlannerNode();

  virtual ~SBPLPlannerNode();

      
  private:

  boost::shared_ptr<mpglue::SBPLEnvironment> env_;
  boost::shared_ptr<mpglue::SBPLPlannerWrap> pWrap_;

  double planner_time_limit_; /* The amount of time given to the planner to find a plan */
  double allocated_time_;

  bool use_cost_map_;
  bool forward_search_;

  std::string planner_type_;
  std::string plan_stats_file_;
  std::string environment_type_;
  std::string cost_map_topic_;

  const old_costmap_2d::CostMapAccessor *cost_map_accessor_; /**< Read-only access to global cost map */
//  old_costmap_2d::CostMap2DMsg cost_map_msg_; /**< The cost map maintained incrementally from laser scans */
//  old_costmap_2d::CostMap2D* cost_map_; /**< The cost map mainatined incrementally from laser scans */

  old_costmap_2d::CostMapNode cost_map_node_;

  bool initializePlannerAndEnvironment();
  void costMapCallBack();

  bool replan(const robot_msgs::JointTrajPoint &start, const robot_msgs::JointTrajPoint &goal, robot_msgs::JointTraj &path);
  bool planPath(sbpl_planner_node::PlanPathSrv::Request &req, sbpl_planner_node::PlanPathSrv::Response &resp);

 std::vector<robot_msgs::Point> footprint_;

 boost::recursive_mutex lock_; /*!< Lock for access to class members in callbacks */
};
