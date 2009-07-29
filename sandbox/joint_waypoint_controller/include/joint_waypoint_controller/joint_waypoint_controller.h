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

#ifndef JOINT_WAYPOINT_CONTROLLER_H_
#define JOINT_WAYPOINT_CONTROLLER_H_

#include <pr2_mechanism_controllers/TrajectoryStart.h>
#include <pr2_mechanism_controllers/TrajectoryCancel.h>
#include <pr2_mechanism_controllers/TrajectoryQuery.h>
#include <ros/ros.h>
#include <filters/filter_chain.h>
#include <manipulation_msgs/WaypointTraj.h>
#include <manipulation_msgs/JointTraj.h>
#include <manipulation_msgs/SplineTraj.h>

namespace joint_waypoint_controller
{

class JointWaypointController
{
public:
  JointWaypointController();
  virtual ~JointWaypointController();

  bool init();
  int run();

  /**
   * @brief Service provided to set trajectories
   * @param req The request containing the trajectory to be executed
   * @param resp The response contains the id assigned to the trajectory
   */
  bool trajectoryStart(pr2_mechanism_controllers::TrajectoryStart::Request &req,
                                              pr2_mechanism_controllers::TrajectoryStart::Response &resp);

  /**
   * @brief Service provided to query trajectories
   * @param req The request contains the id of the trajectory about which you need information (Use the rosmsg tool to see the fields required for the request. e.g. rosmsg show TrajectoryQuery)
   * @param resp The response contains information about the trajectory in the following fields:
   *             (a) done: 1 if trajectory is done, 0 if ongoing, 2 if queued, 3 if deleted, 4 if failed, 5 if canceled
   *             (b) trajectorytime: If active, the current timestamp the trajectory is at, if done the total time taken for the trajectory, if queued 0
   *             (c) jointnames: the names of the joints controlled by this controller
   *             (d) jointpositions: the current joint positions
   */
  bool trajectoryQuery(pr2_mechanism_controllers::TrajectoryQuery::Request &req,
                                                pr2_mechanism_controllers::TrajectoryQuery::Response &resp);

  /**
   * @brief Service provided to cancel trajectories
   * @param req The request contains the id of the trajectory which needs to be canceled (Use the rosmsg tool to see the fields required for the request. e.g. rosmsg show TrajectoryCancel)
   */
  bool trajectoryCancel(pr2_mechanism_controllers::TrajectoryCancel::Request &req,
                                                 pr2_mechanism_controllers::TrajectoryCancel::Response &resp);



private:
  ros::NodeHandle node_handle_;

  ros::ServiceServer trajectory_start_server_;
  ros::ServiceServer trajectory_query_server_;
  ros::ServiceServer trajectory_cancel_server_;

  std::string spline_controller_prefix_;
  ros::ServiceClient set_spline_traj_client_;
  ros::ServiceClient query_spline_traj_client_;
  ros::ServiceClient cancel_spline_traj_client_;

  filters::FilterChain<manipulation_msgs::WaypointTraj> filter_chain_;
  enum {
    TRAJECTORY_TYPE_CUBIC,
    TRAJECTORY_TYPE_QUINTIC
  } trajectory_type_;

  void jointTrajToWaypointTraj(const manipulation_msgs::JointTraj& joint_traj, manipulation_msgs::WaypointTraj& waypoint_traj) const;
  void waypointTrajToSplineTraj(const manipulation_msgs::WaypointTraj& waypoint_traj, manipulation_msgs::SplineTraj& spline_traj) const;
};

}

#endif /* JOINT_WAYPOINT_CONTROLLER_H_ */
