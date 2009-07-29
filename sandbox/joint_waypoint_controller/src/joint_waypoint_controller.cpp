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

#include <joint_waypoint_controller/joint_waypoint_controller.h>
#include <manipulation_srvs/SetSplineTraj.h>
#include <manipulation_srvs/QuerySplineTraj.h>
#include <manipulation_srvs/CancelSplineTraj.h>
#include <manipulation_msgs/WaypointTraj.h>
#include <spline_smoother/splines.h>
#include <algorithm>
#include <string>

namespace joint_waypoint_controller
{

JointWaypointController::JointWaypointController()
{

}

JointWaypointController::~JointWaypointController()
{

}

bool JointWaypointController::init()
{
  //smooth_joint_trajectory_server_ = node_handle_.advertiseService("smooth_joint_trajectory", &SplineSmootherNode::smoothJointTrajectory, this);

  // get some params:
  std::string filters_xml;
  std::string trajectory_type_str;
  node_handle_.param("~filters", filters_xml, std::string("<filters/>"));
  node_handle_.param("~spline_controller_prefix", spline_controller_prefix_, std::string(""));
  node_handle_.param("~trajectory_type", trajectory_type_str, std::string("cubic"));

  // convert it to lower case:
  std::transform(trajectory_type_str.begin(), trajectory_type_str.end(), trajectory_type_str.begin(), ::tolower);
  if (trajectory_type_str=="cubic")
  {
    trajectory_type_ = TRAJECTORY_TYPE_CUBIC;
  }
  else if (trajectory_type_str=="quintic")
  {
    trajectory_type_ = TRAJECTORY_TYPE_QUINTIC;
  }
  else
  {
    ROS_ERROR("Invalid trajectory type %s", trajectory_type_str.c_str());
    return false;
  }

  // initialize the filter chain:
  if (!filter_chain_.configureFromXMLString(1, filters_xml))
    return false;

  // create service clients:
  set_spline_traj_client_ = node_handle_.serviceClient<manipulation_srvs::SetSplineTraj>(spline_controller_prefix_+"SetSplineTrajectory", false);
  query_spline_traj_client_ = node_handle_.serviceClient<manipulation_srvs::QuerySplineTraj>(spline_controller_prefix_+"QuerySplineTrajectory", false);
  cancel_spline_traj_client_ = node_handle_.serviceClient<manipulation_srvs::CancelSplineTraj>(spline_controller_prefix_+"CancelSplineTrajectory", false);



  // advertise services:
  trajectory_start_server_ = node_handle_.advertiseService("TrajectoryStart", &JointWaypointController::trajectoryStart, this);
  trajectory_query_server_ = node_handle_.advertiseService("TrajectoryQuery", &JointWaypointController::trajectoryQuery, this);
  trajectory_cancel_server_ = node_handle_.advertiseService("TrajectoryCancel", &JointWaypointController::trajectoryCancel, this);

  return true;
}

int JointWaypointController::run()
{
  ros::spin();
  return 0;
}

bool JointWaypointController::trajectoryStart(pr2_mechanism_controllers::TrajectoryStart::Request &req,
                                            pr2_mechanism_controllers::TrajectoryStart::Response &resp)
{
  // first convert the input into a "WaypointTraj" message
  manipulation_msgs::WaypointTraj trajectory;
  manipulation_msgs::WaypointTraj trajectory_out;
  jointTrajToWaypointTraj(req.traj, trajectory);

  // run the filters on it:
  if (!filter_chain_.update(trajectory, trajectory_out))
  {
    ROS_ERROR("Filter chain failed to process trajectory");
    return false;
  }

  // then convert it into splines:
  manipulation_srvs::SetSplineTraj::Request sreq;
  manipulation_srvs::SetSplineTraj::Response sresp;
  waypointTrajToSplineTraj(trajectory_out, sreq.spline);

  // make the request
  if (!set_spline_traj_client_.call(sreq, sresp))
    return false;

  // copy the response back:
  resp.trajectoryid = sresp.trajectory_id;

  return true;
}

bool JointWaypointController::trajectoryQuery(pr2_mechanism_controllers::TrajectoryQuery::Request &req,
                                              pr2_mechanism_controllers::TrajectoryQuery::Response &resp)
{
  manipulation_srvs::QuerySplineTraj::Request sreq;
  manipulation_srvs::QuerySplineTraj::Response sresp;

  // copy the request
  sreq.trajectory_id = req.trajectoryid;

  // call the service
  bool result = query_spline_traj_client_.call(sreq, sresp);

  // copy the response and return
  resp.done = sresp.trajectory_status;
  resp.jointnames = sresp.joint_names;
  resp.jointpositions = sresp.joint_positions;
  resp.trajectorytime = sresp.trajectory_time;

  return result;
}

bool JointWaypointController::trajectoryCancel(pr2_mechanism_controllers::TrajectoryCancel::Request &req,
                                               pr2_mechanism_controllers::TrajectoryCancel::Response &resp)
{
  manipulation_srvs::CancelSplineTraj::Request sreq;
  manipulation_srvs::CancelSplineTraj::Response sresp;

  // copy the request
  sreq.trajectory_id = req.trajectoryid;

  // call the service
  bool result = cancel_spline_traj_client_.call(sreq, sresp);

  // copy the response and return

  return result;
}

void JointWaypointController::jointTrajToWaypointTraj(const manipulation_msgs::JointTraj& joint_traj, manipulation_msgs::WaypointTraj& waypoint_traj) const
{
  waypoint_traj.names = joint_traj.names;
  int size = joint_traj.points.size();
  int num_joints = joint_traj.names.size();

  waypoint_traj.points.resize(size);
  for (int i=0; i<size; ++i)
  {
    waypoint_traj.points[i].positions = joint_traj.points[i].positions;
    waypoint_traj.points[i].time = joint_traj.points[i].time;
    waypoint_traj.points[i].velocities.resize(num_joints, 0.0);
    waypoint_traj.points[i].accelerations.resize(num_joints, 0.0);
  }
}

void JointWaypointController::waypointTrajToSplineTraj(const manipulation_msgs::WaypointTraj& waypoint_traj, manipulation_msgs::SplineTraj& spline_traj) const
{
  spline_traj.names = waypoint_traj.names;
  int num_joints = waypoint_traj.names.size();

  int num_points = waypoint_traj.points.size();
  int num_segments = num_points-1;

  std::vector<double> coeffs(6);

  spline_traj.segments.resize(num_segments);
  for (int i=0; i<num_segments; i++)
  {
    double duration = waypoint_traj.points[i+1].time - waypoint_traj.points[i].time;
    spline_traj.segments[i].duration = ros::Time(duration);

    spline_traj.segments[i].a.resize(num_joints);
    spline_traj.segments[i].b.resize(num_joints);
    spline_traj.segments[i].c.resize(num_joints);
    spline_traj.segments[i].d.resize(num_joints);
    spline_traj.segments[i].e.resize(num_joints);
    spline_traj.segments[i].f.resize(num_joints);

    // for each joint, get the spline coefficients:
    for (int j=0; j<num_joints; j++)
    {
      if (trajectory_type_ == TRAJECTORY_TYPE_QUINTIC)
      {
        spline_smoother::getQuinticSplineCoefficients(
            waypoint_traj.points[i].positions[j],
            waypoint_traj.points[i].velocities[j],
            waypoint_traj.points[i].accelerations[j],
            waypoint_traj.points[i+1].positions[j],
            waypoint_traj.points[i+1].velocities[j],
            waypoint_traj.points[i+1].accelerations[j],
            duration,
            coeffs);

        spline_traj.segments[i].a[j] = coeffs[0];
        spline_traj.segments[i].b[j] = coeffs[1];
        spline_traj.segments[i].c[j] = coeffs[2];
        spline_traj.segments[i].d[j] = coeffs[3];
        spline_traj.segments[i].e[j] = coeffs[4];
        spline_traj.segments[i].f[j] = coeffs[5];
      }
      else if (trajectory_type_ == TRAJECTORY_TYPE_CUBIC)
      {
        spline_smoother::getCubicSplineCoefficients(
            waypoint_traj.points[i].positions[j],
            waypoint_traj.points[i].velocities[j],
            waypoint_traj.points[i+1].positions[j],
            waypoint_traj.points[i+1].velocities[j],
            duration,
            coeffs);

        spline_traj.segments[i].a[j] = coeffs[0];
        spline_traj.segments[i].b[j] = coeffs[1];
        spline_traj.segments[i].c[j] = coeffs[2];
        spline_traj.segments[i].d[j] = coeffs[3];
        spline_traj.segments[i].e[j] = 0.0;
        spline_traj.segments[i].f[j] = 0.0;
      }
    }
  }
}

} // namespace joint_waypoint_controller

using namespace joint_waypoint_controller;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_waypoint_controller");
  JointWaypointController jwc;

  if (jwc.init())
    return jwc.run();
  else
    return 1;
}
