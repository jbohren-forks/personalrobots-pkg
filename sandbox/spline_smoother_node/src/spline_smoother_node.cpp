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

#include <spline_smoother_node/spline_smoother_node.h>
#include <spline_smoother/clamped_cubic_spline_smoother.h>
#include <spline_smoother/numerical_differentiation_spline_smoother.h>

using namespace spline_smoother;

namespace spline_smoother_node
{

SplineSmootherNode::SplineSmootherNode():
  smoother_(NULL)
{
}

SplineSmootherNode::~SplineSmootherNode()
{
  if (smoother_)
    delete smoother_;
}

bool SplineSmootherNode::init()
{
  std::string smoother_name;
  node_handle_.param("~smoother", smoother_name, std::string("ClampedCubicSplineSmoother"));

  if (smoother_name == "ClampedCubicSplineSmoother")
  {
    smoother_ = new ClampedCubicSplineSmoother();
  }
  else if (smoother_name == "NumericalDifferentiationSplineSmoother")
  {
    smoother_ = new NumericalDifferentiationSplineSmoother();
  }
  else
  {
    ROS_ERROR("Smoother %s does not exist!", smoother_name.c_str());
    return false;
  }

  smooth_joint_trajectory_server_ = node_handle_.advertiseService("smooth_joint_trajectory", &SplineSmootherNode::smoothJointTrajectory, this);

  return true;
}

int SplineSmootherNode::run()
{
  ros::spin();
  return 0;
}

bool SplineSmootherNode::smoothJointTrajectory(SmoothJointTrajectory::Request& request, SmoothJointTrajectory::Response& response)
{
  unsigned int num_joints = request.input_traj.joint_names.size();
  unsigned int traj_length = request.input_traj.points.size();

  // @TODO stretch the trajectory to handle joint velocity limits!

  std::vector<double> positions(traj_length);
  std::vector<double> velocities(traj_length);
  std::vector<double> accelerations(traj_length);
  std::vector<double> times(traj_length);

  // allocate memory in the outputs:
  response.output_traj = request.input_traj;
  for (unsigned int i=0; i<traj_length; i++)
  {
    response.output_traj.points[i].velocities.resize(num_joints);
    response.output_traj.points[i].accelerations.resize(num_joints);
  }

  for (unsigned int joint=0; joint<num_joints; joint++)
  {
    // create the input for the smoother:
    for (unsigned int i=0; i<traj_length; i++)
    {
      times[i] = request.input_traj.points[i].time;
      positions[i] = request.input_traj.points[i].positions[joint];
      velocities[i] = (request.input_traj.points[i].velocities.size() > joint)?
        request.input_traj.points[i].velocities[joint] : 0.0;
      accelerations[i] = (request.input_traj.points[i].accelerations.size() > joint)?
        request.input_traj.points[i].accelerations[joint] : 0.0;
    }
    if (!smoother_->smooth(positions, velocities, accelerations, times))
      return false;

    // now copy it back to the output:
    for (unsigned int i=0; i<traj_length; i++)
    {
      response.output_traj.points[i].velocities[joint] = velocities[i];
      response.output_traj.points[i].accelerations[joint] = accelerations[i];
    }

  }

  return true;
}

} // namespace spline_smoother_node

using namespace spline_smoother_node;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spline_smoother_node");
  SplineSmootherNode spline_smoother_node;

  if (spline_smoother_node.init())
    return spline_smoother_node.run();
  else
    return 1;
}
