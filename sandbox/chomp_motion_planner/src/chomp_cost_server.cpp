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

#include <chomp_motion_planner/chomp_cost_server.h>
#include <kdl/jntarray.hpp>

namespace chomp
{

ChompCostServer::ChompCostServer()
{

}

ChompCostServer::~ChompCostServer()
{
}

bool ChompCostServer::init(bool advertise_service)
{
  // build the robot model
  if (!chomp_robot_model_.init())
    return false;

  // initialize the collision space
  if (!chomp_collision_space_.init())
    return false;

  // advertise the cost service
  if (advertise_service)
    get_chomp_collision_cost_server_ = node_.advertiseService("get_chomp_collision_cost", &ChompCostServer::getChompCollisionCost, this);

  return true;
}

bool ChompCostServer::getChompCollisionCost(chomp_motion_planner::GetChompCollisionCost::Request& request, chomp_motion_planner::GetChompCollisionCost::Response& response)
{
  KDL::JntArray joint_array(chomp_robot_model_.getNumKDLJoints());
  chomp_robot_model_.jointMsgToArray(request.state, joint_array);

  std::vector<KDL::Vector> joint_axis;
  std::vector<KDL::Vector> joint_pos;
  std::vector<KDL::Frame> segment_frames;
  chomp_robot_model_.getForwardKinematicsSolver()->JntToCart(joint_array, joint_pos, joint_axis, segment_frames);

  int num_links = request.links.size();
  response.costs.resize(num_links);
  response.gradient.resize(num_links);

  std::vector<ChompCollisionPoint> points;
  KDL::Vector position;
  Eigen::Vector3d potential_gradient;
  Eigen::Vector3d position_eigen;
  double potential;

  // for each link, get its collision points and accumulate the cost:
  for (unsigned int l=0; l<request.links.size(); ++l)
  {
    response.costs[l] = 0.0;
    chomp_robot_model_.getLinkCollisionPoints(request.links[l], points);
    for (unsigned int i=0; i<points.size(); i++)
    {
      points[i].getTransformedPosition(segment_frames, position);

      position_eigen = Eigen::Map<Eigen::Vector3d>(position.data);

      //bool colliding =
      chomp_collision_space_.getCollisionPointPotentialGradient(points[i],
          position_eigen, potential, potential_gradient);
      response.costs[l] += potential * points[i].getVolume();
    }
  }

  return true;
}

int ChompCostServer::run()
{
  ros::spin();
  return 0;
}

} // namespace chomp

using namespace chomp;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "chomp_cost_server");
  ChompCostServer chomp_cost_server;

  if (chomp_cost_server.init(true))
    return chomp_cost_server.run();
  else
    return 1;
}
