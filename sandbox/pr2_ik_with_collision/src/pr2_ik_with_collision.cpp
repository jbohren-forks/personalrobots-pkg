/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Sachin Chitta
 */

#include <algorithm>
#include <numeric>
#include "ros/node.h"
#include <tf_conversions/tf_kdl.h>
#include <pr2_ik_with_collision/pr2_ik_with_collision.h>

using namespace KDL;
using namespace tf;
using namespace std;
using namespace ros;

namespace pr2_ik_with_collision
{
  typedef std::pair<double, double> cost_pair;

/*  bool sort_cost(const cost_pair& left, const cost_pair& right)
  {
    return left.first < right.first;
    }*/

  static const std::string IK_WITH_COLLISION_QUERY_NAME = "arm_ik_with_collision_query";
  static const std::string IK_WITH_COLLISION_NAME = "arm_ik_with_collision";
  static const std::string COLLISION_SERVICE = "chomp_collision_service";
  static const std::string ARM_IK_QUERY_NAME = "arm_ik_query";
  static const std::string ARM_IK_NAME = "/pr2_ik_right_arm/ik_service";

  PR2IKWithCollision::PR2IKWithCollision()
  {
    ik_client_= node_handle_.serviceClient<manipulation_srvs::IKService>(ARM_IK_NAME,true);
    ik_query_client_= node_handle_.serviceClient<manipulation_srvs::IKQuery>(ARM_IK_QUERY_NAME,true);

    ik_with_collision_service_ = node_handle_.advertiseService(IK_WITH_COLLISION_NAME,&PR2IKWithCollision::ikService,this);
    ik_with_collision_query_service_ = node_handle_.advertiseService(IK_WITH_COLLISION_QUERY_NAME,&PR2IKWithCollision::ikQuery,this);

    chomp_collision_client_ = node_handle_.serviceClient<chomp_motion_planner::GetChompCollisionCost>(COLLISION_SERVICE,true);

    node_handle_.param<std::string>("~root_name", root_name_, "torso_lift_link");
    node_handle_.param<double>("~free_angle_min", free_angle_min_, -M_PI);
    node_handle_.param<double>("~free_angle_max", free_angle_max_, M_PI);
    node_handle_.param<int>("~free_angle", free_angle_, 2);
    node_handle_.param<int>("~max_search_intervals", max_search_intervals_, 3);

    std::string link_names_string;
    node_handle_.param<std::string>("~collision_link_names",link_names_string,"");
    std::stringstream ss(link_names_string);
    std::string tmp;
    while(ss >> tmp)
    {
      collision_link_names_.push_back(tmp);
    }
    ROS_INFO("Node handle: name:%s ns:%s",node_handle_.getName().c_str(),node_handle_.getNamespace().c_str());
    ROS_INFO("Link names string: %s",link_names_string.c_str());
    ROS_INFO("Number of collision links: %d", collision_link_names_.size());
    for(int i=0; i < collision_link_names_.size(); i++)
      ROS_INFO("Collision link name: %d %s",i,collision_link_names_[i].c_str());
  }

  PR2IKWithCollision::~PR2IKWithCollision()
  {
  }

  bool PR2IKWithCollision::init()
  {
    ROS_DEBUG("Initialized IK controller");
    return true;
  }

  bool PR2IKWithCollision::ikService(manipulation_srvs::IKService::Request &request, manipulation_srvs::IKService::Response &response)
  {
    ROS_INFO("IK with collision received service request");
    int num_intervals(0);
    std::vector<cost_pair> cost_angle_pair;
    chomp_motion_planner::GetChompCollisionCost::Request req;
    chomp_motion_planner::GetChompCollisionCost::Response res;
    double diff_angle = free_angle_max_-free_angle_min_;

    ros::ServiceClient ik_client= node_handle_.serviceClient<manipulation_srvs::IKService>(ARM_IK_NAME,true);
    ros::Duration display_sleep_time(0.2);
    while(num_intervals < max_search_intervals_)
    {   

      for(int i=0; i< request.data.positions.size(); i++)
      {
        request.data.positions[i] = 0.0;
      }
      request.data.positions[free_angle_] = angles::normalize_angle(free_angle_min_ + (double)num_intervals/(double)max_search_intervals_*diff_angle);
      if(ik_client.call(request,response))
      {
        if(!ikResponseToCollisionRequest(request.data.joint_names,response,req))
        {
          ROS_ERROR("Conversion failed");
          return false;
        }
        if(!chomp_collision_client_.call(req,res))
        {
          ROS_ERROR("Could not call chomp collision server");
          return false;
        }
        std::vector<double> tmp = res.costs;
        for(unsigned int i=0; i < tmp.size(); i++)
          tmp[i] *= 1e8;
        double cost = std::accumulate(tmp.begin(),tmp.end(),0);
        //ROS_INFO("Num cost elements: %d",res.costs.size());
        ROS_INFO("IK soln: %f, %f, %f, %f, %f, %f, %f",response.solution[0],response.solution[1],response.solution[2],response.solution[3],response.solution[4],response.solution[5],response.solution[6]);
        ROS_INFO("Angle: %f, Cost: %f:: %f, %f, %f, %f, %f",response.solution[free_angle_],cost,tmp[0],tmp[1],tmp[2],tmp[3],tmp[4]);
        cost_angle_pair.push_back(cost_pair(cost,(response.solution[free_angle_])));
      }
      else
      {
        ROS_ERROR("IK response false");
      }
      num_intervals++;
      display_sleep_time.sleep();
    }

    if(cost_angle_pair.empty())
    {
      ROS_INFO("No ik solutions found");
      return false;
    }
    std::sort(cost_angle_pair.begin(),cost_angle_pair.end());
    double min_angle = cost_angle_pair[0].second;

    request.data.positions[free_angle_] = angles::normalize_angle(min_angle);
    if(ik_client.call(request,response))
    {
      ROS_INFO("Best solution: %f",min_angle);
      for(int i=0; i < response.solution.size(); i++)
      {
        ROS_INFO("%d: %f", i, response.solution[i]);
      }
      ROS_INFO("Best cost: %f",cost_angle_pair[0].first);
      return true;
    }
    return false;
  }

  bool PR2IKWithCollision::ikResponseToCollisionRequest(const std::vector<std::string> &joint_names, const manipulation_srvs::IKService::Response &ik_response, chomp_motion_planner::GetChompCollisionCost::Request &collision_request)
  {
    if(joint_names.size() != ik_response.solution.size())
    {
      ROS_ERROR("Joint names array does not have same size as ik solution");
      return false;
    }

    collision_request.links = collision_link_names_;
    collision_request.state.resize(ik_response.solution.size());
    for(unsigned int i=0; i < collision_request.state.size(); i++)
    {
      collision_request.state[i].header.stamp = ros::Time::now();
      collision_request.state[i].header.frame_id = root_name_;
      collision_request.state[i].value.resize(0);
      collision_request.state[i].value.push_back(ik_response.solution[i]);
      collision_request.state[i].joint_name = joint_names[i];
    }
    return true;
  }

  bool PR2IKWithCollision::ikQuery(manipulation_srvs::IKQuery::Request &request, manipulation_srvs::IKQuery::Response &response)
  {
    if(ik_query_client_.call(request,response))
    {
      return true;
    }
    return false;
  }
} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_ik_with_collision");
  pr2_ik_with_collision::PR2IKWithCollision pr2_ik_wc;
  ros::spin();
  return(0);
}
