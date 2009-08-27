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
 *   * Neither the name of Willow Garage nor the names of its
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
 *
 * $Id: test_executive.cpp 15054 2009-05-07 21:21:18Z meeussen $
 *
 *********************************************************************/

/* Author: Sachin Chitta*/


#include <boost/thread/thread.hpp>
#include <door_msgs/Door.h>
// #include <ros/node.h>
#include <ros/ros.h>
#include <robot_actions/action_client.h>
#include <pr2_robot_actions/Pose2D.h>
#include <pr2_robot_actions/DoorActionState.h>
#include <robot_actions/NoArgumentsActionState.h>
#include <pr2_robot_actions/SwitchControllersState.h>
#include <nav_robot_actions/MoveBaseState.h>
#include <door_functions/door_functions.h>

#include <pr2_robot_actions/MoveArmGoal.h>
#include <pr2_robot_actions/MoveArmState.h>

using namespace ros;
using namespace std;
using namespace door_functions;


namespace planner_utils{

  typedef std::pair<double, geometry_msgs::Pose2D> cost_pair;

  bool findBestCostGoal(const geometry_msgs::Pose2D &p,
                        const geometry_msgs::Pose2D &tolerance, 
                        const geometry_msgs::Pose2D &discretization, 
                        const costmap_2D::WorldModel &world_model,
                        std::vector<geometry_msgs::Point> footprint_spec,
                        geometry_msgs::Pose2D &result, bool return_first_solution = false)
  {
    int num_x_cells  = (int)(tolerance.x/discretization.x)*2 + 1;
    int num_y_cells  = (int)(tolerance.y/discretization.y)*2 + 1;
    int num_th_cells = (int)(tolerance.th/discretization.th)*2 + 1;
    std::vector<cost_pair> cost_pose_pair;
    std::vector<geometry_msgs::Pose2D> pn;

    pn = sampleUniform(p,tolerance,discretization);

    for(int i=0; i < pn.size(); i++)
    {
      double footprint_cost = footprintCost(pn,footprint_spec,world_model,inscribed_radius,circumscribed_radius);
      if(footprint_cost > 0)
      {
        if(return_first_solution)
        {
          result = pn;
          return true;
        }
        else
        {
          cost_pose_pair.push_back(footprint_cost,pn);
        }
      }
    }
    if(cost_pose_pair.empty())
      return false;
    std::sort(cost_pose_pair.begin(),cost_pose_pair.end());
    result = cost_pose_pair.front().second;
    return true;
  }

  geometry_msgs::Pose2D sampleUniform(geometry_msgs::Pose2D &p, geometry_msgs::Pose2D &tolerance, geometry_msgs::Pose2D &discretization)
  {
    std::vector<geometry_msgs::Pose2D> result;
    double x,y,th;
    int num_x_cells  = (int)(tolerance.x/discretization.x)*2 + 1;
    int num_y_cells  = (int)(tolerance.y/discretization.y)*2 + 1;
    int num_th_cells = (int)(tolerance.th/discretization.th)*2 + 1;
    result.resize(num_x_cells*num_y_cells*num_th_cells);
    int count = 0;
    for(int i=0; i< num_x_cells; i++)
    {
      x = p.x+getUniformIndex(i)*discretization.x;
      for(int j=0; j< num_y_cells; j++)
      {
        y = p.y+getUniformIndex(j)*discretization.y;
        for(int k=0; k< num_th_cells; k++)
        {
          th = p.th+getUniformIndex(k)*discretization.th;        
          result[count].x = x;
          result[count].y = y;
          result[count].th = th;
          count++;
        }
      }
    }
    return result;
  }

  int getUniformIndex(int index)
  {
    if(index%2 ==0)
    {
      return -index/2;
    }
    else
    {
      return (index+1)/2;
    }
  }

  bool findValidGoal(const geometry_msgs::Pose2D &p,
                     const geometry_msgs::Pose2D &tolerance, 
                     const geometry_msgs::Pose2D &discretization, 
                     const costmap_2D::WorldModel &world_model,
                     std::vector<geometry_msgs::Point> footprint_spec,
                     geometry_msgs::Pose2D &result)
  {
    return findBestCostGoal(p,tolerance,discretization,world_model,footprint_spec,result,true);
  }

//we need to take the footprint of the robot into account when we calculate cost to obstacles
  double footprintCost(const geometry_msgs::Pose2D &p, 
                       std::vector<geometry_msgs::Point> footprint_spec, 
                       base_local_planner::WorldModel* world_model,
                       double inscribed_radius,
                       double circumscribed_radius)
  {
    if(footprint_spec.size() < 3)
      return -1.0;

    std::vector<geometry_msgs::Point> oriented_footprint;
    getOrientedFootprint(p,footprint_spec,oriented_footprint);

    //check if the footprint is legal
    double footprint_cost = world_model->footprintCost(Pose2DToPoint(p), oriented_footprint, inscribed_radius, circumscribed_radius);
    return footprint_cost;
  }


  void getOrientedFootprint(const geometry_msgs::Pose2D &p,
                            std::vector<geometry_msgs::Point> footprint_spec, 
                            std::vector<geometry_msgs::Point> &oriented_footprint)
  {
    oriented_footprint.clear();
    double cos_th = cos(p.th);
    double sin_th = sin(p.th);
    for(unsigned int i = 0; i < footprint_spec_.size(); ++i)
    {
      geometry_msgs::Point new_pt;
      new_pt.x = p.x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
      new_pt.y = p.y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
      oriented_footprint.push_back(new_pt);
    }
  }

}
