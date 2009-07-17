/*********************************************************************
*
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <carrot_planner/carrot_planner.h>

namespace carrot_planner {
  ROS_REGISTER_BGP(CarrotPlanner);

  CarrotPlanner::CarrotPlanner(std::string name, costmap_2d::Costmap2DROS& costmap_ros)
  : costmap_ros_(costmap_ros){
    ros::NodeHandle n(name);
    n.param("~step_size", step_size_, costmap_ros_.resolution());
    n.param("~min_dist_from_robot", min_dist_from_robot_, 0.10);
  }

  bool CarrotPlanner::makePlan(const robot_msgs::PoseStamped& start, 
      const robot_msgs::PoseStamped& goal, std::vector<robot_msgs::PoseStamped>& plan){

    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    costmap_2d::Costmap2D costmap;
    costmap_ros_.getCostmapCopy(costmap);

    if(goal.header.frame_id != costmap_ros_.globalFrame()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
          costmap_ros_.globalFrame().c_str(), goal.header.frame_id.c_str());
      return false;
    }


    //we want to step back along the vector created by the robot's position and the goal pose until we find a legal cell
    double goal_point_x = goal.pose.position.x;
    double goal_point_y = goal.pose.position.y;

    double robot_point_x = start.pose.position.x;
    double robot_point_y = start.pose.position.y;
    
    double distance = sqrt((robot_point_x - goal_point_x) * (robot_point_x - goal_point_x)
        + (robot_point_y - goal_point_y) * (robot_point_y - goal_point_y));

    double scaling_fact = step_size_ / distance;
    double step_x = scaling_fact * (robot_point_x - goal_point_x);
    double step_y = scaling_fact * (robot_point_y - goal_point_y);
    double sign_step_x = step_x > 0 ? 1 : -1;
    double sign_step_y = step_y > 0 ? 1 : -1;

    double x = goal_point_x;
    double y = goal_point_y;

    //while we haven't made it back to the robot's position
    while(sign_step_x * x < robot_point_x && sign_step_y * y < robot_point_y){
      unsigned int cell_x, cell_y;
      if(costmap.worldToMap(x, y, cell_x, cell_y)){
        unsigned char cost = costmap.getCost(cell_x, cell_y);
        ROS_DEBUG("Cost for point %.2f, %.2f is %d", x, y, cost);
        if(cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
          double current_distance = sqrt((robot_point_x - x) * (robot_point_x - x)
              + (robot_point_y - y) * (robot_point_y - y));

          //push the current position of the robot onto the plan
          plan.push_back(start);

          //we've found a valid point, but if it is super close to us, we don't want to move
          if(current_distance > min_dist_from_robot_){
            robot_msgs::PoseStamped new_goal = goal;
            new_goal.pose.position.x = x;
            new_goal.pose.position.y = y;
            plan.push_back(new_goal);
          }
          return true;
        }
        x += step_x;
        y += step_y;
      }
    }

    ROS_WARN("The carrot planner could not find a valid plan for this goal");

    return false;
  }

};
