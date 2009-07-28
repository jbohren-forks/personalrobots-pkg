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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <navfn/navfn_ros.h>

namespace navfn {
  //register this factory with the ros BaseGlobalPlanner factory
  ROS_REGISTER_BGP(NavfnROS);

  NavfnROS::NavfnROS(std::string name, costmap_2d::Costmap2DROS& costmap_ros) 
    : costmap_ros_(costmap_ros),  planner_(new NavFn(costmap_ros.cellSizeX(), costmap_ros.cellSizeY())), ros_node_(name) {
    //get an initial copy of the costmap
    costmap_ros_.getCostmapCopy(costmap_);

    //advertise our plan visualization
    plan_pub_ = ros_node_.advertise<visualization_msgs::Polyline>("~plan", 1);

    //read parameters for the planner
    global_frame_ = costmap_ros_.globalFrame();
    
    //we'll get the parameters for the robot radius from the costmap we're associated with
    inscribed_radius_ = costmap_ros_.inscribedRadius();
    circumscribed_radius_ = costmap_ros_.circumscribedRadius();
    inflation_radius_ = costmap_ros_.inflationRadius();
  }

  bool NavfnROS::validPointPotential(const geometry_msgs::Point& world_point){
    return getPointPotential(world_point) < COST_OBS_ROS;
  }

  double NavfnROS::getPointPotential(const geometry_msgs::Point& world_point){
    unsigned int mx, my;
    if(!costmap_.worldToMap(world_point.x, world_point.y, mx, my))
      return DBL_MAX;

    unsigned int index = my * planner_->nx + mx;
    return planner_->potarr[index];
  }

  bool NavfnROS::computePotential(const geometry_msgs::Point& world_point){
    //make sure that we have the latest copy of the costmap and that we clear the footprint of obstacles
    costmap_ros_.clearRobotFootprint();
    costmap_ros_.getCostmapCopy(costmap_);

    planner_->setCostmap(costmap_.getCharMap());

    unsigned int mx, my;
    if(!costmap_.worldToMap(world_point.x, world_point.y, mx, my))
      return false;

    int map_start[2];
    map_start[0] = 0;
    map_start[1] = 0;

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_start);
    planner_->setGoal(map_goal);

    return planner_->calcNavFnDijkstra();
  }

  void NavfnROS::clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my){
    double useless_pitch, useless_roll, yaw;
    global_pose.getBasis().getEulerZYX(yaw, useless_pitch, useless_roll);

    //set the associated costs in the cost map to be free
    costmap_.setCost(mx, my, costmap_2d::FREE_SPACE);

    double max_inflation_dist = inflation_radius_ + inscribed_radius_;

    //make sure to re-inflate obstacles in the affected region
    costmap_.reinflateWindow(global_pose.getOrigin().x(), global_pose.getOrigin().y(), max_inflation_dist, max_inflation_dist);

    //just in case we inflate over the point we just cleared
    costmap_.setCost(mx, my, costmap_2d::FREE_SPACE);

  }

  void NavfnROS::getCostmap(costmap_2d::Costmap2D& costmap)
  {
    costmap_ros_.clearRobotFootprint();
    costmap_ros_.getCostmapCopy(costmap);
  }

  bool NavfnROS::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    //clear the plan, just in case
    plan.clear();

    //make sure that we have the latest copy of the costmap and that we clear the footprint of obstacles

    getCostmap(costmap_);

    //get a handle to the global namespace
    ros::NodeHandle global_handle;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if(global_handle.mapName(goal.header.frame_id) != global_handle.mapName(global_frame_)){
      ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                global_handle.mapName(global_frame_).c_str(), global_handle.mapName(goal.header.frame_id).c_str());
      return false;
    }

    if(global_handle.mapName(start.header.frame_id) != global_handle.mapName(global_frame_)){
      ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                global_handle.mapName(global_frame_).c_str(), global_handle.mapName(start.header.frame_id).c_str());
      return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int mx, my;
    if(!costmap_.worldToMap(wx, wy, mx, my))
      return false;

    //clear the starting cell within the costmap because we know it can't be an obstacle
    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);
    clearRobotCell(start_pose, mx, my);

    planner_->setCostmap(costmap_.getCharMap());

    int map_start[2];
    map_start[0] = mx;
    map_start[1] = my;

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if(!costmap_.worldToMap(wx, wy, mx, my))
      return false;

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_start);
    planner_->setGoal(map_goal);

    //bool success = planner_->calcNavFnAstar();
    bool success = planner_->calcNavFnDijkstra(true);

    if(success){
      //extract the plan
      float *x = planner_->getPathX();
      float *y = planner_->getPathY();
      int len = planner_->getPathLen();
      ros::Time plan_time = ros::Time::now();
      for(int i = 0; i < len; ++i){
        unsigned int cell_x, cell_y;
        cell_x = (unsigned int) x[i];
        cell_y = (unsigned int) y[i];

        //convert the plan to world coordinates
        double world_x, world_y;
        costmap_.mapToWorld(cell_x, cell_y, world_x, world_y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame_;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        plan.push_back(pose);
      }
      
      //also make sure that we push the goal pose onto the end of the plan if its not empty
      if(!plan.empty()){
        //make sure the goal we push on has the same timestamp as the rest of the plan
        geometry_msgs::PoseStamped goal_copy = goal;
        goal_copy.header.stamp = plan_time;
        plan.push_back(goal_copy);
      }
    }

    //publish the plan for visualization purposes
    publishPlan(plan, 0.0, 1.0, 0.0, 0.0);
    return success;
  }

  void NavfnROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a){
    //given an empty path we won't do anything
    if(path.empty())
      return;

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    visualization_msgs::Polyline gui_path_msg;
    gui_path_msg.header.frame_id = path[0].header.frame_id;
    gui_path_msg.header.stamp = path[0].header.stamp;
    gui_path_msg.set_points_size(path.size());
    for(unsigned int i=0; i < path.size(); i++){
      gui_path_msg.points[i].x = path[i].pose.position.x;
      gui_path_msg.points[i].y = path[i].pose.position.y;
      gui_path_msg.points[i].z = path[i].pose.position.z;
    }

    gui_path_msg.color.r = r;
    gui_path_msg.color.g = g;
    gui_path_msg.color.b = b;
    gui_path_msg.color.a = a;

    plan_pub_.publish(gui_path_msg);
  }
};
