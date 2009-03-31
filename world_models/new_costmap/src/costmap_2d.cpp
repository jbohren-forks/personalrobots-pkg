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
#include <new_costmap/costmap_2d.h>

using namespace std;
using namespace robot_msgs;

namespace costmap_2d{


  Costmap2D::Costmap2D(double meters_size_x, double meters_size_y, 
      double resolution, double origin_x, double origin_y) : resolution_(resolution), 
  origin_x_(origin_x), origin_y_(origin_y) {
    //make sure that we have a legal sized cost_map
    ROS_ASSERT_MSG(meters_size_x > 0 && meters_size_y > 0, "Both the x and y dimensions of the costmap must be greater than 0.");

    size_x_ = (unsigned int) (meters_size_x / resolution);
    size_y_ = (unsigned int) (meters_size_y / resolution);
  }

  Costmap2D::Costmap2D(unsigned int cells_size_x, unsigned int cells_size_y, 
      double resolution, double origin_x, double origin_y) : size_x_(cells_size_x),
  size_y_(cells_size_y), resolution_(resolution), origin_x_(origin_x), origin_y_(origin_y) {}

  unsigned char Costmap2D::getCellCost(unsigned int mx, unsigned int my) const {
    ROS_ASSERT_MSG(mx < size_x_ && my < size_y_, "You cannot get the cost of a cell that is outside the bounds of the costmap");
    return cost_map_[my * size_x_ + mx];
  }

  void Costmap2D::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const {
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
  }

  bool Costmap2D::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const {
    if(wx < origin_x_ || wy < origin_y_)
      return false;

    mx = (int) ((wx - origin_x_) / resolution_);
    my = (int) ((wy - origin_y_) / resolution_);

    if(mx < size_x_ && my < size_y_)
      return true;

    return false;
  }

  void Costmap2D::worldToMapNoBounds(double wx, double wy, unsigned int& mx, unsigned int& my) const {
    mx = (int) ((wx - origin_x_) / resolution_);
    my = (int) ((wy - origin_y_) / resolution_);
  }

  void Costmap2D::resetMapOutsideWindow(double wx, double wy, double w_size_x, double w_size_y){
    ROS_ASSERT_MSG(w_size_x >= 0 && w_size_y >= 0, "You cannot specify a negative size window");

    double start_point_x = wx - w_size_x / 2;
    double start_point_y = wy - w_size_y / 2;

    unsigned int start_x, start_y, end_x, end_y;
    //we'll do our own bounds checking for the window
    worldToMapNoBounds(start_point_x, start_point_y, start_x, start_y);
    worldToMapNoBounds(start_point_x + w_size_x, start_point_y + w_size_y, end_x, end_y);

    //check start bounds
    start_x = start_point_x < origin_x_ ? 0 : start_x;
    start_y = start_point_y < origin_y_ ? 0 : start_y;

    //check end bounds
    end_x = end_x > size_x_ ? size_x_ : end_x;
    end_y = end_y > size_y_ ? size_y_ : end_y;

    unsigned int cell_size_x = end_x - start_x;
    unsigned int cell_size_y = end_y - start_y;

    //we need a map to store the obstacles in the window temporarily
    unsigned char local_map[cell_size_x * cell_size_y];

    //copy the local window in the costmap to the local map
    unsigned int cost_map_index = getIndex(start_x, start_y);
    unsigned int local_map_index = 0;
    for(unsigned int y = 0; y < cell_size_y; ++y){
      for(unsigned int x = 0; x < cell_size_x; ++x){
        local_map[local_map_index] = cost_map_[cost_map_index];
        local_map_index++;
        cost_map_index++;
      }
      local_map_index += cell_size_x;
      cost_map_index += size_x_;
    }

    //now we'll reset the costmap to the static map
    memcpy(cost_map_, static_map_, size_x_ * size_y_ * sizeof(unsigned char));

    //now we want to copy the local map back into the costmap
    cost_map_index = getIndex(start_x, start_y);
    local_map_index = 0;
    for(unsigned int y = 0; y < cell_size_y; ++y){
      for(unsigned int x = 0; x < cell_size_x; ++x){
        cost_map_[cost_map_index] = local_map[local_map_index];
        local_map_index++;
        cost_map_index++;
      }
      local_map_index += cell_size_x;
      cost_map_index += size_x_;
    }
  }

  void Costmap2D::updateObstacles(const std::vector<Observation>& observations){
    //reset the markers for inflation
    memset(markers_, 0, size_x_ * size_y_ * sizeof(unsigned char));

    //place the new obstacles into a priority queue... each with a priority of zero to begin with
    for(vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it){
      const Observation& obs = *it;
      const PointCloud& cloud =*(obs.cloud_);
      for(unsigned int i = 0; i < cloud.pts.size(); ++i){
        //if the obstacle is too high or too far away from the robot we won't add it
        if(cloud.pts[i].z > max_obstacle_height_)
          continue;

        //compute the squared distance from the hitpoint to the pointcloud's origin
        double sq_dist = (cloud.pts[i].x - obs.origin_.x) * (cloud.pts[i].x - obs.origin_.x)
          + (cloud.pts[i].y - obs.origin_.y) * (cloud.pts[i].y - obs.origin_.y)
          + (cloud.pts[i].z - obs.origin_.z) * (cloud.pts[i].z - obs.origin_.z);

        //if the point is far enough away... we won't consider it
        if(sq_dist >= sq_obstacle_range_)
          continue;

        //now we need to compute the map coordinates for the observation
        unsigned int mx, my;
        if(!worldToMap(cloud.pts[i].x, cloud.pts[i].y, mx, my))
          continue;

        unsigned int index = getIndex(mx, my);

        CostmapCell c;
        c.priority = 0.0;
        c.index = index;

        //push the relevant cell index back onto the inflation queue and mark it
        inflation_queue.push(c);
        markers_[index] = 1;
      }
    }
  }

  void Costmap2D::raytraceFreespace(const std::vector<Observation>& clearing_scans){}

  void Costmap2D::inflateObstaclesInWindow(double wx, double wy, double w_size_x, double w_size_y){}

  unsigned int Costmap2D::cellSizeX() const{}

  unsigned int Costmap2D::cellSizeY() const{}

  double Costmap2D::metersSizeX() const{}

  double Costmap2D::metersSizeY() const{}

  double Costmap2D::originX() const{}

  double Costmap2D::originY() const{}

  double Costmap2D::resolution() const{}

};
