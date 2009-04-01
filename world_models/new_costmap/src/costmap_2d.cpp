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
  Costmap2D::Costmap2D(unsigned int cells_size_x, unsigned int cells_size_y, 
      double resolution, double origin_x, double origin_y, double inscribed_radius,
      double circumscribed_radius, double inflation_radius, double obstacle_range,
      double max_obstacle_height, double raytrace_range, double weight,
      const std::vector<unsigned char>& static_data, unsigned char lethal_threshold) : size_x_(cells_size_x),
  size_y_(cells_size_y), resolution_(resolution), origin_x_(origin_x), origin_y_(origin_y), static_map_(NULL),
  cost_map_(NULL), markers_(NULL), sq_obstacle_range_(obstacle_range * obstacle_range), 
  max_obstacle_height_(max_obstacle_height), raytrace_range_(raytrace_range), cached_costs_(NULL), cached_distances_(NULL), 
  inscribed_radius_(inscribed_radius), circumscribed_radius_(circumscribed_radius), inflation_radius_(inflation_radius),
  weight_(weight){
    //creat the cost_map, static_map, and markers
    cost_map_ = new unsigned char[size_x_ * size_y_];
    static_map_ = new unsigned char[size_x_ * size_y_];
    markers_ = new unsigned char[size_x_ * size_y_];
    memset(markers_, 0, size_x_ * size_y_ * sizeof(unsigned char));

    //convert our inflations from world to cell distance
    cell_inscribed_radius_ = cellDistance(inscribed_radius);
    cell_circumscribed_radius_ = cellDistance(circumscribed_radius);
    cell_inflation_radius_ = cellDistance(inflation_radius);

    //based on the inflation radius... compute distance and cost caches
    cached_costs_ = new unsigned char*[cell_inflation_radius_ + 1];
    cached_distances_ = new double*[cell_inflation_radius_ + 1];
    for(unsigned int i = 0; i <= cell_inflation_radius_; ++i){
      cached_costs_[i] = new unsigned char[cell_inflation_radius_ + 1];
      cached_distances_[i] = new double[cell_inflation_radius_ + 1];
      for(unsigned int j = 0; j <= cell_inflation_radius_; ++j){
        cached_distances_[i][j] = sqrt(i*i + j*j);
        cached_costs_[i][j] = computeCost(cached_distances_[i][j]);
      }
    }

    if(!static_data.empty()){
      ROS_ASSERT_MSG(size_x_ * size_y_ == static_data.size(), "If you want to initialize a costmap with static data, their sizes must match.");

      //make sure the inflation queue is empty at the beginning of the cycle (should always be true)
      ROS_ASSERT_MSG(inflation_queue_.empty(), "The inflation queue must be empty at the beginning of inflation");

      //initialize the costmap with static data
      for(unsigned int i = 0; i < size_x_; ++i){
        for(unsigned int j = 0; j < size_y_; ++j){
          unsigned int index = getIndex(i, j);
          //if the static value is above the threshold... it is a lethal obstacle... otherwise just take the cost
          cost_map_[index] = static_data[index] >= lethal_threshold ? LETHAL_OBSTACLE : static_data[index];
          if(cost_map_[index] == LETHAL_OBSTACLE){
            unsigned int mx, my;
            indexToCells(index, mx, my);
            enqueue(index, mx, my, mx, my, inflation_queue_);
          }
        }
      }
      //now... let's inflate the obstacles
      inflateObstacles(inflation_queue_);

      //we also want to keep a copy of the current costmap as the static map
      memcpy(static_map_, cost_map_, size_x_ * size_y_);
    }
  }

  unsigned int Costmap2D::cellDistance(double world_dist){
    double cells_dist = max(0.0, ceil(world_dist / resolution_));
    return (unsigned int) cells_dist;
  }

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
    unsigned char* cost_map_cell = &cost_map_[getIndex(start_x, start_y)];
    unsigned char* local_map_cell = local_map;
    for(unsigned int y = 0; y < cell_size_y; ++y){
      for(unsigned int x = 0; x < cell_size_x; ++x){
        *local_map_cell = *cost_map_cell;
        local_map_cell++;
        cost_map_cell++;
      }
      local_map_cell += cell_size_x;
      cost_map_cell += size_x_;
    }

    //now we'll reset the costmap to the static map
    memcpy(cost_map_, static_map_, size_x_ * size_y_ * sizeof(unsigned char));

    //now we want to copy the local map back into the costmap
    cost_map_cell = &cost_map_[getIndex(start_x, start_y)];
    local_map_cell = local_map;
    for(unsigned int y = 0; y < cell_size_y; ++y){
      for(unsigned int x = 0; x < cell_size_x; ++x){
        *cost_map_cell = *local_map_cell;
        local_map_cell++;
        cost_map_cell++;
      }
      local_map_cell += cell_size_x;
      cost_map_cell += size_x_;
    }
  }

  void Costmap2D::updateWorld(double robot_x, double robot_y, 
      const vector<Observation>& observations, const vector<Observation>& clearing_observations){
    //reset the markers for inflation
    memset(markers_, 0, size_x_ * size_y_ * sizeof(unsigned char));

    //make sure the inflation queue is empty at the beginning of the cycle (should always be true)
    ROS_ASSERT_MSG(inflation_queue_.empty(), "The inflation queue must be empty at the beginning of inflation");

    //raytrace freespace
    raytraceFreespace(clearing_observations);

    //if we raytrace X meters out... we must re-inflate obstacles within the containing square of that circle
    double inflation_window_size = 2 * (raytrace_range_ + inflation_radius_);

    //reset the inflation window.. clears all costs except lethal costs and adds them to the queue for re-propagation
    resetInflationWindow(robot_x, robot_y, inflation_window_size, inflation_window_size, inflation_queue_);

    //now we also want to add the new obstacles we've received to the cost map
    updateObstacles(observations, inflation_queue_);

    inflateObstacles(inflation_queue_);
  }

  void Costmap2D::updateObstacles(const vector<Observation>& observations, priority_queue<CellData>& inflation_queue){
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

        //push the relevant cell index back onto the inflation queue
        enqueue(index, mx, my, mx, my, inflation_queue);
      }
    }
  }

  void Costmap2D::inflateObstacles(priority_queue<CellData>& inflation_queue){
    while(!inflation_queue.empty()){
      //get the highest priority cell and pop it off the priority queue
      const CellData& current_cell = inflation_queue.top();

      unsigned int index = current_cell.index_;
      unsigned int mx = current_cell.x_;
      unsigned int my = current_cell.y_;
      unsigned int sx = current_cell.src_x_;
      unsigned int sy = current_cell.src_y_;

      //attempt to put the neighbors of the current cell onto the queue
      if(mx > 0)
        enqueue(index - 1, mx - 1, my, sx, sy, inflation_queue); 
      if(my > 0)
        enqueue(index - size_x_, mx, my - 1, sx, sy, inflation_queue);
      if(mx < size_x_ - 1)
        enqueue(index + 1, mx + 1, my, sx, sy, inflation_queue);
      if(my < size_y_ - 1)
        enqueue(index + size_x_, mx, my + 1, sx, sy, inflation_queue);

      //remove the current cell from the priority queue
      inflation_queue.pop();
    }
  }


  void Costmap2D::raytraceFreespace(const std::vector<Observation>& clearing_observations){
    for(unsigned int i = 0; i < clearing_observations.size(); ++i){
      raytraceFreespace(clearing_observations[i]);
    }
  }

  void Costmap2D::raytraceFreespace(const Observation& clearing_observation){
    //pre-comput the squared raytrace range... saves a sqrt in an inner loop
    double sq_raytrace_range = raytrace_range_ * raytrace_range_;

    //create the functor that we'll use to clear cells from the costmap
    ClearCell clearer(cost_map_);

    double ox = clearing_observation.origin_.x;
    double oy = clearing_observation.origin_.y;
    PointCloud* cloud = clearing_observation.cloud_;

    //get the map coordinates of the origin of the sensor 
    unsigned int x0, y0;
    if(!worldToMap(ox, oy, x0, y0))
      return;

    //we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
    double end_x = origin_x_ + metersSizeX();
    double end_y = origin_y_ + metersSizeY();

    //for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
    for(unsigned int i = 0; i < cloud->pts.size(); ++i){
      double wx = cloud->pts[i].x;
      double wy = cloud->pts[i].y;

      //we want to check that we scale the vector so that we only trace to the desired distance
      double sq_distance = (wx - ox) * (wx - ox) + (wy - oy) * (wy - oy);
      double scaling_fact = sq_raytrace_range / sq_distance;
      scaling_fact = scaling_fact > 1.0 ? 1.0 : scaling_fact;

      //now we also need to make sure that the enpoint we're raytracing 
      //to isn't off the costmap and scale if necessary
      double a = wx - ox;
      double b = wy - oy;

      //the minimum value to raytrace from is the origin
      if(wx < origin_x_){
        double t = (origin_x_ - ox) / a;
        wx = ox + a * t;
        wy = oy + b * t;
      }
      if(wy < origin_y_){
        double t = (origin_y_ - oy) / b;
        wx = ox + a * t;
        wy = oy + b * t;
      }

      //the maximum value to raytrace to is the end of the map
      if(wx > end_x){
        double t = (end_x - ox) / a;
        wx = ox + a * t;
        wy = oy + b * t;
      }
      if(wy > end_y){
        double t = (end_y - oy) / b;
        wx = ox + a * t;
        wy = oy + b * t;
      }

      //now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
      unsigned int x1, y1;
      worldToMap(wx, wy, x1, y1);

      //and finally... we can execute our trace to clear obstacles along that line
      raytraceLine(clearer, x0, y0, x1, y1);
    }
  }

  void Costmap2D::resetInflationWindow(double wx, double wy, double w_size_x, double w_size_y,
      priority_queue<CellData>& inflation_queue){
    //get the cell coordinates of the center point of the window
    unsigned int mx, my;
    if(!worldToMap(wx, wy, mx, my))
      return;

    //compute the bounds of the window
    double start_x = wx - w_size_x / 2;
    double start_y = wy - w_size_y / 2;
    double end_x = start_x + w_size_x;
    double end_y = start_y + w_size_y;

    //scale the window based on the bounds of the costmap
    start_x = start_x < origin_x_ ? origin_x_ : start_x;
    start_y = start_y < origin_y_ ? origin_y_ : start_y;

    end_x = end_x > origin_x_ + metersSizeX() ? origin_x_ + metersSizeX() : end_x;
    end_y = end_y > origin_y_ + metersSizeY() ? origin_y_ + metersSizeY() : end_y;

    //get the map coordinates of the bounds of the window
    unsigned int map_sx, map_sy, map_ex, map_ey;
    worldToMap(start_x, start_y, map_sx, map_sy);
    worldToMap(end_x, end_y, map_ex, map_ey);

    //we know that we want to clear all non-lethal obstacles in this window to get it ready for inflation
    unsigned int index = getIndex(map_sx, map_sy);
    unsigned char* current = &cost_map_[index];
    for(unsigned int j = map_sy; j <= map_ey; ++j){
      for(unsigned int i = map_sx; i <= map_ex; ++i){
        //if the cell is a lethal obstacle... we'll keep it and queue it, otherwise... we'll clear it
        if(*current == LETHAL_OBSTACLE)
          enqueue(index, i, j, i, j, inflation_queue);
        else
          *current = 0;
        current++;
        index++;
      }
      current += size_x_;
      index += size_x_;
    }
  }

  unsigned int Costmap2D::cellSizeX() const{
    return size_x_;
  }

  unsigned int Costmap2D::cellSizeY() const{
    return size_y_;
  }

  double Costmap2D::metersSizeX() const{
    return (size_x_ - 1 + 0.5) * resolution_;
  }

  double Costmap2D::metersSizeY() const{
    return (size_y_ - 1 + 0.5) * resolution_;
  }

  double Costmap2D::originX() const{
    return origin_x_;
  }

  double Costmap2D::originY() const{
    return origin_y_;
  }

  double Costmap2D::resolution() const{
    return resolution_;
  }

};
