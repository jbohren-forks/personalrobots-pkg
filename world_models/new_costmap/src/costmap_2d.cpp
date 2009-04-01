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

    size_x_ = (unsigned int) ceil(meters_size_x / resolution);
    size_y_ = (unsigned int) ceil(meters_size_y / resolution);
  }

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
    //convert our inflations from world to cell distance
    inscribed_radius_ = cellDistance(inscribed_radius);
    circumscribed_radius_ = cellDistance(circumscribed_radius);
    inflation_radius_ = cellDistance(inflation_radius);

    //based on the inflation radius... compute distance and cost caches
    cached_costs_ = new unsigned char*[inflation_radius_ + 1];
    cached_distances_ = new double*[inflation_radius_ + 1];
    for(unsigned int i = 0; i <= inflation_radius_; ++i){
      cached_costs_[i] = new unsigned char[inflation_radius_ + 1];
      cached_distances_[i] = new double[inflation_radius_ + 1];
      for(unsigned int j = 0; i <= inflation_radius_; ++j){
        cached_distances_[i][j] = sqrt(i*i + j*j);
        cached_costs_[i][j] = computeCost(cached_distances_[i][j]);
      }
    }

    if(!static_data.empty()){
      ROS_ASSERT_MSG(size_x_ * size_y_ == static_data_.size(), "If you want to initialize a costmap with static data, their sizes must match.");

      //we'll need a priority queue for inflation
      std::priority_queue<CellData*> inflation_queue;

      //initialize the costmap with static data
      for(unsigned int i = 0; i < size_x_; ++i){
        for(unsigned int j = 0; j < size_y_; ++j){
          unsigned int index = getIndex(i, j);
          //if the static value is above the threshold... it is a lethal obstacle... otherwise just take the cost
          cost_map_[index] = static_data[index] >= lethal_threshold ? LETHAL_OBSTACLE : static_data[index];
          if(cost_map_[index] == LETHAL_OBSTACLE){
            unsigned int mx, my;
            indexToCells(index, mx, my);
            enqueue(index, mx, my, mx, my, inflation_queue);
          }
        }
      }
      //now... let's inflate the obstacles
      inflateObstacles(inflation_queue);

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

    //create a priority queue
    std::priority_queue<CellData*> inflation_queue;

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
    inflateObstacles(inflation_queue);
  }

  void Costmap2D::inflateObstacles(priority_queue<CellData*>& inflation_queue){
    CellData* current_cell = NULL;
    while(!inflation_queue.empty()){
      //get the highest priority cell and pop it off the priority queue
      current_cell = inflation_queue.top();
      inflation_queue.pop();

      unsigned int index = current_cell->index_;
      unsigned int mx = current_cell->x_;
      unsigned int my = current_cell->y_;
      unsigned int sx = current_cell->src_x_;
      unsigned int sy = current_cell->src_y_;

      //attempt to put the neighbors of the current cell onto the queue
      if(mx > 0)
        enqueue(index - 1, mx - 1, my, sx, sy, inflation_queue); 
      if(my > 0)
        enqueue(index - size_x_, mx, my - 1, sx, sy, inflation_queue);
      if(mx < size_x_ - 1)
        enqueue(index + 1, mx + 1, my, sx, sy, inflation_queue);
      if(my < size_y_ - 1)
        enqueue(index + size_x_, mx, my + 1, sx, sy, inflation_queue);

      //delete the current_cell b/c it is no longer on the queue and no longer needed
      delete current_cell;
    }
  }


  void Costmap2D::raytraceFreespace(const std::vector<Observation>& clearing_scans){
    //pre-comput the squared raytrace range... saves a sqrt in an inner loop
    double sq_raytrace_range = raytrace_range_ * raytrace_range_;

    //create the functor that we'll use to clear cells from the costmap
    ClearCell clearer(cost_map_);

    for(unsigned int i = 0; i < clearing_scans.size(); ++i){
      double ox = clearing_scans[i].origin_.x;
      double oy = clearing_scans[i].origin_.y;
      PointCloud* cloud = clearing_scans[i].cloud_;

      //get the map coordinates of the origin of the sensor 
      unsigned int x0, y0;
      if(!worldToMap(ox, oy, x0, y0))
        return;

      //we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
      double end_x = origin_x_ + metersSizeX();
      double end_y = origin_y_ + metersSizeY();

      //for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
      for(unsigned int j = 0; j < cloud->pts.size(); ++j){
        double wx = cloud->pts[j].x;
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
  }

  void Costmap2D::inflateObstaclesInWindow(double wx, double wy, double w_size_x, double w_size_y){
  }

  unsigned int Costmap2D::cellSizeX() const{}

  unsigned int Costmap2D::cellSizeY() const{}

  double Costmap2D::metersSizeX() const{}

  double Costmap2D::metersSizeY() const{}

  double Costmap2D::originX() const{}

  double Costmap2D::originY() const{}

  double Costmap2D::resolution() const{}

};
