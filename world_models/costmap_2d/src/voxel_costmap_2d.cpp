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
#include <costmap_2d/voxel_costmap_2d.h>

using namespace std;
using namespace robot_msgs;

namespace costmap_2d{
  VoxelCostmap2D::VoxelCostmap2D(unsigned int cells_size_x, unsigned int cells_size_y, unsigned int cells_size_z,
      double xy_resolution, double z_resolution, double origin_x, double origin_y, double origin_z, double inscribed_radius,
      double circumscribed_radius, double inflation_radius, double obstacle_range,
      double raytrace_range, double weight,
      const std::vector<unsigned char>& static_data, unsigned char lethal_threshold, unsigned int unknown_threshold, unsigned int mark_threshold) 
    : Costmap2D(cells_size_x, cells_size_y, xy_resolution, origin_x, origin_y, inscribed_radius, circumscribed_radius,
        inflation_radius, obstacle_range, (cells_size_z * z_resolution - origin_z), raytrace_range, weight, static_data, lethal_threshold),
    voxel_grid_(cells_size_x, cells_size_y, cells_size_z), xy_resolution_(xy_resolution), z_resolution_(z_resolution),
    origin_z_(origin_z), unknown_threshold_(unknown_threshold + (16 - cells_size_z)), mark_threshold_(mark_threshold)
  {
  }

  VoxelCostmap2D::~VoxelCostmap2D(){}

  void VoxelCostmap2D::resetMapOutsideWindow(double wx, double wy, double w_size_x, double w_size_y){
    ROS_ASSERT_MSG(w_size_x >= 0 && w_size_y >= 0, "You cannot specify a negative size window");

    double start_point_x = wx - w_size_x / 2;
    double start_point_y = wy - w_size_y / 2;
    double end_point_x = start_point_x + w_size_x;
    double end_point_y = start_point_y + w_size_y;

    //check start bounds
    start_point_x = max(origin_x_, start_point_x);
    start_point_y = max(origin_y_, start_point_y);

    //check end bounds
    end_point_x = min(origin_x_ + metersSizeX(), end_point_x);
    end_point_y = min(origin_y_ + metersSizeY(), end_point_y);

    unsigned int start_x, start_y, end_x, end_y;

    //check for legality just in case
    if(!worldToMap(start_point_x, start_point_y, start_x, start_y) || !worldToMap(end_point_x, end_point_y, end_x, end_y))
      return;

    ROS_ASSERT(end_x > start_x && end_y > start_y);
    unsigned int cell_size_x = end_x - start_x;
    unsigned int cell_size_y = end_y - start_y;

    //we need a map to store the obstacles in the window temporarily
    unsigned char* local_map = new unsigned char[cell_size_x * cell_size_y];
    unsigned int* local_voxel_map = new unsigned int[cell_size_x * cell_size_y];

    //copy the local window in the costmap to the local map
    unsigned char* costmap_cell = &costmap_[getIndex(start_x, start_y)];
    unsigned char* local_map_cell = local_map;
    unsigned int* voxel_grid_cell = &(voxel_grid_.getData()[getIndex(start_x, start_y)]);
    unsigned int* local_voxel = local_voxel_map;
    for(unsigned int y = 0; y < cell_size_y; ++y){
      for(unsigned int x = 0; x < cell_size_x; ++x){
        *local_map_cell = *costmap_cell;
        *local_voxel = *voxel_grid_cell;
        local_map_cell++;
        costmap_cell++;
        local_voxel++;
        voxel_grid_cell++;
      }
      costmap_cell += size_x_ - cell_size_x;
      voxel_grid_cell += size_x_ - cell_size_x;
    }

    //now we'll reset the costmap to the static map
    memcpy(costmap_, static_map_, size_x_ * size_y_ * sizeof(unsigned char));

    //the voxel grid will just go back to being unknown
    voxel_grid_.reset();

    //now we want to copy the local map back into the costmap
    costmap_cell = &costmap_[getIndex(start_x, start_y)];
    local_map_cell = local_map;
    voxel_grid_cell = &(voxel_grid_.getData()[getIndex(start_x, start_y)]);
    local_voxel = local_voxel_map;
    for(unsigned int y = 0; y < cell_size_y; ++y){
      for(unsigned int x = 0; x < cell_size_x; ++x){
        *costmap_cell = *local_map_cell;
        local_map_cell++;
        costmap_cell++;
        local_voxel++;
        voxel_grid_cell++;
      }
      costmap_cell += size_x_ - cell_size_x;
      voxel_grid_cell += size_x_ - cell_size_x;
    }

    //clean up
    delete[] local_map;
    delete[] local_voxel_map;
  }

  void VoxelCostmap2D::updateObstacles(const vector<Observation>& observations, priority_queue<CellData>& inflation_queue){
    //place the new obstacles into a priority queue... each with a priority of zero to begin with
    for(vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it){
      const Observation& obs = *it;

      const PointCloud& cloud =obs.cloud_;

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
        unsigned int mx, my, mz;
        if(!worldToMap3D(cloud.pts[i].x, cloud.pts[i].y, cloud.pts[i].z, mx, my, mz))
          continue;

        //mark the cell in the voxel grid
        voxel_grid_.markVoxel(mx, my, mz);
        unsigned int index = getIndex(mx, my);

        //push the relevant cell index back onto the inflation queue
        enqueue(index, mx, my, mx, my, inflation_queue);
      }
    }
  }

  void VoxelCostmap2D::raytraceFreespace(const Observation& clearing_observation){
    if(clearing_observation.cloud_.pts.size() == 0)
      return;

    unsigned int sensor_x, sensor_y, sensor_z;
    double ox = clearing_observation.origin_.x;
    double oy = clearing_observation.origin_.y;
    double oz = clearing_observation.origin_.z;
    
    if(!worldToMap3D(ox, oy, oz, sensor_x, sensor_y, sensor_z))
      return;

    //we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
    double map_end_x = origin_x_ + metersSizeX();
    double map_end_y = origin_y_ + metersSizeY();

    for(unsigned int i = 0; i < clearing_observation.cloud_.pts.size(); ++i){
      double wpx = clearing_observation.cloud_.pts[i].x;
      double wpy = clearing_observation.cloud_.pts[i].y;
      double wpz = clearing_observation.cloud_.pts[i].z;

      double a = wpx - ox;
      double b = wpy - oy;
      double c = wpz - oz;
      //we can only raytrace to a maximum z height
      if(wpz >= max_obstacle_height_){
        //we know we want the vector's z value to be max_z
        double t = (max_obstacle_height_ - .01 - oz) / c;
        wpx = ox + a * t;
        wpy = oy + b * t;
        wpz = oz + c * t;
      }
      //and we can only raytrace down to the floor
      else if(wpz < origin_z_){
        //we know we want the vector's z value to be 0.0
        double t = (origin_z_ - oz) / c;
        wpx = ox + a * t;
        wpy = oy + b * t;
        wpz = oz + c * t;
      }

      //the minimum value to raytrace from is the origin
      if(wpx < origin_x_){
        double t = (origin_x_ - ox) / a;
        wpx = origin_x_;
        wpy = oy + b * t;
        wpz = oz + c * t;
      }
      if(wpy < origin_y_){
        double t = (origin_y_ - oy) / b;
        wpx = ox + a * t;
        wpy = origin_y_;
        wpz = oz + c * t;
      }

      //the maximum value to raytrace to is the end of the map
      if(wpx > map_end_x){
        double t = (map_end_x - ox) / a;
        wpx = map_end_x;
        wpy = oy + b * t;
        wpz = oz + c * t;
      }
      if(wpy > map_end_y){
        double t = (map_end_y - oy) / b;
        wpx = ox + a * t;
        wpy = map_end_y;
        wpz = oz + c * t;
      }

      unsigned int point_x, point_y, point_z;
      if(worldToMap3D(wpx, wpy, wpz, point_x, point_y, point_z)){
        unsigned int cell_raytrace_range = cellDistance(raytrace_range_);
        voxel_grid_.clearVoxelLineInMap(sensor_x, sensor_y, sensor_z, point_x, point_y, point_z, costmap_, unknown_threshold_, mark_threshold_, cell_raytrace_range);
      }
    }
  }


  void VoxelCostmap2D::updateOrigin(double new_origin_x, double new_origin_y){
    //project the new origin into the grid
    int cell_ox, cell_oy;
    cell_ox = int((new_origin_x - origin_x_) / resolution_);
    cell_oy = int((new_origin_y - origin_y_) / resolution_);

    //compute the associated world coordinates for the origin cell
    //beacuase we want to keep things grid-aligned
    double new_grid_ox, new_grid_oy;
    new_grid_ox = origin_x_ + cell_ox * resolution_;
    new_grid_oy = origin_y_ + cell_oy * resolution_;

    //To save casting from unsigned int to int a bunch of times
    int size_x = size_x_;
    int size_y = size_y_;

    //we need to compute the overlap of the new and existing windows
    int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
    lower_left_x = min(max(cell_ox, 0), size_x);
    lower_left_y = min(max(cell_oy, 0), size_y);
    upper_right_x = min(max(cell_ox + size_x, 0), size_x);
    upper_right_y = min(max(cell_oy + size_y, 0), size_y);

    unsigned int cell_size_x = upper_right_x - lower_left_x;
    unsigned int cell_size_y = upper_right_y - lower_left_y;

    //we need a map to store the obstacles in the window temporarily
    unsigned char* local_map = new unsigned char[cell_size_x * cell_size_y];
    unsigned int* local_voxel_map = new unsigned int[cell_size_x * cell_size_y];

    //copy the local window in the costmap to the local map
    unsigned char* costmap_cell = &costmap_[getIndex(lower_left_x, lower_left_y)];
    unsigned char* local_map_cell = local_map;
    unsigned int* voxel_grid_cell = &(voxel_grid_.getData()[getIndex(lower_left_x, lower_left_y)]);
    unsigned int* local_voxel = local_voxel_map;
    for(unsigned int y = 0; y < cell_size_y; ++y){
      for(unsigned int x = 0; x < cell_size_x; ++x){
        *local_map_cell = *costmap_cell;
        local_map_cell++;
        costmap_cell++;
        local_voxel++;
        voxel_grid_cell++;
      }
      costmap_cell += size_x_ - cell_size_x;
      voxel_grid_cell += size_x_ - cell_size_x;
    }

    //now we'll set the costmap to be completely unknown
    memset(costmap_, NO_INFORMATION, size_x_ * size_y_ * sizeof(unsigned char));

    //the voxel grid will just go back to being unknown
    voxel_grid_.reset();

    //update the origin with the appropriate world coordinates
    origin_x_ = new_grid_ox;
    origin_y_ = new_grid_oy;

    //compute the starting cell location for copying data back in
    int start_x = lower_left_x - cell_ox;
    int start_y = lower_left_y - cell_oy;

    //now we want to copy the overlapping information back into the map, but in its new location
    costmap_cell = &costmap_[getIndex(start_x, start_y)];
    local_map_cell = local_map;
    voxel_grid_cell = &(voxel_grid_.getData()[getIndex(start_x, start_y)]);
    local_voxel = local_voxel_map;
    for(unsigned int y = 0; y < cell_size_y; ++y){
      for(unsigned int x = 0; x < cell_size_x; ++x){
        *costmap_cell = *local_map_cell;
        local_map_cell++;
        costmap_cell++;
        local_voxel++;
        voxel_grid_cell++;
      }
      costmap_cell += size_x_ - cell_size_x;
      voxel_grid_cell += size_x_ - cell_size_x;
    }

    //make sure to clean up
    delete[] local_map;
    delete[] local_voxel_map;

  }


};
