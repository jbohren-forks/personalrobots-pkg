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
#ifndef COSTMAP_COSTMAP_2D_H_
#define COSTMAP_COSTMAP_2D_H_

#include <vector>
#include <queue>
#include <new_costmap/observation.h>
#include <robot_msgs/PointCloud.h>

namespace costmap_2d {
  static const unsigned char NO_INFORMATION = 255;
  static const unsigned char LETHAL_OBSTACLE = 254;
  static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;

  //for priority queue propagation
  class CellData {
    public:
      CellData(double d, double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy) : distance_(d), 
      index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy) {}
      double distance_;
      unsigned int index_;
      unsigned int x_, y_;
      unsigned int src_x_, src_y_;
  };

  bool operator<(const CellData &a, const CellData &b){
    return a.distance_ < b.distance_;
  }

  /**
   * @class Costmap
   * @brief A 2D costmap provides a mapping between points in the world and their associated "costs".
   */
  class Costmap2D {
    public:
      /**
       * @brief  Constructor for a costmap
       * @param  meters_size_x The x size of the map in meters
       * @param  meters_size_y The y size of the map in meters
       * @param  resolution The resolution of the map in meters/cell
       * @param  origin_x The x origin of the map
       * @param  origin_y The y origin of the map
       */
      Costmap2D(double meters_size_x, double meters_size_y, 
          double resolution, double origin_x, double origin_y);

      /**
       * @brief  Constructor for a costmap
       * @param  cells_size_x The x size of the map in cells
       * @param  cells_size_y The y size of the map in cells
       * @param  resolution The resolution of the map in meters/cell
       * @param  origin_x The x origin of the map
       * @param  origin_y The y origin of the map
       */
      Costmap2D(unsigned int cells_size_x, unsigned int cells_size_y, 
          double resolution, double origin_x, double origin_y);

      /**
       * @brief  Get the cost of a cell in the costmap
       * @param mx The x coordinate of the cell 
       * @param my The y coordinate of the cell 
       * @return The cost of the cell
       */
      unsigned char getCellCost(unsigned int mx, unsigned int my) const;

      /**
       * @brief  Convert from map coordinates to world coordinates
       * @param  mx The x map coordinate
       * @param  my The y map coordinate
       * @param  wx Will be set to the associated world x coordinate
       * @param  wy Will be set to the associated world y coordinate
       */
      void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const;

      /**
       * @brief  Convert from map coordinates to world coordinates
       * @param  wx The x world coordinate
       * @param  wy The y world coordinate
       * @param  mx Will be set to the associated map x coordinate
       * @param  my Will be set to the associated map y coordinate
       * @return true if the conversion was successful (legal bounds) false otherwise
       */
      bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const;

      inline unsigned int getIndex(unsigned int mx, unsigned int my) const{
        return my * size_x_ + mx;
      }

      inline void updateCellCost(unsigned int index, unsigned char cost){
        unsigned char* cell_cost = &cost_map_[index];
        if(*cell_cost == NO_INFORMATION)
          *cell_cost = cost;
        else
          *cell_cost = std::max(cost, *cell_cost);
      }

      inline unsigned char computeCost(double distance) const {
        unsigned char cost = 0;
        if(distance == 0)
          cost = LETHAL_OBSTACLE;
        else if(distance <= inscribed_radius_)
          cost = INSCRIBED_INFLATED_OBSTACLE;
        else {
          double factor = weight_ / (1 + pow(distance - inscribed_radius_, 2));
          cost = (unsigned char) ((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
        }
        return cost;
      }

      inline char costLookup(int mx, int my, int src_x, int src_y){
        unsigned int dx = abs(mx - src_x);
        unsigned int dy = abs(my - src_y);
        return cached_costs_[dx][dy];
      }

      inline double distanceLookup(int mx, int my, int src_x, int src_y){
        unsigned int dx = abs(mx - src_x);
        unsigned int dy = abs(my - src_y);
        return cached_distances_[dx][dy];
      }

      /**
       * @brief  Convert from map coordinates to world coordinates without checking for legal bounds
       * @param  wx The x world coordinate
       * @param  wy The y world coordinate
       * @param  mx Will be set to the associated map x coordinate
       * @param  my Will be set to the associated map y coordinate
       */
      void worldToMapNoBounds(double wx, double wy, unsigned int& mx, unsigned int& my) const;

      /**
       * @brief  Revert to the static map outside of a specified window centered at a world coordinate
       * @param wx The x coordinate of the center point of the window in world space (meters)
       * @param wy The y coordinate of the center point of the window in world space (meters)
       * @param w_size_x The x size of the window in meters
       * @param w_size_y The y size of the window in meters
       */
      void resetMapOutsideWindow(double wx, double wy, double w_size_x, double w_size_y);

      /**
       * @brief  Insert new obstacles into the cost map
       * @param obstacles The point clouds of obstacles to insert into the map 
       */
      void updateObstacles(const std::vector<Observation>& observations);

      /**
       * @brief  Clear freespace based on any number of planar scans
       * @param clearing_scans The scans used to raytrace 
       */
      void raytraceFreespace(const std::vector<Observation>& clearing_scans);

      /**
       * @brief  Inflate obstacles within a given window centered at a point in world coordinates
       * @param wx The x coordinate of the center point of the window in world space (meters)
       * @param wy The y coordinate of the center point of the window in world space (meters)
       * @param w_size_x The x size of the window in meters
       * @param w_size_y The y size of the window in meters
       */
      void inflateObstaclesInWindow(double wx, double wy, double w_size_x, double w_size_y);

      /**
       * @brief  Accessor for the x size of the costmap in cells
       * @return The x size of the costmap
       */
      unsigned int cellSizeX() const;

      /**
       * @brief  Accessor for the y size of the costmap in cells
       * @return The y size of the costmap
       */
      unsigned int cellSizeY() const;

      /**
       * @brief  Accessor for the x size of the costmap in meters
       * @return The x size of the costmap
       */
      double metersSizeX() const;

      /**
       * @brief  Accessor for the y size of the costmap in meters
       * @return The y size of the costmap
       */
      double metersSizeY() const;

      /**
       * @brief  Accessor for the x origin of the costmap
       * @return The x origin of the costmap
       */
      double originX() const;

      /**
       * @brief  Accessor for the y origin of the costmap
       * @return The y origin of the costmap
       */
      double originY() const;

      /**
       * @brief  Accessor for the resolution of the costmap
       * @return The resolution of the costmap
       */
      double resolution() const;

    private:
      /**
       * @brief  Given an index of a cell in the costmap, place it into a priority queue for obstacle inflation
       * @param  index The index of the cell
       * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
       * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
       * @param  src_x The x index of the obstacle point inflation started at
       * @param  src_y The y index of the obstacle point inflation started at
       * @param  inflation_queue The priority queue to insert into
       * @return 
       */
      inline void enqueue(unsigned int index, unsigned int mx, unsigned int my, 
          unsigned int src_x, unsigned int src_y, std::priority_queue<CellData*>& inflation_queue){
        unsigned char* marked = &markers_[index];
        //set the cost of the cell being inserted
        if(*marked == 0){
          //we compute our distance table one cell further than the inflation radius dictates so we can make the check below
          double distance = distanceLookup(mx, my, src_x, src_y);

          //we only want to put the cell in the queue if it is within the inflation radius of the obstacle point
          if(distance > inflation_radius_)
            return;

          //assign the cost associated with the distance from an obstacle to the cell
          cost_map_[index] = costLookup(mx, my, src_x, src_y);

          //push the cell data onto the queue and mark
          inflation_queue.push(new CellData(distance, index, mx, my, src_x, src_y));
          *marked = 1;
        }
      }

      /**
       * @brief  Given a priority queue with the actual obstacles, compute the inflated costs for the costmap
       * @param  inflation_queue A priority queue contatining the cell data for the actual obstacles
       */
      void inflateObstacles(std::priority_queue<CellData*>& inflation_queue);

      unsigned int size_x_;
      unsigned int size_y_;
      double resolution_;
      double origin_x_;
      double origin_y_;
      unsigned char* static_map_;
      unsigned char* cost_map_;
      unsigned char* markers_;
      double sq_obstacle_range_;
      double max_obstacle_height_;
      unsigned char** cached_costs_;
      double** cached_distances_;
      double inscribed_radius_, circumscribed_radius, inflation_radius_;
      double weight_;
  };
};

#endif
