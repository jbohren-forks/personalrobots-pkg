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
#include <new_costmap/observation.h>

namespace costmap_2d {
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
      void updateObstacles(const std::vector<Observation>& obstacles);

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
      unsigned int size_x_;
      unsigned int size_y_;
      double resolution_;
      double origin_x_;
      double origin_y_;
      unsigned char* static_map_;
      unsigned char* cost_map_;
  };
};

#endif
