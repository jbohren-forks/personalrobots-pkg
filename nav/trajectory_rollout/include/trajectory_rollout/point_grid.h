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
#ifndef POINT_GRID_H_
#define POINT_GRID_H_
#include <vector>
#include <list>
#include <std_msgs/Point2DFloat32.h>
#include <std_msgs/Point32.h>
#include <costmap_2d/observation.h>
#include <trajectory_rollout/world_model.h>

namespace trajectory_rollout {
  /**
   * @class PointGrid
   * @brief A class that implements the WorldModel interface to provide
   * free-space collision checks for the trajectory controller. This class
   * stores points binned into a grid and performs point-in-polygon checks when
   * necessary to determine the legality of a footprint at a given
   * position/orientation.
   */
  class PointGrid : public WorldModel {
    public:
      /**
       * @brief  Constuctor for a grid that stores points in the plane
       * @param  width The width in meters of the grid
       * @param  height The height in meters of the gird
       * @param  resolution The resolution of the grid in meters/cell
       * @param  origin The origin of the bottom left corner of the grid
       * @param  max_z The maximum height for an obstacle to be added to the grid
       * @param  obstacle_range The maximum distance for obstacles to be added to the grid
       */
      PointGrid(double width, double height, double resolution, std_msgs::Point2DFloat32 origin, double max_z, double obstacle_range);

      /**
       * @brief  Destructor for a point grid
       */
      virtual ~PointGrid(){}

      /**
       * @brief  Returns the points that lie within the cells contained in the specified range. Some of these points may be outside the range itself.
       * @param  lower_left The lower left corner of the range search 
       * @param  upper_right The upper right corner of the range search
       * @param points A vector of pointers to lists of the relevant points
       */
      void getPointsInRange(std_msgs::Point2DFloat32 lower_left, std_msgs::Point2DFloat32 upper_right, std::vector< std::list<std_msgs::Point32>* > points);

      /**
       * @brief  Checks if any points in the grid lie inside a convex footprint
       * @param  position The position of the robot in world coordinates
       * @param  footprint The specification of the footprint of the robot in world coordinates
       * @param  inscribed_radius The radius of the inscribed circle of the robot
       * @param  circumscribed_radius The radius of the circumscribed circle of the robot
       * @return True if all points lie outside the footprint, false otherwise
       */
      virtual bool legalFootprint(const std_msgs::Point2DFloat32& position, const std::vector<std_msgs::Point2DFloat32>& footprint,
          double inscribed_radius, double circumscribed_radius);

      /**
       * @brief  Inserts observations from sensors into the point grid
       * @param observations The observations from various sensors 
       * @param laser_outline The polygon of the active sensor region
       */
      virtual void updateWorld(const std::vector<costmap_2d::Observation>& observations, const std::vector<std_msgs::Point2DFloat32>& laser_outline);

      /**
       * @brief  Convert from world coordinates to grid coordinates
       * @param  pt A point in world space 
       * @param  gx The x coordinate of the corresponding grid cell to be set by the function
       * @param  gy The y coordinate of the corresponding grid cell to be set by the function
       * @return True if the conversion was successful, false otherwise
       */
      inline bool gridCoords(std_msgs::Point2DFloat32 pt, unsigned int& gx, unsigned int& gy) const {
        if(pt.x < origin_.x || pt.y < origin_.y){
          gx = 0;
          gy = 0;
          return false;
        }
        gx = (int) ((pt.x - origin_.x)/resolution_);
        gy = (int) ((pt.y - origin_.y)/resolution_);

        if(gx >= width_ || gy >= height_){
          gx = 0;
          gy = 0;
          return false;
        }

        return true;
      }

      /**
       * @brief  Convert from world coordinates to grid coordinates
       * @param  pt A point in world space 
       * @param  gx The x coordinate of the corresponding grid cell to be set by the function
       * @param  gy The y coordinate of the corresponding grid cell to be set by the function
       * @return True if the conversion was successful, false otherwise
       */
      inline bool gridCoords(std_msgs::Point32 pt, unsigned int& gx, unsigned int& gy) const {
        if(pt.x < origin_.x || pt.y < origin_.y){
          gx = 0;
          gy = 0;
          return false;
        }
        gx = (int) ((pt.x - origin_.x)/resolution_);
        gy = (int) ((pt.y - origin_.y)/resolution_);

        if(gx >= width_ || gy >= height_){
          gx = 0;
          gy = 0;
          return false;
        }

        return true;
      }

      /**
       * @brief  Converts cell coordinates to an index value that can be used to look up the correct grid cell
       * @param gx The x coordinate of the cell 
       * @param gy The y coordinate of the cell 
       * @return The index of the cell in the stored cell list
       */
      inline unsigned int gridIndex(unsigned int gx, unsigned int gy) const {
        /*
         * (0, 0) ---------------------- (width, 0)
         *  |                               |
         *  |                               |
         *  |                               |
         *  |                               |
         *  |                               |
         * (0, height) ----------------- (width, height)
         */
        return(gx + gy * width_);
      }

      /**
       * @brief  Check the orientation of a pt c with respect to the vector a->b
       * @param a The start point of the vector 
       * @param b The end point of the vector 
       * @param c The point to compute orientation for
       * @return orient(a, b, c) < 0 ----> Left, orient(a, b, c) > 0 ----> Right 
       */
      inline double orient(const std_msgs::Point2DFloat32& a, const std_msgs::Point2DFloat32& b, const std_msgs::Point32& c){
        double acx = a.x - c.x;
        double bcx = b.x - c.x;
        double acy = a.y - c.y;
        double bcy = b.y - c.y;
        return acx * bcy - acy * bcx;
      }

      /**
       * @brief  Check if a point is in a polygon
       * @param pt The point to be checked 
       * @param poly The polygon to check against
       * @return True if the point is in the polygon, false otherwise
       */
      bool ptInPolygon(const std_msgs::Point32& pt, const std::vector<std_msgs::Point2DFloat32>& poly);

      /**
       * @brief  Insert a point into the point grid
       * @param pt The point to be inserted 
       */
      void insert(std_msgs::Point32 pt);

      /**
       * @brief  Removes points from the grid that lie within the polygon
       * @param poly A specification of the polygon to clear from the grid 
       */
      void removePointsInPolygon(const std::vector<std_msgs::Point2DFloat32> poly);

    private:
      double resolution_; ///< @brief The resolution of the grid in meters/cell
      std_msgs::Point2DFloat32 origin_; ///< @brief The origin point of the grid
      unsigned int width_; ///< @brief The width of the grid in cells
      unsigned int height_; ///< @brief The height of the grid in cells
      std::vector< std::list<std_msgs::Point32> > cells_; ///< @brief Storage for the cells in the grid
      double max_z_;  ///< @brief The height cutoff for adding points as obstacles
      double sq_obstacle_range_;  ///< @brief The square distance at which we no longer add obstacles to the grid
      std::vector< std::list<std_msgs::Point32>* > points_;  ///< @brief The lists of points returned by a range search, made a member to save on memory allocation

  };
};
#endif
