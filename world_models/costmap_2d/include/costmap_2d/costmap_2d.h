/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: E. Gil Jones
 */

#ifndef COSTMAP_2D_COSTMAP_2D_H
#define COSTMAP_2D_COSTMAP_2D_H

/**

@mainpage 

@htmlinclude manifest.html

This library provides functionality for maintaining a 2D cost map.  The primary inputs are:
  - a priori 2-D occupancy grid map
  - obstacle data
This library is initialized with the a priori map, and thereafter updated 
with obstacle data.  A sliding window model of persistence is used, in which
obstacle data lives in the map for a fixed (but configurable) amount of time
before being discarded.  The static map lives forever.

*/

//c++
#include <list>

// For time support
#include "ros/time.h"

// For point clouds
#include "std_msgs/PointCloudFloat32.h"

// A bunch of x,y points, with a timestamp
typedef struct ObstaclePts
{
  ObstaclePts() {
    pts_ = NULL;
    pts_num_ = 0;
    ts_ = 0;
  }
  void setSize(size_t num) {
    if(pts_ != NULL) {
      delete pts_;
      pts_ = NULL;
    }
    pts_ = new double[num*2];
    pts_num_ = num;
  }
  void setPoint(size_t i, double x, double y) {
    if(pts_ == NULL) return;
    if(i > pts_num_) return;
    pts_[i*2] = x;
    pts_[i*2+1] = y;
  }
  double* pts_;
  size_t pts_num_;
  ros::Time ts_;
};

class CostMap2D
{
public:
  /**
   * @brief Constructor.
   *
   * @param window_length how long to hold onto obstacle data [sec]
   */
  CostMap2D(double window_length);
  
  /**
   * @brief Destructor.
   */
  ~CostMap2D();
  
  /** 
   * @brief Initialize with a static map.
   *
   * @param width width of map [cells]
   * @param height height of map [cells]
   * @param resolution resolution of map [m/cell]
   * @data row-major obstacle data, each value in the range [0,100] to
   *       indicate probability of occupancy, -1 to indicate no
   *       information
   */
  void setStaticMap(size_t width, size_t height, 
                    double resolution, const unsigned char* data);
  
  /**
   * @brief Add dynamic obstacles to the map.
   *
   * Updates the map to contain the new dynamic obstacles, in addition to
   * any previously added obstacles (but only those that are still within
   * the window_length).
   */
  void addObstacles(const std_msgs::PointCloudFloat32* cloud);
  
  /**
   * @brief A hook to allow time-based maintenance to occur (e.g.,
   * removing stale obstacles, in the case that addObstacles() doesn't
   * get called that often).
   *
   * @param ts current time
   */
  void update(ros::Time ts);
  
  /**
   * @brief Get pointer into the obstacle map (which contains both static
   * and dynamic obstacles)
   */
  const unsigned char* getMap() const;
  
  /**
   * @brief Get index of given (x,y) point into data
   * 
   * @param x x-index of the cell
   * @param y y-index of the cell
   */
  unsigned int getMapIndex(unsigned int x, unsigned int y) const; 

  /**
   * @brief Get index of given world (x,y) point in map indexes
   * 
   * @param wx world x location of the cell
   * @param wy world y location of the cell
   * @param mx map x index return value
   * @param my map y index return value
   */
  void convertFromWorldCoordToIndexes(double wx, double wy,
                                    size_t& mx, size_t& my) const;
  /**
   * @brief Get world (x,y) point given map indexes
   * 
   * @param mx map x index location
   * @param my map y index location
   * @param wx world x return value
   * @param wy world y return value
   */
  void convertFromIndexesToWorldCoord(size_t mx, size_t my,
                                      double& wx, double& wy) const;
    
  //accessors
  size_t getWidth() const;
  size_t getHeight() const;
  unsigned int getTotalObsPoints() const;
  
private:
  
  //refreshes the full_map, chucking old data
  void refreshFullMap();
  
  //adds the points in the pts arg to the full_map
  void addObstaclePointsToFullMap(const ObstaclePts* pts);
  
  //checks if the time param t falls within the window of our current time
  bool isTimeWithinWindow(ros::Time t) const;
  
  double window_length_; /**< window length for remembering data */
  size_t width_; /**< width of the map */
  size_t height_; /**< height of the map */
  double resolution_; /**< resolution of the map, currently unused */ 
  unsigned char* static_data_; /**< data loaded from the static map */
  unsigned char* full_data_; /**< the full map data that has both static and obstacle data */
  unsigned int* obs_count_; /**< this holds the count among current scans of a particular cell as an obstacle cell */

  std::list<ObstaclePts*> obstacle_pts_; /**< a list of obstacle points corresponding to scans */
  ros::Time cur_time_; /**< the map's current notion of time */
  
  unsigned int total_obs_points_; /**< running total of number of non-static obstacle points in the map */

};



#endif
