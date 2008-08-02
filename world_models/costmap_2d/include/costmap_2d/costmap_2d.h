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

// For time support
#include "ros/time.h"

// For point clouds
#include "std_msgs/PointCloudFloat32.h"

// A bunch of x,y points, with a timestamp
typedef struct
{
  double* pts;
  size_t pts_num;
  double ts;
} ObstaclePts;

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
                      double resolution, unsigned char* data);

    /**
     * @brief Add dynamic obstacles to the map.
     *
     * Updates the map to contain the new dynamic obstacles, in addition to
     * any previously added obstacles (but only those that are still within
     * the window_length).
     */
    void addObstacles(std_msg::PointCloudFloat32* cloud);

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
    unsigned char* getMap();
};



#endif
