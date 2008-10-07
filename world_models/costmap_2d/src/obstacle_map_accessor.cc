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
 *********************************************************************/

#include "costmap_2d/obstacle_map_accessor.h"
#include <stdio.h>

namespace costmap_2d {
	
  ObstacleMapAccessor::ObstacleMapAccessor(double origin_x, double origin_y, unsigned int width, unsigned int height, double resolution)
    : origin_x_(origin_x), origin_y_(origin_y), width_(width), height_(height), resolution_(resolution){}

  void ObstacleMapAccessor::getOriginInWorldCoordinates(double& wx, double& wy) const{
    wx = origin_x_;
    wy = origin_y_;
  }

  /**
   * @brief Obtain world co-ordinates for the given index
   * @param ind index
   * @param wx world x location of the cell
   * @param wy world y location of the cell
   */
  void ObstacleMapAccessor::IND_WC(unsigned int ind, double& wx, double& wy) const{
    unsigned int mx, my;
    IND_MC(ind, mx, my);
    MC_WC(mx, my, wx, wy);
  }

  /**
   * @brief Converts from 1D map index into x y map coords
   * 
   * @param ind 1d map index
   * @param x 2d map return value
   * @param y 2d map return value
   */
  void ObstacleMapAccessor::IND_MC(unsigned int ind, unsigned int& mx, unsigned int& my) const{
    my = ind / width_;
    mx = ind - (my*width_);
  }

  /**
   * @brief Get index of given (x,y) point into data
   * 
   * @param x x-index of the cell
   * @param y y-index of the cell
   */
  unsigned int ObstacleMapAccessor::MC_IND(unsigned int mx, unsigned int my) const {
    return(mx+my*width_);
  }

  /**
   * @brief Get world (x,y) point given map indexes
   * 
   * @param mx map x index location
   * @param my map y index location
   * @param wx world x return value
   * @param wy world y return value
   */
  void ObstacleMapAccessor::MC_WC(unsigned int mx, unsigned int my, double& wx, double& wy) const {
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
  }

  /**
   * @brief Get index of given (x,y) point into data
   * 
   * @param wx world x location of the cell
   * @param wy world y location of the cell
   */
  unsigned int ObstacleMapAccessor::WC_IND(double wx, double wy) const{
    unsigned int mx, my;
    WC_MC(wx, wy, mx, my);
    return MC_IND(mx, my);
  }

  /**
   * @brief Get index of given world (x,y) point in map indexes
   * 
   * @param wx world x location of the cell
   * @param wy world y location of the cell
   * @param mx map x index return value
   * @param my map y index return value
   */
  bool ObstacleMapAccessor::WC_MC(double wx, double wy, unsigned int& mx, unsigned int& my) const {

    if(wx < 0 || wy < 0) {
      mx = 0;
      my = 0;
      return false;
    }

    //not much for now
    mx = (int) ((wx - origin_x_)/resolution_);
    my = (int) ((wy - origin_y_)/resolution_);

    //printf("x: %.2f y: %.2f or_x: %.2f, or_y: %.2f, resolution: %.2f\n   ", wx, wy, origin_x_, origin_y_, resolution_);
    if(mx >= width_) {
      mx = 0;
      return false;
    } 

    if(my >= height_) {
      my = 0;
      return false;
    }

    return true;
  }
}
