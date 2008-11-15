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
	

  const unsigned char ObstacleMapAccessor::NO_INFORMATION(255);
  const unsigned char ObstacleMapAccessor::LETHAL_OBSTACLE(254);
  const unsigned char ObstacleMapAccessor::INSCRIBED_INFLATED_OBSTACLE(253);

  ObstacleMapAccessor::ObstacleMapAccessor(double origin_x, double origin_y, unsigned int width, unsigned int height, double resolution)
    : origin_x_(origin_x), origin_y_(origin_y), width_(width), height_(height), resolution_(resolution), costLB_(0){}

  void ObstacleMapAccessor::getOriginInWorldCoordinates(double& wx, double& wy) const{
    wx = origin_x_;
    wy = origin_y_;
  }

  bool ObstacleMapAccessor::isCircumscribedCell(unsigned int mx, unsigned int my) const{
    const unsigned char cost = getCost(mx, my);
    return cost < INSCRIBED_INFLATED_OBSTACLE && cost >= costLB_;
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
      //printf("WC_MC converted  %d greater than height %d\n", my, height_);
      my = 0;
      return false;
    }

    return true;
  }
}
