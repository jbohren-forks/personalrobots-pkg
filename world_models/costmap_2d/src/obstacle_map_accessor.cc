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
#include <cstring>

namespace costmap_2d {
	

  const unsigned char ObstacleMapAccessor::NO_INFORMATION(255);
  const unsigned char ObstacleMapAccessor::LETHAL_OBSTACLE(254);
  const unsigned char ObstacleMapAccessor::INSCRIBED_INFLATED_OBSTACLE(253);

  ObstacleMapAccessor::ObstacleMapAccessor(double origin_x, double origin_y, unsigned int width, unsigned int height, double resolution)
    : origin_x_(origin_x), origin_y_(origin_y), width_(width), height_(height), resolution_(resolution), costData_(NULL), costLB_(0){
    costData_ = new unsigned char[width_*height_];
    memset(costData_, 0, width_*height_);
  }


  ObstacleMapAccessor::~ObstacleMapAccessor(){
    if(costData_ != NULL) 
      delete[] costData_;
  }

  void ObstacleMapAccessor::getOriginInWorldCoordinates(double& wx, double& wy) const{
    wx = origin_x_;
    wy = origin_y_;
  }

  bool ObstacleMapAccessor::isCircumscribedCell(unsigned int mx, unsigned int my) const{
    const unsigned char cost = getCost(mx, my);
    return cost < INSCRIBED_INFLATED_OBSTACLE && cost >= costLB_;
  }

  std::string ObstacleMapAccessor::toString() const {
    std::stringstream ss;
    ss << std::endl;
    for(unsigned j = 0; j < getHeight(); j++){
      for (unsigned int i = 0; i < getWidth(); i++){
        unsigned char cost = getCost(i, j);
        if(cost == LETHAL_OBSTACLE)
          ss << "O";
        else if (cost == INSCRIBED_INFLATED_OBSTACLE)
          ss << "I";
        else if (isCircumscribedCell(i, j))
          ss << "C";
        else if (cost == NO_INFORMATION)
          ss << "?";
        else if (cost > 0)
          ss << "f";
        else
          ss << "F";

        ss << ",";
      }

      ss << std::endl;
    }
    return ss.str();
  }
}
