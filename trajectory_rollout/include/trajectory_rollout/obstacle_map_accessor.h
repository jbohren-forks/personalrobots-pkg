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
#ifndef OBSTACLE_MAP_ACCESSOR_H
#define OBSTACLE_MAP_ACCESSOR_H

#include <trajectory_rollout/map_grid.h>

//Interface call for accessing obstacle data
class ObstacleMapAccessor{
  public:
  virtual ~ObstacleMapAccessor(){}
    virtual unsigned int getWidth() const = 0;
    virtual unsigned int getHeight() const = 0;
    virtual double getResolution() const = 0;
    virtual bool isObstacle(unsigned int mx, unsigned int my) const = 0;
    virtual bool isInflatedObstacle(unsigned int mx, unsigned int my) const = 0;
    virtual void getOriginInWorldCoordinates(double& wx, double& wy) const = 0;
};

//So we can still use wavefront player we'll fake an obs accessor
class WavefrontMapAccessor : public ObstacleMapAccessor {
  public:
    WavefrontMapAccessor(MapGrid &map) : map_(map) {}
  virtual ~WavefrontMapAccessor(){};
    virtual unsigned int getWidth() const {return  map_.size_x_;}
    virtual unsigned int getHeight() const {return map_.size_y_;}
    virtual double getResolution() const {return map_.scale;}

    virtual bool isObstacle(unsigned int mx, unsigned int my) const {
      if(map_(mx, my).occ_state == 1)
        return true;
      return false;
    }

    virtual bool isInflatedObstacle(unsigned int mx, unsigned int my) const {
      return isObstacle(mx, my);
    }

    virtual void getOriginInWorldCoordinates(double& wx, double& wy) const {
      wx = map_.origin_x;
      wy = map_.origin_y;
    }

  private:
    MapGrid& map_;

};
#endif
