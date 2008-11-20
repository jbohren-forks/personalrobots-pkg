/*********************************************************************
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

/** \file costmap_wrap.h Uniform handling of various costmap implementations. */

#ifndef OMPL_COSTMAP_WRAP_HPP
#define OMPL_COSTMAP_WRAP_HPP

#include <unistd.h>

namespace costmap_2d {
  class ObstacleMapAccessor;
}

namespace sfl {
  class RDTravmap;
  class GridFrame;
}

namespace ompl {
  
  
  template<typename cost_type,
	   typename index_type>
  class GenericCostmapWrap
  {
  public:
    typedef cost_type cost_t;
    typedef index_type index_t;
    
    virtual ~GenericCostmapWrap() {}
    
    virtual cost_t getWSpaceObstacleCost() const = 0;
    virtual cost_t getCSpaceObstacleCost() const = 0;
    virtual cost_t getFreespaceCost() const = 0;
    
    virtual index_t getXBegin() const = 0;
    virtual index_t getXEnd() const = 0;
    virtual index_t getYBegin() const = 0;
    virtual index_t getYEnd() const = 0;
    
    virtual bool isValidIndex(index_t index_x, index_t index_y) const = 0;
    virtual bool isWSpaceObstacle(index_t index_x, index_t index_y, bool out_of_bounds_is_obstacle) const = 0;
    virtual bool isCSpaceObstacle(index_t index_x, index_t index_y, bool out_of_bounds_is_obstacle) const = 0;
    virtual bool isFreespace(index_t index_x, index_t index_y, bool out_of_bounds_is_freespace) const = 0;
    
    virtual bool getCost(index_t index_x, index_t index_y, cost_t * cost) const = 0;
  };
  
  typedef GenericCostmapWrap<int, ssize_t> CostmapWrap;
  
  
  template<typename index_type>
  class GenericIndexTransformWrap
  {
  public:
    typedef index_type index_t;
    
    virtual ~GenericIndexTransformWrap() {}
    
    virtual void globalToIndex(double global_x, double global_y, index_t * index_x, index_t * index_y) const = 0;
    virtual void indexToGlobal(index_t index_x, index_t index_y, double * global_x, double * global_y) const = 0;
    virtual double getResolution() const = 0;
    
    template<typename other_index_t>
    void globalToIndex(double global_x, double global_y, other_index_t * index_x, other_index_t * index_y) const {
      index_t ix, iy;
      globalToIndex(global_x, global_y, &ix, &iy);
      *index_x = ix;
      *index_y = iy;
    }
  };
  
  typedef GenericIndexTransformWrap<ssize_t> IndexTransformWrap;
  
  
  CostmapWrap * createCostmapWrap(costmap_2d::ObstacleMapAccessor const * oma);
  CostmapWrap * createCostmapWrap(sfl::RDTravmap const * rdt);
  
  IndexTransformWrap * createIndexTransformWrap(costmap_2d::ObstacleMapAccessor const * oma);
  IndexTransformWrap * createIndexTransformWrap(sfl::GridFrame const * gf);
  
}

#endif // OMPL_COSTMAP_WRAP_HPP
