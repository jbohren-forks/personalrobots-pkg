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

/** \file costmap.h Uniform handling of various costmap implementations. */

#ifndef MPGLUE_COSTMAP_HPP
#define MPGLUE_COSTMAP_HPP

#include <unistd.h>

namespace old_costmap_2d {
  // CostMap2D is actually a subclass of ObstacleMapAccessor, so it
  // suffices to forward-declare that.
  class ObstacleMapAccessor;
}

namespace sfl {
  class TraversabilityMap;
  class RDTravmap;
  class GridFrame;
}

namespace mpglue {
  
  typedef ssize_t index_t;
  typedef int cost_t;
  
  
  /**
     \note There is no getFreespaceCost() because that is implicitly
     assumed to be zero for all implementations.
  */
  class CostmapAccessor
  {
  public:
    virtual ~CostmapAccessor() {}
    
    virtual cost_t getLethalCost() const = 0;
    virtual cost_t getInscribedCost() const = 0;
    virtual cost_t getPossiblyCircumcribedCost() const = 0;
    
    virtual index_t getXBegin() const = 0;
    virtual index_t getXEnd() const = 0;
    virtual index_t getYBegin() const = 0;
    virtual index_t getYEnd() const = 0;
    
    virtual bool isValidIndex(index_t index_x, index_t index_y) const = 0;
    
    /** Subclasses should return true and place the cost value in the
	third parameter, unless the index lies out of bounds, in which
	case they should return false. */
    virtual bool getCost(index_t index_x, index_t index_y, cost_t * cost) const = 0;
    
    /** Default implementation uses getCost() and
	getLethalCost(). Subclasses can override this with a more
	efficient method. */
    virtual bool isLethal(index_t index_x, index_t index_y,
			  bool out_of_bounds_reply) const;
    
    /** Default implementation uses getCost() and
	getInscribedCost(). Subclasses can override this with a more
	efficient method. */
    virtual bool isInscribed(index_t index_x, index_t index_y,
			     bool out_of_bounds_reply) const;
    
    /** Default implementation uses getCost() and
	getPossiblyCircumcribedCost(). Subclasses can override this
	with a more efficient method. */
    virtual bool isPossiblyCircumcribed(index_t index_x, index_t index_y,
					bool out_of_bounds_reply) const;
    
    /** Default implementation uses getCost() and assumes freespace is
	represented by zero cost values. Subclasses can override this
	with a more efficient method. */
    virtual bool isFreespace(index_t index_x, index_t index_y,
			     bool out_of_bounds_reply) const;
  };
  
  
  class IndexTransform
  {
  public:
    typedef ssize_t index_t;
    
    virtual ~IndexTransform() {}
    
    virtual void globalToIndex(double global_x, double global_y,
			       index_t * index_x, index_t * index_y) const = 0;
    virtual void indexToGlobal(index_t index_x, index_t index_y,
			       double * global_x, double * global_y) const = 0;
    virtual double getResolution() const = 0;
    
    template<typename other_index_t>
    void globalToIndex(double global_x, double global_y,
		       other_index_t * index_x, other_index_t * index_y) const {
      index_t ix, iy;
      globalToIndex(global_x, global_y, &ix, &iy);
      *index_x = ix;
      *index_y = iy;
    }
  };
  
  
  template<typename index_type>
  double interpolateIndexToGlobal(index_type dis_idx, double dis_glob, double idx, double res)
  { return dis_glob + res * (idx - dis_idx); }
  
  
  CostmapAccessor * createCostmapAccessor(old_costmap_2d::ObstacleMapAccessor const * cm);

  CostmapAccessor * createCostmapAccessor(sfl::RDTravmap const * rdt,
					  int possibly_circumscribed_cost);

  CostmapAccessor * createCostmapAccessor(sfl::TraversabilityMap const * rdt,
					  int possibly_circumscribed_cost);
  
  IndexTransform * createIndexTransform(old_costmap_2d::ObstacleMapAccessor const * cm);

  IndexTransform * createIndexTransform(sfl::GridFrame const * gf);
  
}

#endif // MPGLUE_COSTMAP_HPP
