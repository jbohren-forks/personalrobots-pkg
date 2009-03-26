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

/** \file costmapper.h Uniform handling of various costmap implementations. */

#ifndef MPGLUE_COSTMAPPER_HPP
#define MPGLUE_COSTMAPPER_HPP

#include <mpglue/costmap.h>
#include <boost/shared_ptr.hpp>
#include <set>

namespace sfl {
  class Mapper2d;
}

namespace mpglue {
  
  
  struct index_pair {
    index_pair(index_t _ix, index_t _iy): ix(_ix), iy(_iy) {}
    
    bool operator < (index_pair const & rhs) const
    { return (ix < rhs.ix) || ((ix == rhs.ix) && (iy < rhs.iy)); }
    
    index_t ix, iy;
  };
  
  
  typedef std::set<index_pair> index_collection_t;
  
  
  /**
     Container and utility for managing additions and removals of
     workspace obstacles.
  */
  class ObstacleDelta {
  public:
    bool empty() const { return added_.empty() && removed_.empty(); }
    
    index_collection_t const * getAddedIndices() const { return &added_; }
    index_collection_t const * getRemovedIndices() const { return &removed_; }
    
    /** Add an index to the added_ set, and make sure it is not in the removed_ set. */
    void addIndex(index_pair const & idx) {
      added_.insert(idx);
      removed_.erase(idx);
    }

    /** Add an index to the added_ set, and make sure it is not in the removed_ set. */
    void addIndex(index_t ix, index_t iy) { addIndex(index_pair(ix, iy)); }

    /** Add an index to the removed_ set, and make sure it is not in the added_ set. */
    void removeIndex(index_pair const & idx) {
      removed_.insert(idx);
      added_.erase(idx);
    }
    
    /** Add an index to the removed_ set, and make sure it is not in the added_ set. */
    void removeIndex(index_t ix, index_t iy) { removeIndex(index_pair(ix, iy)); }
    
  protected:
    index_collection_t added_;
    index_collection_t removed_;
  };
  
  
  class Costmapper
  {
  public:
    typedef ssize_t index_t;
    
    virtual ~Costmapper() {}
    
    virtual boost::shared_ptr<CostmapAccessor const> getAccessor() const = 0;
    virtual boost::shared_ptr<IndexTransform const> getIndexTransform() const = 0;
    
    virtual size_t updateObstacles(/** use null if there are no added obstacles */
				   index_collection_t const * added_obstacle_indices,
				   /** use null if there are no removed obstacles */
				   index_collection_t const * removed_obstacle_indices,
				   /** use null if you are not interested in debug messages */
				   std::ostream * dbgos) = 0;
    
    size_t updateObstacles(ObstacleDelta const & delta, std::ostream * dbgos)
    { return updateObstacles(delta.getAddedIndices(), delta.getRemovedIndices(), dbgos); }
    
    size_t addObstacles(index_collection_t const & obstacle_indices, std::ostream * dbgos)
    { return updateObstacles(&obstacle_indices, 0, dbgos); }
    
    size_t removeObstacles(index_collection_t const & freespace_indices, std::ostream * dbgos)
    { return updateObstacles(0, &freespace_indices, dbgos); }
  };
  
  
  boost::shared_ptr<Costmapper> createCostmapper(boost::shared_ptr<sfl::Mapper2d> m2d,
						 int possibly_circumscribed_cost);
  
}

#endif // MPGLUE_COSTMAPPER_HPP
