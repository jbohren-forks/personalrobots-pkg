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

namespace costmap_2d {
  class Costmap2D;
}

namespace mpglue {
  
  
  /** Container for a grid index, consisting of an index for both X and Y. */
  struct index_pair {
    inline index_pair(index_t _ix, index_t _iy): ix(_ix), iy(_iy) {}
    
    /** Grid indices are ordered by their X-coordinate, or by their
	Y-coordinate in case of equal X-coordinates.  Useful mainly
	for stuffing grid indices into e.g. a std::list<> or using
	them as key type of e.g. a std::map<>. */
    inline bool operator < (index_pair const & rhs) const
    { return (ix < rhs.ix) || ((ix == rhs.ix) && (iy < rhs.iy)); }
    
    index_t ix, iy;
  };
  
  /** Container for a collection of grid indices. */
  typedef std::set<index_pair> index_collection_t;
  
  
  /**
     Container and utility for managing additions and removals of
     workspace obstacles.
  */
  class ObstacleDelta {
  public:
    inline bool empty() const { return added_.empty() && removed_.empty(); }
    
    inline index_collection_t const * getAddedIndices() const { return &added_; }
    inline index_collection_t const * getRemovedIndices() const { return &removed_; }
    
    /** Add an index to the added_ set, and make sure it is not in the removed_ set. */
    void addIndex(index_pair const & idx);

    /** Add an index to the added_ set, and make sure it is not in the removed_ set. */
    inline void addIndex(index_t ix, index_t iy) { addIndex(index_pair(ix, iy)); }

    /** Add an index to the removed_ set, and make sure it is not in the added_ set. */
    void removeIndex(index_pair const & idx);
    
    /** Add an index to the removed_ set, and make sure it is not in the added_ set. */
    inline void removeIndex(index_t ix, index_t iy) { removeIndex(index_pair(ix, iy)); }
    
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
  
  
  /**
     Parameters common to all Costmapper factory functions. Their
     interpretation might vary slightly depending on the subclass you
     are instantiating.
  */
  struct costmapper_params {
    costmapper_params(costmapper_params const & orig);
    
    costmapper_params(double origin_x,
		      double origin_y,
		      double resolution,
		      double inscribed_radius,
		      double circumscribed_radius,
		      double inflation_radius,
		      index_t ix_end,
		      index_t iy_end);
    
    virtual ~costmapper_params() {}
    
    double origin_x, origin_y, resolution, inscribed_radius, circumscribed_radius, inflation_radius;
    index_t ix_end, iy_end;
  };
  
  
  /**
     Parameters that are specific to sfl::Mapper2d instances when
     wrapped by mpglue::Costmapper.
  */
  struct sfl_costmapper_params: public costmapper_params {
    explicit sfl_costmapper_params(costmapper_params const & base);
    

    sfl_costmapper_params(costmapper_params const & base,
			  double origin_th,
			  index_t ix_begin,
			  index_t iy_begin,
			  int obstacle_cost);
    
    sfl_costmapper_params(double origin_x,
			  double origin_y,
			  double origin_th,
			  double resolution,
			  double inscribed_radius,
			  double circumscribed_radius,
			  double inflation_radius,
			  index_t ix_begin,
			  index_t iy_begin,
			  index_t ix_end,
			  index_t iy_end,
			  int obstacle_cost);
    
    double origin_th;
    index_t ix_begin, iy_begin;
    int obstacle_cost;
  };
  
  
  /**
     Parameters that are specific to costmap_2d::Costmap2D instances
     when wrapped by mpglue::Costmapper.
  */
  struct cm2d_costmapper_params: public costmapper_params {
    explicit cm2d_costmapper_params(costmapper_params const & base);
    
    cm2d_costmapper_params(costmapper_params const & base,
			   double obstacle_range,
			   double max_obstacle_height,
			   double raytrace_range,
			   double weight);

    cm2d_costmapper_params(double origin_x,
			   double origin_y,
			   double resolution,
			   double inscribed_radius,
			   double circumscribed_radius,
			   double inflation_radius,
			   index_t ix_end,
			   index_t iy_end,
			   double obstacle_range,
			   double max_obstacle_height,
			   double raytrace_range,
			   double weight);
    
    double obstacle_range, max_obstacle_height, raytrace_range, weight;
  };
  
  
  boost::shared_ptr<Costmapper> createCostmapper(boost::shared_ptr<sfl::Mapper2d> m2d,
						 int possibly_circumscribed_cost);
  
  boost::shared_ptr<Costmapper> createCostmapper(boost::shared_ptr<costmap_2d::Costmap2D> cm);
  
  boost::shared_ptr<Costmapper> createCostmapper(sfl_costmapper_params const & sfl_params);
  
  /**
     \note The created costmap_2d::Costmap2D instance is initialized
     with all cells set to freespace, instead of the NO_INFORMATION
     value (which gets interpreted as obstacles).
  */
  boost::shared_ptr<Costmapper> createCostmapper(cm2d_costmapper_params const & cm2d_params);
  
  /**
     Attempts to cast the parameters to one of the known subtypes. If
     that succeeds, it forwards the call to one of the known
     factories. Otherwise it defaults to cm2d_costmapper_params.
  */
  boost::shared_ptr<Costmapper> createCostmapper(costmapper_params const * params);
  
}

#endif // MPGLUE_COSTMAPPER_HPP
