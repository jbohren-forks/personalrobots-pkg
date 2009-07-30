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

#include <iosfwd>
#include <unistd.h>

namespace costmap_2d {
  class Costmap2D;
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
    virtual cost_t getPossiblyCircumscribedCost() const = 0;
    
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
	getPossiblyCircumscribedCost(). Subclasses can override this
	with a more efficient method. */
    virtual bool isPossiblyCircumscribed(index_t index_x, index_t index_y,
					 bool out_of_bounds_reply) const;
    
    /** Default implementation uses getCost() and assumes freespace is
	represented by zero cost values. Subclasses can override this
	with a more efficient method. */
    virtual bool isFreespace(index_t index_x, index_t index_y,
			     bool out_of_bounds_reply) const;
    
    void dumpHistogram(char const * prefix, std::ostream & os) const;
  };
  
  
  class IndexTransform
  {
  public:
    typedef ssize_t index_t;
    
    virtual ~IndexTransform() {}
    
    /**
       Transform a global (x, y) point into costmap indices.
       
       \note The computed indices are not guaranteed to lie within the
       map bounds, you have to check that separately (i.e. using
       CostmapAccessor::isValidIndex()). The idea behind this is that
       you can thus easily determine by how much you'd have to grow
       the costmap.
    */
    virtual void globalToIndex(double global_x, double global_y,
			       index_t * index_x, index_t * index_y) const = 0;
    
    /**
       Compute the global coordinates (x, y) of the center of a cell
       given by its indices.
    */
    virtual void indexToGlobal(index_t index_x, index_t index_y,
			       double * global_x, double * global_y) const = 0;
    
    /**
       Compute the costmap-relative (x, y, theta) coordinates of a
       pose given in the global frame. This does NOT involve scaling
       by the resolution.
    */
    virtual void globalToLocal(double global_x, double global_y, double global_th,
			       double * local_x, double * local_y, double * local_th) const = 0;
    
    template<typename value_t>
    void globalToLocal(value_t global_x, value_t global_y, value_t global_th,
		       value_t * local_x, value_t * local_y, value_t * local_th) const
    {
      double gx(global_x);
      double gy(global_y);
      double gth(global_th);
      double lx, ly, lth;
      globalToLocal(gx, gy, gth, &lx, &ly, &lth);
      *local_x = lx;
      *local_y = ly;
      *local_th = lth;
    }
    
    /**
       Perform only the rotational part of the localToGlobal()
       transform. Use this for vectors (as opposed to points).
    */
    virtual void rotateGlobalToLocal(double global_x, double global_y, double global_th,
				     double * local_x, double * local_y, double * local_th) const = 0;
    
    template<typename value_t>
    void rotateGlobalToLocal(value_t global_x, value_t global_y, value_t global_th,
			     value_t * local_x, value_t * local_y, value_t * local_th) const
    {
      double gx(global_x);
      double gy(global_y);
      double gth(global_th);
      double lx, ly, lth;
      rotateGlobalToLocal(gx, gy, gth, &lx, &ly, &lth);
      *local_x = lx;
      *local_y = ly;
      *local_th = lth;
    }
    
    /**
       Compute the global (x, y, theta) coordinates of a pose given in
       the local costmap frame. This does NOT involve scaling by the
       resolution.
    */
    virtual void localToGlobal(double local_x, double local_y, double local_th,
			       double * global_x, double * global_y, double * global_th) const = 0;
    
    template<typename value_t>
    void localToGlobal(value_t global_x, value_t global_y, value_t global_th,
		       value_t * local_x, value_t * local_y, value_t * local_th) const
    {
      double gx(global_x);
      double gy(global_y);
      double gth(global_th);
      double lx, ly, lth;
      localToGlobal(gx, gy, gth, &lx, &ly, &lth);
      *local_x = lx;
      *local_y = ly;
      *local_th = lth;
    }
    
    /**
       Perform only the rotational part of the localToGlobal()
       transform. Use this for vectors (as opposed to points).
    */
    virtual void rotateLocalToGlobal(double global_x, double global_y, double global_th,
				     double * local_x, double * local_y, double * local_th) const = 0;
    
    template<typename value_t>
    void rotateLocalToGlobal(value_t global_x, value_t global_y, value_t global_th,
			     value_t * local_x, value_t * local_y, value_t * local_th) const
    {
      double gx(global_x);
      double gy(global_y);
      double gth(global_th);
      double lx, ly, lth;
      rotateLocalToGlobal(gx, gy, gth, &lx, &ly, &lth);
      *local_x = lx;
      *local_y = ly;
      *local_th = lth;
    }
    
    /**
       Get the origin of the costmap grid relative to its containing
       frame of reference, typically "the global" frame. (ox, oy) is
       the center of the cell that has the indices [0][0], and oth is
       the angle from the global X-axis to the grid X-axis
     */
    virtual void getOrigin(double * ox, double * oy, double * oth) const = 0;
    
    /**
       Get the resolution of the costmap grid, i.e. the length of one
       side of a grid cell.
     */
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
  
  
  /**
     The recommended way of using the costmap_2d package is to
     instantiate a costmap_2d::Costmap2DROS and then, whenever you need
     access to the actual data, create a copy of the underlying
     costmap_2d::Costmap2D object by calling
     costmap_2d::Costmap2DROS::getCostMapCopy(). There is (currently)
     no way to know from the outside when the underlying
     costmap_2d::Costmap2D has actually changed, and the typical use
     case is for clients to grab a copy at the beginning of their
     cycle and work with that.
     
     In order to keep the mpglue::CostmapAccessor independent of when
     you actually refresh your copy of the costmap_2d::Costmap2D
     object, you have to subclass the mpglue::costmap_2d_getter functor,
     which should return a pointer to your costmap_2d::Costmap2D,
     assuming that your code knows when it has to get a fresh copy.
     
     \see http://pr.willowgarage.com/pr-docs/ros-packages/costmap_2d/html/index.html
  */
  struct costmap_2d_getter {
    virtual ~costmap_2d_getter() {}
    virtual costmap_2d::Costmap2D const * operator () () = 0;
  };
  
  
  /**
     Create a CostmapAccessor wrapper for a costmap_2d::Costmap2D or
     costmap_2d::Costmap2DROS.
     
     \note The calling code retains ownership of the costmap_2d_getter object.
     
     \see mpglue::costmap_2d_getter for the reason of yet another indirection.
  */
  CostmapAccessor * createCostmapAccessor(costmap_2d_getter * get_costmap);
  
  CostmapAccessor * createCostmapAccessor(sfl::RDTravmap const * rdt,
					  int possibly_circumscribed_cost);

  CostmapAccessor * createCostmapAccessor(sfl::TraversabilityMap const * rdt,
					  int possibly_circumscribed_cost);
  
  
  /**
     Create an IndexTransform wrapper for a costmap_2d::Costmap2D or
     costmap_2d::Costmap2DROS.
     
     \note The calling code retains ownership of the costmap_2d_getter object.
     
     \see mpglue::costmap_2d_getter for the reason of yet another indirection.
  */
  IndexTransform * createIndexTransform(mpglue::costmap_2d_getter * get_costmap);
  
  /**
     Create an IndexTransform wrapper for a sfl::GridFrame.
     
     \note The calling code retains ownership of the sfl::GridFrame
     object, beware of passing in local variables.
     
     \see mpglue::costmap_2d_getter for the reason of yet another indirection.
  */
  IndexTransform * createIndexTransform(sfl::GridFrame const * gf);
  
  
  /**
     Wraps a "raw" array-of-some-type into a
     CostmapAccessor-interface. The user is entirely responsible for
     managing the raw memory, and for keeping that in synch with the
     RawCostmapAccessor.
     
     \note All fields are public to allow you to manage the raw memory
     if required, without keeping a separate pointer around.
  */
  template<typename raw_cost_type,
	   typename raw_cost_pointer_type = raw_cost_type const *,
	   typename raw_index_type = index_t>
  class RawCostmapAccessor
    : public CostmapAccessor {
  public:
    typedef raw_cost_type raw_cost_t;
    typedef raw_cost_pointer_type raw_cost_pointer_t;
    typedef raw_index_type raw_index_t;
    
    RawCostmapAccessor(raw_cost_pointer_t _raw,
		       raw_index_t _ncells_x,
		       raw_index_t _ncells_y,
		       raw_index_t _lethal_cost,
		       raw_index_t _inscribed_cost,
		       raw_index_t _circumscribed_cost)
      : raw(_raw),
	ncells_x(_ncells_x),
	ncells_y(_ncells_y),
	lethal_cost(_lethal_cost),
	inscribed_cost(_inscribed_cost),
	circumscribed_cost(_circumscribed_cost)
    {}
    
    virtual cost_t getLethalCost() const { return lethal_cost; }
    virtual cost_t getInscribedCost() const { return inscribed_cost; }
    virtual cost_t getPossiblyCircumscribedCost() const { return circumscribed_cost; }
    
    virtual index_t getXBegin() const { return 0; }
    virtual index_t getXEnd() const { return ncells_x; }
    virtual index_t getYBegin() const { return 0; }
    virtual index_t getYEnd() const { return ncells_y; }
    
    virtual bool isValidIndex(index_t index_x, index_t index_y) const
    { return checkIndex(index_x, index_y); }
    
    virtual bool getCost(index_t index_x, index_t index_y, cost_t * cost) const {
      if (checkIndex(index_x, index_y)) {
	*cost = raw[indexToRaw(index_x, index_y)];
	return true;
      }
      return false;
    }
    
    template<typename other_index_t>
    raw_index_t indexToRaw(other_index_t ix, other_index_t iy) const
    { return ix + ncells_x * iy; }
    
    template<typename other_index_t>
    bool checkIndex(other_index_t ix, other_index_t iy) const
    { return (ix >= 0) && (ix < ncells_x) && (iy >= 0) && (iy < ncells_y); }
    
    raw_cost_pointer_t raw;
    raw_index_t ncells_x, ncells_y;
    raw_cost_t lethal_cost;
    raw_cost_t inscribed_cost;
    raw_cost_t circumscribed_cost;
  };
  
}

#endif // MPGLUE_COSTMAP_HPP
