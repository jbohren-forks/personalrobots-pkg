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

#include <mpglue/costmap.h>
#include <costmap_2d/obstacle_map_accessor.h>
#include <sfl/gplan/RWTravmap.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <math.h>

namespace {
  
  
  class cm2dCostmapAccessor: public mpglue::CostmapAccessor {
  public:
    cm2dCostmapAccessor(costmap_2d::ObstacleMapAccessor const * cm): cm_(cm) {}
    
    virtual mpglue::cost_t getLethalCost() const
    { return costmap_2d::ObstacleMapAccessor::LETHAL_OBSTACLE; }
    
    virtual mpglue::cost_t getInscribedCost() const
    { return costmap_2d::ObstacleMapAccessor::INSCRIBED_INFLATED_OBSTACLE; }
    
    virtual mpglue::cost_t getPossiblyCircumcribedCost() const
    { return cm_->getCircumscribedCostLowerBound(); }
    
    virtual ssize_t getXBegin() const { return 0; }
    virtual ssize_t getXEnd() const { return cm_->getWidth(); }
    virtual ssize_t getYBegin() const { return 0; }
    virtual ssize_t getYEnd() const { return cm_->getHeight(); }
    
    virtual bool isValidIndex(ssize_t index_x, ssize_t index_y) const
    { return (index_x >= 0) && (static_cast<size_t>(index_x) < cm_->getWidth())
	&&   (index_y >= 0) && (static_cast<size_t>(index_y) < cm_->getHeight()); }
    
    virtual bool isLethal(ssize_t index_x, ssize_t index_y,
			  bool out_of_bounds_reply) const {
      if (isValidIndex(index_x, index_y))
	return cm_->getCost(index_x, index_y) >= costmap_2d::ObstacleMapAccessor::LETHAL_OBSTACLE;
      return out_of_bounds_reply;
    }
    
    virtual bool isInscribed(ssize_t index_x, ssize_t index_y,
			     bool out_of_bounds_reply) const {
      if (isValidIndex(index_x, index_y))
	return cm_->getCost(index_x, index_y)
	  >= costmap_2d::ObstacleMapAccessor::INSCRIBED_INFLATED_OBSTACLE;
      return out_of_bounds_reply;
    }
    
    virtual bool isPossiblyCircumcribed(ssize_t index_x, ssize_t index_y,
					bool out_of_bounds_reply) const {
      if (isValidIndex(index_x, index_y))
	return cm_->isCircumscribedCell(index_x, index_y);
      return out_of_bounds_reply;
    }
    
    virtual bool isFreespace(ssize_t index_x, ssize_t index_y,
			     bool out_of_bounds_reply) const {
      if (isValidIndex(index_x, index_y))
	return cm_->getCost(index_x, index_y) == 0;
      return out_of_bounds_reply;
    }
    
    virtual bool getCost(ssize_t index_x, ssize_t index_y, mpglue::cost_t * cost) const {
      if ( ! isValidIndex(index_x, index_y))
	return false;
      *cost = cm_->getCost(index_x, index_y);
      return true;
    }
    
    costmap_2d::ObstacleMapAccessor const * cm_;
  };
  
  
  class cm2dTransform: public mpglue::IndexTransform {
  public:
    cm2dTransform(costmap_2d::ObstacleMapAccessor const * cm): cm_(cm) {}
    
    virtual void globalToIndex(double global_x, double global_y,
			       ssize_t * index_x, ssize_t * index_y) const {
      unsigned int ix, iy;
      cm_->WC_MC(global_x, global_y, ix, iy);
      *index_x = ix;
      *index_y = iy;
    }
    
    virtual void indexToGlobal(ssize_t index_x, ssize_t index_y,
			       double * global_x, double * global_y) const
    { cm_->MC_WC(index_x, index_y, *global_x, *global_y); }
    
    virtual double getResolution() const { return cm_->getResolution(); }
    
    costmap_2d::ObstacleMapAccessor const * cm_;
  };
  
  
  class sflRDTAccessor: public mpglue::CostmapAccessor {
  public:
    sfl::RDTravmap const * rdt_;
    int const wobstCost_;
    int const cobstCost_;
    int const possibly_circumscribed_cost_;
    
    sflRDTAccessor(sfl::RDTravmap const * rdt,
		   int possibly_circumscribed_cost)
      : rdt_(rdt),
	// Beware: W-space means "non-inflated", which is tagged by
	// making the cost higher than necessary in sfl::Mapper2d.
	wobstCost_(rdt->GetObstacle() + 1),
	cobstCost_(rdt->GetObstacle()),
	possibly_circumscribed_cost_(possibly_circumscribed_cost) {}
    
    virtual mpglue::cost_t getLethalCost() const { return wobstCost_; }
    virtual mpglue::cost_t getInscribedCost() const { return cobstCost_; }
    virtual mpglue::cost_t getPossiblyCircumcribedCost() const { return possibly_circumscribed_cost_; }
    
    virtual ssize_t getXBegin() const { return rdt_->GetXBegin(); }
    virtual ssize_t getXEnd() const { return rdt_->GetXEnd(); }
    virtual ssize_t getYBegin() const { return rdt_->GetYBegin(); }
    virtual ssize_t getYEnd() const { return rdt_->GetYEnd(); }
    
    virtual bool isValidIndex(ssize_t index_x, ssize_t index_y) const
    { return rdt_->IsValid(index_x, index_y); }
    
    virtual bool isLethal(ssize_t index_x, ssize_t index_y,
			  bool out_of_bounds_reply) const {
      if (rdt_->IsValid(index_x, index_y))
	return rdt_->IsWObst(index_x, index_y);
      return out_of_bounds_reply;
    }
    
    virtual bool isInscribed(ssize_t index_x, ssize_t index_y,
			     bool out_of_bounds_reply) const {
      if (rdt_->IsValid(index_x, index_y))
	return rdt_->IsObst(index_x, index_y);
      return out_of_bounds_reply;
    }
    
    virtual bool isPossiblyCircumcribed(ssize_t index_x, ssize_t index_y,
					bool out_of_bounds_reply) const {
      int value;
      if (rdt_->GetValue(index_x, index_y, value))
	return value >= possibly_circumscribed_cost_;
      return out_of_bounds_reply;
    }
    
    virtual bool isFreespace(ssize_t index_x, ssize_t index_y,
			     bool out_of_bounds_reply) const {
      if (rdt_->IsValid(index_x, index_y))
	return rdt_->IsFree(index_x, index_y);
      return out_of_bounds_reply;
    }
    
    virtual bool getCost(ssize_t index_x, ssize_t index_y, mpglue::cost_t * cost) const
    { return rdt_->GetValue(index_x, index_y, *cost); }
  };
  
  
  class sflTravmapAccessor: public mpglue::CostmapAccessor {
  public:
    sfl::TraversabilityMap const * travmap_;
    int const wobstCost_;
    int const cobstCost_;
    int const possibly_circumscribed_cost_;
    
    sflTravmapAccessor(sfl::TraversabilityMap const * travmap,
		       int possibly_circumscribed_cost)
      : travmap_(travmap), wobstCost_(travmap->obstacle + 1), cobstCost_(travmap->obstacle),
	possibly_circumscribed_cost_(possibly_circumscribed_cost) {}
    
    virtual mpglue::cost_t getLethalCost() const { return wobstCost_; }
    virtual mpglue::cost_t getInscribedCost() const { return cobstCost_; }
    virtual mpglue::cost_t getPossiblyCircumcribedCost() const { return possibly_circumscribed_cost_; }
    
    virtual ssize_t getXBegin() const { return travmap_->grid.xbegin(); }
    virtual ssize_t getXEnd() const { return travmap_->grid.xend(); }
    virtual ssize_t getYBegin() const { return travmap_->grid.ybegin(); }
    virtual ssize_t getYEnd() const { return travmap_->grid.yend(); }
    
    virtual bool isValidIndex(ssize_t index_x, ssize_t index_y) const
    { return travmap_->IsValid(index_x, index_y); }
    
    virtual bool isLethal(ssize_t index_x, ssize_t index_y,
			  bool out_of_bounds_reply) const {
      if (travmap_->IsValid(index_x, index_y))
	return travmap_->IsWObst(index_x, index_y);
      return out_of_bounds_reply;
    }
    
    virtual bool isInscribed(ssize_t index_x, ssize_t index_y,
			     bool out_of_bounds_reply) const {
      if (travmap_->IsValid(index_x, index_y))
	return travmap_->IsObst(index_x, index_y);
      return out_of_bounds_reply;
    }
    
    virtual bool isPossiblyCircumcribed(ssize_t index_x, ssize_t index_y,
					bool out_of_bounds_reply) const {
      int value;
      if (travmap_->GetValue(index_x, index_y, value))
	return value >= possibly_circumscribed_cost_;
      return out_of_bounds_reply;
    }
    
    virtual bool isFreespace(ssize_t index_x, ssize_t index_y,
			     bool out_of_bounds_reply) const {
      if (travmap_->IsValid(index_x, index_y))
	return travmap_->IsFree(index_x, index_y);
      return out_of_bounds_reply;
    }
    
    virtual bool getCost(ssize_t index_x, ssize_t index_y, mpglue::cost_t * cost) const
    { return travmap_->GetValue(index_x, index_y, *cost); }
  };
  
  
  class sflTransform: public mpglue::IndexTransform {
  public:
    sfl::GridFrame const * gf_;
    
    sflTransform(sfl::GridFrame const * gf): gf_(gf) {}
    
    virtual void globalToIndex(double global_x, double global_y,
			       ssize_t * index_x, ssize_t * index_y) const {
      sfl::GridFrame::index_t const ii(gf_->GlobalIndex(global_x, global_y));
      *index_x = ii.v0;
      *index_y = ii.v1;
    }
    
    virtual void indexToGlobal(ssize_t index_x, ssize_t index_y,
			       double * global_x, double * global_y) const {
      sfl::GridFrame::position_t const pp(gf_->GlobalPoint(index_x, index_y));
      *global_x = pp.v0;
      *global_y = pp.v1;
    }
    
    virtual double getResolution() const { return gf_->Delta(); }
  };
  
}

namespace mpglue {
  
  bool CostmapAccessor::
  isLethal(index_t index_x, index_t index_y,
	   bool out_of_bounds_reply) const
  {
    cost_t cost;
    if ( ! getCost(index_x, index_y, &cost))
      return out_of_bounds_reply;
    return cost <= getLethalCost();
  }
  
  
  bool CostmapAccessor::
  isInscribed(index_t index_x, index_t index_y,
	      bool out_of_bounds_reply) const
  {
    cost_t cost;
    if ( ! getCost(index_x, index_y, &cost))
      return out_of_bounds_reply;
    return cost <= getInscribedCost();
  }
  
  
  bool CostmapAccessor::
  isPossiblyCircumcribed(index_t index_x, index_t index_y,
			 bool out_of_bounds_reply) const
  {
    cost_t cost;
    if ( ! getCost(index_x, index_y, &cost))
      return out_of_bounds_reply;
    return cost <= getPossiblyCircumcribedCost();
  }
  
  
  bool CostmapAccessor::
  isFreespace(index_t index_x, index_t index_y,
	      bool out_of_bounds_reply) const
  {
    cost_t cost;
    if ( ! getCost(index_x, index_y, &cost))
      return out_of_bounds_reply;
    return cost == 0;
  }
  
  
  CostmapAccessor * createCostmapAccessor(costmap_2d::ObstacleMapAccessor const * cm)
  { return new cm2dCostmapAccessor(cm); }
  
  CostmapAccessor * createCostmapAccessor(sfl::RDTravmap const * rdt,
					  int possibly_circumscribed_cost)
  { return new sflRDTAccessor(rdt, possibly_circumscribed_cost); }
  
  CostmapAccessor * createCostmapAccessor(sfl::TraversabilityMap const * rdt,
					  int possibly_circumscribed_cost)
  { return new sflTravmapAccessor(rdt, possibly_circumscribed_cost); }
  
  IndexTransform * createIndexTransform(costmap_2d::ObstacleMapAccessor const * cm)
  { return new cm2dTransform(cm); }
  
  IndexTransform * createIndexTransform(sfl::GridFrame const * gf)
  { return new sflTransform(gf); }
  
}
