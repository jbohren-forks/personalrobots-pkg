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

#include <mpglue/costmapper.h>
#include <sfl/gplan/Mapper2d.hpp>

using namespace mpglue;
using namespace boost;
using namespace std;

namespace {
  
  class sflCostmapper: public mpglue::Costmapper {
  public:
    shared_ptr<sfl::Mapper2d> m2d_;
    mutable shared_ptr<sfl::RDTravmap> rdt_;
    mutable shared_ptr<mpglue::CostmapAccessor> cma_;
    mutable shared_ptr<mpglue::IndexTransform> idxt_;
    int const possibly_circumscribed_cost_;
    
    sflCostmapper(shared_ptr<sfl::Mapper2d> m2d,
		  int possibly_circumscribed_cost)
      : m2d_(m2d), possibly_circumscribed_cost_(possibly_circumscribed_cost) {}
    
    virtual boost::shared_ptr<mpglue::CostmapAccessor const> getAccessor() const
    {
      if ( ! cma_) {
	rdt_ = m2d_->CreateRDTravmap();
	cma_.reset(mpglue::createCostmapAccessor(rdt_.get(), possibly_circumscribed_cost_));
      }
      return cma_;
    }
    
    virtual boost::shared_ptr<mpglue::IndexTransform const> getIndexTransform() const
    {
      if ( ! idxt_)
	idxt_.reset(mpglue::createIndexTransform(&m2d_->GetGridFrame()));
      return idxt_;
    }
    
    virtual size_t updateObstacles(index_collection_t const * added_obstacle_indices,
				   index_collection_t const * removed_obstacle_indices,
				   ostream * dbgos)
    {
      sfl::Mapper2d::index_buffer_t add_b;
      sfl::Mapper2d::index_buffer_t * add_p(0);
      if (added_obstacle_indices) {
	add_p = &add_b;
	for (index_collection_t::const_iterator ia(added_obstacle_indices->begin());
	     ia != added_obstacle_indices->end(); ++ia)
	  add_b.insert(sfl::Mapper2d::index_t(ia->ix, ia->iy));
	if (dbgos)
	  *dbgos << "sflCostmapper::updateObstacles(): adding " << add_b.size() << " cells\n";
      }
      else if (dbgos)
	*dbgos << "sflCostmapper::updateObstacles(): no cells to add\n";
      
      sfl::Mapper2d::index_buffer_t remove_b;
      sfl::Mapper2d::index_buffer_t * remove_p(0);
      if (removed_obstacle_indices) {
	remove_p = &remove_b;
	for (index_collection_t::const_iterator ir(removed_obstacle_indices->begin());
	     ir != removed_obstacle_indices->end(); ++ir)
	  remove_b.insert(sfl::Mapper2d::index_t(ir->ix, ir->iy));
	if (dbgos)
	  *dbgos << "sflCostmapper::updateObstacles(): removing " << remove_b.size() << " cells\n";
      }
      else if (dbgos)
	*dbgos << "sflCostmapper::updateObstacles(): no cells to remove\n";
      
      return m2d_->UpdateObstacles(add_p, false, remove_p, 0);
    }
    
  };

}

namespace mpglue {
  
  shared_ptr<Costmapper> createCostmapper(shared_ptr<sfl::Mapper2d> m2d,
					  int possibly_circumscribed_cost)
  {
    shared_ptr<Costmapper> foo(new sflCostmapper(m2d, possibly_circumscribed_cost));
    return foo;
  }

}
