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

/** \file world.cpp  */

#include "world.h"
#include <sfl/gplan/Mapper2d.hpp>
#include <sfl/util/numeric.hpp>
#include <sfl/util/strutil.hpp>
// #include <errno.h>
// #include <cstring>
// #include <err.h>

using sfl::minval;
using sfl::maxval;
using sfl::to_string;
using namespace boost;
using namespace std;


namespace {
  
  struct obstupdate: public sfl::GridFrame::draw_callback {
    obstupdate(mpglue::ObstacleDelta & _od, bool _add, ostream * _dbgos)
      : od(_od), add(_add), dbgos(_dbgos) {}
    
    virtual void operator () (ssize_t ix, ssize_t iy) {
      if (dbgos) {
	*dbgos << "obstupdate: ";
	if (add)
	  *dbgos << "add ";
	else
	  *dbgos << "rem ";
	*dbgos << ix << " " << iy << "\n";
      }
      if (add)
	od.addIndex(ix, iy);
      else
	od.removeIndex(ix, iy);
    }
    
    mpglue::ObstacleDelta & od;
    bool add;
    ostream * dbgos;
  };
  
}


namespace mpbench {  
  
  World::
  World(SetupOptions const & options, std::ostream * verbose_os, std::ostream * debug_os)
    : opt_(options),
      verbose_os_(verbose_os),
      debug_os_(debug_os),
      gridframe_(options.costmap_resolution),
      bbx0_(0),
      bby0_(0),
      bbx1_(0),
      bby1_(0)
  {
  }
  
  
  boost::shared_ptr<mpglue::ObstacleDelta> World::
  getObstdelta(size_t episode_id) const
  {
    if (episode_id >= update_.size()) {
      if (debug_os_)
	*debug_os_ << "World::getObstdelta(): making space for episode " << episode_id << "\n";
      update_.resize(episode_id + 1);
    }
    
    shared_ptr<mpglue::ObstacleDelta> & obstd(update_[episode_id]);
    if ( ! obstd) {
      if (debug_os_)
	*debug_os_ << "World::getObstdelta(): allocating delta for episode " << episode_id << "\n";
      obstd.reset(new mpglue::ObstacleDelta());
    }
    
    return obstd;
  }
  
  
  void World::
  drawLine(size_t episode_id, bool add,
	   double x0, double y0, double x1, double y1)
  {
    if (verbose_os_)
      *verbose_os_ << "World::drawLine(" << episode_id << "  " << sfl::to_string(add)
		   << "  " << x0 << "  " << y0 << "  " << x1 << "  " << y1 << ")\n" << flush;
    
    obstupdate wu(*getObstdelta(episode_id), add, debug_os_);
    gridframe_.DrawGlobalLine(x0, y0, x1, y1,
			      0, std::numeric_limits<ssize_t>::max(),
			      0, std::numeric_limits<ssize_t>::max(),
			      wu);
    
    if (add) {
      if (debug_os_)
	*debug_os_ << "World::drawLine(): updating bounding box\n";
      bbx0_ = minval(bbx0_, minval(x0, x1));
      bby0_ = minval(bby0_, minval(y0, y1));
      bbx1_ = maxval(bbx1_, maxval(x0, x1));
      bby1_ = maxval(bby1_, maxval(y0, y1));
    }
  }
  
  
  void World::
  drawPoint(size_t episode_id, bool add,
	    double xx, double yy)
  {
    if (verbose_os_)
      *verbose_os_ << "World::drawPoint(" << episode_id << "  " << sfl::to_string(add)
		   << "  " << xx << "  " << yy << ")\n" << flush;
    
    shared_ptr<mpglue::ObstacleDelta> obstd(getObstdelta(episode_id));
    sfl::GridFrame const gridframe(opt_.costmap_resolution);
    sfl::GridFrame::index_t const idx(gridframe_.GlobalIndex(xx, yy));
    if (add)
      obstd->addIndex(idx.v0, idx.v1);
    else
      obstd->removeIndex(idx.v0, idx.v1);
    
    if (add) {
      if (debug_os_)
	*debug_os_ << "World::drawPoint(): updating bounding box\n";
      bbx0_ = minval(bbx0_, xx);
      bby0_ = minval(bby0_, yy);
      bbx1_ = maxval(bbx1_, xx);
      bby1_ = maxval(bby1_, yy);
    }
  }
  
  
  void World::
  getWorkspaceBounds(double & x0, double & y0, double & x1, double & y1) const
  {
    x0 = bbx0_ - opt_.costmap_resolution;
    y0 = bby0_ - opt_.costmap_resolution;
    x1 = bbx1_ + opt_.costmap_resolution;
    y1 = bby1_ + opt_.costmap_resolution;
  }
  
  
  void World::
  getInscribedBounds(double & x0, double & y0, double & x1, double & y1) const
  {
    x0 = bbx0_ - opt_.costmap_inscribed_radius;
    y0 = bby0_ - opt_.costmap_inscribed_radius;
    x1 = bbx1_ + opt_.costmap_inscribed_radius;
    y1 = bby1_ + opt_.costmap_inscribed_radius;
  }
  
  
  void World::
  getCircumscribedBounds(double & x0, double & y0, double & x1, double & y1) const
  {
    x0 = bbx0_ - opt_.costmap_circumscribed_radius;
    y0 = bby0_ - opt_.costmap_circumscribed_radius;
    x1 = bbx1_ + opt_.costmap_circumscribed_radius;
    y1 = bby1_ + opt_.costmap_circumscribed_radius;
  }
  
  
  void World::
  getInflatedBounds(double & x0, double & y0, double & x1, double & y1) const
  {
    x0 = bbx0_ - opt_.costmap_inflation_radius;
    y0 = bby0_ - opt_.costmap_inflation_radius;
    x1 = bbx1_ + opt_.costmap_inflation_radius;
    y1 = bby1_ + opt_.costmap_inflation_radius;
  }
  
  
  boost::shared_ptr<mpglue::CostmapAccessor const> World::
  getCostmap(size_t task_id) const
  {
    if (task_episode_map_.find(task_id) == task_episode_map_.end())
      const_cast<World*>(this)->select(task_id, 0);
    return getCostmapper(task_id)->getAccessor();
  }
  
  
  boost::shared_ptr<mpglue::IndexTransform const> World::
  getIndexTransform() const
  {
    return getCostmapper(0)->getIndexTransform(); // XXXX all transforms "should be" equal
  }
  
  
  boost::shared_ptr<mpglue::Costmapper> World::
  getCostmapper(size_t task_id) const
  {
    if (task_id >= costmapper_.size()) {
      if (debug_os_)
	*debug_os_ << "World::getCostmapper(): making space for task " << task_id << "\n";
      costmapper_.resize(task_id + 1);
    }
    
    shared_ptr<mpglue::Costmapper> & cm(costmapper_[task_id]);
    if (cm)
      return cm;
    
    if (verbose_os_)
      *verbose_os_ << "World::getCostmapper(): allocating costmapper for task " << task_id
		   << " (currently hardcoded to sfl::Mapper2d)\n";
    boost::shared_ptr<sfl::Mapper2d::travmap_grow_strategy>
      growstrategy(new sfl::Mapper2d::always_grow());
    double const buffer_zone(opt_.costmap_circumscribed_radius - opt_.costmap_inscribed_radius);
    double const padding_factor(0);
    shared_ptr<sfl::Mapper2d> m2d(new sfl::Mapper2d(gridframe_,
						    0, 0, // grid_xbegin, grid_xend
						    0, 0, // grid_ybegin, grid_yend
						    opt_.costmap_inscribed_radius,
						    sfl::maxval(0.0, buffer_zone),
						    padding_factor,
						    0, // freespace cost
						    opt_.costmap_obstacle_cost,
						    // costmap_2d seems to use
						    // a quadratic decay in r7215
						    sfl::exponential_travmap_cost_decay(2),
						    "m2d",
						    sfl::RWlock::Create("m2d"),
						    growstrategy));
    cm = mpglue::createCostmapper(m2d);
    
    return cm;
  }
  
  
  void World::
  select(size_t task_id, size_t episode_id) throw(std::exception)
  {
    task_episode_map_t::iterator item(task_episode_map_.find(task_id));
    size_t delta_id(0);
    if (task_episode_map_.end() != item) {
      if (item->second == episode_id) {
	if (debug_os_)
	  *debug_os_ << "mpbench::World::select(" << task_id << ", " << episode_id
		     << "): already up to date\n";
	return;
      }
      if (item->second > episode_id)
	throw runtime_error("mpbench::World::select(" + to_string(task_id) + ", "
			    + to_string(episode_id) + "): rewinding is not supported, and task "
			    + to_string(task_id) + " is already at episode "
			    + to_string(item->second));
      delta_id = item->second + 1;
    }
    
    shared_ptr<mpglue::Costmapper> cm(getCostmapper(task_id));
    while (delta_id <= episode_id) {
      if (verbose_os_)
	*verbose_os_ << "World::select(" << task_id << ", " << episode_id
		     << "): applying obstacle delta ID " << delta_id << "\n";
      cm->updateObstacles(*getObstdelta(delta_id), debug_os_);
      ++delta_id;
    }
    
    if (task_episode_map_.end() != item)
      item->second = episode_id;
    else
      task_episode_map_.insert(make_pair(task_id, episode_id));
  }

}
