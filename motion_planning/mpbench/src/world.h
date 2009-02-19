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

/** \file world.h */

#ifndef MPBENCH_WORLD_HPP
#define MPBENCH_WORLD_HPP

#include <mpbench/setup.h>

namespace mpbench {
  
  /**
     We need one world per planner and per task, because we cycle
     through episodes per planner and do not want to support either
     rewinding mpglue::ObstacleDelta or changing the costmap which a
     planner uses after it has been set up for it.
  */
  class World
  {
  public:
    World(SetupOptions const & options, std::ostream * verbose_os, std::ostream * debug_os);
    
    /**
       Draw an obstacle line into the mpglue::ObstacleDelta associated
       with a given episode. The line will be expanded by the robot
       radius and with costs descending out up to the freespace
       distance.
    */
    void drawLine(size_t episode_id, bool add,
		  double x0, double y0, double x1, double y1);
    
    void drawPoint(size_t episode_id, bool add,
		   double xx, double yy);
    
    /**
       For each task_id, you can never go backwards in episode_id.
       
       \note The way things are used, you probably won't see a true
       return value for episode_id==0 because that gets initialized
       implicitly the first time getCostmap() is called.
       
       \return true if the costmap has changed, false otherwise.
    */
    bool select(size_t task_id, size_t episode_id) throw(std::exception);
    
    boost::shared_ptr<mpglue::CostmapAccessor const> getCostmap(size_t task_id) const;
    //future//boost::shared_ptr<mpglue::CostmapAccessor const> getSnapshot(size_t episode_id) const;
    boost::shared_ptr<mpglue::IndexTransform const> getIndexTransform() const;
    
    void getWorkspaceBounds(double & x0, double & y0, double & x1, double & y1) const;
    void getInscribedBounds(double & x0, double & y0, double & x1, double & y1) const;
    void getCircumscribedBounds(double & x0, double & y0, double & x1, double & y1) const;
    void getInflatedBounds(double & x0, double & y0, double & x1, double & y1) const;
    
  protected:
    typedef std::vector<boost::shared_ptr<mpglue::ObstacleDelta> > update_t;
    typedef std::map<size_t, size_t> task_episode_map_t; // task_id --> episode_id
    
    boost::shared_ptr<mpglue::ObstacleDelta> getObstdelta(size_t episode_id) const;
    
    boost::shared_ptr<mpglue::Costmapper> getCostmapper(size_t task_id) const;
    
    SetupOptions const opt_;
    std::ostream * verbose_os_;
    std::ostream * debug_os_;
    sfl::GridFrame gridframe_;
    mutable update_t update_;
    double bbx0_, bby0_, bbx1_, bby1_; // workspace bounding box
    
    /** one per task, lazy init */
    mutable std::vector<boost::shared_ptr<mpglue::Costmapper> > costmapper_;
    mutable task_episode_map_t task_episode_map_;
  };
  
}

#endif // MPBENCH_WORLD_HPP
