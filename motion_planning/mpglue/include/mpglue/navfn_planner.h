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

/** \file navfn_planner.h Interface NavFn via mpglue::CostmapPlanner. */

#ifndef MPGLUE_NAVFN_PLANNER_HPP
#define MPGLUE_NAVFN_PLANNER_HPP

#include <mpglue/planner.h>

class NavFn;

namespace mpglue {
  
  /**
     \todo For the moment, assume the costmap is initialized at
     construction time, and that it does not change (using lazy init
     though). Later, some cost change management must be added.
  */
  class NavFnPlanner
    : public CostmapPlanner
  {
  public:
    NavFnPlanner(boost::shared_ptr<CostmapAccessor const> costmap,
		 boost::shared_ptr<IndexTransform const> itransform,
		 /** whether to use convertPlanInterpolate() or not */
		 bool interpolate_plan);
    
  protected:
    virtual void doFlushCostChanges(cost_delta_map_t const & delta);
    virtual void preCreatePlan() throw(std::exception);
    virtual boost::shared_ptr<waypoint_plan_t> doCreatePlan() throw(std::exception);
    //    virtual void postCreatePlan() throw(std::exception);
    
    boost::shared_ptr<NavFn> planner_;
    CostmapPlannerStats stats_;
    bool interpolate_plan_;
  };
  
}

#endif // MPGLUE_NAVFN_PLANNER_HPP
