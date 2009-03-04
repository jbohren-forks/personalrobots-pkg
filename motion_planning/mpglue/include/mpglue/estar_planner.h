/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/** \file estar_planner.h Interface Estar via mpglue::CostmapPlanner. */

#ifndef MPGLUE_ESTAR_PLANNER_HPP
#define MPGLUE_ESTAR_PLANNER_HPP

#include <mpglue/planner.h>

namespace estar {
  class Facade;
}

namespace mpglue {
  
  /**
     \todo For the moment, assume the costmap is initialized at
     construction time, and that it does not change (using lazy init
     though). Later, some cost change management must be added.
  */
  class EstarPlanner
    : public CostmapPlanner
  {
  public:
    EstarPlanner(boost::shared_ptr<CostmapAccessor const> costmap,
		 boost::shared_ptr<IndexTransform const> itransform,
		 std::ostream * erros);
    
  protected:
    virtual void preCreatePlan() throw(std::exception);
    virtual boost::shared_ptr<waypoint_plan_t> doCreatePlan() throw(std::exception);
    
    boost::shared_ptr<estar::Facade> planner_;
    CostmapPlannerStats stats_;
    std::ostream * erros_;
  };
  
}

#endif // MPGLUE_ESTAR_PLANNER_HPP
