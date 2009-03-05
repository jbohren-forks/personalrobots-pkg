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

#include <mpglue/estar_planner.h>
#include <mpglue/costmap.h>
#include <estar/Facade.hpp>
#include <estar/Region.hpp>
#include <sfl/util/numeric.hpp>
#include <sfl/util/strutil.hpp>

using namespace estar;
using namespace std;

namespace {
  
  typedef enum {
    NO_GOAL,
    ROBOT_OUT_OF_GRID,
    PLANNING,
    AT_GOAL,
    ROBOT_IN_OBSTACLE,
    ERROR,
    UNREACHABLE,
    HAVE_PLAN,
    BUFFERING
  } status_t;
  
  
  static status_t getStatus(estar::Facade const & planner,
			    ssize_t start_ix, ssize_t start_iy,
			    double wavefront_buffer)
  {
    if ( ! planner.HaveGoal())
      return NO_GOAL;
    
    Facade::node_status_t const nstat(planner.GetStatus(start_ix, start_iy));
    switch (nstat) {
    case Facade::UPWIND:
      break; // do the stuff after this switch
    case Facade::OUT_OF_GRID:
      return ROBOT_OUT_OF_GRID;
    case Facade::DOWNWIND:
      return PLANNING;
    case Facade::WAVEFRONT:
      return PLANNING;
    case Facade::GOAL:
      return AT_GOAL;
    case Facade::OBSTACLE:
      return ROBOT_IN_OBSTACLE;
    default:
      return ERROR;
    }
    
    double const value(planner.GetValue(start_ix, start_iy));
    double thresh;
    if ( ! planner.GetLowestInconsistentValue(thresh)) {
      if (value >= infinity) {
	return UNREACHABLE;
      }
      return HAVE_PLAN;
    }
    if (value + wavefront_buffer < thresh) {
      return HAVE_PLAN;
    }
    if (value < thresh) {
      return BUFFERING;
    }
    return PLANNING;
  }
  
  
  struct get_meta: public Grid::get_meta {
    mpglue::CostmapAccessor const & cm;
    double const power;
    double const obstcost;
    double const delta;
    
    get_meta(mpglue::CostmapAccessor const & _cm, double _power)
      : cm(_cm), power(_power), obstcost(cm.getInscribedCost()),
	delta(obstcost /* - cm.getFreespaceCost() but that is implicitly always zero */) {}
    
    virtual double operator () (ssize_t ix, ssize_t iy) const {
      int cost;
      if ( ! cm.getCost(ix, iy, &cost))
	return 1;
      return sfl::boundval(0.0, pow((obstcost - cost) / delta, power), 1.0);
    }
  };
  
}


namespace mpglue {
  
  EstarPlanner::
  EstarPlanner(boost::shared_ptr<CostmapAccessor const> costmap,
	       boost::shared_ptr<IndexTransform const> itransform,
	       std::ostream * erros)
    : CostmapPlanner(stats_, costmap, itransform),
      erros_(erros)
  {
  }
  
  
  /**
     \todo find a way to handle cost changes
  */
  void EstarPlanner::
  preCreatePlan() throw(std::exception)
  {
    if ( ! planner_) {
      planner_.reset(estar::Facade::CreateDefault(0, 0, itransform_->getResolution()));
      get_meta gm(*costmap_, 2); // XXXX hardcoded quadratic, should be parameter
      planner_->AddRange(costmap_->getXBegin(), costmap_->getXEnd(),
			 costmap_->getYBegin(), costmap_->getYEnd(),
			 &gm);
    }
    
    if (goal_pose_changed_) {
      estar::Region const goalregion(stats_.goal_tol_distance,
				     itransform_->getResolution(),
				     stats_.goal_x,
				     stats_.goal_y,
				     costmap_->getXBegin(), costmap_->getXEnd(),
				     costmap_->getYBegin(), costmap_->getYEnd());
      if (goalregion.GetArea().empty()) {
	throw runtime_error("mpglue::EstarPlanner::preCreatePlan(): empty goal area (is the goal inside of the grid?)");
      }
      
      planner_->RemoveAllGoals();
      for (estar::Region::indexlist_t::const_iterator
	     in(goalregion.GetArea().begin());
	   in != goalregion.GetArea().end(); ++in)
	planner_->AddGoal(in->x, in->y, in->r);
    }
  }
  
  
  boost::shared_ptr<waypoint_plan_t> EstarPlanner::
  doCreatePlan() throw(std::exception)
  {
    double const wavefront_buffer(3 * itransform_->getResolution()); // XXXX hardcoded
    static size_t const batch_nsteps(30); // XXXX hardcoded
    
    status_t status(getStatus(*planner_, stats_.start_ix, stats_.start_iy, wavefront_buffer));
    for (bool run((status != HAVE_PLAN) && (status != AT_GOAL)); run; /**/) {
      switch (status) {
      case NO_GOAL:
	throw runtime_error("mpglue::EstarPlanner::doCreatePlan(): NO_GOAL");	
      case ROBOT_OUT_OF_GRID:
	throw runtime_error("mpglue::EstarPlanner::doCreatePlan(): ROBOT_OUT_OF_GRID");
      case PLANNING:
	break;
      case AT_GOAL:
	run = false;
	break;
      case ROBOT_IN_OBSTACLE:
	//throw runtime_error("mpglue::EstarPlanner::doCreatePlan(): ROBOT_IN_OBSTACLE");
	if (erros_)
	  *erros_ << "mpglue::EstarPlanner::doCreatePlan(): ROBOT_IN_OBSTACLE\n";
	run = false;
	break;
      case ERROR:
	throw runtime_error("mpglue::EstarPlanner::doCreatePlan(): ERROR (bug?)");
      case UNREACHABLE:
	//throw runtime_error("mpglue::EstarPlanner::doCreatePlan(): UNREACHABLE");
	if (erros_)
	  *erros_ << "mpglue::EstarPlanner::doCreatePlan(): UNREACHABLE\n";
	run = false;
	break;
      case HAVE_PLAN:
	run = false;
	break;
      case BUFFERING:
	break;
      default:
	throw runtime_error("mpglue::EstarPlanner::doCreatePlan(): unknown status "
			    + sfl::to_string(status) + " (bug!)");	
      }
      if (run) {
	if ( ! planner_->HaveWork()) {
	  // "never" happens though
	  status = UNREACHABLE;
	  run = false;
	}
	else {
	  for (size_t ii(0); ii < batch_nsteps; ++ii)
	    planner_->ComputeOne();
	  status = getStatus(*planner_, stats_.start_ix, stats_.start_iy, wavefront_buffer);
	}
      }
    }
    
    boost::shared_ptr<waypoint_plan_t> plan;
    if ((status == HAVE_PLAN) || (status == AT_GOAL)) {
      carrot_trace carrot;
      static double const lookahead(30); // XXXX hardcoded
      double const stepsize(0.5 * itransform_->getResolution()); // XXXX hardcoded
      static size_t const maxnsteps(1000); // XXXX hardcoded
      int const retval(planner_->TraceCarrot(stats_.start_x, stats_.start_y,
					     lookahead, stepsize, maxnsteps,
					     carrot, erros_));
      if (0 > retval) {
	if (erros_)
	  *erros_ << "mpglue::EstarPlanner::doCreatePlan(): TraceCarrot() failed (retval "
		  << retval << ")\n";
	return plan;
      }
      
      plan.reset(new waypoint_plan_t());
      PlanConverter pc(plan.get());
      for (carrot_trace::const_iterator ic(carrot.begin()); ic != carrot.end(); ++ic)
	pc.addWaypoint(ic->cx, ic->cy, 0);
      stats_.plan_length = pc.plan_length;
      stats_.plan_angle_change = pc.tangent_change;
    }
    return plan;
  }
  
}
