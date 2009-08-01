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

#include <mpglue/costmap.h>
#include <mpglue/planner.h>
#include <mpglue/setup.h>
#include <ros/ros.h>
#include <mpglue/SetIndexTransform.h>
#include <mpglue/SelectPlanner.h>
#include <navfn/SetCostmap.h>
#include <navfn/MakeNavPlan.h>
#include <costmap_2d/cost_values.h>
#include <sfl/gplan/GridFrame.hpp>


namespace mpglue_node {
  
  using namespace mpglue;
  using namespace navfn;
  using namespace boost;
  using namespace std;
  
  typedef RawCostmapAccessor<uint8_t, uint8_t *, uint16_t> raw_costmap_t;
  
  // XXXX to do: make these configurable via a service call or ROS parameter.
  static uint8_t const lethal_cost(       costmap_2d::LETHAL_OBSTACLE);
  static uint8_t const inscribed_cost(    costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  static uint8_t const circumscribed_cost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE / 2);
  static string const default_planner_spec("NavFn");
  static string const default_robot_spec("pr2");
  
  
  class CostmapPlannerNode
  {
  public:
    CostmapPlannerNode();
    ~CostmapPlannerNode();
    
    bool setCostmap(SetCostmap::Request & req, SetCostmap::Response & resp);
    bool setIndexTransform(SetIndexTransform::Request & req, SetIndexTransform::Response & resp);
    bool selectPlanner(SelectPlanner::Request & req, SelectPlanner::Response & resp);
    bool makeNavPlan(MakeNavPlan::Request & req, MakeNavPlan::Response & resp);
    
  protected:
    ros::NodeHandle node_;
    list<ros::ServiceServer> services_;
    shared_ptr<raw_costmap_t> costmap_;
    sfl::GridFrame gridframe_;
    shared_ptr<mpglue::IndexTransform> indexTransform_;
    requestspec spec_;
    shared_ptr<CostmapPlanner> planner_;
  };
  
}


int main (int argc, char ** argv)
{
  ros::init(argc, argv, "costmap_planner_node");
  mpglue_node::CostmapPlannerNode cpn;
  ros::spin();
  return 0;
}


namespace mpglue_node {
  
  CostmapPlannerNode::
  CostmapPlannerNode()
    : costmap_(new raw_costmap_t(0, // start with NULL raw data
				 0, 0, // start with 0-by-0 dimensions
				 lethal_cost,
				 inscribed_cost,
				 circumscribed_cost)),
      gridframe_(0, 0, 0, 1),
      spec_(default_planner_spec, default_robot_spec)
  {
    services_.push_back(node_.advertiseService("~set_costmap",
					       &CostmapPlannerNode::setCostmap,
					       this));
    services_.push_back(node_.advertiseService("~set_index_transform",
					       &CostmapPlannerNode::setIndexTransform,
					       this));
    services_.push_back(node_.advertiseService("~select_planner",
					       &CostmapPlannerNode::selectPlanner,
					       this));
    services_.push_back(node_.advertiseService("~make_nav_plan",
					       &CostmapPlannerNode::makeNavPlan,
					       this));
  }
  
  
  CostmapPlannerNode::
  ~CostmapPlannerNode()
  {
    delete[] costmap_->raw;
  }
  
  
  bool CostmapPlannerNode::
  setCostmap(SetCostmap::Request& req, SetCostmap::Response& resp)
  {
    ROS_INFO("CostmapPlannerNode::setCostmap(): w = %u  h = %u", req.width, req.height);
    size_t const ttncells(req.width * req.height);
    
    // if we have a costmap with different size, just ditch it
    if ((costmap_->raw) && ((size_t) costmap_->ncells_x * costmap_->ncells_y != ttncells)) {
      ROS_INFO("  existing costmap differs in size");
      delete[] costmap_->raw;
      costmap_->raw = 0;
    }
    costmap_->ncells_x = req.width;
    costmap_->ncells_y = req.height;
    
    // handle empty ranges
    if (0 == ttncells) {
      ROS_INFO("  detected empty costmap");
      return true;
    }
    
    if ( ! costmap_->raw) {
      ROS_INFO("  (re)allocating costmap with %zu cells", ttncells);
      costmap_->raw = new uint8_t[ttncells];
    }
    if ( ! costmap_->raw) {
      ROS_ERROR("CostmapPlannerNode::setCostmap(): failed to allocate %zu cells", ttncells);
      return false;		// alloc error
    }
    
    memcpy(costmap_->raw, &req.costs[0], sizeof(*costmap_->raw) * ttncells);
    return true;
  }
  
  
  bool CostmapPlannerNode::
  setIndexTransform(SetIndexTransform::Request & req, SetIndexTransform::Response & resp)
  {
    ROS_INFO("CostmapPlannerNode::setIndexTransform(): x = %g  y = %g  th = %g  res = %g",
	     req.origin_x, req.origin_y, req.origin_th, req.resolution);
    
    if (0 > req.resolution) {
      ROS_ERROR("CostmapPlannerNode::setIndexTransform(): invalid resolution %g (must be >0)",
		req.resolution);
      return false;
    }
    gridframe_.Configure(req.origin_x, req.origin_y, req.origin_th, req.resolution);
    
    if ( ! indexTransform_) {
      ROS_INFO("  allocating index transform (wrapper)");
      indexTransform_.reset(mpglue::createIndexTransform(&gridframe_));
    }
    
    return true;
  }
  
  
  bool CostmapPlannerNode::
  selectPlanner(SelectPlanner::Request & req, SelectPlanner::Response & resp)
  {
    ROS_INFO("CostmapPlannerNode::selectPlanner(): planner_spec = \"%s\"  robot_spec = \"%s\"",
	     req.planner_spec.c_str(), req.robot_spec.c_str());
    
    if (req.planner_spec.empty()) {
      ROS_INFO("  using default planner spec = \"%s\"", default_planner_spec.c_str());
      req.planner_spec = default_planner_spec;
    }
    if (req.robot_spec.empty()) {
      ROS_INFO("  using default robot spec = \"%s\"", default_robot_spec.c_str());
      req.robot_spec = default_robot_spec;
    }
    
    requestspec const newspec(req.planner_spec, req.robot_spec);
    if (planner_ && (newspec == spec_)) {
      ROS_INFO("  already have a planner, and spec has not changed: doing nothing");
    }
    else {
      ROS_INFO("  creating costmap planner");
      if ( ! costmap_->raw) {
	ROS_WARN("  no costmap (yet), will not be able to plan until you send one...");
      }
      // XXXX to do: configurable footprint, optional doorspec
      footprint_t footprint;
      initSimpleFootprint(footprint,
			  newspec.robot_inscribed_radius,
			  newspec.robot_circumscribed_radius);
      doorspec * optional_door(0);
      ostream * verbose_os(0);
      try {
	planner_.reset(createCostmapPlanner(newspec, costmap_, indexTransform_,
					    footprint, optional_door, verbose_os));
      }
      catch (runtime_error const & ee) {
	ROS_ERROR("CostmapPlannerNode::selectPlanner():"
		  " planner_spec = \"%s\"  robot_spec = \"%s\":"
		  " EXCEPTION from mpglue::createCostmapPlanner(): %s",
		  req.planner_spec.c_str(), req.robot_spec.c_str(), ee.what());
	ostringstream os;
	requestspec::help(os, "  requestspec::help()", "    ");
	ROS_INFO("%s", os.str().c_str());
	resp.ok = 0;
	resp.error_message = string("EXCEPTION from mpglue::createCostmapPlanner(): ") + ee.what();
	//	return false;
	return true; // if we return false, the resp does not get sent to the caller
      }
      spec_ = newspec;
    }
    
    resp.ok = 1;
    resp.error_message = "success";
    return true;
  }
  
  
  bool CostmapPlannerNode::
  makeNavPlan(MakeNavPlan::Request & req, MakeNavPlan::Response & resp)
  {
    waypoint_s const start(req.start.pose);
    waypoint_s const goal(req.goal.pose);
    ROS_INFO("CostmapPlannerNode::makeNavPlan():"
	     " start %.2f %.2f %.2f"
	     "  goal %.2f %.2f %.2f %.2f %.2f",
	     start.x, start.y, start.theta,
	     goal.x, goal.y, goal.theta, goal.dr, goal.dtheta);
    
    resp.plan_found = 0;
    
    if ( ! costmap_->raw) {
      ROS_ERROR("CostmapPlannerNode::makeNavPlan():"
		" no costmap - use set_costmap service!");
      // XXXX to do: add an error message to the reply
      return true; // if we return false, the resp does not get sent to the caller
    }
    
    if ( ! planner_) {
      ROS_ERROR("CostmapPlannerNode::makeNavPlan():"
		" no planner - use select_planner service!");
      // XXXX to do: add an error message to the reply
      return true; // if we return false, the resp does not get sent to the caller
    }
    
    // XXXX to do: find an explicit or heuristic way to determine
    // from_scratch. Eg for SBPL planners, you only really need it if
    // a lot of the costs in the vicinity of the old and new path have
    // changed.
    bool force_from_scratch(true);
    
    // XXXX to do: this should only be true if costs have really
    // changed, but maybe also if planner spec got changed since last
    // time this service was called.
    bool flush_cost_changes(true);
    
    planner_->setGoal(goal.x, goal.y, goal.theta);
    planner_->setGoalTolerance(goal.dr, goal.dtheta);
    planner_->setStart(start.x, start.y, start.theta);
    planner_->forcePlanningFromScratch(force_from_scratch);
    planner_->flushCostChanges(flush_cost_changes);
    
    // we should treat SBPL planners specially, because they are
    // capable of incremental planning...
    //   sbpl_planner->stopAtFirstSolution(start.use_initial_solution);
    //   sbpl_planner->setAllocatedTime(start.alloc_time);
    
    shared_ptr<waypoint_plan_t> plan;
    try {
      plan = planner_->createPlan();
    }
    catch (std::exception const & ee) {
      ROS_ERROR("CostmapPlannerNode::makeNavPlan(): createPlan() EXCEPTION %s", ee.what());
      // XXXX to do: add an error message to the reply
      return true; // if we return false, the resp does not get sent to the caller
    }
    
    // XXXX to do: if the log level is above INFO, we should skip
    // creating this stats message.
    ostringstream os;
    if (plan)
      planner_->getStats().logStream(os, "  planning success", "    ");
    else
      planner_->getStats().logStream(os, "  planning FAILURE", "    ");
    ROS_INFO("%s", os.str().c_str());
    
    // XXXX to do: convert path into response message
    return true;
  }
  
}
