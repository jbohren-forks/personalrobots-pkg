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
#include <ros/ros.h>
#include <navfn/SetCostmap.h>
#include <costmap_2d/cost_values.h>


namespace mpglue_node {
  
  using namespace mpglue;
  using namespace navfn;
  
  typedef RawCostmapAccessor<uint8_t, uint8_t *, uint16_t> raw_costmap_t;
  
  // XXXX to do: make these configurable via a service call or ROS parameter.
  static uint8_t const lethal_cost(       costmap_2d::LETHAL_OBSTACLE);
  static uint8_t const inscribed_cost(    costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  static uint8_t const circumscribed_cost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE / 2);
  
  
  class CostmapPlannerNode
  {
  public:
    CostmapPlannerNode();
    ~CostmapPlannerNode();
    
    bool setCostmap(SetCostmap::Request& req, SetCostmap::Response& resp);
    
  protected:
    ros::NodeHandle node_;
    ros::ServiceServer set_costmap_service_;
    raw_costmap_t costmap_;
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
    : costmap_(0,		// start with NULL raw data
	       0, 0,		// start with 0-by-0 dimensions
	       lethal_cost,
	       inscribed_cost,
	       circumscribed_cost)
  {
    set_costmap_service_
      = node_.advertiseService("~set_costmap", &CostmapPlannerNode::setCostmap, this);
  }
  
  
  CostmapPlannerNode::
  ~CostmapPlannerNode()
  {
    delete[] costmap_.raw;
  }
  
  
  bool CostmapPlannerNode::
  setCostmap(SetCostmap::Request& req, SetCostmap::Response& resp)
  {
    ROS_INFO("CostmapPlannerNode::setCostmap(): w = %u  h = %u", req.width, req.height);
    size_t const ttncells(req.width * req.height);
    
    // if we have a costmap with different size, just ditch it
    if ((costmap_.raw) && (costmap_.ncells_x * costmap_.ncells_y != ttncells)) {
      ROS_INFO("  existing costmap differs in size");
      delete[] costmap_.raw;
      costmap_.raw = 0;
    }
    costmap_.ncells_x = req.width;
    costmap_.ncells_y = req.height;
    
    // handle empty ranges
    if (0 == ttncells) {
      ROS_INFO("  detected empty costmap");
      return true;
    }
    
    if ( ! costmap_.raw) {
      ROS_INFO("  (re)allocating costmap with %zu cells", ttncells);
      costmap_.raw = new uint8_t[ttncells];
    }
    if ( ! costmap_.raw) {
      ROS_ERROR("CostmapPlannerNode::setCostmap(): failed to allocate %zu cells", ttncells);
      return false;		// alloc error
    }
    
    memcpy(costmap_.raw, &req.costs[0], sizeof(*costmap_.raw) * ttncells);
    return true;
  }
  
}
