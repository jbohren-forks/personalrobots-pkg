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

namespace mpglue_node {
  
  using namespace mpglue;
  using namespace navfn;
  
  typedef RawCostmapAccessor<uint8_t, uint8_t *, uint16_t> raw_costmap_t;
  
  
  class CostmapPlannerNode
  {
  public:
    CostmapPlannerNode();
    ~CostmapPlannerNode();
    
    bool setCostmap(SetCostmap::Request& req, SetCostmap::Response& resp);
    
    raw_costmap_t costmap_;
  };
  
}

// main ...


namespace mpglue_node {
  
  CostmapPlannerNode::
  ~CostmapPlannerNode()
  {
    delete[] costmap_.raw;
  }
  
  
  bool CostmapPlannerNode::
  setCostmap(SetCostmap::Request& req, SetCostmap::Response& resp)
  {
    size_t const ttncells(req.width * req.height);
    
    // if we have a costmap with different size, just ditch it
    if ((costmap_.raw) && (costmap_.ncells_x * costmap_.ncells_y != ttncells)) {
      delete[] costmap_.raw;
      costmap_.raw = 0;
    }
    costmap_.ncells_x = req.width;
    costmap_.ncells_y = req.height;
    
    // handle empty ranges
    if (0 == ttncells)
      return true;
    
    if ( ! costmap_.raw)
      costmap_.raw = new uint8_t[ttncells];
    if ( ! costmap_.raw)
      return false;		// alloc error
    
    memcpy(costmap_.raw, &req.costs[0], sizeof(*costmap_.raw) * ttncells);
    return true;
  }
  
}
