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

/** \file setup.hpp */

#include <std_msgs/Pose2DFloat32.h>
#include <std_msgs/Point2DFloat32.h>
// #include <string>
// #include <vector>

namespace costmap_2d {
  class CostMap2D;
}

namespace ompl {
  
  
  class SBPLBenchmarkSetup
  {
  public:
    struct task {
      task(bool _from_scratch, std_msgs::Pose2DFloat32 _start, std_msgs::Pose2DFloat32 _goal)
	: from_scratch(_from_scratch), start(_start), goal(_goal) {}
      
      bool from_scratch;
      std_msgs::Pose2DFloat32 start, goal;
    };
    
    typedef std::vector<task> tasklist_t;
    
    
    virtual ~SBPLBenchmarkSetup();
    
    virtual costmap_2d::CostMap2D const & getCostmap() const = 0;
    virtual void dumpDescription(char const * filename, char const * title, char const * prefix) const = 0;
    
    tasklist_t const & getTasks() const;
    
  protected:
    tasklist_t tasklist_;
  };
  
  
  class OfficeBenchmark1
    : public SBPLBenchmarkSetup
  {
  public:
    OfficeBenchmark1(/** cell size [m] (square cells) */
		     double resolution,
		     /** (inscribed) radius of the robot [m] */
		     double robot_radius,
		     /** distance from obstacles where cells become
			 freespace [m] (e.g. the circumscribed robot
			 radius) */
		     double freespace_distance,
		     /** the cost value at or above which a cell is
			 considered an obstacle */
		     int obstacle_cost);
    
    virtual ~OfficeBenchmark1();
    virtual costmap_2d::CostMap2D const & getCostmap() const;
    virtual void dumpDescription(char const * filename, char const * title, char const * prefix) const;
    
  protected:
    costmap_2d::CostMap2D * costmap_;
  };

}
