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

/** \file result.h */

#ifndef MPBENCH_BENCHMARK_RESULT_HPP
#define MPBENCH_BENCHMARK_RESULT_HPP

#include <mpbench/setup.h>

namespace mpbench {
  
  namespace result {
    
    typedef enum {
      TASK_ID,
      EPISODE_ID,
      ITERATION_ID
    } id_t;
        
    struct entry {
      entry(size_t task_id,
	    size_t episode_id,
	    size_t iteration_id,
	    episode::startspec const & start,
	    episode::goalspec const & goal,
	    boost::shared_ptr<mpglue::waypoint_plan_t> plan,
	    boost::shared_ptr<mpglue::CostmapPlannerStats> stats);
      
      size_t select(id_t id) const throw(std::exception);
      
      size_t task_id;
      size_t episode_id;
      size_t iteration_id;
      episode::startspec start;
      episode::goalspec goal;
      boost::shared_ptr<mpglue::waypoint_plan_t> plan;
      boost::shared_ptr<mpglue::CostmapPlannerStats> stats;
    };
    
    typedef std::vector<boost::shared_ptr<entry> > list_t;
    typedef std::map<size_t, boost::shared_ptr<entry> > view1_t;
    typedef std::map<size_t, view1_t> view2_t;
    typedef std::map<size_t, view2_t> view3_t;
    
  }
  
  
  class ResultCollection
  {
  public:
    explicit ResultCollection(SetupOptions const & setup_options);
    
    void insert(size_t task_id,
		size_t episode_id,
		size_t iteration_id,
		episode::startspec const & start,
		episode::goalspec const & goal,
		boost::shared_ptr<mpglue::waypoint_plan_t> plan,
		boost::shared_ptr<mpglue::CostmapPlannerStats> stats);
    
    result::list_t const & getAll() const { return list_; }
    
    void createView(result::id_t id1, size_t begin1, size_t end1,
		    result::id_t id2, size_t begin2, size_t end2,
		    result::id_t id3, size_t begin3, size_t end3,
		    result::view3_t & view) const throw(std::exception);
    
    void dumpXML(std::ostream & os,
		 std::string const & prefix) const;
    
  protected:
    SetupOptions const & setup_options_;
    result::list_t list_;
  };
  
}

#endif // MPBENCH_BENCHMARK_RESULT_HPP
