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

#include "result.h"
#include <sfl/util/strutil.hpp>

using namespace boost;
using namespace std;

namespace mpbench {
  
  namespace result {

    entry::
    entry(size_t _task_id,
	  size_t _episode_id,
	  size_t _iteration_id,
	  episode::startspec const & _start,
	  episode::goalspec const & _goal,
	  boost::shared_ptr<mpglue::waypoint_plan_t> _plan,
	  boost::shared_ptr<mpglue::CostmapPlannerStats> _stats)
      : task_id(_task_id),
	episode_id(_episode_id),
	iteration_id(_iteration_id),
	start(_start),
	goal(_goal),
	plan(_plan),
	stats(_stats)      
    {
    }
    
    size_t entry::
    select(id_t id) const throw(std::exception)
    {
      switch (id) {
      case TASK_ID: return task_id;
      case EPISODE_ID: return episode_id;
      case ITERATION_ID: return iteration_id;
      }
      throw runtime_error("mpbench::result::entry::select(" + sfl::to_string(id)
			  + "): invalid ID, must be <= " + sfl::to_string(ITERATION_ID));
    }
    
  }
  
  
  ResultCollection::
  ResultCollection(SetupOptions const & setup_options)
    : setup_options_(setup_options)
  {
  }
  
  
  void ResultCollection::
  insert(size_t task_id,
	 size_t episode_id,
	 size_t iteration_id,
	 episode::startspec const & start,
	 episode::goalspec const & goal,
	 boost::shared_ptr<mpglue::waypoint_plan_t> plan,
	 boost::shared_ptr<mpglue::CostmapPlannerStats> stats)
  {
    shared_ptr<result::entry>
      entry(new result::entry(task_id, episode_id, iteration_id, start, goal, plan, stats));
    list_.push_back(entry);
  }
  
  
  void ResultCollection::
  createView(result::id_t id1, size_t begin1, size_t end1,
	     result::id_t id2, size_t begin2, size_t end2,
	     result::id_t id3, size_t begin3, size_t end3,
	     result::view3_t & view) const throw(std::exception)
  {
    if ((id1 == id2) || (id1 == id3) || (id2 == id3))
      throw runtime_error("mpbench::ResultCollection::createView(" + sfl::to_string(id1)
			  + ", " + sfl::to_string(begin1)
			  + ", " + sfl::to_string(end1)
			  + ", " + sfl::to_string(id2)
			  + ", " + sfl::to_string(begin2)
			  + ", " + sfl::to_string(end2)
			  + ", " + sfl::to_string(id3)
			  + ", " + sfl::to_string(begin3)
			  + ", " + sfl::to_string(end3)
			  + ",...): IDs must be different from each other");
    
    for (result::list_t::const_iterator il(list_.begin()); il != list_.end(); ++il) {
      shared_ptr<result::entry> entry(*il);
      if ( ! entry)		// "never" happens though
	continue;
      size_t const s1(entry->select(id1));
      if ((s1 < begin1) || (s1 >= end1))
	continue;
      size_t const s2(entry->select(id2));
      if ((s2 < begin2) || (s2 >= end2))
	continue;
      size_t const s3(entry->select(id3));
      if ((s3 < begin3) || (s3 >= end3))
	continue;
      view[s1][s2][s3] = entry;
    }
    
  }
  
  
  void ResultCollection::
  dumpXML(std::ostream & os,
	  std::string const & prefix) const
  {
    os << "<mpbench_result_collection>\n"
       << "  <world_spec>" << setup_options_.world_spec << "</world_spec>\n"
       << "  <planner_spec>" << setup_options_.planner_spec << "</planner_spec>\n"
       << "  <robot_spec>" << setup_options_.robot_spec << "</robot_spec>\n"
       << "  <costmap_spec>" << setup_options_.costmap_spec << "</costmap_spec>\n";
    for (result::list_t::const_iterator ir(list_.begin()); ir != list_.end(); ++ir) {
      os << "  <result>\n"
	 << "    <task_id>" << (*ir)->task_id << "</task_id>\n"
	 << "    <episode_id>" << (*ir)->episode_id << "</episode_id>\n"
	 << "    <iteration_id>" << (*ir)->iteration_id << "</iteration_id>\n";
      (*ir)->stats->dumpXML(os, prefix + "    ");
      os << "  </result>\n";
    }
    os << "</mpbench_result_collection>\n";
  }
  
}
