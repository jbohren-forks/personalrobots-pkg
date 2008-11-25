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

#ifndef OMPL_BENCHMARK_GFX_HPP
#define OMPL_BENCHMARK_GFX_HPP

// XXXX should be <sbpl_util/blah> or something like that
#include <sbpl_util.hh>
#include <footprint.h>
#include <plan_wrap.h>

// XXXX should be <something>
#include "setup.hpp"

namespace ompl {
  
  /** Indexed by taskNumber, and allow more than one entry because
      incremental planners can produce more than one plan per task. */
  typedef std::multimap<size_t, boost::shared_ptr<waypoint_plan_t const> > planList_t;
  
  namespace gfx {
    
    struct Configuration {
      Configuration(SBPLBenchmarkSetup const & setup,
		    EnvironmentWrapper const & environment,
		    SBPLBenchmarkOptions const & opt,
		    bool websiteMode,
		    std::string const & baseFilename,
		    footprint_t const & footprint,
		    planList_t const & planList,
		    bool ignorePlanTheta,
		    std::ostream & logOs);
      
      SBPLBenchmarkSetup const & setup;
      EnvironmentWrapper const & environment;
      double const resolution;
      double const inscribedRadius;
      double const circumscribedRadius;
      bool const websiteMode;
      std::string const baseFilename;
      footprint_t const & footprint;
      planList_t const & planList;
      bool const ignorePlanTheta;
      mutable std::ostream & logOs;
    };
    
    /** Does not return, ends up calling exit() when the user presses
	'q'. Does not support being called multiple times from within
	a given process. It always takes one screenshot without
	prefix. You can make further screenshots by pressing 'p'. In
	websiteMode, it just ends up taking two screenshots (one big
	one small) and then calls exit(). */
    void display(Configuration const & config, char const * windowName,
		 /** XXXX temporary hack */
		 size_t layoutID,
		 int * argc, char ** argv);
    
  }
}

#endif // OMPL_BENCHMARK_GFX_HPP
