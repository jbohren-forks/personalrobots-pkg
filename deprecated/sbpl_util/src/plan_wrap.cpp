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

#include "plan_wrap.h"
#include "environment_wrap.h"
#include <sfl/util/numeric.hpp>

namespace ompl {
  
  void convertPlan(EnvironmentWrapper const & environment,
		   raw_sbpl_plan_t const & raw,
		   waypoint_plan_t * plan,
		   double * optPlanLengthM,
		   double * optTangentChangeRad,
		   double * optDirectionChangeRad)
  {
    double tmpPlanLengthM;
    double tmpTangentChangeRad;
    double tmpDirectionChangeRad;
    double * planLengthM(optPlanLengthM ? optPlanLengthM : &tmpPlanLengthM);
    double * tangentChangeRad(optTangentChangeRad ? optTangentChangeRad : &tmpTangentChangeRad);
    double * directionChangeRad(optDirectionChangeRad ? optDirectionChangeRad : &tmpDirectionChangeRad);
    *planLengthM = 0;
    *tangentChangeRad = 0;
    *directionChangeRad = 0;
    
    double prevx(0), prevy(0), prevtan(0), prevdir(0);
    prevtan = 42.17;	// to detect when it has been initialized (see 42 below)
    for (raw_sbpl_plan_t::const_iterator it(raw.begin()); it != raw.end(); ++it) {
      std_msgs::Pose2DFloat32 const waypoint(environment.GetPoseFromState(*it));
      
      // update stats:
      // - first round, nothing to do
      // - second round, update path length only
      // - third round, update path length and angular change
      if (plan->empty()) {
	prevx = waypoint.x;
	prevy = waypoint.y;
	prevdir = waypoint.th;
      }
      else {
	double const dx(waypoint.x - prevx);
	double const dy(waypoint.y - prevy);
	*planLengthM += sqrt(pow(dx, 2) + pow(dy, 2));
	*directionChangeRad += fabs(sfl::mod2pi(waypoint.th - prevdir));
	double const tangent(atan2(dy, dx));
	if (42 > prevtan) // see 42.17 above
	  *tangentChangeRad += fabs(sfl::mod2pi(tangent - prevtan));
	prevx = waypoint.x;
	prevy = waypoint.y;
	prevdir = waypoint.th;
	prevtan = tangent;
      }
      
      plan->push_back(waypoint);
    }
  }
  
}
