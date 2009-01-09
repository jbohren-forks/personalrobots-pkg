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

#include "footprint.h"

namespace mpglue {
  
  footprint_t createSimpleFootprint(double inscribedRadius, double circumscribedRadius)
  {
    footprint_t footprint;
    initSimpleFootprint(footprint, inscribedRadius, circumscribedRadius);
    return footprint;
  }
  
  
  // copy-pasted and adapted from highlevel_controllers/MoveBase
  // constructor
  void initSimpleFootprint(footprint_t & footprint,
			   double inscribedRadius, double circumscribedRadius)
  {
    std_msgs::Point2DFloat32 pt;
    
    //create a square footprint
    pt.x = inscribedRadius;
    pt.y = -1 * inscribedRadius;
    footprint.push_back(pt);
    pt.x = -1 * inscribedRadius;
    pt.y = -1 * inscribedRadius;
    footprint.push_back(pt);
    pt.x = -1 * inscribedRadius;
    pt.y = inscribedRadius;
    footprint.push_back(pt);
    pt.x = inscribedRadius;
    pt.y = inscribedRadius;
    footprint.push_back(pt);
    
    //give the robot a nose
    pt.x = circumscribedRadius;
    pt.y = 0;
    footprint.push_back(pt);
  }
  
}
