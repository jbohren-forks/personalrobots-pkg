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

#ifndef MATH_UTILS_ANGLES
#define MATH_UTILS_ANGLES

#include <cmath>


namespace math_utils
{
    
    /** Convert degrees to radians */
    
    static inline double from_degrees(double degrees)
    {
	return degrees * M_PI / 180.0;   
    }
    
    /** Convert radians to degrees */
    static inline double to_degrees(double radians)
    {
	return radians * 180.0 / M_PI;
    }
    
    /*
     * normalize_angle_positive
     *
     * Normalizes the angle to be 0 circle to 1 circle
     * It takes and returns native units.
     */
    
    static inline double normalize_angle_positive(double angle)
    {
	return fmod(fmod(angle, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI);
    }
    /*
     * normalize
     *
     * Normalizes the angle to be -1/2 circle to +1/2 circle
     * It takes and returns native units.
     *
     */
    
    static inline double normalize_angle(double angle)
    {
	double a = normalize_angle_positive(angle);
	if (a > M_PI)
	    a -= 2.0 *M_PI;
	return a;
    }
    
    /*
     * shortest_angular_distance
     *
     * Given 2 angles, this returns the shortest angular
     * difference.  The inputs and ouputs are of course native
     * units.
     *
     * As an example, if native units are degrees, the result
     * would always be -180 <= result <= 180.  Adding the result
     * to "from" will always get you an equivelent angle to "to".
     */
    
    static inline double shortest_angular_distance(double from, double to)
    {
	double result = normalize_angle_positive(normalize_angle_positive(to) - normalize_angle_positive(from));
	
	if (result > M_PI)
	    // If the result > 180,
	    // It's shorter the other way.
	    result = -(2.0*M_PI - result);
	
	return normalize_angle(result);
    }
    
}

#endif
