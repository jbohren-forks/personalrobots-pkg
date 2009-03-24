/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu@cs.tum.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: scan_shadows_filter.cpp,v 1.0 2008/12/04 12:00:00 rusu Exp $
 *
 */

/*
  \author Radu Bogdan Rusu <rusu@cs.tum.edu>


*/

#ifndef LASER_SCAN_SHADOWS_FILTER_H
#define LASER_SCAN_SHADOWS_FILTER_H

#include "filters/filter_base.h"
#include <laser_scan/LaserScan.h>
#include "angles/angles.h"

namespace laser_scan{

/** @b ScanShadowsFilter is a simple filter that filters shadow points in a laser scan line 
 */
template <typename T>
class ScanShadowsFilter : public filters::FilterBase<T>
{
public:

  double laser_max_range_;           // Used in laser scan projection
  double min_angle_, max_angle_;          // Filter angle threshold
  int window_;
    

  ////////////////////////////////////////////////////////////////////////////////
  ScanShadowsFilter () 
  {


  }

  /**@b Configure the filter from XML */
  bool configure()
  {
    if (!filters::FilterBase<T>::getDoubleParam(std::string("min_angle"), min_angle_, 10))
    {
      ROS_ERROR("Error: ShadowsFilter was not given min_angle.\n");
      return false;
    }
    if (!filters::FilterBase<T>::getDoubleParam(std::string("max_angle"), max_angle_, 170))
    {
      ROS_ERROR("Error: ShadowsFilter was not given min_angle.\n");
      return false;
    }
    if (!filters::FilterBase<T>::getIntParam(std::string("window"), window_, 1))
    {
      ROS_ERROR("Error: ShadowsFilter was not given window.\n");
      return false;
    }

    return true;
  }

  ////////////////////////////////////////////////////////////////////////////////
  virtual ~ScanShadowsFilter () { }

  /** @brief calculate the perpendicular angle at the end of r1 to get to r2 
   * See http://en.wikipedia.org/wiki/Law_of_cosines */
  inline double getAngleWithViewpoint(float r1, float r2, float included_angle)
  {
    return atan2(r2 * sin(included_angle), r1 - r2 * cos(included_angle));
  }


  ////////////////////////////////////////////////////////////////////////////////
  /** \brief Filter shadow points based on 3 global parameters: min_angle, max_angle
   * and window. {min,max}_angle specify the allowed angle interval (in degrees)
   * between the created lines (see getAngleWithViewPoint). Window specifies how many
   * consecutive measurements to take into account for one point.
   * \param scan_in the input LaserScan message
   * \param scan_out the output LaserScan message
   */
  bool update(const std::vector<laser_scan::LaserScan>& scans_in, std::vector<laser_scan::LaserScan>& scans_out)
  {
    if (scans_in.size() != 1)
    {
      ROS_ERROR("ScanShadowsFilter is not vectorized");
      return false;
    }
    if (scans_out.size() != 1)
    {
      ROS_WARN("ScanShadowsFilter had to resize output data");
      scans_out.resize(1);
    }
    
    //Local references for ease of use
    const laser_scan::LaserScan& scan_in = scans_in[0];
    laser_scan::LaserScan& scan_out = scans_out[0];

    //copy across all data first
    scan_out = scan_in;

    // For each point in the current line scan
    for (unsigned int i = 0; i < scan_in.ranges.size (); i++)
    {
      for (int y = -window_; y < window_ + 1; y++)
      {
        int j = i + y;
        if ( j < 0 || j >= (int)scan_in.ranges.size () || (int)i == j ) // Out of scan bounds or itself
          continue;
          
        double angle = angles::to_degrees(getAngleWithViewpoint (scan_in.ranges[i],scan_in.ranges[j], scan_in.angle_increment));
        if (angle < min_angle_ || angle > max_angle_) 
        {
          //          printf("\nFailed test with angle %g", angle);
          scan_out.ranges[i] = -1.0 * fabs(scan_in.ranges[i]); //Failed test so set the ranges to invalid value
        }

      }
    }
    return true;
  }

  ////////////////////////////////////////////////////////////////////////////////

} ;
typedef laser_scan::LaserScan laser_scan_laser_scan;
ROS_REGISTER_FILTER(ScanShadowsFilter, laser_scan_laser_scan);
}

#endif //LASER_SCAN_SHADOWS_FILTER_H
