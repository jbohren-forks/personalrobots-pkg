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


using namespace robot_msgs;

/** @b ScanShadowsFilter is a simple filter that filters shadow points in a laser scan line 
 */
template <typename T>
class ScanShadowsFilter : public FilterBase<T>
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
  ScanShadowsFilter::configure()
  {
    if (!FilterBase<T>::getDoubleParam(std::string("min_angle"), min_angle_, 10))
    {
      ROS_ERROR("Error: ShadowsFilter was not given min_angle.\n");
      return false;
    }
    if (!FilterBase<T>::getDoubleParam(std::string("max_angle"), man_angle_, 170))
    {
      ROS_ERROR("Error: ShadowsFilter was not given min_angle.\n");
      return false;
    }
    if (!FilterBase<T>::getIntParam(std::string("window"), window_, 1))
    {
      ROS_ERROR("Error: ShadowsFilter was not given window.\n");
      return false;
    }

    return true;
  }

    ////////////////////////////////////////////////////////////////////////////////
    virtual ~ScanShadowsFilter () { }

    ////////////////////////////////////////////////////////////////////////////////
    /**
     * \brief Computes the angle between the two lines created from 2 points and the
     * viewpoint. Returns the angle (in degrees).
     * \param px X coordinate for the first point
     * \param py Y coordinate for the first point
     * \param pz Z coordinate for the first point
     * \param qx X coordinate for the second point
     * \param qy Y coordinate for the second point
     * \param qz Z coordinate for the second point
     */
    inline double
      getAngleWithViewpoint (float px, float py, float pz, float qx, float qy, float qz)
    {
      double dir_a[3], dir_b[3];
      dir_a[0] =    - px; dir_a[1] =    - py; dir_a[2] =    - pz;   // Assume viewpoint is 0,0,0
      dir_b[0] = qx - px; dir_b[1] = qy - py; dir_b[2] = qz - pz;

      // sqrt (sqr (x) + sqr (y) + sqr (z))
      double norm_a = sqrt (dir_a[0]*dir_a[0] + dir_a[1]*dir_a[1] + dir_a[2]*dir_a[2]);
      // Check for bogus 0,0,0 points
      if (norm_a == 0) return (0);
      double norm_b = sqrt (dir_b[0]*dir_b[0] + dir_b[1]*dir_b[1] + dir_b[2]*dir_b[2]);
      if (norm_b == 0) return (0);
      // dot_product (x, y)
      double dot_pr = dir_a[0]*dir_b[0] + dir_a[1]*dir_b[1] + dir_a[2]*dir_b[2];
      if (dot_pr != dot_pr)     // Check for NaNs
        return (0);

      return ( acos (dot_pr / (norm_a * norm_b) ) * 180.0 / M_PI);
    }


    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Filter shadow points based on 3 global parameters: min_angle, max_angle
     * and window. {min,max}_angle specify the allowed angle interval (in degrees)
     * between the created lines (see getAngleWithViewPoint). Window specifies how many
     * consecutive measurements to take into account for one point.
     * \param scan_in the input LaserScan message
     * \param scan_out the output LaserScan message
     */
  bool update(const std::vector<laser_scan::LaserScan>& scans_in, std::vector<laser_scan::LaserScan>& scans_out);    
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
    laser_scan::LaserScan& scan_in = scans_in[0];
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
          
          ///\todo Radu fix this function. 
          double angle = getAngleWithViewpoint (scan_in.ranges[i].x, scan_in.ranges[i].y, scan_in.ranges[i].z,
                                                scan_in.ranges[j].x, scan_in.ranges[j].y, scan_in.ranges[j].z);
          if (angle < min_angle_ || angle > max_angle_) 
            scan_out.ranges[i] = -1.0; //Failed test so set the ranges to invalid value
        }
      }
      return true;
    }

    ////////////////////////////////////////////////////////////////////////////////

} ;
typedef laser_scan::LaserScan laser_scan_laser_scan;
ROS_REGISTER_FILTER(ScanShadowsFilter, laser_scan_laser_scan);


#endif //LASER_SCAN_SHADOWS_FILTER_H
