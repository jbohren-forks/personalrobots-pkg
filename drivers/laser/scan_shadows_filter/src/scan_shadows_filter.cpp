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

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu <rusu@cs.tum.edu>

@b ScanShadowsFilter is a simple node that filters shadow points in a laser scan line and publishes the results in a cloud.

 **/

#include "ros/node.h"
#include "std_msgs/PointCloud.h"
#include "std_msgs/LaserScan.h"

#include <float.h>

// Laser projection
#include "laser_scan/laser_scan.h"

using namespace std_msgs;

class ScanShadowsFilter : public ros::node
{
  public:

    // ROS related
    LaserScan tilt_scan_msg_;               // Filled by subscriber with new tilte laser scans
    laser_scan::LaserProjection projector_; // Used to project laser scans

    double tilt_laser_max_range_;           // Used in laser scan projection
    double min_angle_, max_angle_;          // Filter angle threshold
    int window_;

    ////////////////////////////////////////////////////////////////////////////////
    ScanShadowsFilter () : ros::node ("scan_shadows_filter"), tilt_laser_max_range_ (DBL_MAX)
    {
      subscribe("tilt_scan",  tilt_scan_msg_,  &ScanShadowsFilter::tiltScanCallback, 10);

      advertise<PointCloud> ("tilt_laser_cloud_filtered", 10);

      param ("~filter_min_angle", min_angle_, 10.0);
      param ("~filter_max_angle", max_angle_, 170.0);
      param ("~filter_window", window_, 2);
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
    /** \brief Given a PointCloud representing a single laser scan (usually obtained
     * after LaserProjection's projectLaser(), and the index of the channel
     * representing the true measurement "index", create a complete PointCloud
     * representation which replaces the invalid measurements with 0 values.
     * \param c_idx the channel index for the "index"
     * \param cloud_in the input PointCloud message
     * \param cloud_out the output PointCloud message
     */
    void
      constructCompleteLaserScanCloud (int c_idx, PointCloud cloud_in, PointCloud &cloud_out)
    {
      // Use the index to revert to a full laser scan cloud (inefficient locally, but efficient globally)
      int idx = 0;
      for (unsigned int i = 0; i < cloud_out.pts.size (); i++)
      {
        unsigned int j = (int)cloud_in.chan[c_idx].vals[idx];  // Find out the true index value
        if (i == j)
        {
          // Copy relevant data
          cloud_out.pts[i].x = cloud_in.pts[idx].x;
          cloud_out.pts[i].y = cloud_in.pts[idx].y;
          cloud_out.pts[i].z = cloud_in.pts[idx].z;
          for (unsigned int d = 0; d < cloud_out.get_chan_size (); d++)
            cloud_out.chan[d].vals[i] = cloud_in.chan[d].vals[idx];

          idx++;                                        // Assume chan['index'] is sorted (which should be true)
          if (idx >= (int)cloud_in.chan[c_idx].vals.size ()) idx = cloud_in.chan[c_idx].vals.size () - 1;
        }
        else
        {
          // Bogus XYZ entry. No need to copy channels.
          cloud_out.pts[i].x = cloud_out.pts[i].y = cloud_out.pts[i].z = 0;
        }
      }
    }

    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Filter shadow points based on 3 global parameters: min_angle, max_angle
     * and window. {min,max}_angle specify the allowed angle interval (in degrees)
     * between the created lines (see getAngleWithViewPoint). Window specifies how many
     * consecutive measurements to take into account for one point.
     * \param cloud_in the input PointCloud message
     * \param cloud_out the output PointCloud message
     */
    void
      filterShadowPoints (PointCloud cloud_in, PointCloud &cloud_out)
    {
      // For each point in the current line scan
      int n_pts_filtered = 0;
      for (unsigned int i = 0; i < cloud_in.pts.size (); i++)
      {
        bool valid_point = true;
        for (int y = -window_; y < window_ + 1; y++)
        {
          int j = i + y;
          if ( j < 0 || j >= (int)cloud_in.pts.size () || (int)i == j ) // Out of scan bounds or itself
            continue;

          double angle = getAngleWithViewpoint (cloud_in.pts[i].x, cloud_in.pts[i].y, cloud_in.pts[i].z,
                                                cloud_in.pts[j].x, cloud_in.pts[j].y, cloud_in.pts[j].z);
          if (angle < min_angle_ || angle > max_angle_)
            valid_point = false;
        }

        // If point found as 'ok', copy the relevant data
        if (valid_point)
        {
          cloud_out.pts[n_pts_filtered].x = cloud_in.pts[i].x;
          cloud_out.pts[n_pts_filtered].y = cloud_in.pts[i].y;
          cloud_out.pts[n_pts_filtered].z = cloud_in.pts[i].z;

          for (unsigned int d = 0; d < cloud_out.get_chan_size (); d++)
            cloud_out.chan[d].vals[n_pts_filtered] = cloud_in.chan[d].vals[i];

          n_pts_filtered++;
        }
      }

      // Resize output vectors
      cloud_out.pts.resize (n_pts_filtered);
      for (unsigned int d = 0; d < cloud_out.get_chan_size (); d++)
        cloud_out.chan[d].vals.resize (n_pts_filtered);
    }

    ////////////////////////////////////////////////////////////////////////////////
    void
      tiltScanCallback ()
    {
      // Project laser into point cloud
      PointCloud tilt_cloud;
      int n_scan = tilt_scan_msg_.ranges.size ();      // Save the number of measurements

      // Transform into a PointCloud message
      projector_.projectLaser (tilt_scan_msg_, tilt_cloud, tilt_laser_max_range_);//, true);

      /// ---[ Perhaps unnecessary, but find out which channel contains the index
      int c_idx = -1;
      for (unsigned int d = 0; d < tilt_cloud.get_chan_size (); d++)
      {
        if (tilt_cloud.chan[d].name == "index")
        {
          c_idx = d;
          break;
        }
      }
      if (c_idx == -1 || tilt_cloud.chan[c_idx].vals.size () == 0) return;
      /// ]--

      // Prepare the storage for the temporary array ([] and resize are faster than push_back)
      PointCloud tilt_full_cloud (tilt_cloud);
      tilt_full_cloud.pts.resize (n_scan);
      for (unsigned int d = 0; d < tilt_cloud.get_chan_size (); d++)
        tilt_full_cloud.chan[d].vals.resize (n_scan);

      // Prepare data storage for the output array ([] and resize are faster than push_back)
      PointCloud filtered_cloud (tilt_cloud);
      filtered_cloud.pts.resize (n_scan);
      for (unsigned int d = 0; d < tilt_cloud.get_chan_size (); d++)
        filtered_cloud.chan[d].vals.resize  (n_scan);

      // Construct a complete laser cloud resembling the original LaserScan (0..LASER_MAX measurements)
      constructCompleteLaserScanCloud (c_idx, tilt_cloud, tilt_full_cloud);

      // Filter points
      filterShadowPoints (tilt_full_cloud, filtered_cloud);

      // Set timestamp/frameid and publish
      filtered_cloud.header = tilt_scan_msg_.header;
      publish ("tilt_laser_cloud_filtered", filtered_cloud);
    }

} ;

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  ScanShadowsFilter f;
  f.spin ();

  ros::fini ();

  return (0);
}
/* ]--- */

