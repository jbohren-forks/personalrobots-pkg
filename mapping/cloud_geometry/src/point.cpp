/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
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
 * $Id: point.h,v 1.0 2008/12/04 12:00:00 rusu Exp $
 *
 */

/** \author Radu Bogdan Rusu */

#include <cfloat>
#include "cloud_geometry/point.h"

namespace cloud_geometry
{

  /////////////////////////////////////////////////////////////////////////////////
  /** \brief Compute the median value of a 3D point cloud and return it as a Point32.
    * \param points the point cloud data message
    */
  std_msgs::Point32
    computeMedian (std_msgs::PointCloud points)
  {
    std_msgs::Point32 median;

    // Copy the values to vectors for faster sorting
    std::vector<double> x (points.pts.size ());
    std::vector<double> y (points.pts.size ());
    std::vector<double> z (points.pts.size ());
    for (unsigned int i = 0; i < points.pts.size (); i++)
    {
      x[i] = points.pts[i].x;
      y[i] = points.pts[i].y;
      z[i] = points.pts[i].z;
    }
    sort (x.begin (), x.end ());
    sort (y.begin (), y.end ());
    sort (z.begin (), z.end ());

    int mid = points.pts.size () / 2;
    if (points.pts.size () % 2 == 0)
    {
      median.x = (x[mid-1] + x[mid]) / 2;
      median.y = (y[mid-1] + y[mid]) / 2;
      median.z = (z[mid-1] + z[mid]) / 2;
    }
    else
    {
      median.x = x[mid];
      median.y = y[mid];
      median.z = z[mid];
    }
    return (median);
  }

  /////////////////////////////////////////////////////////////////////////////////
  /** \brief Compute the median value of a 3D point cloud using a given set point
    * indices and return it as a Point32.
    * \param points the point cloud data message
    * \param indices the point indices
    */
  std_msgs::Point32
    computeMedian (std_msgs::PointCloud points, std::vector<int> indices)
  {
    std_msgs::Point32 median;

    // Copy the values to vectors for faster sorting
    std::vector<double> x (indices.size ());
    std::vector<double> y (indices.size ());
    std::vector<double> z (indices.size ());
    for (unsigned int i = 0; i < indices.size (); i++)
    {
      x[i] = points.pts.at (indices.at (i)).x;
      y[i] = points.pts.at (indices.at (i)).y;
      z[i] = points.pts.at (indices.at (i)).z;
    }
    sort (x.begin (), x.end ());
    sort (y.begin (), y.end ());
    sort (z.begin (), z.end ());

    int mid = indices.size () / 2;
    if (indices.size () % 2 == 0)
    {
      median.x = (x[mid-1] + x[mid]) / 2;
      median.y = (y[mid-1] + y[mid]) / 2;
      median.z = (z[mid-1] + z[mid]) / 2;
    }
    else
    {
      median.x = x[mid];
      median.y = y[mid];
      median.z = z[mid];
    }
    return (median);
  }


  /////////////////////////////////////////////////////////////////////////////////
  /** \brief Compute the median absolute deviation:
    * \f[
    * MAD = \sigma * median_i (| Xi - median_j(Xj) |)
    * \f]
    * \note Sigma needs to be chosen carefully (a good starting sigma value is 1.4826)
    * \param points the point cloud data message
    * \param sigma the sigma value
    */
  double
    computeMedianAbsoluteDeviation (std_msgs::PointCloud points, double sigma)
  {
    // median (dist (x - median (x)))
    std_msgs::Point32 median = cloud_geometry::computeMedian (points);

    std::vector<double> distances (points.pts.size ());

    for (unsigned int i = 0; i < points.pts.size (); i++)
      distances[i] = (points.pts[i].x - median.x) * (points.pts[i].x - median.x) +
                     (points.pts[i].y - median.y) * (points.pts[i].y - median.y) +
                     (points.pts[i].z - median.z) * (points.pts[i].z - median.z);

    sort (distances.begin (), distances.end ());

    double result;
    int mid = points.pts.size () / 2;
    // Do we have a "middle" point or should we "estimate" one ?
    if (points.pts.size () % 2 == 0)
      result = (sqrt (distances[mid-1]) + sqrt (distances[mid])) / 2;
    else
      result = sqrt (distances[mid]);
    return (sigma * result);
  }


  /////////////////////////////////////////////////////////////////////////////////
  /** \brief Compute the median absolute deviation:
    * \f[
    * MAD = \sigma * median_i (| Xi - median_j(Xj) |)
    * \f]
    * \note Sigma needs to be chosen carefully (a good starting sigma value is 1.4826)
    * \param points the point cloud data message
    * \param sigma the sigma value
    */
  double
    computeMedianAbsoluteDeviation (std_msgs::PointCloud points, std::vector<int> indices, double sigma)
  {
    // median (dist (x - median (x)))
    std_msgs::Point32 median = computeMedian (points, indices);

    std::vector<double> distances (indices.size ());

    for (unsigned int i = 0; i < indices.size (); i++)
      distances[i] = (points.pts.at (indices.at (i)).x - median.x) * (points.pts.at (indices.at (i)).x - median.x) +
                     (points.pts.at (indices.at (i)).y - median.y) * (points.pts.at (indices.at (i)).y - median.y) +
                     (points.pts.at (indices.at (i)).z - median.z) * (points.pts.at (indices.at (i)).z - median.z);

    sort (distances.begin (), distances.end ());

    double result;
    int mid = indices.size () / 2;
    // Do we have a "middle" point or should we "estimate" one ?
    if (indices.size () % 2 == 0)
      result = (sqrt (distances[mid-1]) + sqrt (distances[mid])) / 2;
    else
      result = sqrt (distances[mid]);

    return (sigma * result);
  }

  /////////////////////////////////////////////////////////////////////////////////
  /** \brief Get the minimum and maximum values on each of the 3 (x-y-z) dimensions
    * in a given pointcloud.
    * \param points the point cloud data message
    */
  void
    getMinMax (std_msgs::PointCloud points, std_msgs::Point32 &min_pt, std_msgs::Point32 &max_pt)
  {
    min_pt.x = min_pt.y = min_pt.z = FLT_MAX;
    max_pt.x = max_pt.y = max_pt.z = FLT_MIN;

    for (unsigned int i = 0; i < points.pts.size (); i++)
    {
      min_pt.x = (points.pts[i].x < min_pt.x) ? points.pts[i].x : min_pt.x;
      min_pt.y = (points.pts[i].y < min_pt.y) ? points.pts[i].y : min_pt.y;
      min_pt.z = (points.pts[i].z < min_pt.z) ? points.pts[i].z : min_pt.z;

      max_pt.x = (points.pts[i].x > max_pt.x) ? points.pts[i].x : max_pt.x;
      max_pt.y = (points.pts[i].y > max_pt.y) ? points.pts[i].y : max_pt.y;
      max_pt.z = (points.pts[i].z > max_pt.z) ? points.pts[i].z : max_pt.z;
    }
  }

  /////////////////////////////////////////////////////////////////////////////////
  /** \brief Get the minimum and maximum values on each of the 3 (x-y-z) dimensions
    * in a given point cloud using a set of given indices.
    * \param points the point cloud data message
    * \param indices the point indices
    */
  void
    getMinMax (std_msgs::PointCloud points, std::vector<int> indices,
               std_msgs::Point32 &min_pt, std_msgs::Point32 &max_pt)
  {
    min_pt.x = min_pt.y = min_pt.z = FLT_MAX;
    max_pt.x = max_pt.y = max_pt.z = FLT_MIN;

    for (unsigned int i = 0; i < indices.size (); i++)
    {
      min_pt.x = (points.pts.at (indices.at (i)).x < min_pt.x) ? points.pts.at (indices.at (i)).x : min_pt.x;
      min_pt.y = (points.pts.at (indices.at (i)).y < min_pt.y) ? points.pts.at (indices.at (i)).y : min_pt.y;
      min_pt.z = (points.pts.at (indices.at (i)).z < min_pt.z) ? points.pts.at (indices.at (i)).z : min_pt.z;

      max_pt.x = (points.pts.at (indices.at (i)).x > max_pt.x) ? points.pts.at (indices.at (i)).x : max_pt.x;
      max_pt.y = (points.pts.at (indices.at (i)).y > max_pt.y) ? points.pts.at (indices.at (i)).y : max_pt.y;
      max_pt.z = (points.pts.at (indices.at (i)).z > max_pt.z) ? points.pts.at (indices.at (i)).z : max_pt.z;
    }
  }
}
