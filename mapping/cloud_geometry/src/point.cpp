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
 * $Id$
 *
 */

/** \author Radu Bogdan Rusu */

#include <algorithm>
#include <cfloat>
#include "cloud_geometry/point.h"

namespace cloud_geometry
{

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Get the index of a specified dimension/channel in a point cloud
    * \param points the point cloud
    * \param channel_name the string defining the channel name
    */
  int
    getChannelIndex (std_msgs::PointCloud *points, std::string channel_name)
  {
    for (unsigned int d = 0; d < points->chan.size (); d++)
      if (points->chan[d].name == channel_name)
        return (d);
    return (-1);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
    std::sort (x.begin (), x.end ());
    std::sort (y.begin (), y.end ());
    std::sort (z.begin (), z.end ());

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

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
    std::sort (x.begin (), x.end ());
    std::sort (y.begin (), y.end ());
    std::sort (z.begin (), z.end ());

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


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

    std::sort (distances.begin (), distances.end ());

    double result;
    int mid = points.pts.size () / 2;
    // Do we have a "middle" point or should we "estimate" one ?
    if (points.pts.size () % 2 == 0)
      result = (sqrt (distances[mid-1]) + sqrt (distances[mid])) / 2;
    else
      result = sqrt (distances[mid]);
    return (sigma * result);
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

    std::sort (distances.begin (), distances.end ());

    double result;
    int mid = indices.size () / 2;
    // Do we have a "middle" point or should we "estimate" one ?
    if (indices.size () % 2 == 0)
      result = (sqrt (distances[mid-1]) + sqrt (distances[mid])) / 2;
    else
      result = sqrt (distances[mid]);

    return (sigma * result);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Get the point indices from a cloud, whose normals are close to parallel with a given axis direction.
    * \param points the point cloud message
    * \param nx the normal X channel index
    * \param ny the normal Y channel index
    * \param nz the normal Z channel index
    * \param axis the given axis direction
    * \param eps_angle the maximum allowed difference threshold between the normal and the axis (in radians)
    * \param indices the resultant indices
    */
  void
    getPointIndicesAxisParallelNormals (std_msgs::PointCloud *points, int nx, int ny, int nz, double eps_angle,
                                        std_msgs::Point32 axis, std::vector<int> &indices)
  {
    // Check all points
    for (unsigned int i = 0; i < points->pts.size (); i++)
    {
      std_msgs::Point32 p;
      p.x = points->chan[nx].vals[i];
      p.y = points->chan[ny].vals[i];
      p.z = points->chan[nz].vals[i];
      // Compute the angle between their normal and the given axis
      double angle = acos (dot (p, axis));
      if ( (angle < eps_angle) || ( (M_PI - angle) < eps_angle ) )
        indices.push_back (i);
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Get the point indices from a cloud, whose normals are close to perpendicular to a given axis direction.
    * \param points the point cloud message
    * \param nx the normal X channel index
    * \param ny the normal Y channel index
    * \param nz the normal Z channel index
    * \param axis the given axis direction
    * \param eps_angle the maximum allowed difference threshold between the normal and the axis (in radians)
    * \param indices the resultant indices
    */
  void
    getPointIndicesAxisPerpendicularNormals (std_msgs::PointCloud *points, int nx, int ny, int nz, double eps_angle,
                                             std_msgs::Point32 axis, std::vector<int> &indices)
  {
    // Check all points
    for (unsigned int i = 0; i < points->pts.size (); i++)
    {
      std_msgs::Point32 p;
      p.x = points->chan[nx].vals[i];
      p.y = points->chan[ny].vals[i];
      p.z = points->chan[nz].vals[i];
      // Compute the angle between their normal and the given axis
      double angle = acos (dot (p, axis));
      if (fabs (M_PI / 2.0 - angle) < eps_angle)
        indices.push_back (i);
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Downsample a Point Cloud using a voxelized grid approach
    * \param points a pointer to the point cloud message
    * \param points_down the resultant downsampled point cloud
    * \param leaf_size the voxel leaf dimensions
    * \param leaves a vector of already existing leaves (empty for the first call)
    * \param d_idx the index of the channel providing distance data (set to -1 if nonexistant)
    * \param cut_distance the maximum admissible distance of a point from the viewpoint (default: FLT_MAX)
    */
  void
    downsamplePointCloud (std_msgs::PointCloud *points, std_msgs::PointCloud &points_down, std_msgs::Point leaf_size,
                          std::vector<Leaf> &leaves, int d_idx, double cut_distance)
  {
    if (d_idx == -1)
      cut_distance = DBL_MAX;
    // Copy the header (and thus the frame_id) + allocate enough space for points
    points_down.header = points->header;
    points_down.pts.resize (points->pts.size ());

    std_msgs::Point32 minP, maxP, minB, maxB, divB;
    getMinMax (points, minP, maxP, d_idx, cut_distance);

    // Compute the minimum and maximum bounding box values
    minB.x = (int)(floor (minP.x / leaf_size.x));
    maxB.x = (int)(floor (maxP.x / leaf_size.x));

    minB.y = (int)(floor (minP.y / leaf_size.y));
    maxB.y = (int)(floor (maxP.y / leaf_size.y));

    minB.z = (int)(floor (minP.z / leaf_size.z));
    maxB.z = (int)(floor (maxP.z / leaf_size.z));

    // Compute the number of divisions needed along all axis
    divB.x = (int)(maxB.x - minB.x + 1);
    divB.y = (int)(maxB.y - minB.y + 1);
    divB.z = (int)(maxB.z - minB.z + 1);

    // Allocate the space needed
    try
    {
      if (leaves.capacity () < divB.x * divB.y * divB.z)
        leaves.reserve (divB.x * divB.y * divB.z);             // fallback to x*y*z from 2*x*y*z due to memory problems
      leaves.resize (divB.x * divB.y * divB.z);
    }
    catch (std::bad_alloc)
    {
      ROS_ERROR ("Attempting to allocate a vector of %f (%g x %g x %g) leaf elements (%f bytes total)", divB.x * divB.y * divB.z,
                 divB.x, divB.y, divB.z, divB.x * divB.y * divB.z * sizeof (Leaf));
    }

    for (unsigned int cl = 0; cl < leaves.size (); cl++)
    {
      if (leaves[cl].nr_points > 0)
      {
        leaves[cl].centroid_x = leaves[cl].centroid_y = leaves[cl].centroid_z = 0.0;
        leaves[cl].nr_points = 0;
      }
    }

    // First pass: go over all points and insert them into the right leaf
    for (unsigned int cp = 0; cp < points->pts.size (); cp++)
    {
      if (d_idx != -1 && points->chan[d_idx].vals[cp] > cut_distance)        // Use a threshold for cutting out points which are too far away
        continue;

      int i = (int)(floor (points->pts[cp].x / leaf_size.x));
      int j = (int)(floor (points->pts[cp].y / leaf_size.y));
      int k = (int)(floor (points->pts[cp].z / leaf_size.z));

      int idx = ( (k - minB.z) * divB.y * divB.x ) + ( (j - minB.y) * divB.x ) + (i - minB.x);
      leaves[idx].centroid_x += points->pts[cp].x;
      leaves[idx].centroid_y += points->pts[cp].y;
      leaves[idx].centroid_z += points->pts[cp].z;
      leaves[idx].nr_points++;
    }

    // Second pass: go over all leaves and compute centroids
    int nr_p = 0;
    for (unsigned int cl = 0; cl < leaves.size (); cl++)
    {
      if (leaves[cl].nr_points > 0)
      {
        points_down.pts[nr_p].x = leaves[cl].centroid_x / leaves[cl].nr_points;
        points_down.pts[nr_p].y = leaves[cl].centroid_y / leaves[cl].nr_points;
        points_down.pts[nr_p].z = leaves[cl].centroid_z / leaves[cl].nr_points;
        nr_p++;
      }
    }
    points_down.pts.resize (nr_p);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Downsample a Point Cloud using a voxelized grid approach
    * \note this method should not be used in fast loops as it always reallocs the std::vector<Leaf> internally (!)
    * \param points a pointer to the point cloud message
    * \param points_down the resultant downsampled point cloud
    * \param leaf_size the voxel leaf dimensions
    */
  void
    downsamplePointCloud (std_msgs::PointCloud *points, std_msgs::PointCloud &points_down, std_msgs::Point leaf_size)
  {
    std::vector<Leaf> leaves;
    downsamplePointCloud (points, points_down, leaf_size, leaves, -1);
  }
}
