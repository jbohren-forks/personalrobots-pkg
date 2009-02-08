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
#include <cloud_geometry/point.h>
#include <cloud_geometry/statistics.h>

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
                                        std_msgs::Point32 *axis, std::vector<int> &indices)
  {
    // Check all points
    for (unsigned int i = 0; i < points->pts.size (); i++)
    {
      std_msgs::Point32 p;
      p.x = points->chan[nx].vals[i];
      p.y = points->chan[ny].vals[i];
      p.z = points->chan[nz].vals[i];
      // Compute the angle between their normal and the given axis
      double angle = acos (dot (&p, axis));
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
                                             std_msgs::Point32 *axis, std::vector<int> &indices)
  {
    // Check all points
    for (unsigned int i = 0; i < points->pts.size (); i++)
    {
      std_msgs::Point32 p;
      p.x = points->chan[nx].vals[i];
      p.y = points->chan[ny].vals[i];
      p.z = points->chan[nz].vals[i];
      // Compute the angle between their normal and the given axis
      double angle = acos (dot (&p, axis));
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
    downsamplePointCloud (std_msgs::PointCloud *points, std::vector<int> *indices, std_msgs::PointCloud &points_down, std_msgs::Point leaf_size,
                          std::vector<Leaf> &leaves, int d_idx, double cut_distance)
  {
    if (d_idx == -1)
      cut_distance = DBL_MAX;
    // Copy the header (and thus the frame_id) + allocate enough space for points
    points_down.header = points->header;
    points_down.pts.resize (points->pts.size ());

    std_msgs::Point32 minP, maxP, minB, maxB, divB;
    cloud_geometry::statistics::getMinMax (points, indices, minP, maxP, d_idx, cut_distance);

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
    for (unsigned int cp = 0; cp < indices->size (); cp++)
    {
      if (d_idx != -1 && points->chan[d_idx].vals[indices->at (cp)] > cut_distance)        // Use a threshold for cutting out points which are too far away
        continue;

      int i = (int)(floor (points->pts[indices->at (cp)].x / leaf_size.x));
      int j = (int)(floor (points->pts[indices->at (cp)].y / leaf_size.y));
      int k = (int)(floor (points->pts[indices->at (cp)].z / leaf_size.z));

      int idx = ( (k - minB.z) * divB.y * divB.x ) + ( (j - minB.y) * divB.x ) + (i - minB.x);
      leaves[idx].centroid_x += points->pts[indices->at (cp)].x;
      leaves[idx].centroid_y += points->pts[indices->at (cp)].y;
      leaves[idx].centroid_z += points->pts[indices->at (cp)].z;
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
    * \param points a pointer to the point cloud message
    * \param indices a set of point indices
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
    cloud_geometry::statistics::getMinMax (points, minP, maxP, d_idx, cut_distance);

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
