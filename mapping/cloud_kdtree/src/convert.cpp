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

#include "cloud_kdtree/kdtree.h"

namespace cloud_kdtree
{

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Converts a ROS PointCloud message to the internal ANN point array representation. Returns the number of
    * points.
    * \param ros_cloud the ROS PointCloud message
    * \param ann_cloud the ANN point cloud array
    */
  int
    KdTree::convertCloudToArray (std_msgs::PointCloud *ros_cloud, ANNpointArray &ann_cloud)
  {
    // No point in doing anything if the array is empty
    if (ros_cloud->pts.size () == 0)
    {
      ann_cloud = NULL;
      return (0);
    }

    int nr_points = ros_cloud->pts.size ();

    ann_cloud = annAllocPts (nr_points, 3);       // default number of dimensions (3 = xyz)

    for (int cp = 0; cp < nr_points; cp++)
    {
#ifdef USE_ANN
      ann_cloud[cp][0] = ros_cloud->pts[cp].x;
      ann_cloud[cp][1] = ros_cloud->pts[cp].y;
      ann_cloud[cp][2] = ros_cloud->pts[cp].z;
#else
      ann_cloud[cp * 3 + 0] = ros_cloud->pts[cp].x;
      ann_cloud[cp * 3 + 1] = ros_cloud->pts[cp].y;
      ann_cloud[cp * 3 + 2] = ros_cloud->pts[cp].z;
#endif
    }

    return (nr_points);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Converts a ROS PointCloud message to the internal ANN point array representation. Returns the number of
    * points.
    * \param ros_cloud the ROS PointCloud message
    * \param nr_dimensions the number of extra dimensions (channels) that we want to copy into the point data
    * \param ann_cloud the ANN point cloud array
    */
  int
    KdTree::convertCloudToArray (std_msgs::PointCloud *ros_cloud, unsigned int nr_dimensions, ANNpointArray &ann_cloud)
  {
    // No point in doing anything if the array is empty, or if the requested number of dimensions is bigger than
    // what the ros_cloud message holds
    if ( (ros_cloud->pts.size () == 0) || (nr_dimensions > ros_cloud->chan.size ()) )
    {
      ann_cloud = NULL;
      return (0);
    }

    int nr_points = ros_cloud->pts.size ();

    ann_cloud = annAllocPts (nr_points, 3 + nr_dimensions);

    for (int cp = 0; cp < nr_points; cp++)
    {
      // Copy the point data
#ifdef USE_ANN
      ann_cloud[cp][0] = ros_cloud->pts[cp].x;
      ann_cloud[cp][1] = ros_cloud->pts[cp].y;
      ann_cloud[cp][2] = ros_cloud->pts[cp].z;
#else
      ann_cloud[cp * (3 + nr_dimensions) + 0] = ros_cloud->pts[cp].x;
      ann_cloud[cp * (3 + nr_dimensions) + 1] = ros_cloud->pts[cp].y;
      ann_cloud[cp * (3 + nr_dimensions) + 2] = ros_cloud->pts[cp].z;
#endif
      // Copy the remaining dimensions
      for (unsigned int d = 0; d < nr_dimensions; d++)
#ifdef USE_ANN
        ann_cloud[cp][d + 3] = ros_cloud->chan[d].vals[cp];
#else
        ann_cloud[cp * (3 + nr_dimensions) + (d + 3)] = ros_cloud->chan[d].vals[cp];
#endif
    }

    return (nr_points);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Converts a ROS PointCloud message to the internal ANN point array representation. Returns the number of
    * points.
    * \param ros_cloud the ROS PointCloud message
    * \param dimensions the indices of the extra dimensions (channels) that we want to copy into the point data
    * \param ann_cloud the ANN point cloud array
    */
  int
    KdTree::convertCloudToArray (std_msgs::PointCloud *ros_cloud, std::vector<unsigned int> dimensions, ANNpointArray &ann_cloud)
  {
    // No point in doing anything if the array is empty, or if the requested number of dimensions is bigger than
    // what the ros_cloud message holds
    if ( (ros_cloud->pts.size () == 0) || (dimensions.size () > ros_cloud->chan.size ()) )
    {
      ann_cloud = NULL;
      return (0);
    }

    int nr_points = ros_cloud->pts.size ();

    ann_cloud = annAllocPts (nr_points, 3 + dimensions.size ());

    for (int cp = 0; cp < nr_points; cp++)
    {
#ifdef USE_ANN
      ann_cloud[cp][0] = ros_cloud->pts[cp].x;
      ann_cloud[cp][1] = ros_cloud->pts[cp].y;
      ann_cloud[cp][2] = ros_cloud->pts[cp].z;
#else
      ann_cloud[cp * (3 + dimensions.size ()) + 0] = ros_cloud->pts[cp].x;
      ann_cloud[cp * (3 + dimensions.size ()) + 1] = ros_cloud->pts[cp].y;
      ann_cloud[cp * (3 + dimensions.size ()) + 2] = ros_cloud->pts[cp].z;
#endif
      // Copy the remaining dimensions
      for (unsigned int d = 0; d < dimensions.size (); d++)
      {
        // Check if the values in 'dimensions' are valid
        if ( (dimensions.at (d) > 0) && (dimensions.at (d) < ros_cloud->chan.size ()) )
#ifdef USE_ANN
          ann_cloud[cp][d + 3] = ros_cloud->chan[dimensions.at (d)].vals[cp];
#else
          ann_cloud[cp * (3 + dimensions.size ()) + (d + 3)] = ros_cloud->chan[dimensions.at (d)].vals[cp];
#endif
        else
        {
          annDeallocPts (ann_cloud);
          ann_cloud = NULL;
          return (0);
        }
      }
    }

    return (nr_points);
  }

}
