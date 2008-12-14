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

#ifndef _CLOUD_GEOMETRY_POINT_H_
#define _CLOUD_GEOMETRY_POINT_H_

// ROS includes
#include "std_msgs/PointCloud.h"
#include "std_msgs/Point32.h"

namespace cloud_geometry
{

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Create a quick copy of a point and its associated channels and return the data as a float vector.
    * \param points the point cloud data message
    * \param index the index of the point to return
    * \param array the vector containing the result
    */
  inline void
    getPointAsFloatArray (std_msgs::PointCloud points, int index, std::vector<float> &array)
  {
    // Resize for XYZ (3) + NR_CHANNELS
    array.resize (3 + points.get_chan_size ());
    array[0] = points.pts[index].x;
    array[1] = points.pts[index].y;
    array[2] = points.pts[index].z;

    for (unsigned int d = 0; d < points.get_chan_size (); d++)
      array[3 + d] = points.chan[d].vals[index];
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Create a quick copy of a point and its associated channels and return the data as a float vector.
    * \param points the point cloud data message
    * \param index the index of the point to return
    * \param array the vector containing the result
    * \param nr_channels the number of channels to copy (starting with the first channel)
    */
  inline void
    getPointAsFloatArray (std_msgs::PointCloud points, int index, std::vector<float> &array, int nr_channels)
  {
    // Resize for XYZ (3) + NR_CHANNELS
    array.resize (3 + nr_channels);
    array[0] = points.pts[index].x;
    array[1] = points.pts[index].y;
    array[2] = points.pts[index].z;

    for (int d = 0; d < nr_channels; d++)
      array[3 + d] = points.chan[d].vals[index];
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Create a quick copy of a point and its associated channels and return the data as a float vector.
    * \param points the point cloud data message
    * \param index the index of the point to return
    * \param array the vector containing the result
    * \param start_channel the first channel to start copying data from
    * \param end_channel the last channel to stop copying data at
    */
  inline void
    getPointAsFloatArray (std_msgs::PointCloud points, int index, std::vector<float> &array, int start_channel, int end_channel)
  {
    // Resize for XYZ (3) + NR_CHANNELS
    array.resize (3 + end_channel - start_channel);
    array[0] = points.pts[index].x;
    array[1] = points.pts[index].y;
    array[2] = points.pts[index].z;

    for (int d = start_channel; d < end_channel; d++)
      array[3 + d - start_channel] = points.chan[d].vals[index];
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Create a quick copy of a point and its associated channels and return the data as a float vector.
    * \param points the point cloud data message
    * \param index the index of the point to return
    * \param array the vector containing the result
    * \param start_channel the first channel to start copying data from
    * \param end_channel the last channel to stop copying data at
    */
  inline void
    getPointAsFloatArray (std_msgs::PointCloud points, int index, std::vector<float> &array, std::vector<int> channels)
  {
    if (channels.size () > points.get_chan_size ())
      return;
    // Resize for XYZ (3) + NR_CHANNELS
    array.resize (3 + channels.size ());
    array[0] = points.pts[index].x;
    array[1] = points.pts[index].y;
    array[2] = points.pts[index].z;

    for (unsigned int d = 0; d < channels.size (); d++)
      array[3 + d] = points.chan[channels.at (d)].vals[index];
  }

  void getMinMax (std_msgs::PointCloud points, std_msgs::Point32 &min_pt, std_msgs::Point32 &max_pt);
  void getMinMax (std_msgs::PointCloud points, std::vector<int> indices, std_msgs::Point32 &min_pt, std_msgs::Point32 &max_pt);

  std_msgs::Point32 computeMedian (std_msgs::PointCloud points);
  std_msgs::Point32 computeMedian (std_msgs::PointCloud points, std::vector<int> indices);

  double computeMedianAbsoluteDeviation (std_msgs::PointCloud points, double sigma);
  double computeMedianAbsoluteDeviation (std_msgs::PointCloud points, std::vector<int> indices, double sigma);


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Compute the cross product between two points (vectors).
    * \param p1 the first point/vector
    * \param p2 the second point/vector
    */
  inline std_msgs::Point32
    cross (std_msgs::Point32 p1, std_msgs::Point32 p2)
  {
    std_msgs::Point32 r;
    r.x = p1.y*p2.z - p1.z*p2.y;
    r.y = p1.z*p2.x - p1.x*p2.z;
    r.z = p1.x*p2.y - p1.y*p2.x;
    return (r);
  }


}

#endif
