/*
 * Copyright (c) 2009 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
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

#ifndef _CLOUD_GEOMETRY_ANGLES_H_
#define _CLOUD_GEOMETRY_ANGLES_H_

// ROS includes
#include <robot_msgs/Point32.h>
#include <robot_msgs/PointCloud.h>
#include <robot_msgs/Polygon3D.h>
#include <robot_msgs/Polyline2D.h>

#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/nearest.h>

namespace cloud_geometry
{

  namespace angles
  {
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the angle between two planes
      * \param plane_a the normalized coefficients of the first plane
      * \param plane_b the normalized coefficients of the second plane
      */
    inline double
      getAngleBetweenPlanes (std::vector<double> plane_a, std::vector<double> plane_b)
    {
      return (acos (plane_a[0] * plane_b[0] + plane_a[1] * plane_b[1] + plane_a[2] * plane_b[2]));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the angle in the [ 0, 2*PI ) interval of a point (direction) with a reference (0, 0) in 2D.
      * \param point a 2D point
      */
    inline double
      getAngle2D (double point[2])
    {
      double rad;
      if (point[0] == 0)
        rad = (point[1] < 0) ? -M_PI / 2.0 : M_PI / 2.0;
      else
      {
        rad = atan (point[1] / point[0]);
        if (point[0] < 0)
          rad += M_PI;
      }
      if (rad < 0)
        rad += 2 * M_PI;

      return (rad);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the smallest angle between two vectors in the [ 0, PI ) interval in 3D.
      * \param v1 the first 3D vector
      * \param v2 the second 3D vector
      */
    inline double
      getAngle3D (robot_msgs::Point32 *v1, robot_msgs::Point32 *v2)
    {
      // Compute the vectors norms
      double norm_v1 = (v1->x * v1->x) + (v1->y * v1->y) + (v1->z * v1->z);
      double norm_v2 = (v2->x * v2->x) + (v2->y * v2->y) + (v2->z * v2->z);

      // Compute the actual angle
      double rad = acos ( cloud_geometry::dot (v1, v2) / sqrt (norm_v1 * norm_v2) );

      // Check against NaN
      if (std::isnan (rad))
        ROS_ERROR ("[cloud_geometry::angles::getAngle3D] got a NaN angle!");
      return (rad);
    }

  }
}

#endif
