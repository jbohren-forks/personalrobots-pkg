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

#ifndef _CLOUD_GEOMETRY_AREAS_H_
#define _CLOUD_GEOMETRY_AREAS_H_

// ROS includes
#include <std_msgs/Point32.h>
#include <std_msgs/PointCloud.h>
#include <std_msgs/Point2DFloat32.h>
#include <std_msgs/Polygon3D.h>
#include <std_msgs/Polyline2D.h>

#include <cloud_geometry/nearest.h>

namespace cloud_geometry
{

  namespace areas
  {

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Sort the Point2DFloat32 points in vector structures according to their .x/.y values
      * \param p1 the first Point2DFloat32 point
      * \param p2 the second Point2DFloat32 point
      */
    inline bool
      comparePoint2DFloat32 (const std_msgs::Point2DFloat32& p1, const std_msgs::Point2DFloat32& p2)
    {
      if (p1.x < p2.x)      return true;
      else if (p1.x > p2.x) return false;
      else if (p1.y < p2.y) return true;
      else                  return false;
    }

    double compute2DPolygonalArea (std_msgs::PointCloud points, std::vector<double> normal);
    double compute2DPolygonalArea (std_msgs::Polygon3D polygon, std::vector<double> normal);
    void convexHull2D (std_msgs::PointCloud *points, std::vector<int> indices, std::vector<double> coeff, std_msgs::Polygon3D &hull);
    void convexHull2D (std::vector<std_msgs::Point2DFloat32> points, std_msgs::Polyline2D &hull);

  }
}

#endif
