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

#include "cloud_geometry/point.h"
#include "cloud_geometry/distances.h"

namespace cloud_geometry
{

  namespace distances
  {
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the distance from a point to a line (represented by a point and a direction)
      * \param p a point
      * \param q the point on the line
      * \param dir the direction of the line
      */
    double
      pointToLineDistance (std_msgs::Point32 p, std_msgs::Point32 q, std_msgs::Point32 dir)
    {
      // Calculate the distance from the point to the line
      // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
      std_msgs::Point32 r, p_t;
      r.x = q.x + dir.x;
      r.y = q.y + dir.y;
      r.z = q.z + dir.z;
      p_t.x = r.x - p.x;
      p_t.y = r.y - p.y;
      p_t.z = r.z - p.z;

      std_msgs::Point32 c = cross (p_t, dir);
      double sqr_distance = (c.x * c.x + c.y * c.y + c.z * c.z) / (dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
      return (sqrt (sqr_distance));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the distance from a point to a line (represented by a point and a direction)
      * \param p a point
      * \param line_coefficients the line coefficients (point.x point.y point.z direction.x direction.y direction.z)
      */
    double
      pointToLineDistance (std_msgs::Point32 p, std::vector<double> line_coefficients)
    {
      // Calculate the distance from the point to the line
      // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
      std_msgs::Point32 r, p_t, dir;
      dir.x = line_coefficients[3];
      dir.y = line_coefficients[4];
      dir.z = line_coefficients[5];
      r.x = line_coefficients[0] + dir.x;
      r.y = line_coefficients[1] + dir.y;
      r.z = line_coefficients[2] + dir.z;
      p_t.x = r.x - p.x;
      p_t.y = r.y - p.y;
      p_t.z = r.z - p.z;

      std_msgs::Point32 c = cross (p_t, dir);
      double sqr_distance = (c.x * c.x + c.y * c.y + c.z * c.z) / (dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
      return (sqrt (sqr_distance));
    }

  }
}
