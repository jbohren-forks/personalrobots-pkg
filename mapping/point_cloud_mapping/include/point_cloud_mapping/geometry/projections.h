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

#ifndef _CLOUD_GEOMETRY_PROJECTIONS_H_
#define _CLOUD_GEOMETRY_PROJECTIONS_H_

// ROS includes
#include <robot_msgs/Point32.h>
#include <robot_msgs/Polygon3D.h>

namespace cloud_geometry
{

  namespace projections
  {

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Project a point onto a plane defined by ax+by+cz+d=0
      * \param p the point to project
      * \param q the resultant projected point
      * \param plane_coefficients the normalized coefficients (a, b, c, d) of a plane
      */
    inline void
      pointToPlane (robot_msgs::Point32 *p, robot_msgs::Point32 &q, std::vector<double> *plane_coefficients)
    {
      double distance = plane_coefficients->at (0) * p->x +
                        plane_coefficients->at (1) * p->y +
                        plane_coefficients->at (2) * p->z +
                        plane_coefficients->at (3);
      // Calculate the projection of the point on the plane
      q.x = p->x - distance * plane_coefficients->at (0);
      q.y = p->y - distance * plane_coefficients->at (1);
      q.z = p->z - distance * plane_coefficients->at (2);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Project a set of points onto a plane defined by ax+by+cz+d=0
      * \param p the points to project
      * \param q the resultant projected points
      * \param plane_coefficients the normalized coefficients (a, b, c, d) of a plane
      */
    inline void
      pointsToPlane (robot_msgs::Polygon3D *p, robot_msgs::Polygon3D &q, std::vector<double> *plane_coefficients)
    {
      q.points.resize (p->points.size ());
      for (unsigned int i = 0; i < p->points.size (); i++)
        pointToPlane (&p->points[i], q.points[i], plane_coefficients);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Project a point onto a plane defined by ax+by+cz+d=0, and return the point to plane distance
      * \param p the point to project
      * \param q the resultant projected point
      * \param plane_coefficients the normalized coefficients (a, b, c, d) of a plane
      * \param distance the computed distance from p to the plane
      */
    inline void
      pointToPlane (robot_msgs::Point32 *p, robot_msgs::Point32 &q, std::vector<double> *plane_coefficients,
                    double &distance)
    {
      distance = plane_coefficients->at (0) * p->x +
                 plane_coefficients->at (1) * p->y +
                 plane_coefficients->at (2) * p->z +
                 plane_coefficients->at (3);
      // Calculate the projection of the point on the plane
      q.x = p->x - distance * plane_coefficients->at (0);
      q.y = p->y - distance * plane_coefficients->at (1);
      q.z = p->z - distance * plane_coefficients->at (2);
    }

  }
}

#endif
