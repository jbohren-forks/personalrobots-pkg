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

#ifndef _CLOUD_GEOMETRY_TRANSFORMS_H_
#define _CLOUD_GEOMETRY_TRANSFORMS_H_

// ROS includes
#include <robot_msgs/PointCloud.h>
#include <robot_msgs/Point32.h>

#include <Eigen/Core>

namespace cloud_geometry
{
  namespace transforms
  {

    void getPlaneToPlaneTransformation (std::vector<double> plane_a, std::vector<double> plane_b, float tx, float ty, float tz,
                                        Eigen::Matrix4d &transformation);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Transform a 3D point using a given 4x4 rigid transformation
      * \param point_in the input point
      * \param point_out the resultant transformed point
      * \param transform the 4x4 rigid transformation
      */
    inline void
      transformPoint (robot_msgs::Point32 *point_in, robot_msgs::Point32 &point_out, Eigen::Matrix4d transformation)
    {
      point_out.x = transformation (0, 0) * point_in->x + transformation (0, 1) * point_in->y + transformation (0, 2) * point_in->z + transformation (0, 3);
      point_out.y = transformation (1, 0) * point_in->x + transformation (1, 1) * point_in->y + transformation (1, 2) * point_in->z + transformation (1, 3);
      point_out.z = transformation (2, 0) * point_in->x + transformation (2, 1) * point_in->y + transformation (2, 2) * point_in->z + transformation (2, 3);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Transform a set of 3D points using a given 4x4 rigid transformation
      * \param points_in the input points
      * \param points_out the resultant transformed points
      * \param transform the 4x4 rigid transformation
      */
    inline void
      transformPoints (std::vector<robot_msgs::Point32> *points_in, std::vector<robot_msgs::Point32> &points_out, Eigen::Matrix4d transformation)
    {
      points_out.resize (points_in->size ());
      for (unsigned i = 0; i < points_in->size (); i++)
        transformPoint (&points_in->at (i), points_out[i], transformation);
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Obtain the inverse of a 4x4 rigid transformation matrix
      * \param transformation the input transformation
      * \param transformation_inverse the output transformation (the inverse of \a transformation)
      */
    inline void
      getInverseTransformation (Eigen::Matrix4d transformation, Eigen::Matrix4d &transformation_inverse)
    {
      float tx = transformation (0, 3);
      float ty = transformation (1, 3);
      float tz = transformation (2, 3);

      transformation_inverse (0, 0) = transformation (0, 0);
      transformation_inverse (0, 1) = transformation (1, 0);
      transformation_inverse (0, 2) = transformation (2, 0);
      transformation_inverse (0, 3) = - (transformation (0, 0) * tx + transformation (0, 1) * ty + transformation (0, 2) * tz);


      transformation_inverse (1, 0) = transformation (0, 1);
      transformation_inverse (1, 1) = transformation (1, 1);
      transformation_inverse (1, 2) = transformation (2, 1);
      transformation_inverse (1, 3) = - (transformation (1, 0) * tx + transformation (1, 1) * ty + transformation (1, 2) * tz);

      transformation_inverse (2, 0) = transformation (0, 2);
      transformation_inverse (2, 1) = transformation (1, 2);
      transformation_inverse (2, 2) = transformation (2, 2);
      transformation_inverse (2, 3) = - (transformation (2, 0) * tx + transformation (2, 1) * ty + transformation (2, 2) * tz);

      transformation_inverse (3, 0) = 0;
      transformation_inverse (3, 1) = 0;
      transformation_inverse (3, 2) = 0;
      transformation_inverse (3, 3) = 1;
    }

    void convertAxisAngleToRotationMatrix (robot_msgs::Point32 axis, double angle, Eigen::Matrix3d &rotation);
  }
}

#endif
