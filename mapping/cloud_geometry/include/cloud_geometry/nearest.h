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

#ifndef _CLOUD_GEOMETRY_NEAREST_H_
#define _CLOUD_GEOMETRY_NEAREST_H_

// ROS includes
#include <std_msgs/PointCloud.h>
#include <std_msgs/Point32.h>
#include <std_msgs/Point2DFloat32.h>

#include <Eigen/Core>
#include <Eigen/QR>

namespace cloud_geometry
{

  namespace nearest
  {

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the centroid of a set of points and return it as a Point32 message.
      * \param points the input point cloud
      * \param centroid the output centroid
      */
    inline void
      computeCentroid (std_msgs::PointCloud *points, std_msgs::Point32 &centroid)
    {
      centroid.x = centroid.y = centroid.z = 0;
      // For each point in the cloud
      for (unsigned int i = 0; i < points->pts.size (); i++)
      {
        centroid.x += points->pts.at (i).x;
        centroid.y += points->pts.at (i).y;
        centroid.z += points->pts.at (i).z;
      }

      centroid.x /= points->pts.size ();
      centroid.y /= points->pts.size ();
      centroid.z /= points->pts.size ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the centroid of a set of points using their indices and return it as a Point32 message.
      * \param points the input point cloud
      * \param indices the point cloud indices that need to be used
      * \param centroid the output centroid
      */
    inline void
      computeCentroid (std_msgs::PointCloud *points, std::vector<int> *indices, std_msgs::Point32 &centroid)
    {
      centroid.x = centroid.y = centroid.z = 0;
      // For each point in the cloud
      for (unsigned int i = 0; i < indices->size (); i++)
      {
        centroid.x += points->pts.at (indices->at (i)).x;
        centroid.y += points->pts.at (indices->at (i)).y;
        centroid.z += points->pts.at (indices->at (i)).z;
      }

      centroid.x /= indices->size ();
      centroid.y /= indices->size ();
      centroid.z /= indices->size ();
    }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the 3x3 covariance matrix of a given set of points. The result is returned as a Eigen::Matrix3d.
      * \note The (x-y-z) centroid is also returned as a Point32 message.
      * \param points the input point cloud
      * \param covariance_matrix the 3x3 covariance matrix
      * \param centroid the computed centroid
      */
    inline void
      computeCovarianceMatrix (std_msgs::PointCloud *points, Eigen::Matrix3d &covariance_matrix, std_msgs::Point32 &centroid)
    {
      computeCentroid (points, centroid);

      // Initialize to 0
      covariance_matrix = Eigen::Matrix3d::Zero ();

      // Sum of outer products
      // covariance_matrix (k, i)  += points_c (j, k) * points_c (j, i);
      for (unsigned int j = 0; j < points->pts.size (); j++)
      {
        covariance_matrix (0, 0) += (points->pts[j].x - centroid.x) * (points->pts[j].x - centroid.x);
        covariance_matrix (0, 1) += (points->pts[j].x - centroid.x) * (points->pts[j].y - centroid.y);
        covariance_matrix (0, 2) += (points->pts[j].x - centroid.x) * (points->pts[j].z - centroid.z);

        covariance_matrix (1, 0) += (points->pts[j].y - centroid.y) * (points->pts[j].x - centroid.x);
        covariance_matrix (1, 1) += (points->pts[j].y - centroid.y) * (points->pts[j].y - centroid.y);
        covariance_matrix (1, 2) += (points->pts[j].y - centroid.y) * (points->pts[j].z - centroid.z);

        covariance_matrix (2, 0) += (points->pts[j].z - centroid.z) * (points->pts[j].x - centroid.x);
        covariance_matrix (2, 1) += (points->pts[j].z - centroid.z) * (points->pts[j].y - centroid.y);
        covariance_matrix (2, 2) += (points->pts[j].z - centroid.z) * (points->pts[j].z - centroid.z);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the 3x3 covariance matrix of a given set of points. The result is returned as a Eigen::Matrix3d.
      * \param points the input point cloud
      * \param covariance_matrix the 3x3 covariance matrix
      */
    inline void
      computeCovarianceMatrix (std_msgs::PointCloud *points, Eigen::Matrix3d &covariance_matrix)
    {
      std_msgs::Point32 centroid;
      computeCovarianceMatrix (points, covariance_matrix, centroid);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the 3x3 covariance matrix of a given set of points using their indices.
      * The result is returned as a Eigen::Matrix3d.
      * \note The (x-y-z) centroid is also returned as a Point32 message.
      * \param points the input point cloud
      * \param indices the point cloud indices that need to be used
      * \param covariance_matrix the 3x3 covariance matrix
      * \param centroid the computed centroid
      */
    inline void
      computeCovarianceMatrix (std_msgs::PointCloud *points, std::vector<int> *indices, Eigen::Matrix3d &covariance_matrix, std_msgs::Point32 &centroid)
    {
      computeCentroid (points, indices, centroid);

      // Initialize to 0
      covariance_matrix = Eigen::Matrix3d::Zero ();

      for (unsigned int j = 0; j < indices->size (); j++)
      {
        covariance_matrix (0, 0) += (points->pts[indices->at (j)].x - centroid.x) * (points->pts[indices->at (j)].x - centroid.x);
        covariance_matrix (0, 1) += (points->pts[indices->at (j)].x - centroid.x) * (points->pts[indices->at (j)].y - centroid.y);
        covariance_matrix (0, 2) += (points->pts[indices->at (j)].x - centroid.x) * (points->pts[indices->at (j)].z - centroid.z);

        covariance_matrix (1, 0) += (points->pts[indices->at (j)].y - centroid.y) * (points->pts[indices->at (j)].x - centroid.x);
        covariance_matrix (1, 1) += (points->pts[indices->at (j)].y - centroid.y) * (points->pts[indices->at (j)].y - centroid.y);
        covariance_matrix (1, 2) += (points->pts[indices->at (j)].y - centroid.y) * (points->pts[indices->at (j)].z - centroid.z);

        covariance_matrix (2, 0) += (points->pts[indices->at (j)].z - centroid.z) * (points->pts[indices->at (j)].x - centroid.x);
        covariance_matrix (2, 1) += (points->pts[indices->at (j)].z - centroid.z) * (points->pts[indices->at (j)].y - centroid.y);
        covariance_matrix (2, 2) += (points->pts[indices->at (j)].z - centroid.z) * (points->pts[indices->at (j)].z - centroid.z);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the 3x3 covariance matrix of a given set of points using their indices.
      * The result is returned as a Eigen::Matrix3d.
      * \param points the input point cloud
      * \param indices the point cloud indices that need to be used
      * \param covariance_matrix the 3x3 covariance matrix
      */
    inline void
      computeCovarianceMatrix (std_msgs::PointCloud *points, std::vector<int> *indices, Eigen::Matrix3d &covariance_matrix)
    {
      std_msgs::Point32 centroid;
      computeCovarianceMatrix (points, indices, covariance_matrix, centroid);
    }


    void computeCentroid (std_msgs::PointCloud *points, std_msgs::PointCloud &centroid);
    void computeCentroid (std_msgs::PointCloud *points, std::vector<int> *indices, std_msgs::PointCloud &centroid);

    void computeSurfaceNormalCurvature (std_msgs::PointCloud *points, Eigen::Vector4d &plane_parameters, double &curvature);
    void computeSurfaceNormalCurvature (std_msgs::PointCloud *points, std::vector<int> *indices, Eigen::Vector4d &plane_parameters, double &curvature);

    void computeMomentInvariants (std_msgs::PointCloud *points, double &j1, double &j2, double &j3);
    void computeMomentInvariants (std_msgs::PointCloud *points, std::vector<int> *indices, double &j1, double &j2, double &j3);
    
    bool isBoundaryPoint (std_msgs::PointCloud *points, int q_idx, std::vector<int> *neighbors, Eigen::Vector3d u, Eigen::Vector3d v, double angle_threshold = M_PI / 2.0);
  }
}

#endif
