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
 * $Id: nearest.cpp,v 1.0 2008/12/04 12:00:00 rusu Exp $
 *
 */

/** \author Radu Bogdan Rusu */

#include "cloud_geometry/nearest.h"

namespace cloud_geometry
{

  namespace nearest
  {

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the centroid of a set of points and return it as a PointCloud message with 1 value.
      * \param points the input point cloud
      * \param centroid the output centroid
      */
    void
      computeCentroid (std_msgs::PointCloud points, std_msgs::PointCloud &centroid)
    {
      // Prepare the data output
      centroid.pts.resize (1);
      centroid.chan.resize (points.get_chan_size ());
      for (unsigned int d = 0; d < points.get_chan_size (); d++)
      {
        centroid.chan[d].name = points.chan[d].name;
        centroid.chan[d].vals.resize (1);
      }

      // For each point in the cloud
      for (unsigned int i = 0; i < points.get_pts_size (); i++)
      {
        centroid.pts[0].x += points.pts[i].x;
        centroid.pts[0].y += points.pts[i].y;
        centroid.pts[0].z += points.pts[i].z;

        for (unsigned int d = 0; d < points.get_chan_size (); d++)
          centroid.chan[d].vals[0] += points.chan[d].vals[i];
      }

      centroid.pts[0].x /= points.get_pts_size ();
      centroid.pts[0].y /= points.get_pts_size ();
      centroid.pts[0].z /= points.get_pts_size ();
      for (unsigned int d = 0; d < points.get_chan_size (); d++)
        centroid.chan[d].vals[0] /= points.get_pts_size ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the centroid of a set of points using their indices and return it as a PointCloud message with 1 value.
      * \param points the input point cloud
      * \param indices the point cloud indices that need to be used
      * \param centroid the output centroid
      */
    void
      computeCentroid (std_msgs::PointCloud points, std::vector<int> indices, std_msgs::PointCloud &centroid)
    {
      // Prepare the data output
      centroid.pts.resize (1);
      centroid.chan.resize (points.get_chan_size ());
      for (unsigned int d = 0; d < points.get_chan_size (); d++)
      {
        centroid.chan[d].name = points.chan[d].name;
        centroid.chan[d].vals.resize (1);
      }

      // For each point in the cloud
      for (unsigned int i = 0; i < indices.size (); i++)
      {
        centroid.pts[0].x += points.pts.at (indices.at (i)).x;
        centroid.pts[0].y += points.pts.at (indices.at (i)).y;
        centroid.pts[0].z += points.pts.at (indices.at (i)).z;

        for (unsigned int d = 0; d < points.get_chan_size (); d++)
          centroid.chan[d].vals[0] += points.chan[d].vals[i];
      }

      centroid.pts[0].x /= indices.size ();
      centroid.pts[0].y /= indices.size ();
      centroid.pts[0].z /= indices.size ();
      for (unsigned int d = 0; d < points.get_chan_size (); d++)
        centroid.chan[d].vals[0] /= indices.size ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the 3x3 covariance matrix of a given set of points. The result is returned as a Eigen::Matrix3d.
      * \note The (x-y-z) centroid is also returned as a Point32 message.
      * \param points the input point cloud
      * \param covariance_matrix the 3x3 covariance matrix
      * \param centroid the computed centroid
      */
    void
      computeCovarianceMatrix (std_msgs::PointCloud points, Eigen::Matrix3d &covariance_matrix, std_msgs::Point32 &centroid)
    {
      computeCentroid (points, centroid);

      // Initialize to 0
      covariance_matrix = Eigen::Matrix3d::Zero ();

      // Sum of outer products
      // covariance_matrix (k, i)  += points_c (j, k) * points_c (j, i);
      for (unsigned int j = 0; j < points.pts.size (); j++)
      {
        covariance_matrix (0, 0) += (points.pts[j].x - centroid.x) * (points.pts[j].x - centroid.x);
        covariance_matrix (0, 1) += (points.pts[j].x - centroid.x) * (points.pts[j].y - centroid.y);
        covariance_matrix (0, 2) += (points.pts[j].x - centroid.x) * (points.pts[j].z - centroid.z);

        covariance_matrix (1, 0) += (points.pts[j].y - centroid.y) * (points.pts[j].x - centroid.x);
        covariance_matrix (1, 1) += (points.pts[j].y - centroid.y) * (points.pts[j].y - centroid.y);
        covariance_matrix (1, 2) += (points.pts[j].y - centroid.y) * (points.pts[j].z - centroid.z);

        covariance_matrix (2, 0) += (points.pts[j].z - centroid.z) * (points.pts[j].x - centroid.x);
        covariance_matrix (2, 1) += (points.pts[j].z - centroid.z) * (points.pts[j].y - centroid.y);
        covariance_matrix (2, 2) += (points.pts[j].z - centroid.z) * (points.pts[j].z - centroid.z);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the 3x3 covariance matrix of a given set of points. The result is returned as a Eigen::Matrix3d.
      * \param points the input point cloud
      * \param covariance_matrix the 3x3 covariance matrix
      */
    void
      computeCovarianceMatrix (std_msgs::PointCloud points, Eigen::Matrix3d &covariance_matrix)
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
    void
      computeCovarianceMatrix (std_msgs::PointCloud points, std::vector<int> indices, Eigen::Matrix3d &covariance_matrix, std_msgs::Point32 &centroid)
    {
      computeCentroid (points, indices, centroid);

      // Initialize to 0
      covariance_matrix = Eigen::Matrix3d::Zero ();

      for (unsigned int j = 0; j < indices.size (); j++)
      {
        covariance_matrix (0, 0) += (points.pts[indices.at (j)].x - centroid.x) * (points.pts[indices.at (j)].x - centroid.x);
        covariance_matrix (0, 1) += (points.pts[indices.at (j)].x - centroid.x) * (points.pts[indices.at (j)].y - centroid.y);
        covariance_matrix (0, 2) += (points.pts[indices.at (j)].x - centroid.x) * (points.pts[indices.at (j)].z - centroid.z);

        covariance_matrix (1, 0) += (points.pts[indices.at (j)].y - centroid.y) * (points.pts[indices.at (j)].x - centroid.x);
        covariance_matrix (1, 1) += (points.pts[indices.at (j)].y - centroid.y) * (points.pts[indices.at (j)].y - centroid.y);
        covariance_matrix (1, 2) += (points.pts[indices.at (j)].y - centroid.y) * (points.pts[indices.at (j)].z - centroid.z);

        covariance_matrix (2, 0) += (points.pts[indices.at (j)].z - centroid.z) * (points.pts[indices.at (j)].x - centroid.x);
        covariance_matrix (2, 1) += (points.pts[indices.at (j)].z - centroid.z) * (points.pts[indices.at (j)].y - centroid.y);
        covariance_matrix (2, 2) += (points.pts[indices.at (j)].z - centroid.z) * (points.pts[indices.at (j)].z - centroid.z);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the 3x3 covariance matrix of a given set of points using their indices.
      * The result is returned as a Eigen::Matrix3d.
      * \param points the input point cloud
      * \param indices the point cloud indices that need to be used
      * \param covariance_matrix the 3x3 covariance matrix
      */
    void
      computeCovarianceMatrix (std_msgs::PointCloud points, std::vector<int> indices, Eigen::Matrix3d &covariance_matrix)
    {
      std_msgs::Point32 centroid;
      computeCovarianceMatrix (points, indices, covariance_matrix, centroid);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the Least-Squares plane fit for a given set of points, and return the estimated plane parameters
      * together with the surface curvature.
      * \param points the input point cloud
      * \param plane_parameters the plane parameters as: a, b, c, d (ax + by + cz + d = 0)
      * \param curvature the estimated surface curvature as a measure of
      * \f[
      * \lambda_0 / (\lambda_0 + \lambda_1 + \lambda_2)
      * \f]
      */
    void
      computeSurfaceNormalCurvature (std_msgs::PointCloud points, Eigen::Vector4d &plane_parameters, double &curvature)
    {
      std_msgs::Point32 centroid;
      // Compute the 3x3 covariance matrix
      Eigen::Matrix3d covariance_matrix;
      computeCovarianceMatrix (points, covariance_matrix, centroid);

      // Extract the eigenvalues and eigenvectors
      Eigen::Vector3d eigen_values;
      Eigen::Matrix3d eigen_vectors;
      eigen_cov (covariance_matrix, eigen_values, eigen_vectors);

      // Normalize the surface normal (eigenvector corresponding to the smallest eigenvalue)
      double norm = sqrt ( eigen_vectors (0, 0) * eigen_vectors (0, 0) +
                           eigen_vectors (0, 1) * eigen_vectors (0, 1) +
                           eigen_vectors (0, 2) * eigen_vectors (0, 2));
      plane_parameters (0) = eigen_vectors (0, 0) / norm;
      plane_parameters (1) = eigen_vectors (0, 1) / norm;
      plane_parameters (2) = eigen_vectors (0, 2) / norm;

      // Hessian form (D = nc . p_plane (centroid here) + p)
      plane_parameters (3) = -1 * (plane_parameters (0) * centroid.x + plane_parameters[1] * centroid.y + plane_parameters[2] * centroid.z);

      // Compute the curvature surface change
      curvature = fabs ( eigen_values (0) / (eigen_values (0) + eigen_values (1) + eigen_values (2)) );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the Least-Squares plane fit for a given set of points, using their indices,
      * and return the estimated plane parameters together with the surface curvature.
      * \param points the input point cloud
      * \param indices the point cloud indices that need to be used
      * \param plane_parameters the plane parameters as: a, b, c, d (ax + by + cz + d = 0)
      * \param curvature the estimated surface curvature as a measure of
      * \f[
      * \lambda_0 / (\lambda_0 + \lambda_1 + \lambda_2)
      * \f]
      */
    void
      computeSurfaceNormalCurvature (std_msgs::PointCloud points, std::vector<int> indices, Eigen::Vector4d &plane_parameters, double &curvature)
    {
      std_msgs::Point32 centroid;
      // Compute the 3x3 covariance matrix
      Eigen::Matrix3d covariance_matrix;
      computeCovarianceMatrix (points, indices, covariance_matrix, centroid);

      // Extract the eigenvalues and eigenvectors
      Eigen::Vector3d eigen_values;
      Eigen::Matrix3d eigen_vectors;
      eigen_cov (covariance_matrix, eigen_values, eigen_vectors);

      // Normalize the surface normal (eigenvector corresponding to the smallest eigenvalue)
      // Note: Remember to take care of the eigen_vectors ordering ! Check lapack.cpp
      double norm = sqrt ( eigen_vectors (0, 0) * eigen_vectors (0, 0) +
                           eigen_vectors (1, 0) * eigen_vectors (1, 0) +
                           eigen_vectors (2, 0) * eigen_vectors (2, 0));
      plane_parameters (0) = eigen_vectors (0, 0) / norm;
      plane_parameters (1) = eigen_vectors (1, 0) / norm;
      plane_parameters (2) = eigen_vectors (2, 0) / norm;

      // Hessian form (D = nc . p_plane (centroid here) + p)
      plane_parameters (3) = -1 * (plane_parameters (0) * centroid.x + plane_parameters[1] * centroid.y + plane_parameters[2] * centroid.z);

      // Compute the curvature surface change
      curvature = fabs ( eigen_values (0) / (eigen_values (0) + eigen_values (1) + eigen_values (2)) );
    }

  }
}
