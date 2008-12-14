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
 * $Id: sac_model_plane.cpp,v 1.0 2008/12/04 12:00:00 rusu Exp $
 *
 */

/** \author Radu Bogdan Rusu */

#include "sample_consensus/sac_model_plane.h"
#include "cloud_geometry/nearest.h"

namespace sample_consensus
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Get 3 random non-collinear points as data samples and return them as point indices.
    * \param iterations the internal number of iterations used by SAC methods
    * \note assumes unique points!
    */
  std::vector<int>
    SACModelPlane::getSamples (int &iterations)
  {
    std::vector<int> random_idx (3);

    // Get a random number between 1 and max_indices
    int idx = (int)(indices_.size () * (rand () / (RAND_MAX + 1.0)));
    // Get the index
    random_idx[0] = indices_.at (idx);

    // Get a second point which is different than the first
    do
    {
      idx = (int)(indices_.size () * (rand () / (RAND_MAX + 1.0)));
      random_idx[1] = indices_.at (idx);
      iterations++;
    } while (random_idx[1] == random_idx[0]);
    iterations--;

    double Dx1, Dy1, Dz1, Dx2, Dy2, Dz2, Dy1Dy2;
    // Compute the segment values (in 3d) between XY
    Dx1 = cloud_->pts[random_idx[1]].x - cloud_->pts[random_idx[0]].x;
    Dy1 = cloud_->pts[random_idx[1]].y - cloud_->pts[random_idx[0]].y;
    Dz1 = cloud_->pts[random_idx[1]].z - cloud_->pts[random_idx[0]].z;

    int iter = 0;
    do
    {
      // Get the third point, different from the first two
      do
      {
        idx = (int)(indices_.size () * (rand () / (RAND_MAX + 1.0)));
        random_idx[2] = indices_.at (idx);
        iterations++;
      } while ( (random_idx[2] == random_idx[1]) || (random_idx[2] == random_idx[0]) );
      iterations--;

      // Compute the segment values (in 3d) between XZ
      Dx2 = cloud_->pts[random_idx[2]].x - cloud_->pts[random_idx[0]].x;
      Dy2 = cloud_->pts[random_idx[2]].y - cloud_->pts[random_idx[0]].y;
      Dz2 = cloud_->pts[random_idx[2]].z - cloud_->pts[random_idx[0]].z;

      Dy1Dy2 = Dy1 / Dy2;
      iter++;

      if (iter > MAX_ITERATIONS_COLLINEAR )
      {
        std::cerr << "WARNING: Could not select 3 non collinear points in " << MAX_ITERATIONS_COLLINEAR << " iterations!!!" << std::endl;
        break;
      }
      iterations++;
    }
    // Use Zoli's method for collinearity check
    while (((Dx1 / Dx2) == Dy1Dy2) && (Dy1Dy2 == (Dz1 / Dz2)));
    iterations--;

    return (random_idx);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Select all the points which respect the given model coefficients as inliers.
    * \param model_coefficients the coefficients of a plane model that we need to compute distances to
    * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
    * \note: we should compare (e^2) < (T^2) but instead we use fabs(e) because threshold is always positive
    * \note: To get the refined inliers of a model, use:
    * ANNpoint refined_coeff = refitModel (...); selectWithinDistance (refined_coeff, threshold);
    */
  std::vector<int>
    SACModelPlane::selectWithinDistance (std::vector<double> model_coefficients, double threshold)
  {
    std::vector<int> inliers;

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (unsigned int i = 0; i < indices_.size (); i++)
    {
      // Calculate the distance from the point to the plane normal as the dot product
      // D = (P-A).N/|N|
      if (fabs (model_coefficients.at (0) * cloud_->pts.at (indices_.at (i)).x +
                model_coefficients.at (1) * cloud_->pts.at (indices_.at (i)).y +
                model_coefficients.at (2) * cloud_->pts.at (indices_.at (i)).z +
                model_coefficients.at (3)) < threshold)
        // Returns the indices of the points whose distances are smaller than the threshold
        inliers.push_back (indices_[i]);
    }
    return (inliers);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Compute all distances from the cloud data to a given plane model.
    * \param model_coefficients the coefficients of a plane model that we need to compute distances to
    */
  std::vector<double>
    SACModelPlane::getDistancesToModel (std::vector<double> model_coefficients)
  {
    std::vector<double> distances (indices_.size ());

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (unsigned int i = 0; i < indices_.size (); i++)
      // Calculate the distance from the point to the plane normal as the dot product
      // D = (P-A).N/|N|
      distances[i] = fabs (model_coefficients.at (0) * cloud_->pts.at (indices_[i]).x +
                           model_coefficients.at (1) * cloud_->pts.at (indices_[i]).y +
                           model_coefficients.at (2) * cloud_->pts.at (indices_[i]).z +
                           model_coefficients.at (3));
    return (distances);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Create a new point cloud with inliers projected onto the plane model.
    * \param inliers the data inliers that we want to project on the plane model
    * \param model_coefficients the coefficients of a plane model
    */
  std_msgs::PointCloud
    SACModelPlane::projectPoints (std::vector<int> inliers, std::vector<double> model_coefficients)
  {
    std_msgs::PointCloud projected_cloud;
    // Allocate enough space
    projected_cloud.pts.resize (inliers.size ());

    projected_cloud.set_chan_size (cloud_->get_chan_size ());

    // Create the channels
    for (unsigned int d = 0; d < projected_cloud.get_chan_size (); d++)
    {
      projected_cloud.chan[d].name = cloud_->chan[d].name;
      projected_cloud.chan[d].vals.resize (inliers.size ());
    }

    // Get the plane normal
    // Calculate the 2-norm: norm (x) = sqrt (sum (abs (v)^2))
    double n_norm = sqrt (model_coefficients.at (0) * model_coefficients.at (0) +
                          model_coefficients.at (1) * model_coefficients.at (1) +
                          model_coefficients.at (2) * model_coefficients.at (2));
    model_coefficients.at (0) /= n_norm;
    model_coefficients.at (1) /= n_norm;
    model_coefficients.at (2) /= n_norm;
    model_coefficients.at (3) /= n_norm;

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (unsigned int i = 0; i < inliers.size (); i++)
    {
      // Calculate the distance from the point to the plane
      double distance_to_plane = model_coefficients.at (0) * cloud_->pts.at (inliers.at (i)).x +
                                 model_coefficients.at (1) * cloud_->pts.at (inliers.at (i)).y +
                                 model_coefficients.at (2) * cloud_->pts.at (inliers.at (i)).z +
                                 model_coefficients.at (3) * 1;
      // Calculate the projection of the point on the plane
      projected_cloud.pts[i].x = cloud_->pts.at (inliers.at (i)).x - distance_to_plane * model_coefficients.at (0);
      projected_cloud.pts[i].y = cloud_->pts.at (inliers.at (i)).y - distance_to_plane * model_coefficients.at (1);
      projected_cloud.pts[i].z = cloud_->pts.at (inliers.at (i)).z - distance_to_plane * model_coefficients.at (2);
      // Copy the other attributes
      for (unsigned int d = 0; d < projected_cloud.get_chan_size (); d++)
        projected_cloud.chan[d].vals[i] = cloud_->chan[d].vals[inliers.at (i)];
    }
    return (projected_cloud); 
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Project inliers (in place) onto the given plane model.
    * \param inliers the data inliers that we want to project on the plane model
    * \param model_coefficients the coefficients of a plane model
    */
  void
    SACModelPlane::projectPointsInPlace (std::vector<int> inliers, std::vector<double> model_coefficients)
  {
    // Get the plane normal
    // Calculate the 2-norm: norm (x) = sqrt (sum (abs (v)^2))
    double n_norm = sqrt (model_coefficients.at (0) * model_coefficients.at (0) +
                          model_coefficients.at (1) * model_coefficients.at (1) +
                          model_coefficients.at (2) * model_coefficients.at (2));
    model_coefficients.at (0) /= n_norm;
    model_coefficients.at (1) /= n_norm;
    model_coefficients.at (2) /= n_norm;
    model_coefficients.at (3) /= n_norm;

    // Iterate through the 3d points and calculate the distances from them to the plane
    for (unsigned int i = 0; i < inliers.size (); i++)
    {
      // Calculate the distance from the point to the plane
      double distance_to_plane = model_coefficients.at (0) * cloud_->pts.at (inliers.at (i)).x +
                                 model_coefficients.at (1) * cloud_->pts.at (inliers.at (i)).y +
                                 model_coefficients.at (2) * cloud_->pts.at (inliers.at (i)).z +
                                 model_coefficients.at (3) * 1;
      // Calculate the projection of the point on the plane
      cloud_->pts.at (inliers.at (i)).x = cloud_->pts.at (inliers.at (i)).x - distance_to_plane * model_coefficients.at (0);
      cloud_->pts.at (inliers.at (i)).y = cloud_->pts.at (inliers.at (i)).y - distance_to_plane * model_coefficients.at (1);
      cloud_->pts.at (inliers.at (i)).z = cloud_->pts.at (inliers.at (i)).z - distance_to_plane * model_coefficients.at (2);
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Check whether the given index samples can form a valid plane model, compute the model coefficients from
    * these samples and store them internally in model_coefficients_. The plane coefficients are:
    * a, b, c, d (ax+by+cz+d=0)
    * \param indices the point indices found as possible good candidates for creating a valid model
    */
  bool
    SACModelPlane::computeModelCoefficients (std::vector<int> indices)
  {
    model_coefficients_.resize (4);
    double Dx1, Dy1, Dz1, Dx2, Dy2, Dz2, Dy1Dy2;
    // Compute the segment values (in 3d) between XY
    Dx1 = cloud_->pts.at (indices.at (1)).x - cloud_->pts.at (indices.at (0)).x;
    Dy1 = cloud_->pts.at (indices.at (1)).y - cloud_->pts.at (indices.at (0)).y;
    Dz1 = cloud_->pts.at (indices.at (1)).z - cloud_->pts.at (indices.at (0)).z;

    // Compute the segment values (in 3d) between XZ
    Dx2 = cloud_->pts.at (indices.at (2)).x - cloud_->pts.at (indices.at (0)).x;
    Dy2 = cloud_->pts.at (indices.at (2)).y - cloud_->pts.at (indices.at (0)).y;
    Dz2 = cloud_->pts.at (indices.at (2)).z - cloud_->pts.at (indices.at (0)).z;

    Dy1Dy2 = Dy1 / Dy2;
    if (((Dx1 / Dx2) == Dy1Dy2) && (Dy1Dy2 == (Dz1 / Dz2)))     // Check for collinearity
      return (false);

    // Compute the plane coefficients from the 3 given points in a straightforward manner
    // calculate the plane normal n = (p2-p1) x (p3-p1) = cross (p2-p1, p3-p1)
    model_coefficients_.at (0) = (cloud_->pts.at (indices.at (1)).y - cloud_->pts.at (indices.at (0)).y) *
                                 (cloud_->pts.at (indices.at (2)).z - cloud_->pts.at (indices.at (0)).z) -
                                 (cloud_->pts.at (indices.at (1)).z - cloud_->pts.at (indices.at (0)).z) *
                                 (cloud_->pts.at (indices.at (2)).y - cloud_->pts.at (indices.at (0)).y);

    model_coefficients_.at (1) = (cloud_->pts.at (indices.at (1)).z - cloud_->pts.at (indices.at (0)).z) *
                                 (cloud_->pts.at (indices.at (2)).x - cloud_->pts.at (indices.at (0)).x) -
                                 (cloud_->pts.at (indices.at (1)).x - cloud_->pts.at (indices.at (0)).x) *
                                 (cloud_->pts.at (indices.at (2)).z - cloud_->pts.at (indices.at (0)).z);

    model_coefficients_.at (2) = (cloud_->pts.at (indices.at (1)).x - cloud_->pts.at (indices.at (0)).x) *
                                 (cloud_->pts.at (indices.at (2)).y - cloud_->pts.at (indices.at (0)).y) -
                                 (cloud_->pts.at (indices.at (1)).y - cloud_->pts.at (indices.at (0)).y) *
                                 (cloud_->pts.at (indices.at (2)).x - cloud_->pts.at (indices.at (0)).x);
    // calculate the 2-norm: norm (x) = sqrt (sum (abs (v)^2))
    // nx ny nz (aka: ax + by + cz ...
    double n_norm = sqrt (model_coefficients_.at (0) * model_coefficients_.at (0) +
                          model_coefficients_.at (1) * model_coefficients_.at (1) +
                          model_coefficients_.at (2) * model_coefficients_.at (2));
    model_coefficients_.at (0) /= n_norm;
    model_coefficients_.at (1) /= n_norm;
    model_coefficients_.at (2) /= n_norm;

    // ... + d = 0
    model_coefficients_.at (3) = -1 * (model_coefficients_.at (0) * cloud_->pts.at (indices.at (0)).x +
                                       model_coefficients_.at (1) * cloud_->pts.at (indices.at (0)).y +
                                       model_coefficients_.at (2) * cloud_->pts.at (indices.at (0)).z);

    return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Recompute the plane coefficients using the given inlier set and return them to the user.
    * @note: these are the coefficients of the plane model after refinement (eg. after SVD)
    * \param inliers the data inliers found as supporting the model
    */
  std::vector<double>
    SACModelPlane::refitModel (std::vector<int> inliers)
  {
    if (inliers.size () == 0)
    {
      // std::cerr << "[SACModelPlanes::RefitModel] Cannot re-fit 0 inliers!" << std::endl;
      return (model_coefficients_);
    }

    Eigen::Vector4d plane_coefficients;
    double curvature;

    // Use Least-Squares to fit the plane through all the given sample points and find out its coefficients
    cloud_geometry::nearest::computeSurfaceNormalCurvature (*cloud_, inliers, plane_coefficients, curvature);

    std::vector<double> refit (4);
    for (int d = 0; d < 4; d++)
      refit[d] = plane_coefficients (d);

    return (refit);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Verify whether a subset of indices verifies the internal plane model coefficients.
    * \param indices the data indices that need to be tested against the plane model
    * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
    */
  bool
    SACModelPlane::doSamplesVerifyModel (std::set<int> indices, double threshold)
  {
    for (std::set<int>::iterator it = indices.begin (); it != indices.end (); ++it)
      if (fabs (model_coefficients_.at (0) * cloud_->pts.at (*it).x +
                model_coefficients_.at (1) * cloud_->pts.at (*it).y +
                model_coefficients_.at (2) * cloud_->pts.at (*it).z +
                model_coefficients_.at (3)) > threshold)
        return (false);

    return (true);
  }
}
