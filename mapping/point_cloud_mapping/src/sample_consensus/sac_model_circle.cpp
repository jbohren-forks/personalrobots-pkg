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

#include "sample_consensus/sac_model_circle.h"
#include "cloud_geometry/nearest.h"

namespace sample_consensus
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Get 3 random non-collinear points as data samples and return them as point indices.
    * \param iterations the internal number of iterations used by SAC methods
    * \note assumes unique points!
    */
  std::vector<int>
    SACModelCircle2D::getSamples (int &iterations)
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
        std::cerr << "[SACModelCircle2D::getSamples] WARNING: Could not select 3 non collinear points in " << MAX_ITERATIONS_COLLINEAR << " iterations!!!" << std::endl;
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
    * \param model_coefficients the coefficients of a circle model that we need to compute distances to
    * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
    * \note: To get the refined inliers of a model, use:
    * ANNpoint refined_coeff = refitModel (...); selectWithinDistance (refined_coeff, threshold);
    */
  std::vector<int>
    SACModelCircle2D::selectWithinDistance (std::vector<double> model_coefficients, double threshold)
  {
    std::vector<int> inliers;

    // Iterate through the 3d points and calculate the distances from them to the circle
    for (unsigned int i = 0; i < indices_.size (); i++)
    {
      // Calculate the distance from the point to the circle as the difference between 
      //dist(point,circle_origin) and circle_radius
      double distance_to_circle = fabs (sqrt (
                                              ( cloud_->pts.at (indices_[i]).x - model_coefficients.at (0) ) *
                                              ( cloud_->pts.at (indices_[i]).x - model_coefficients.at (0) ) +

                                              ( cloud_->pts.at (indices_[i]).y - model_coefficients.at (1) ) *
                                              ( cloud_->pts.at (indices_[i]).y - model_coefficients.at (1) )
                                             ) - model_coefficients.at (2));
      if (distance_to_circle < threshold)
        // Returns the indices of the points whose distances are smaller than the threshold
        inliers.push_back (indices_[i]);
    }
    return (inliers);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Compute all distances from the cloud data to a given 2D circle model.
    * \param model_coefficients the coefficients of a 2D circle model that we need to compute distances to
    */
  std::vector<double>
    SACModelCircle2D::getDistancesToModel (std::vector<double> model_coefficients)
  {
    std::vector<double> distances (indices_.size ());

    // Iterate through the 3d points and calculate the distances from them to the circle
    for (unsigned int i = 0; i < indices_.size (); i++)
      // Calculate the distance from the point to the circle as the difference between 
      //dist(point,circle_origin) and circle_radius
      distances[i] = fabs (sqrt (
                                 ( cloud_->pts.at (indices_[i]).x - model_coefficients.at (0) ) *
                                 ( cloud_->pts.at (indices_[i]).x - model_coefficients.at (0) ) +

                                 ( cloud_->pts.at (indices_[i]).y - model_coefficients.at (1) ) *
                                 ( cloud_->pts.at (indices_[i]).y - model_coefficients.at (1) )
                                ) - model_coefficients.at (2));
    return (distances);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Create a new point cloud with inliers projected onto the 2D circle model.
    * \param inliers the data inliers that we want to project on the 2D circle model
    * \param model_coefficients the coefficients of a 2D circle model
    * \todo implement this.
    */
  std_msgs::PointCloud
    SACModelCircle2D::projectPoints (std::vector<int> inliers, std::vector<double> model_coefficients)
  {
    std::cerr << "[SACModelCircle2D::projecPoints] Not implemented yet." << std::endl;
    return (*cloud_);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Project inliers (in place) onto the given 2D circle model.
    * \param inliers the data inliers that we want to project on the 2D circle model
    * \param model_coefficients the coefficients of a 2D circle model
    * \todo implement this.
    */
  void
    SACModelCircle2D::projectPointsInPlace (std::vector<int> inliers, std::vector<double> model_coefficients)
  {
    std::cerr << "[SACModelCircle2D::projecPointsInPlace] Not implemented yet." << std::endl;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Check whether the given index samples can form a valid 2D circle model, compute the model coefficients from
    * these samples and store them internally in model_coefficients_.
    * \param indices the point indices found as possible good candidates for creating a valid model
    */
  bool
    SACModelCircle2D::computeModelCoefficients (std::vector<int> indices)
  {
    model_coefficients_.resize (3);

    std_msgs::Point32 u, v, m;
    u.x = ( cloud_->pts.at (indices.at (0)).x + cloud_->pts.at (indices.at (1)).x ) / 2;
    u.y = ( cloud_->pts.at (indices.at (1)).x + cloud_->pts.at (indices.at (2)).x ) / 2;

    v.x = ( cloud_->pts.at (indices.at (0)).y + cloud_->pts.at (indices.at (1)).y ) / 2;
    v.y = ( cloud_->pts.at (indices.at (1)).y + cloud_->pts.at (indices.at (2)).y ) / 2;

    m.x = -( cloud_->pts.at (indices.at (1)).x - cloud_->pts.at (indices.at (0)).x ) /
           ( cloud_->pts.at (indices.at (1)).y - cloud_->pts.at (indices.at (0)).y );
    m.y = -( cloud_->pts.at (indices.at (2)).x - cloud_->pts.at (indices.at (1)).x ) /
           ( cloud_->pts.at (indices.at (2)).y - cloud_->pts.at (indices.at (1)).y );

    // Center (x, y)
    model_coefficients_[0] = (m.x * u.x -  m.y * u.y  - (v.x - v.y) )           / (m.x - m.y);
    model_coefficients_[1] = (m.x * m.y * (u.x - u.y) +  m.x * v.y - m.y * v.x) / (m.x - m.y);

    // Radius
    model_coefficients_[2] = sqrt (
                                   ( model_coefficients_[0] - cloud_->pts.at (indices.at (0)).x ) *
                                   ( model_coefficients_[0] - cloud_->pts.at (indices.at (0)).x ) +

                                   ( model_coefficients_[1] - cloud_->pts.at (indices.at (0)).y ) *
                                   ( model_coefficients_[1] - cloud_->pts.at (indices.at (0)).y )
                                  );
    return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Recompute the plane coefficients using the given inlier set and return them to the user.
    * @note: these are the coefficients of the circle model after refinement (eg. after SVD)
    * \param inliers the data inliers found as supporting the model
    */
  std::vector<double>
    SACModelCircle2D::refitModel (std::vector<int> inliers)
  {
    if (inliers.size () == 0)
    {
      // std::cerr << "[SACModelCircle2D::RefitModel] Cannot re-fit 0 inliers!" << std::endl;
      return (model_coefficients_);
    }

    std::vector<double> refit (4);
//     for (int d = 0; d < 4; d++)
//       refit[d] = plane_coefficients (d);

    return (refit);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Verify whether a subset of indices verifies the internal circle model coefficients.
    * \param indices the data indices that need to be tested against the circle model
    * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
    */
  bool
    SACModelCircle2D::doSamplesVerifyModel (std::set<int> indices, double threshold)
  {
    // Iterate through the 3d points and calculate the distances from them to the circle
    for (std::set<int>::iterator it = indices.begin (); it != indices.end (); ++it)
    {
      // Calculate the distance from the point to the circle as the difference between
      //dist(point,circle_origin) and circle_radius
      double distance_to_circle = fabs (sqrt (
                                              ( cloud_->pts.at (*it).x - model_coefficients_.at (0) ) *
                                              ( cloud_->pts.at (*it).x - model_coefficients_.at (0) ) +

                                              ( cloud_->pts.at (*it).y - model_coefficients_.at (1) ) *
                                              ( cloud_->pts.at (*it).y - model_coefficients_.at (1) )
                                             ) - model_coefficients_.at (2));
      if (distance_to_circle > threshold)
        return (false);
    }
    return (true);
  }
}
