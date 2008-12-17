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

#include <stdlib.h>
#include "sample_consensus/sac_model_cylinder.h"
#include "cloud_geometry/point.h"
#include "cloud_geometry/distances.h"

namespace sample_consensus
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Get 4 random points (3 non-collinear) as data samples and return them as point indices.
    * \param iterations the internal number of iterations used by SAC methods
    * \note assumes unique points!
    */
  std::vector<int>
    SACModelCylinder::getSamples (int &iterations)
  {
    std::vector<int> random_idx (4);

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
        std::cerr << "[SACModelCylinder::getSamples] WARNING: Could not select 3 non collinear points in " << MAX_ITERATIONS_COLLINEAR << " iterations!!!" << std::endl;
        break;
      }
      iterations++;
    }
    // Use Zoli's method for collinearity check
    while (((Dx1 / Dx2) == Dy1Dy2) && (Dy1Dy2 == (Dz1 / Dz2)));
    iterations--;

    // Need to improve this: we need 4 points, 3 non-collinear always, and the 4th should not be in the same plane as the other 3
    // otherwise we can encounter degenerate cases
    do
    {
      random_idx[3] = (int)(indices_.size () * (rand () / (RAND_MAX + 1.0)));
      iterations++;
    } while ( (random_idx[3] == random_idx[2]) || (random_idx[3] == random_idx[1]) || (random_idx[3] == random_idx[0]) );
    iterations--;

    return (random_idx);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Select all the points which respect the given model coefficients as inliers.
    * \param model_coefficients the coefficients of a cylinder model that we need to compute distances to
    * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
    * \note: To get the refined inliers of a model, use:
    * ANNpoint refined_coeff = refitModel (...); selectWithinDistance (refined_coeff, threshold);
    */
  std::vector<int>
    SACModelCylinder::selectWithinDistance (std::vector<double> model_coefficients, double threshold)
  {
    std::vector<int> inliers;

    // Model coefficients: [point_on_axis axis_direction radius]
    // Iterate through the 3d points and calculate the distances from them to the cylinder
    for (unsigned int i = 0; i < indices_.size (); i++)
    {
      // Aproximate the distance from the point to the cylinder as the difference between
      //dist(point,cylinder_axis) and cylinder radius
      // NOTE: need to revise this.
      if (fabs (
                cloud_geometry::distances::pointToLineDistance (cloud_->pts.at (indices_[i]), model_coefficients) - model_coefficients[6]
               ) < threshold)
        // Returns the indices of the points whose distances are smaller than the threshold
        inliers.push_back (indices_[i]);
    }
    return (inliers);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Compute all distances from the cloud data to a given cylinder model.
    * \param model_coefficients the coefficients of a cylinder model that we need to compute distances to
    */
  std::vector<double>
    SACModelCylinder::getDistancesToModel (std::vector<double> model_coefficients)
  {
    std::vector<double> distances (indices_.size ());

    // Iterate through the 3d points and calculate the distances from them to the cylinder
    for (unsigned int i = 0; i < indices_.size (); i++)
      // Aproximate the distance from the point to the cylinder as the difference between
      //dist(point,cylinder_axis) and cylinder radius
      // NOTE: need to revise this.
      distances[i] = fabs (cloud_geometry::distances::pointToLineDistance (cloud_->pts.at (indices_[i]), model_coefficients) - model_coefficients[6]);
    return (distances);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Create a new point cloud with inliers projected onto the cylinder model.
    * \param inliers the data inliers that we want to project on the cylinder model
    * \param model_coefficients the coefficients of a cylinder model
    * \todo implement this.
    */
  std_msgs::PointCloud
    SACModelCylinder::projectPoints (std::vector<int> inliers, std::vector<double> model_coefficients)
  {
    std::cerr << "[SACModelCylinder::projecPoints] Not implemented yet." << std::endl;
    return (*cloud_);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Project inliers (in place) onto the given cylinder model.
    * \param inliers the data inliers that we want to project on the cylinder model
    * \param model_coefficients the coefficients of a cylinder model
    * \todo implement this.
    */
  void
    SACModelCylinder::projectPointsInPlace (std::vector<int> inliers, std::vector<double> model_coefficients)
  {
    std::cerr << "[SACModelCylinder::projecPointsInPlace] Not implemented yet." << std::endl;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Check whether the given index samples can form a valid cylinder model, compute the model coefficients from
    * these samples and store them internally in model_coefficients_. The cylinder coefficients are: x, y, z, R.
    * \param indices the point indices found as possible good candidates for creating a valid model
    */
  bool
    SACModelCylinder::computeModelCoefficients (std::vector<int> indices)
  {
    model_coefficients_.resize (7);

    // Save the nx/ny/nz channel indices the first time we run this
    if (nx_idx_ == -1)
    {
      nx_idx_ = cloud_geometry::getChannelIndex (*cloud_, "nx");
      if (nx_idx_ == -1) return (false);
    }
    if (ny_idx_ == -1)
    {
      ny_idx_ = cloud_geometry::getChannelIndex (*cloud_, "ny");
      if (ny_idx_ == -1) return (false);
    }
    if (nz_idx_ == -1)
    {
      nz_idx_ = cloud_geometry::getChannelIndex (*cloud_, "nz");
      if (nz_idx_ == -1) return (false);
    }

    std_msgs::Point32 u, v, w;

    u.x = cloud_->chan[nx_idx_].vals.at (indices.at (0));
    u.y = cloud_->chan[ny_idx_].vals.at (indices.at (0));
    u.z = cloud_->chan[nz_idx_].vals.at (indices.at (0));

    v.x = cloud_->chan[nx_idx_].vals.at (indices.at (1));
    v.y = cloud_->chan[ny_idx_].vals.at (indices.at (1));
    v.z = cloud_->chan[nz_idx_].vals.at (indices.at (1));

    w.x = (u.x + cloud_->pts.at (indices.at (0)).x) - cloud_->pts.at (indices.at (1)).x;
    w.y = (u.y + cloud_->pts.at (indices.at (0)).y) - cloud_->pts.at (indices.at (1)).y;
    w.z = (u.z + cloud_->pts.at (indices.at (0)).z) - cloud_->pts.at (indices.at (1)).z;

    double a = cloud_geometry::dot (u, u);
    double b = cloud_geometry::dot (u, v);
    double c = cloud_geometry::dot (v, v);
    double d = cloud_geometry::dot (u, w);
    double e = cloud_geometry::dot (v, w);
    double denominator = a*c - b*b;
    double sc, tc;
    // Compute the line parameters of the two closest points
    if (denominator < 1e-8)          // The lines are almost parallel
    {
      sc = 0.0;
      tc = (b > c ? d / b : e / c);  // Use the largest denominator
    }
    else
    {
      sc = (b*e - c*d) / denominator;
      tc = (a*e - b*d) / denominator;
    }

    // point_on_axis, axis_direction
    model_coefficients_[0] = cloud_->pts.at (indices.at (0)).x + cloud_->chan[nx_idx_].vals.at (indices.at (0)) + (sc * u.x);
    model_coefficients_[1] = cloud_->pts.at (indices.at (0)).y + cloud_->chan[ny_idx_].vals.at (indices.at (0)) + (sc * u.y);
    model_coefficients_[2] = cloud_->pts.at (indices.at (0)).z + cloud_->chan[nz_idx_].vals.at (indices.at (0)) + (sc * u.z);
    model_coefficients_[3] = cloud_->pts.at (indices.at (1)).x + (tc * v.x) - model_coefficients_[0];
    model_coefficients_[4] = cloud_->pts.at (indices.at (1)).y + (tc * v.y) - model_coefficients_[1];
    model_coefficients_[5] = cloud_->pts.at (indices.at (1)).z + (tc * v.z) - model_coefficients_[2];

    double norm = sqrt (
                        (model_coefficients_[3] * model_coefficients_[3]) +
                        (model_coefficients_[4] * model_coefficients_[4]) +
                        (model_coefficients_[5] * model_coefficients_[5])
                      );
    model_coefficients_[3] /= norm;
    model_coefficients_[4] /= norm;
    model_coefficients_[5] /= norm;

    // cylinder radius
    model_coefficients_[6] = cloud_geometry::distances::pointToLineDistance (cloud_->pts.at (indices.at (0)), model_coefficients_);

    return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Recompute the cylinder coefficients using the given inlier set and return them to the user.
    * @note: these are the coefficients of the cylinder model after refinement (eg. after SVD)
    * \param inliers the data inliers found as supporting the model
    */
  std::vector<double>
    SACModelCylinder::refitModel (std::vector<int> inliers)
  {
    if (inliers.size () == 0)
    {
      // std::cerr << "[SACModelCylinder::RefitModel] Cannot re-fit 0 inliers!" << std::endl;
      return (model_coefficients_);
    }

/*
    */
    std::vector<double> refit (7);
    for (int d = 0; d < 4; d++)
      refit[d] = model_coefficients_[d];

    return (refit);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Verify whether a subset of indices verifies the internal cylinder model coefficients.
    * \param indices the data indices that need to be tested against the cylinder model
    * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
    */
  bool
    SACModelCylinder::doSamplesVerifyModel (std::set<int> indices, double threshold)
  {
    for (std::set<int>::iterator it = indices.begin (); it != indices.end (); ++it)
      // Aproximate the distance from the point to the cylinder as the difference between
      //dist(point,cylinder_axis) and cylinder radius
      // NOTE: need to revise this.
      if (fabs (
                cloud_geometry::distances::pointToLineDistance (cloud_->pts.at (*it), model_coefficients_) - model_coefficients_[6]
               ) > threshold)
        return (false);

    return (true);
  }
}
