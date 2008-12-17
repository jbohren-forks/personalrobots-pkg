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

#include "sample_consensus/sac_model_sphere.h"
#include "cloud_geometry/nearest.h"
#include <Eigen/LU>

namespace sample_consensus
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Get 4 random points (3 non-collinear) as data samples and return them as point indices.
    * \param iterations the internal number of iterations used by SAC methods
    * \note assumes unique points!
    * \note Two different points could be enough in theory, to infere some sort of a center and a radius,
    *       but in practice, we might end up with a lot of points which are just 'close' to one another.
    *       Therefore we have two options:
    *       a) use normal information (good but I wouldn't rely on it in extremely noisy point clouds, no matter what)
    *       b) get two more points and uniquely identify a sphere in space (3 unique points define a circle)
    */
  std::vector<int>
    SACModelSphere::getSamples (int &iterations)
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
        std::cerr << "[SACModelSphere::getSamples] WARNING: Could not select 3 non collinear points in " << MAX_ITERATIONS_COLLINEAR << " iterations!!!" << std::endl;
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
    * \param model_coefficients the coefficients of a sphere model that we need to compute distances to
    * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
    * \note: To get the refined inliers of a model, use:
    * ANNpoint refined_coeff = refitModel (...); selectWithinDistance (refined_coeff, threshold);
    */
  std::vector<int>
    SACModelSphere::selectWithinDistance (std::vector<double> model_coefficients, double threshold)
  {
    std::vector<int> inliers;

    // Iterate through the 3d points and calculate the distances from them to the sphere 
    for (unsigned int i = 0; i < indices_.size (); i++)
    {
      // Calculate the distance from the point to the sphere as the difference between
      //dist(point,sphere_origin) and sphere_radius
      if (fabs (sqrt (
                      ( cloud_->pts.at (indices_[i]).x - model_coefficients.at (0) ) *
                      ( cloud_->pts.at (indices_[i]).x - model_coefficients.at (0) ) +

                      ( cloud_->pts.at (indices_[i]).y - model_coefficients.at (1) ) *
                      ( cloud_->pts.at (indices_[i]).y - model_coefficients.at (1) ) +

                      ( cloud_->pts.at (indices_[i]).z - model_coefficients.at (2) ) *
                      ( cloud_->pts.at (indices_[i]).z - model_coefficients.at (2) )
                     ) - model_coefficients.at (3)) < threshold)
        // Returns the indices of the points whose distances are smaller than the threshold
        inliers.push_back (indices_[i]);
    }
    return (inliers);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Compute all distances from the cloud data to a given sphere model.
    * \param model_coefficients the coefficients of a sphere model that we need to compute distances to
    */
  std::vector<double>
    SACModelSphere::getDistancesToModel (std::vector<double> model_coefficients)
  {
    std::vector<double> distances (indices_.size ());

    // Iterate through the 3d points and calculate the distances from them to the sphere
    for (unsigned int i = 0; i < indices_.size (); i++)
      // Calculate the distance from the point to the sphere as the difference between
      //dist(point,sphere_origin) and sphere_radius
      distances[i] = fabs (sqrt (
                                 ( cloud_->pts.at (indices_[i]).x - model_coefficients.at (0) ) *
                                 ( cloud_->pts.at (indices_[i]).x - model_coefficients.at (0) ) +

                                 ( cloud_->pts.at (indices_[i]).y - model_coefficients.at (1) ) *
                                 ( cloud_->pts.at (indices_[i]).y - model_coefficients.at (1) ) +

                                 ( cloud_->pts.at (indices_[i]).z - model_coefficients.at (2) ) *
                                 ( cloud_->pts.at (indices_[i]).z - model_coefficients.at (2) )
                                ) - model_coefficients.at (3));
    return (distances);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Create a new point cloud with inliers projected onto the sphere model.
    * \param inliers the data inliers that we want to project on the sphere model
    * \param model_coefficients the coefficients of a sphere model
    * \todo implement this.
    */
  std_msgs::PointCloud
    SACModelSphere::projectPoints (std::vector<int> inliers, std::vector<double> model_coefficients)
  {
    std::cerr << "[SACModelSphere::projecPoints] Not implemented yet." << std::endl;
    return (*cloud_);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Project inliers (in place) onto the given sphere model.
    * \param inliers the data inliers that we want to project on the sphere model
    * \param model_coefficients the coefficients of a sphere model
    * \todo implement this.
    */
  void
    SACModelSphere::projectPointsInPlace (std::vector<int> inliers, std::vector<double> model_coefficients)
  {
    std::cerr << "[SACModelSphere::projecPointsInPlace] Not implemented yet." << std::endl;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Check whether the given index samples can form a valid sphere model, compute the model coefficients from
    * these samples and store them internally in model_coefficients_. The sphere coefficients are: x, y, z, R.
    * \param indices the point indices found as possible good candidates for creating a valid model
    */
  bool
    SACModelSphere::computeModelCoefficients (std::vector<int> indices)
  {
    model_coefficients_.resize (4);

    Eigen::Matrix4d temp;
    for (int i = 0; i < 4; i++)
    {
      temp (i, 0) = cloud_->pts.at (indices.at (i)).x;
      temp (i, 1) = cloud_->pts.at (indices.at (i)).y;
      temp (i, 2) = cloud_->pts.at (indices.at (i)).z;
      temp (i, 3) = 1;
    }
    double m11 = temp.determinant ();
    if (m11 == 0)
      return (false);             // the points don't define a sphere!

    for (int i = 0; i < 4; i++)
      temp (i, 0) = (cloud_->pts.at (indices.at (i)).x) * (cloud_->pts.at (indices.at (i)).x) +
                    (cloud_->pts.at (indices.at (i)).y) * (cloud_->pts.at (indices.at (i)).y) +
                    (cloud_->pts.at (indices.at (i)).z) * (cloud_->pts.at (indices.at (i)).z);
    double m12 = temp.determinant ();

    for (int i = 0; i < 4; i++)
    {
      temp (i, 1) = temp (i, 0);
      temp (i, 0) = cloud_->pts.at (indices.at (i)).x;
    }
    double m13 = temp.determinant ();

    for (int i = 0; i < 4; i++)
    {
      temp (i, 2) = temp (i, 1);
      temp (i, 1) = cloud_->pts.at (indices.at (i)).y;
    }
    double m14 = temp.determinant ();

    for (int i = 0; i < 4; i++)
    {
      temp (i, 0) = temp (i, 2);
      temp (i, 1) = cloud_->pts.at (indices.at (i)).x;
      temp (i, 2) = cloud_->pts.at (indices.at (i)).y;
      temp (i, 3) = cloud_->pts.at (indices.at (i)).z;
    }
    double m15 = temp.determinant ();

    // Center (x , y, z)
    model_coefficients_[0] = 0.5 * m12 / m11;
    model_coefficients_[1] = 0.5 * m13 / m11;
    model_coefficients_[2] = 0.5 * m14 / m11;
    // Radius
    model_coefficients_[3] = sqrt (
                                   model_coefficients_[0] * model_coefficients_[0] +
                                   model_coefficients_[1] * model_coefficients_[1] +
                                   model_coefficients_[2] * model_coefficients_[2] - m15 / m11);

    return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Recompute the sphere coefficients using the given inlier set and return them to the user.
    * @note: these are the coefficients of the sphere model after refinement (eg. after SVD)
    * \param inliers the data inliers found as supporting the model
    */
  std::vector<double>
    SACModelSphere::refitModel (std::vector<int> inliers)
  {
    if (inliers.size () == 0)
    {
      // std::cerr << "[SACModelSphere::RefitModel] Cannot re-fit 0 inliers!" << std::endl;
      return (model_coefficients_);
    }

/*    LMStrucData data;
    
    data.points  = points;
    data.samples = samples;
    
    double *x (new double[samples.size ()]);
    for (unsigned int i = 0; i < samples.size (); i++) 
      x[i] = 0;
    
    // I: minim. options [\tau, \epsilon1, \epsilon2, \epsilon3]. Respectively the scale factor for initial \mu,
    // stopping thresholds for ||J^T e||_inf, ||Dp||_2 and ||e||_2. Set to NULL for defaults to be used    
    double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
    opts[0] = LM_INIT_MU;
    opts[1] = 1E-15;
    opts[2] = 1E-15;
    opts[3] = 1E-20;
    opts[4] = LM_DIFF_DELTA;

    int m = 4;               /// I: parameter vector dimension (i.e. #unknowns)
    int n = samples.size (); /// I: measurement vector dimension
    int itmax = 5000;        // I: maximum number of iterations

    printf ("Using the following initial estimates: %g, %g, %g, %g\n", bestCoefficients[0], bestCoefficients[1], bestCoefficients[2], bestCoefficients[3]);
    // I/O: initial parameter estimates. On output contains the estimated solution
    double p[m];
    // Position of center point
    p[0] = bestCoefficients[0];
    p[1] = bestCoefficients[1];
    p[2] = bestCoefficients[2];
    // Radius
    p[3] = bestCoefficients[3];

    // double *work,  // I: pointer to working memory, allocated internally if NULL. 
    // If !=NULL, it is assumed to point to a memory chunk at least LM_DER_WORKSZ(m, n)*sizeof(double) bytes long
    // double *covar, // O: Covariance matrix corresponding to LS solution; Assumed to point to a mxm matrix. Set to NULL if not needed.
    // void *adata)   // I: pointer to possibly needed additional data, passed uninterpreted to func & jacf. Set to NULL if not needed
    int ret = dlevmar_der (sphere_func, sphere_jac, p, x, m, n, itmax, opts, info, NULL, NULL, (void *) &data);
//    int ret = dlevmar_dif (spherefit_func, p, X, m, n, itmax, opts, info, NULL, NULL, data);

    printf ("Levenberg-Marquardt returned %d in %g iter, reason %g\nSolution: ", ret, info[5], info[6]);
    for (int i = 0; i < 4; ++i)
      printf ("%.7g ", p[i]);
    
    printf ("\n\nMinimization info:\n");
    for (int i = 0; i < LM_INFO_SZ; ++i)
      printf ("%g ", info[i]);
    printf ("\n");

    ANNpoint newcoeff = annAllocPt (4);
    newcoeff[0] = p[0]; newcoeff[1] = p[1];
    newcoeff[2] = p[2]; newcoeff[3] = p[3];
//    newcoeff[0] = bestCoefficients[0]; newcoeff[1] = bestCoefficients[1];
//    newcoeff[2] = bestCoefficients[2]; newcoeff[3] = bestCoefficients[3];
    return newcoeff;
    */
    std::vector<double> refit (4);
//     for (int d = 0; d < 4; d++)
//       refit[d] = plane_coefficients (d);

    return (refit);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Verify whether a subset of indices verifies the internal sphere model coefficients.
    * \param indices the data indices that need to be tested against the sphere model
    * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
    */
  bool
    SACModelSphere::doSamplesVerifyModel (std::set<int> indices, double threshold)
  {
    for (std::set<int>::iterator it = indices.begin (); it != indices.end (); ++it)
      // Calculate the distance from the point to the sphere as the difference between
      //dist(point,sphere_origin) and sphere_radius
      if (fabs (sqrt (
                      ( cloud_->pts.at (*it).x - model_coefficients_.at (0) ) *
                      ( cloud_->pts.at (*it).x - model_coefficients_.at (0) ) +

                      ( cloud_->pts.at (*it).y - model_coefficients_.at (1) ) *
                      ( cloud_->pts.at (*it).y - model_coefficients_.at (1) ) +

                      ( cloud_->pts.at (*it).z - model_coefficients_.at (2) ) *
                      ( cloud_->pts.at (*it).z - model_coefficients_.at (2) )
                     ) - model_coefficients_.at (3)) > threshold)
        return (false);

    return (true);
  }
}
