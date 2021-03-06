#ifndef __D3D_CURVATURE_H__
#define __D3D_CURVATURE_H__
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <vector>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.hpp>

#include <descriptors_3d/generic/neighborhood_feature.h>
#include <descriptors_3d/shared/spectral_analysis.h>

// --------------------------------------------------------------
/*!
 * \file curvature.h
 *
 * \brief A Curvature descriptor computes the local curvature
 */
// --------------------------------------------------------------

// --------------------------------------------------------------
/*!
 * \brief A Curvature descriptor computes the local curvature around
 *        an intersest point/region.
 *
 * The feature value is \lambda_1 / (\lambda_1 + \lambda_2 + \lambda_3)
 * where \lambda_i are the 3 eigenvalues such that
 * \lambda_1 < \lambda_2 < \lambda_3
 */
// --------------------------------------------------------------
class Curvature: public Descriptor3D
{
  public:
    // --------------------------------------------------------------
    /*!
     * \brief Instantiates the curvature descriptor to use the given spectral
     *        information
     *
     * \param spectral_information Class to retrieve the eigenvalues from
     */
    // --------------------------------------------------------------
    Curvature(SpectralAnalysis& spectral_information);

    // --------------------------------------------------------------
    /*!
     * \brief Clears any already-computed spectral information
     */
    // --------------------------------------------------------------
    virtual void clearShared();

  protected:
    // --------------------------------------------------------------
    /*!
     * \brief Computes/retrieves the eigenvalues for each interest point
     *
     * \param data The point cloud to process from Descriptor3D::compute()
     * \param data_kdtree The efficient neighborhood data structure
     * \param interest_pts  The list of interest points to be processed
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    virtual int precompute(const sensor_msgs::PointCloud& data,
                           cloud_kdtree::KdTree& data_kdtree,
                           const cv::Vector<const geometry_msgs::Point32*>& interest_pts);

    // --------------------------------------------------------------
    /*!
     * \brief Computes/retrieves the eigenvalues for each interest region
     *
     * \param data The point cloud to process from Descriptor3D::compute()
     * \param data_kdtree The efficient neighborhood data structure
     * \param interest_pts  The list of interest regions to be processed
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    virtual int precompute(const sensor_msgs::PointCloud& data,
                           cloud_kdtree::KdTree& data_kdtree,
                           const cv::Vector<const std::vector<int>*>& interest_region_indices);

    // --------------------------------------------------------------
    /*!
     * \brief Computes the local curvature around each interest point
     *
     * \see Descriptor3D::compute
     */
    // --------------------------------------------------------------
    virtual void doComputation(const sensor_msgs::PointCloud& data,
                               cloud_kdtree::KdTree& data_kdtree,
                               const cv::Vector<const geometry_msgs::Point32*>& interest_pts,
                               cv::Vector<cv::Vector<float> >& results);

    // --------------------------------------------------------------
    /*!
     * \brief Computes the local curvature around/for each interest region
     *
     * \see Descriptor3D::compute
     */
    // --------------------------------------------------------------
    virtual void doComputation(const sensor_msgs::PointCloud& data,
                               cloud_kdtree::KdTree& data_kdtree,
                               const cv::Vector<const std::vector<int>*>& interest_region_indices,
                               cv::Vector<cv::Vector<float> >& results);

    // --------------------------------------------------------------
    /*!
     * \brief Computes local curvature feature for the specified interest point/region
     */
    // --------------------------------------------------------------
    virtual void
    computeCurvature(const unsigned int interest_sample_idx, cv::Vector<float>& result) const;

  private:
    /*! \brief The eigenvalues for each interest point/region to be processed */
    const std::vector<const Eigen::Vector3d*>* eig_vals_;

    SpectralAnalysis* spectral_information_;
};

#endif
