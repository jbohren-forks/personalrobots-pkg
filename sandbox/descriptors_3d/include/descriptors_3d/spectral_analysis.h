#ifndef __D3D_SPECTRAL_ANALYSIS_H__
#define __D3D_SPECTRAL_ANALYSIS_H__
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

#include <Eigen/Core>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.hpp>

#include <ros/console.h>

#include <robot_msgs/PointCloud.h>

#include <point_cloud_mapping/kdtree/kdtree.h>
#include <point_cloud_mapping/geometry/nearest.h>

// --------------------------------------------------------------
//* SpectralAnalysis
/**
 * \brief An abstract class for descriptors that require the results
 *        from spectral analysis of a given volume of points
 *
 * This class is meant to hold intermediate results needed for the
 * computation of other descriptors so that computation is not
 * unnecessarily repeated.
 */
// --------------------------------------------------------------
class SpectralAnalysis
{
  public:
    // --------------------------------------------------------------
    /*!
     * \brief Instantiates the class indicating no spectral information
     *        or parameters have been defined
     */
    // --------------------------------------------------------------
    SpectralAnalysis(double support_radius);

    ~SpectralAnalysis();

    // --------------------------------------------------------------
    /*!
     * \brief Clears & frees previously computed spectral data
     *
     * This function should be called when calling compute() on different
     * sequential point clouds.
     *
     * This method has no affect if called on an instance that did not
     * compute the spectral data.
     */
    // --------------------------------------------------------------
    void clearSpectral();

    // ===================================================================
    /*! \name Accessors */
    // ===================================================================
    //@{
    // TODO comment
    inline const bool isSpectralComputed() const
    {
      return spectral_computed_;
    }

    // --------------------------------------------------------------
    /*!
     * \brief Returns the saved normals estimated for each interest point/region
     *        that was passed to compute()
     */
    // --------------------------------------------------------------
    inline const std::vector<const Eigen::Vector3d*>& getNormals() const
    {
      return normals_;
    }

    // --------------------------------------------------------------
    /*!
     * \brief Returns the saved tangents estimated for each interest point/region
     *        that was passed to compute()
     */
    // --------------------------------------------------------------
    inline const std::vector<const Eigen::Vector3d*>& getTangents() const
    {
      return tangents_;
    }

    // --------------------------------------------------------------
    /*!
     * \brief Returns the saved middle component direction
     */
    // --------------------------------------------------------------
    inline const std::vector<const Eigen::Vector3d*>& getMiddleEigenVectors() const
    {
      return middle_eig_vecs_;
    }

    // --------------------------------------------------------------
    /*!
     * \brief Returns the saved eigen values of the covariance matrix for each
     *        interest point/region that was passed to compute()
     */
    // --------------------------------------------------------------
    inline const std::vector<const Eigen::Vector3d*>& getEigenValues() const
    {
      return eigen_values_;
    }
    //@}

    int analyzeInterestPoints(const robot_msgs::PointCloud& data,
                              cloud_kdtree::KdTree& data_kdtree,
                              const cv::Vector<const robot_msgs::Point32*>& interest_pts);

    int analyzeInterestRegions(const robot_msgs::PointCloud& data,
                               cloud_kdtree::KdTree& data_kdtree,
                               const cv::Vector<const std::vector<int>*>& interest_region_indices);

  private:
    void computeSpectralInfo(const robot_msgs::PointCloud& data,
                             const std::vector<int>& curr_region_indices,
                             const size_t idx);
    double support_radius_;
    bool spectral_computed_;

    std::vector<const Eigen::Vector3d*> normals_;
    std::vector<const Eigen::Vector3d*> tangents_;
    std::vector<const Eigen::Vector3d*> middle_eig_vecs_;
    std::vector<const Eigen::Vector3d*> eigen_values_;
};

#endif
