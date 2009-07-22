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

#include <descriptors_3d/descriptor_3d.h>

using namespace std;

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
class SpectralAnalysis: public Descriptor3D
{
  public:
    // --------------------------------------------------------------
    /*!
     * \brief Instantiates the class indicating no spectral information
     *        or parameters have been defined
     */
    // --------------------------------------------------------------
    SpectralAnalysis() :
      spectral_info_(NULL), support_radius_(-1.0), support_radius_defined_(false)
    {
    }

    virtual ~SpectralAnalysis() = 0;

    // --------------------------------------------------------------
    /*!
     * \brief Clears out all computed spectral information.
     */
    // --------------------------------------------------------------
    void clear();

    // ===================================================================
    /*! \name Required settings (one or the other) */
    // ===================================================================
    //@{
    // --------------------------------------------------------------
    /*!
     * \brief
     *
     * \warning This method cannot be called if setSpectralRadius has already
     *          been called.
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    int useSpectralInformation(SpectralAnalysis* spectral_info);

    // --------------------------------------------------------------
    /*!
     * \brief Defines the neighborhood radius when doing spectral analysis
     *
     * The radius must be positive when computing features for interest
     * points.  However, the value can be negative when giving interest
     * REGIONs to indicate to use only the points within the interest region
     * for spectral analysis
     *
     * \warning This method cannot be called if useSpectralInformation has already
     *          been called.
     *
     * \param support_radius The radius value
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    int setSpectralRadius(double support_radius);
    //@}

    // ===================================================================
    /*! \name Accessors */
    // ===================================================================
    //@{
    // --------------------------------------------------------------
    /*!
     * \brief Returns the saved normals estimated for each interest point/region
     *        that was passed to compute()
     */
    // --------------------------------------------------------------
    inline const vector<Eigen::Vector3d*>& getNormals()
    {
      return normals_;
    }

    // --------------------------------------------------------------
    /*!
     * \brief Returns the saved tangents estimated for each interest point/region
     *        that was passed to compute()
     */
    // --------------------------------------------------------------
    inline const vector<Eigen::Vector3d*>& getTangents()
    {
      return tangents_;
    }

    // --------------------------------------------------------------
    /*!
     * \brief Returns the saved middle component direction
     */
    // --------------------------------------------------------------
    inline const vector<Eigen::Vector3d*>& getMiddleEigenVectors()
    {
      return middle_eig_vecs_;
    }

    // --------------------------------------------------------------
    /*!
     * \brief Returns the saved eigen values of the covariance matrix for each
     *        interest point/region that was passed to compute()
     */
    // --------------------------------------------------------------
    inline const vector<Eigen::Vector3d*>& getEigenValues()
    {
      return eigen_values_;
    }

    // --------------------------------------------------------------
    /*!
     * \brief Returns the saved centroids for each interest point/region
     *        that was passed to compute()
     */
    // --------------------------------------------------------------
    inline const vector<Eigen::Vector3d*>& getCentroids()
    {
      return centroids_;
    }
    //@}

  protected:
    int analyzeInterestPoints(const robot_msgs::PointCloud& data,
                              cloud_kdtree::KdTree& data_kdtree,
                              const cv::Vector<robot_msgs::Point32*>& interest_pts);

    int analyzeInterestRegions(const robot_msgs::PointCloud& data,
                               cloud_kdtree::KdTree& data_kdtree,
                               const cv::Vector<vector<int>*>& interest_region_indices);

    SpectralAnalysis* spectral_info_;

    vector<Eigen::Vector3d*> normals_;
    vector<Eigen::Vector3d*> tangents_;
    vector<Eigen::Vector3d*> middle_eig_vecs_;
    vector<Eigen::Vector3d*> eigen_values_;
    vector<Eigen::Vector3d*> centroids_;

  private:
    double support_radius_;
    bool support_radius_defined_;

    void populateContainers(const robot_msgs::PointCloud& data, vector<int>& curr_region_indices, size_t idx);
};

#endif
