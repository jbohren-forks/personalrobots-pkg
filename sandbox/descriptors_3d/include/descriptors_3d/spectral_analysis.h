#ifndef __D3D_SPECTRAL_ANALYSIS_H__
#define __D3D_SPECTRAL_ANALYSIS_H__
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Daniel Munoz
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

#include <descriptors_3d/descriptors_3d.h>

using namespace std;

// --------------------------------------------------------------
//* SpectralAnalysis
/**
 * \brief An abstract class for descriptors that require computing
 *        spectral information from spectral/pca analysis
 */
// --------------------------------------------------------------
class SpectralAnalysis: public Descriptor3D
{
  public:
    // --------------------------------------------------------------
    /*!
     * \brief
     */
    // --------------------------------------------------------------
    SpectralAnalysis() :
      spectral_info_(NULL), support_radius_(-1.0)
    {
    }

    // --------------------------------------------------------------
    /*!
     * \brief
     */
    // --------------------------------------------------------------
    virtual ~SpectralAnalysis() = 0;

    // --------------------------------------------------------------
    /*!
     * \brief
     */
    // --------------------------------------------------------------
    inline void useSpectralInformation(SpectralAnalysis* spectral_info)
    {
      spectral_info_ = spectral_info;
    }

    virtual void compute(const robot_msgs::PointCloud& data,
                         cloud_kdtree::KdTree& data_kdtree,
                         const cv::Vector<robot_msgs::Point32*>& interest_pts,
                         cv::Vector<cv::Vector<float> >& results);

    virtual void compute(const robot_msgs::PointCloud& data,
                         cloud_kdtree::KdTree& data_kdtree,
                         const cv::Vector<vector<int>*>& interest_region_indices,
                         cv::Vector<cv::Vector<float> >& results);

    // --------------------------------------------------------------
    /*!
     * \brief
     */
    // --------------------------------------------------------------
    inline void setSupportRadius(double support_radius)
    {
      support_radius_ = support_radius;
    }

    inline const vector<Eigen::Vector3d*>& getNormals()
    {
      return normals_;
    }

    inline const vector<Eigen::Vector3d*>& getTangents()
    {
      return tangents_;
    }

    inline const vector<Eigen::Vector3d*>& getEigenValues()
    {
      return eigen_values_;
    }

    inline const vector<Eigen::Vector3d*>& getCentroids()
    {
      return centroids_;
    }

  protected:
    virtual void computeFeatures(cv::Vector<cv::Vector<float> >& results) = 0;

    SpectralAnalysis* spectral_info_;

    double support_radius_;

    vector<Eigen::Vector3d*> normals_;
    vector<Eigen::Vector3d*> tangents_;
    vector<Eigen::Vector3d*> eigen_values_;
    vector<Eigen::Vector3d*> centroids_;

  private:
    // --------------------------------------------------------------
    /*!
     * \brief
     */
    // --------------------------------------------------------------
    void clear();

    void populateContainers(const robot_msgs::PointCloud& data, vector<int>& curr_region_indices, size_t idx);
};

#endif
