#ifndef __D3D_SPIN_IMAGE_H__
#define __D3D_SPIN_IMAGE_H__
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
#include <Eigen/Geometry>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.hpp>

#include <point_cloud_mapping/geometry/nearest.h>

#include <descriptors_3d/spectral_analysis.h>

using namespace std;

// --------------------------------------------------------------
//* SpinImage
/**
 * \brief
 */
// --------------------------------------------------------------
class SpinImage: public SpectralAnalysis
{
  public:
    typedef enum spin_axis
    {
      UNDEFINED = 0, NORMAL = 0, TANGENT = 1, CUSTOM = 2
    } spin_axis_type_t;

    SpinImage() :
      spin_axis_type_(SpinImage::UNDEFINED), use_interest_region_only_(false), dimensions_defined_(false),
          row_res_(-1.0), col_res_(-1.0), nbr_rows_(0), nbr_cols_(0)
    {
      result_size_ = 0;
    }

    inline void setAxisNormal()
    {
      spin_axis_type_ = SpinImage::NORMAL;
    }

    inline void setAxisTangent()
    {
      spin_axis_type_ = SpinImage::TANGENT;
    }

    void setAxisCustom(double ref_x, double ref_y, double ref_z);

    // ^
    // |_ _ _ _
    // |_|_|_|_|
    // x_|_|_|_|
    // |_|_|_|_|
    // 3 rows, 4 cols
    int setImageDimensions(double row_res, double col_res, unsigned int nbr_rows, unsigned int nbr_cols);

    virtual void compute(const robot_msgs::PointCloud& data,
                         cloud_kdtree::KdTree& data_kdtree,
                         const cv::Vector<robot_msgs::Point32*>& interest_pts,
                         cv::Vector<cv::Vector<float> >& results);

    // ===================================================================
    /*! \name Interest region related  */
    // ===================================================================
    //@{

    virtual void compute(const robot_msgs::PointCloud& data,
                         cloud_kdtree::KdTree& data_kdtree,
                         const cv::Vector<vector<int>*>& interest_region_indices,
                         cv::Vector<cv::Vector<float> >& results);

    inline void useInterestRegionOnly()
    {
      use_interest_region_only_ = true;
    }

    //@}

  private:
    // --------------------------------------------------------------
    /*!
     * \brief Computes a spin image of the neighboring points around center_pt
     *
     */
    // --------------------------------------------------------------
    void computeSpinImage(const robot_msgs::PointCloud& data,
                          const vector<int>& neighbor_indices,
                          const Eigen::Vector3d& center_pt,
                          cv::Vector<float>& spin_image);
    Eigen::Vector3d spin_axis_;
    spin_axis_type_t spin_axis_type_;

    bool use_interest_region_only_;

    bool dimensions_defined_;
    double row_res_;
    double col_res_;
    unsigned int nbr_rows_;
    unsigned int nbr_cols_;
};

#endif
