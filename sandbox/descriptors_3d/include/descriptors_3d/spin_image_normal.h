#ifndef __D3D_SPIN_IMAGE_NORMAL_H__
#define __D3D_SPIN_IMAGE_NORMAL_H__
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

#include <descriptors_3d/generic/spin_image_generic.h>
#include <descriptors_3d/spectral_analysis.h>

// --------------------------------------------------------------
//* Orientation
/*!
 * \brief An Orientation descriptor looks at the angle of an interest
 *        point/region's neighborhood principle directions with respect
 *        to a given reference direction.
 *
 * The principle directions are the tangent (biggest eigenvector)
 * and normal (smallest eigenvector) vectors from the extracted spectral
 * information.
 */
// --------------------------------------------------------------
class SpinImageNormal: public SpinImageGeneric
{
  public:
    // --------------------------------------------------------------
    /*!
     * \brief Instantiates the Orientation descriptor
     *
     * The computed feature is the cosine of the angle between the extracted
     * tangent/normal vector against the specified reference directions.
     *
     * \warning Since the sign of the extracted vectors have no meaning, the
     *          computed feature is always between 0 and 1
     *
     * If computing using both tangent and normal vectors, the feature vector
     * format is: [a b] where a is the projection of the tangent vector and b
     * is the projection of the normal vector
     */
    // --------------------------------------------------------------
    SpinImageNormal(const double row_res,
                    const double col_res,
                    const unsigned int nbr_rows,
                    const unsigned int nbr_cols,
                    const bool use_interest_regions_only,
                    SpectralAnalysis& spectral_information);

  protected:
    virtual int precompute(const robot_msgs::PointCloud& data,
                           cloud_kdtree::KdTree& data_kdtree,
                           const cv::Vector<const robot_msgs::Point32*>& interest_pts);

    virtual int precompute(const robot_msgs::PointCloud& data,
                           cloud_kdtree::KdTree& data_kdtree,
                           const cv::Vector<const std::vector<int>*>& interest_region_indices);
  private:
    SpectralAnalysis* spectral_information_;
};

#endif
