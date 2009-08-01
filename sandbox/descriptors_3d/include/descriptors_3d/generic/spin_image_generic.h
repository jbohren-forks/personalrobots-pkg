#ifndef __D3D_SPIN_IMAGE_GENERIC_H__
#define __D3D_SPIN_IMAGE_GENERIC_H__
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

#include <descriptors_3d/generic/neighborhood_feature.h>

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
class SpinImageGeneric: public NeighborhoodFeature
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
    SpinImageGeneric();

    virtual ~SpinImageGeneric() = 0;

  protected:
    // --------------------------------------------------------------
    /*!
     * \brief Computes the bounding box information of the given neighborhood
     *
     * \param data The overall point cloud data
     * \param neighbor_indices List of indices in data that constitute the neighborhood
     * \param result The vector to hold the computed bounding box dimensions
     */
    // --------------------------------------------------------------
    virtual void computeNeighborhoodFeature(const robot_msgs::PointCloud& data,
                                            const std::vector<int>& neighbor_indices,
                                            const unsigned int interest_sample_idx,
                                            cv::Vector<float>& result) const;

    const std::vector<const Eigen::Vector3d*>* spin_axes_;
    std::vector<Eigen::Vector3d> spin_image_centers_;

    /*! \brief The cell resolution along the beta axis */
    double row_res_;
    /*! \brief The cell resolution along the alpha axis */
    double col_res_;
    /*! \brief The number of cells along the beta axis */
    unsigned int nbr_rows_;
    /*! \brief The number of cells along the alpha axis */
    unsigned int nbr_cols_;
};

#endif
