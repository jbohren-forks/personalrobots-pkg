#ifndef __D3D_BOUNDING_BOX_RAW_H__
#define __D3D_BOUNDING_BOX_RAW_H__
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

#include <descriptors_3d/generic/neighborhood_feature.h>

// --------------------------------------------------------------
//* BoundingBox
/*!
 * \brief A BoundingBox descriptor computes the dimensions of the
 *        3-D box that encloses a group of points.
 *
 * When computing the feature for an interest point, the bounding box
 * is defined by the neighboring points within some specified radius.
 *
 * When computing the feature for an interest region of points, the
 * bounding box can either be the box that encloses the given region of
 * points, or from the neighboring points within some specified radius
 * from the region's centroid.
 *
 * The bounding box can be computed in the given coordinate frame
 * and/or in the projected principle component space.
 */
// --------------------------------------------------------------
class BoundingBoxRaw: public NeighborhoodFeature
{
  public:
    // --------------------------------------------------------------
    /*!
     * \brief Instantiates the BoundingBox descriptor with specified parameters
     *
     * If computing the bounding box in principle component space, then features are
     * in order: [a,b,c] where a is the length along the principle eigenvector, b
     * is the length along the middle eigenvector, and c is the length along the
     * smallest eigenvector.
     *
     * If computing the bounding box in the given coordinate frame, then the
     * features are in order: [x,y,z] where x is the length along the first dimension,
     * y is the length along the second dimension, z is the length along the third
     * dimension.
     *
     * If computing both bounding boxes, the values are in order: [a b c x y z]
     *
     * \param use_pca_bbox Flag to compute the bounding box in the principle
     *                     component space
     * \param use_raw_bbox Flag to compute the bounding box in the given
     *                     coordinate xyz space
     */
    // --------------------------------------------------------------
    BoundingBoxRaw(double bbox_radius);

  protected:
    virtual int precompute(const sensor_msgs::PointCloud& data,
                           cloud_kdtree::KdTree& data_kdtree,
                           const cv::Vector<const geometry_msgs::Point32*>& interest_pts);

    virtual int precompute(const sensor_msgs::PointCloud& data,
                           cloud_kdtree::KdTree& data_kdtree,
                           const cv::Vector<const std::vector<int>*>& interest_region_indices);
    // --------------------------------------------------------------
    /*!
     * \brief Computes the bounding box information of the given neighborhood
     *
     * \param data The overall point cloud data
     * \param neighbor_indices List of indices in data that constitute the neighborhood
     * \param result The vector to hold the computed bounding box dimensions
     */
    // --------------------------------------------------------------
    virtual void computeNeighborhoodFeature(const sensor_msgs::PointCloud& data,
                                            const std::vector<int>& neighbor_indices,
                                            const unsigned int interest_sample_idx,
                                            cv::Vector<float>& result) const;
};

#endif
