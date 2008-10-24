/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 */

#ifndef POINT_CLOUD_UTIL_FIND_TRANSFORM_H_
#define POINT_CLOUD_UTIL_FIND_TRANSFORM_H_

// Messages
#include "std_msgs/PointCloud.h"
#include "std_msgs/Transform.h"

namespace point_cloud_utils
{

/**
 * \brief Finds rigid transform between 2 point clouds (whose correspondences are known)
 * Wrapper around cv::willow_garage::visual_odometry::PoseEstimate::estimate to work more seamlessly in the ros framework
 */
class RigidTransformFinder
{
public:
  RigidTransformFinder() { }
  ~RigidTransformFinder() { }
  
  /**
   * \brief Executes the rigid transform search
   * 
   * Note that we must know the correspondence between both clouds for this optimization to work.
   * That is, A.pts[i] <=> B.pts[i]
   * This also implies that A.pts and B.pts must be of the same size. Also, the headers of both clouds are completely ignored
   * \param A (input) The first set of points
   * \param B (input) The second set of points
   * \param transform (output) Calculated transform. Stores a Rotation R and translation T, such that R*A + T = B
   * \return The number of points used to actually compute the transform (ie. # of inliers). \
   * Returns negative when there's an error
   */
  static int FindTransform(const std_msgs::PointCloud& A, const std_msgs::PointCloud& B, std_msgs::Transform& transform) ;
  
private:

} ;

}

#endif // POINT_CLOUD_UTIL_FIND_TRANSFORM_H_
