/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef POINT_CLOUD_MAPPING_H_
#define POINT_CLOUD_MAPPING_H_

// functions that should eventually go into the point_cloud_mapping package


#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <vector>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <point_cloud_mapping/geometry/point.h>

namespace cloud_geometry {
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief GB Downsample a Point Cloud using a voxelized grid approach
* \param xyzd            pointer to a 2D float CvMat of xyzd points where "d" is disparity and 0=> invalid.
* \param indices         integer offsets into xyzd indicating the points to use.  No invalid points allowed here
* \param points_down     integer offsets into xyzd indicating the resultant downsampled points
* \param leaf_size         the voxel leaf dimensions
* \param leaves         a vector of already existing leaves (empty for the first call)
* \param cut_distance     the maximum admissible distance of a point from the viewpoint (default: DBL_MAX)
*/
void
downsamplePointCloud (const CvMat *xyzd, const std::vector<int> &indices, std::vector<int> &points_down,
		geometry_msgs::Point leaf_size, std::vector<Leaf> &leaves, double cut_distance = DBL_MAX);


namespace statistics
{

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief GB Get the minimum and maximum values on each of the 3 (x-y-z) dimensions
  * in a given pointcloud, without considering points outside of a distance threshold from the laser origin
  * \param xyzd            pointer to a 2D float CvMat of xyzd points where "d" is disparity and 0=> invalid.
  * \param indices         integer offsets into xyzd indicating the points to use.  No invalid points allowed here
  * \param min_pt         the resultant minimum bounds
  * \param max_pt         the resultant maximum bounds
  * \param cut_distance a maximum admissible distance threshold (z distance) for points from the imager origin
  */
inline void
getMinMax (const CvMat *xyzd, const std::vector<int> &indices,
		geometry_msgs::Point32 &min_pt, geometry_msgs::Point32 &max_pt,
        double cut_distance);

}

}

#endif /* POINT_CLOUD_MAPPING_H_ */
