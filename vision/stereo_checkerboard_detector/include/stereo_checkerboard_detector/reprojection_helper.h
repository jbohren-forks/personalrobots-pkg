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

//! \author Vijay Pradeep

#ifndef STEREO_CHECKERBOARD_DETECTOR_MONO_REPROJECTION_HELPER_H_
#define STEREO_CHECKERBOARD_DETECTOR_MONO_REPROJECTION_HELPER_H_

#include <vector>
#include "opencv/cv.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/CameraInfo.h"

namespace stereo_checkerboard_detector
{

/**
 * Performs the necessary openCV calls to reproject a sparse set of stereo points (in pixel
 * coordinates) into 3D coordinates
 */
class ReprojectionHelper
{
public:

  ReprojectionHelper() { } ;
  ~ReprojectionHelper() { } ;

  /**
   * Computes the disparty for the lists of left and right Points, using ROS message types
   * \param left_pts List of points in the left cam
   * \param right_pts List of points in the right cam
   * \param result Output: Each point stores a xyd tuple, with xy being the left
   *                       cam point, and d being the disparity
   */
  void computeDisparity(const std::vector<geometry_msgs::Point>& left_pts,
                        const std::vector<geometry_msgs::Point>& right_pts,
                        std::vector<geometry_msgs::Point>& result) ;

  /**
   * Computes the disparty for the lists of left and right Points, using
   * openCV datatypes.
   * \param left_pts List of points in the left cam
   * \param right_pts List of points in the right cam
   * \param Output array. Should be of form cvCreateMat(N, 1, CV_64FC3)
   */
  void computeDisparity(const std::vector<CvPoint2D32f>& left_pts,
                        const std::vector<CvPoint2D32f>& right_pts,
                        CvMat* uvd) ;

  /**
   * Reprojects the list of disparity points, using standard ROS datatypes
   * \param ros_uvd Disparity point of the form (left_u, left_v, disparity), using ROS messages
   * \param left_info  CameraInfo message holding intrinsics for the left camera, using ROS messages
   * \param right_info CameraInfo message holding intrinsics for the right camera, using ROS messages
   * \param ros_xyz Output vector with reprojected points from uvd, using ROS messages
   */
  void reproject(const std::vector<geometry_msgs::Point>& uvd,
                 const sensor_msgs::CameraInfo& left_info,
                 const sensor_msgs::CameraInfo& right_info,
                 std::vector<geometry_msgs::Point>& xyz) ;


  void reproject(const CvMat* uvd,
                 const sensor_msgs::CameraInfo& left_info,
                 const sensor_msgs::CameraInfo& right_info,
                 CvMat* xyz) ;


  /**
   * Build the reprojection matrix Q, based on the left and right cameras' projection matricies
   * \param left_info CameraInfo message holding intrinsics for the left camera, using ROS messages
   * \param right_info CameraInfo message holding intrinsics for the right camera, using ROS messages
   */
  void buildQ(const sensor_msgs::CameraInfo& left_info,
              const sensor_msgs::CameraInfo& right_info,
              CvMat* Q) ;


private:

  // Disparity Point
  struct Disp_Point_t
  {
    float u ;
    float v ;
    float d ;
  };

  // Cartesian Point
  struct Cart_Point_t
  {
    float x ;
    float y ;
    float z ;
  };

};

}

#endif // STEREO_CHECKERBOARD_DETECTOR_MONO_REPROJECTION_HELPER_H_

