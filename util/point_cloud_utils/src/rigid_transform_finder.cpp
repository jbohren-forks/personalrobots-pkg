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

#include "point_cloud_utils/rigid_transform_finder.h"

// visual_odometry includes
#include "PoseEstimate.h"               

// TF Includes
#include "LinearMath/btMatrix3x3.h"
#include "tf/transform_datatypes.h"

using namespace point_cloud_utils ;

int RigidTransformFinder::findTransform(const std_msgs::PointCloud& A, const std_msgs::PointCloud& B, std_msgs::Transform& transform)
{
  if (A.get_pts_size() != B.get_pts_size())
    return -1 ;

  const int N = A.get_pts_size() ;

  CvMat* mat_A = cvCreateMat(N,3,CV_64FC1) ;
  CvMat* mat_B = cvCreateMat(N,3,CV_64FC1) ;

  CvMat* R = cvCreateMat(3,3,CV_64FC1) ;              // Allocate rotation matrix
  CvMat* T = cvCreateMat(3,1,CV_64FC1) ;              // Allocate translation vector

  // Copy point clouds into openCV datatypes
  for (int i=0; i < N; i++)
  {
    cvmSet(mat_A, i, 0, A.pts[i].x) ;
    cvmSet(mat_A, i, 1, A.pts[i].y) ;
    cvmSet(mat_A, i, 2, A.pts[i].z) ;

    cvmSet(mat_B, i, 0, B.pts[i].x) ;
    cvmSet(mat_B, i, 1, B.pts[i].y) ;
    cvmSet(mat_B, i, 2, B.pts[i].z) ;
  }

  cv::willow::PoseEstimate poseEstimate ;

  int inliers ;
  inliers = poseEstimate.estimate(mat_A, mat_B, R, T, true) ;

  // Convert openCV Rotation matrix to bullet rotation matrix
  btMatrix3x3 bt_R ( cvmGet(R,0,0), cvmGet(R,0,1), cvmGet(R,0,2),
                     cvmGet(R,1,0), cvmGet(R,1,1), cvmGet(R,1,2),
                     cvmGet(R,2,0), cvmGet(R,2,1), cvmGet(R,2,2) ) ;

  // Convert bullet rotation matrix to a TF quaternion
  tf::Quaternion tf_quaternion ;
  bt_R.getRotation(tf_quaternion) ;
  std_msgs::Quaternion msg_quaternion ;

  // Convert TF quaternion to a message
  tf::QuaternionTFToMsg(tf_quaternion, msg_quaternion) ;

  transform.rotation = msg_quaternion ;

  transform.translation.x = cvmGet(T,0,0) ;
  transform.translation.y = cvmGet(T,1,0) ;
  transform.translation.z = cvmGet(T,2,0) ;

  cvReleaseMat(&T) ;
  cvReleaseMat(&R) ;
  cvReleaseMat(&mat_B) ;
  cvReleaseMat(&mat_A) ;

  return inliers ;
}
