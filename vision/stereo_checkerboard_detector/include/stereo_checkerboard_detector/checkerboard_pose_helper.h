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

#ifndef STEREO_CHECKERBOARD_DETECTOR_CHECKERBOARD_POSE_HELPER_H_
#define STEREO_CHECKERBOARD_DETECTOR_CHECKERBOARD_POSE_HELPER_H_

#include <vector>

#include "opencv/cv.h"
#include "opencv_latest/CvBridge.h"

#include "robot_msgs/Pose.h"
#include "robot_msgs/Point.h"
#include "tf/transform_datatypes.h"

namespace stereo_checkerboard_detector
{

/**
 * Given a list of checkerboard corners, fit a checkerboard to these points.
 */
class CheckerboardPoseHelper
{

public :
  CheckerboardPoseHelper(int w=2, int h=2, int spacing=.1)
  {
    cb_expected_ = NULL ;
    setSize(w,h,spacing) ;
  }

  ~CheckerboardPoseHelper()
  {
    if (cb_expected_)
      cvReleaseMat(&cb_expected_) ;
  }

  /**
   * Sets the size of the checkerboard we're looking for
   * \param w Num checkerboard squares Wide
   * \param h Num checkerboard squares high
   * \param spacing between checkerboard corners
   */
  void setSize(int w, int h, double spacing) ;


  /**
   * Compare the sensed cloud and expected cloud to determine the pose of the checkerboard
   * \param cb_sensed Nx3 CV_32FC1 matrix storing the sensed xyz locations of the checkerboard corners. These points
   *        must be ordered in the same ordering that cvFindChessboardCorners returns points.
   * \param pose_tf \b Output Checkerboard pose stored as a bullet/tf datatype
   */
  void getPose(const CvMat* cb_sensed, tf::Pose& pose_tf) ;

  /**
   * Compare the sensed cloud and expected cloud to determine the pose of the checkerboard
   * \param cb_sensed Nx3 CV_32FC1 matrix storing the sensed xyz locations of the checkerboard corners. These points
   *        must be ordered in the same ordering that cvFindChessboardCorners returns points.
   * \param R \b Output. Preallocated 3x3 CV_32FC1 matrix. Stores the calculated rotation matrix to the checkerboard
   * \param trans \b Output. Preallocated 3x1 CV_32FC1 matrix. Stores the calculated translation to the checkerboard
   */
  void getPose(const CvMat* cb_sensed, CvMat* R, CvMat* trans) ;


private :
  CvSize board_size_ ;                   //!< Size of the checkboard
  CvMat* cb_expected_ ;
} ;


}



#endif // STEREO_CHECKERBOARD_DETECTOR_CHECKERBOARD_POSE_HELPER_H_
