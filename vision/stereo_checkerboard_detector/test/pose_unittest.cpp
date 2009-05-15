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

#include <gtest/gtest.h>


#include "stereo_checkerboard_detector/checkerboard_pose_helper.h"

using namespace stereo_checkerboard_detector ;
using namespace std ;

static const double eps = 1e-5 ;
static const bool DEBUG = false ;

struct pt3d
{
  pt3d(float x, float y, float z) : x(x), y(y), z(z) { }
  float x ;
  float y ;
  float z ;
};

void populatePointsEasy(CvMat* pts)
{
  *( (pt3d*)CV_MAT_ELEM_PTR(*pts, 0, 0) ) = pt3d(1.0, 0.0, 2.0) ;
  *( (pt3d*)CV_MAT_ELEM_PTR(*pts, 1, 0) ) = pt3d(1.0,-0.1, 2.0) ;
  *( (pt3d*)CV_MAT_ELEM_PTR(*pts, 2, 0) ) = pt3d(1.0,-0.2, 2.0) ;
  *( (pt3d*)CV_MAT_ELEM_PTR(*pts, 3, 0) ) = pt3d(1.0, 0.0, 1.9) ;
  *( (pt3d*)CV_MAT_ELEM_PTR(*pts, 4, 0) ) = pt3d(1.0,-0.1, 1.9) ;
  *( (pt3d*)CV_MAT_ELEM_PTR(*pts, 5, 0) ) = pt3d(1.0,-0.2, 1.9) ;
}

TEST(CHECKERBOARD_DETECTOR, pose_cv_easy)
{
  CheckerboardPoseHelper helper ;

  helper.setSize(3,2, .1) ;
  CvMat* sensed_pts = cvCreateMat(6, 3, CV_32FC1) ;
  populatePointsEasy(sensed_pts) ;

  float R[9] ;
  float R_expected[9] = {  0.0,  0.0,  1.0,
                           -1.0,  0.0,  0.0,
                            0.0, -1.0,  0.0 } ;

  float trans[3] ;
  float trans_expected[3] = {1.0, 0.0, 2.0} ;

  CvMat cv_R = cvMat(3, 3, CV_32FC1, R) ;
  CvMat cv_trans = cvMat(3, 1, CV_32FC1, trans) ;

  helper.getPose(sensed_pts, &cv_R, &cv_trans) ;

  //  printf("R =\n") ;
  //  for (int i=0; i<3; i++)
  //  {
  //    printf("  % .3f, % .3f, % .3f\n", *( (float*) CV_MAT_ELEM_PTR( cv_R, i, 0)),
  //                                      *( (float*) CV_MAT_ELEM_PTR( cv_R, i, 1)),
  //                                      *( (float*) CV_MAT_ELEM_PTR( cv_R, i, 2)) ) ;
  //  }
  //
  //  printf("trans=  % .3f, % .3f, % .3f\n", *( (float*) CV_MAT_ELEM_PTR( cv_trans, 0, 0)),
  //                                          *( (float*) CV_MAT_ELEM_PTR( cv_trans, 1, 0)),
  //                                          *( (float*) CV_MAT_ELEM_PTR( cv_trans, 2, 0)) ) ;


  for (int i=0; i<9; i++)
    EXPECT_NEAR(R[i], R_expected[i], eps) ;

  for (int i=0; i<3; i++)
    EXPECT_NEAR(trans[i], trans_expected[i], eps) ;

  cvReleaseMat(&sensed_pts) ;
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(CHECKERBOARD_DETECTOR, pose_tf_easy)
{
  CheckerboardPoseHelper helper ;

  helper.setSize(3,2, .1) ;
  CvMat* sensed_pts = cvCreateMat(6, 3, CV_32FC1) ;
  populatePointsEasy(sensed_pts) ;

  tf::Pose pose ;
  helper.getPose(sensed_pts, pose) ;

  // Make sure the top left and bottom right point end up in the correct place with the applied transform
  tf::Point top_left = pose*tf::Point(0.0, 0.0, 0.0) ;
  tf::Point bot_right = pose*tf::Point(0.2, 0.1, 0.0) ;

  EXPECT_NEAR(top_left.x(), 1.0, eps) ;
  EXPECT_NEAR(top_left.y(), 0.0, eps) ;
  EXPECT_NEAR(top_left.z(), 2.0, eps) ;

  EXPECT_NEAR(bot_right.x(), 1.0, eps) ;
  EXPECT_NEAR(bot_right.y(),-0.2, eps) ;
  EXPECT_NEAR(bot_right.z(), 1.9, eps) ;
}
