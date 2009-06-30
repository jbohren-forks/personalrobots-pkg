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


#include "stereo_checkerboard_detector/reprojection_helper.h"

using namespace stereo_checkerboard_detector ;
using namespace std ;

static const double eps = 1e-5 ;
static const bool DEBUG = false ;

struct point_t
{
  float x ;
  float y ;
  float d ;
};

TEST(CHECKERBOARD_DETECTOR, disparity_easy)
{
  ReprojectionHelper helper ;

  vector<CvPoint2D32f> left_pts(2) ;
  vector<CvPoint2D32f> right_pts(2) ;

  left_pts[0].x = 1 ;
  left_pts[0].y = 2 ;
  left_pts[1].x = 3 ;
  left_pts[1].y = 4 ;

  right_pts[0].x = 10 ;
  right_pts[0].y = 20 ;
  right_pts[1].x = 30 ;
  right_pts[1].y = 40 ;

  CvMat* xyd = cvCreateMat(left_pts.size(), 1, CV_64FC3) ;
  helper.computeDisparity(left_pts, right_pts, xyd) ;

  EXPECT_NEAR( *( (float*)CV_MAT_ELEM_PTR(*xyd, 0, 0) + 0 ), 1, eps) ;
  EXPECT_NEAR( *( (float*)CV_MAT_ELEM_PTR(*xyd, 0, 0) + 1 ), 2, eps) ;
  EXPECT_NEAR( *( (float*)CV_MAT_ELEM_PTR(*xyd, 0, 0) + 2 ),-9, eps) ;

  cvReleaseMat(&xyd) ;
}

TEST(CHECKERBOARD_DETECTOR, reprojection_easy)
{
  ReprojectionHelper helper ;

  sensor_msgs::CamInfo left_info ;
  sensor_msgs::CamInfo right_info ;
  double L[12] = {500,   0, 320, 0,
                    0, 500, 240, 0,
                    0,   0,   1, 0 } ;
  double R[12] = {500,   0, 320, -50,
                    0, 500, 240, 0,
                    0,   0,   1, 0 } ;
  left_info.P.resize(12) ;
  right_info.P.resize(12) ;

  for (unsigned int i=0; i<12; i++)
  {
    left_info.P[i]  = L[i] ;
    right_info.P[i] = R[i] ;
  }

  // Define some easy uvd testpoints
  vector<robot_msgs::Point> uvd(2) ;
  uvd[0].x = 330 ;      // u
  uvd[0].y = 250 ;      // v
  uvd[0].z =  2 ;      // Disparity

  uvd[1].x = 320 ;      // u
  uvd[1].y = 240 ;      // v
  uvd[1].z =  4 ;      // Disparity

  vector<robot_msgs::Point> xyz(2) ;
  helper.reproject(uvd, left_info, right_info, xyz) ;

  EXPECT_GT(xyz[0].x, 0.0) ;
  EXPECT_GT(xyz[0].y, 0.0) ;
  EXPECT_NEAR(xyz[0].z, 50/2.0, eps) ;

  EXPECT_NEAR(xyz[1].x, 0.0, eps) ;
  EXPECT_NEAR(xyz[1].y, 0.0, eps) ;
  EXPECT_NEAR(xyz[1].z, 50/4.0, eps) ;
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
