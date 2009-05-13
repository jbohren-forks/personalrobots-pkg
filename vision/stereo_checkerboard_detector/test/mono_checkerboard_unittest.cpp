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


#include "stereo_checkerboard_detector/mono_checkerboard_helper.h"
#include "opencv/highgui.h"

using namespace stereo_checkerboard_detector ;
using namespace std ;

static const double pix_eps = .5 ;
static const bool DEBUG = false ;

TEST(CHECKERBOARD_DETECTOR, easy_cb_3x4)
{
  MonoCheckerboardHelper mono_helper(3,4) ;
  IplImage* img ;
  img = cvLoadImage("test/data/cb_3x4.png",0) ;         // 0 -> Force image to grayscale
  EXPECT_TRUE(img != NULL) ;

  vector<CvPoint2D32f> corners ;
  bool found = mono_helper.getCorners(img, corners) ;

  EXPECT_TRUE(found) ;
  EXPECT_EQ(corners.size(), (unsigned int) 12) ;

  // Top Right Corner
  EXPECT_NEAR(corners[0].x, 223.5, pix_eps) ;
  EXPECT_NEAR(corners[0].y,  71.5, pix_eps) ;

  // Bottom Left Corner
  EXPECT_NEAR(corners[11].x, 81.5,  pix_eps) ;
  EXPECT_NEAR(corners[11].y, 166.5, pix_eps) ;

  if (DEBUG)
  {
    printf("All Corners:\n") ;
    for(unsigned int i=0; i<corners.size(); i++)
    {
      printf("  %2u) % .2f % .2f\n", i, corners[i].x, corners[i].y) ;
    }
  }

  cvReleaseImage(&img) ;
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
