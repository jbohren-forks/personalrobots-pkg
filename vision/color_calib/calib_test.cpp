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

#include "ros/node.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

IplImage* g_img;
CvMat* g_real_corners;
CvMat* g_meas_corners;
CvMat* g_hom;
int g_ind;

//Mouse event handler
void on_mouse(int event, int x, int y, int flags, void *params)
{
  switch(event) {
  case CV_EVENT_LBUTTONDOWN:
   printf("Got click at %d %d\n", x, y);
   cvCircle(g_img, cvPoint(x,y), 5, cvScalar(0,0,255));

   // For first 4 clicks, set measured corners:
   if (g_ind < 4) {
     cvmSet(g_meas_corners, 0, g_ind, float(x));
     cvmSet(g_meas_corners, 1, g_ind, float(y));
     cvmSet(g_meas_corners, 2, g_ind, 1.0);
     g_ind++;
   }

   // On 4th click, compute and display homography
   if (g_ind == 4)
   {
     cvFindHomography( g_real_corners, g_meas_corners, g_hom );

     printf("Homography:\n");
     for (int i = 0; i < 3; i++)
     {
       for (int j = 0; j < 3; j++)
         printf("%f ", cvmGet(g_hom,i,j));
       printf("\n");
     }

     // Use homography to reproject real corners
     CvMat* reproj = cvCreateMat(3,4,CV_32FC1);
     cvMatMul(g_hom, g_real_corners, reproj);     

     for (int i = 0; i < 4; i++)
     {
       cvCircle(g_img, cvPoint(cvmGet(reproj,0,i),cvmGet(reproj,1,i)), 5, cvScalar(255,0,0));
       printf("Projected click from %f %f maps to %f %f\n", cvmGet(g_real_corners,0,i), cvmGet(g_real_corners,1,i), cvmGet(reproj,0,i), cvmGet(reproj,1,i));
     }

     cvReleaseMat(&reproj);

   }

   cvShowImage("image", g_img);
   break;
  }    
}


int main(int argc, char** argv)
{
  // Load image
  g_img = cvLoadImage(argv[1]);

  if (g_img)
  {

    //Allocate matrices
    g_real_corners = cvCreateMat( 3, 4, CV_32FC1);
    g_meas_corners = cvCreateMat( 3, 4, CV_32FC1);
    g_hom = cvCreateMat( 3, 3, CV_32FC1);
  

    //Set coordinates of real corners of a macbeth chart in perfect view
    cvmSet(g_real_corners, 0, 0, 0.0);
    cvmSet(g_real_corners, 1, 0, 0.0);
    cvmSet(g_real_corners, 2, 0, 1.0);

    cvmSet(g_real_corners, 0, 1, 6.0);
    cvmSet(g_real_corners, 1, 1, 0.0);
    cvmSet(g_real_corners, 2, 1, 1.0);

    cvmSet(g_real_corners, 0, 2, 6.0);
    cvmSet(g_real_corners, 1, 2, 4.0);
    cvmSet(g_real_corners, 2, 2, 1.0);

    cvmSet(g_real_corners, 0, 3, 0.0);
    cvmSet(g_real_corners, 1, 3, 4.0);
    cvmSet(g_real_corners, 2, 3, 1.0);

    // Create display
    cvNamedWindow("image", CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback("image", on_mouse, 0);
    cvShowImage("image", g_img);
    cvWaitKey(0);

    cvReleaseImage(&g_img);
    cvReleaseMat(&g_real_corners);
    cvReleaseMat(&g_meas_corners);

  } else {
    printf("Could not load image: %s\n", argv[1]);
  }
}
