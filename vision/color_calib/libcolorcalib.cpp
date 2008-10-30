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

#include "colorcalib.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

IplImage* g_img;

CvMat* g_real_corners;
CvMat* g_meas_corners;

CvMat* g_colors_pos;

CvMat* g_real_colors;
CvMat* g_meas_colors;

CvMat* g_hom;
CvMat* g_color_cal;

int g_ind;

float g_srgb_colors_dat[] = { 115, 82, 68,
                              194, 150, 130,
                              98, 122, 157,
                              87, 108, 67,
                              133, 128, 177,
                              103, 189, 170,
                              214, 126, 44,
                              80, 91, 166,
                              193, 90, 99,
                              94, 60, 108,
                              157, 188, 64,
                              224, 163, 64,
                              56, 61, 150,
                              70, 148, 73,
                              175, 54, 60,
                              231, 199, 31,
                              187, 86, 149,
                              8, 133, 161,
                              243, 243, 242,
                              200, 200, 200,
                              160, 160, 160,
                              122, 122, 121,
                              85, 85, 85,
                              52, 52, 52};



float srgb2lrgb(float x)
{
  float k = x / 255.0;
  if (k < 0.04045)
  {
    k = k/12.92;
  } else {
    k = pow( (k + 0.055)/(1.055), 2.4 );
  }
  return k*255.0;
}

float lrgb2srgb(float x)
{
  float k = x / 255.0;
  if (k < 0.0031308)
  {
    k = k*12.92;
  } else {
    k = (1.055)*pow(k, 1.0/2.4) - 0.055;
  }
  return k*255.0;
}

//Mouse event handler
void on_mouse(int event, int x, int y, int flags, void *params)
{
  switch(event) {
  case CV_EVENT_LBUTTONDOWN:
    cvCircle(g_img, cvPoint(x,y), 5, cvScalar(0,0,255));

   // For first 4 clicks, set measured corners:
   if (g_ind < 4) {
     cvSet2D(g_meas_corners, g_ind, 0, cvScalar(float(x), float(y)));
     g_ind++;
   }

   // On 4th click, compute and display homography
   if (g_ind == 4)
   {
     cvFindHomography( g_real_corners, g_meas_corners, g_hom );

     // Use homography to reproject real corners
     CvMat* reproj = cvCreateMat(g_colors_pos->rows, g_colors_pos->cols, CV_32FC2);

     cvPerspectiveTransform(g_colors_pos, reproj, g_hom);

     for (int i = 0; i < g_colors_pos->rows; i++)
     {
       CvPoint* p[1];
       CvPoint poly[4];
       p[0] = &(poly[0]);
       int n[] = {4};
       int minx=10000;
       int miny=10000;
       int maxx=0;
       int maxy=0;

       for (int j = 0; j < g_colors_pos->cols; j++)
       {
         CvScalar p = cvGet2D(reproj,i,j);
         int x = int(p.val[0]);
         int y = int(p.val[1]);

         minx = MIN(minx,x);
         miny = MIN(miny,y);
         maxx = MAX(maxx,x);
         maxy = MAX(maxy,y);

         poly[j].x = x;
         poly[j].y = y;
       }

       IplImage* mask = cvCreateImage(cvGetSize(g_img), IPL_DEPTH_8U, 1);
       cvFillConvexPoly(mask, poly, 4, cvScalar(1));

       CvScalar color = cvScalar(0,0,0);
       int cnt = 0;

       for( int u = minx; u < maxx; u++)
         for (int v = miny; v < maxy; v++)
           if (cvGetReal2D(mask,v,u) == 1)
           {
             CvScalar c = cvGet2D(g_img,v,u);
             color.val[0] += c.val[0];
             color.val[1] += c.val[1];
             color.val[2] += c.val[2];
             cnt++;
           }
       
       color.val[0] /= cnt;
       color.val[1] /= cnt;
       color.val[2] /= cnt;

       cvDrawPolyLine(g_img, p, n, 1, 1, cvScalar(255,0,0));

       cvmSet(g_meas_colors, i, 0, color.val[0]);
       cvmSet(g_meas_colors, i, 1, color.val[1]);
       cvmSet(g_meas_colors, i, 2, color.val[2]);
       
     }

     cvShowImage("macbeth image", g_img);

     cvReleaseMat(&reproj);

     cvSolve(g_meas_colors, g_real_colors, g_color_cal, CV_SVD);

     cvTranspose(g_color_cal, g_color_cal);
    
     g_ind++;
   }
   cvShowImage("macbeth image", g_img);
   break;
  }


}

bool find_calib(IplImage* img, CvMat* mat)
{
  g_img = cvCloneImage(img);

  if (g_img && mat)
  { 
    //Allocate matrices
    g_real_corners = cvCreateMat( 4, 1, CV_32FC2);
    cvSet2D(g_real_corners, 0, 0, cvScalar(0,0));
    cvSet2D(g_real_corners, 1, 0, cvScalar(6,0));
    cvSet2D(g_real_corners, 2, 0, cvScalar(6,4));
    cvSet2D(g_real_corners, 3, 0, cvScalar(0,4));

    g_meas_corners = cvCreateMat( 4, 1, CV_32FC2);
    g_hom = cvCreateMat( 3, 3, CV_32FC1);

    g_colors_pos = cvCreateMat( 24, 4, CV_32FC2);
    for (int i = 0; i < 24; i++) {
      cvSet2D(g_colors_pos, i, 0, cvScalar(float(i%6 + .25), float(i/6 + .25)));
      cvSet2D(g_colors_pos, i, 1, cvScalar(float(i%6 + .75), float(i/6 + .25)));
      cvSet2D(g_colors_pos, i, 2, cvScalar(float(i%6 + .75), float(i/6 + .75)));
      cvSet2D(g_colors_pos, i, 3, cvScalar(float(i%6 + .25), float(i/6 + .75)));
    }

    g_real_colors = cvCreateMatHeader( 24, 3, CV_32FC1);
    cvSetData( g_real_colors, g_srgb_colors_dat, 3*sizeof(float));

    // Swap our srgb colors to lbgr colors for opencv
    for (int i = 0; i < 24; i++)
    {
      float r = srgb2lrgb(cvmGet(g_real_colors, i, 0));
      float g = srgb2lrgb(cvmGet(g_real_colors, i, 1));
      float b = srgb2lrgb(cvmGet(g_real_colors, i, 2));
      cvmSet(g_real_colors, i, 0, b);
      cvmSet(g_real_colors, i, 1, g);
      cvmSet(g_real_colors, i, 2, r);
    }

    g_meas_colors = cvCreateMat( 24, 3, CV_32FC1);
  
    g_color_cal = mat;

    // Create display
    cvNamedWindow("macbeth image", CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback("macbeth image", on_mouse, 0);
    cvShowImage("macbeth image", g_img);

    
    while (g_ind < 5)
    {
      cvWaitKey(3);
      usleep(1000);
    }

    cvDestroyWindow("macbeth image");

    cvReleaseImage(&g_img);
    cvReleaseMat(&g_real_corners);
    cvReleaseMat(&g_meas_corners);
    cvReleaseMat(&g_real_colors);
    cvReleaseMat(&g_meas_colors);
    cvReleaseMat(&g_colors_pos);
    cvReleaseMat(&g_hom);

    return true;
  } else {
    return false;
  }

}
