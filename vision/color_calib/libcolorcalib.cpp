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
IplImage* g_disp_img;

CvMat* g_real_corners;
CvMat* g_meas_corners;

CvMat* g_colors_pos;

CvMat* g_real_colors;
CvMat* g_meas_colors;
CvMat* g_reproj_colors;

CvMat* g_hom;
CvMat* g_color_cal;

int g_ind;

float g_srgb_colors_dat[] = { 115.0/255.0, 82.0/255.0, 68.0/255.0,
                              194.0/255.0, 150.0/255.0, 130.0/255.0,
                              98.0/255.0, 122.0/255.0, 157.0/255.0,
                              87.0/255.0, 108.0/255.0, 67.0/255.0,
                              133.0/255.0, 128.0/255.0, 177.0/255.0,
                              103.0/255.0, 189.0/255.0, 170.0/255.0,
                              214.0/255.0, 126.0/255.0, 44.0/255.0,
                              80.0/255.0, 91.0/255.0, 166.0/255.0,
                              193.0/255.0, 90.0/255.0, 99.0/255.0,
                              94.0/255.0, 60.0/255.0, 108.0/255.0,
                              157.0/255.0, 188.0/255.0, 64.0/255.0,
                              224.0/255.0, 163.0/255.0, 64.0/255.0,
                              56.0/255.0, 61.0/255.0, 150.0/255.0,
                              70.0/255.0, 148.0/255.0, 73.0/255.0,
                              175.0/255.0, 54.0/255.0, 60.0/255.0,
                              231.0/255.0, 199.0/255.0, 31.0/255.0,
                              187.0/255.0, 86.0/255.0, 149.0/255.0,
                              8.0/255.0, 133.0/255.0, 161.0/255.0,
                              243.0/255.0, 243.0/255.0, 242.0/255.0,
                              200.0/255.0, 200.0/255.0, 200.0/255.0,
                              160.0/255.0, 160.0/255.0, 160.0/255.0,
                              122.0/255.0, 122.0/255.0, 121.0/255.0,
                              85.0/255.0, 85.0/255.0, 85.0/255.0,
                              52.0/255.0, 52.0/255.0, 52.0/255.0};



float srgb2lrgb(float x)
{
  if (x < 0.04045)
  {
    x = x/12.92;
  } else {
    x = pow( (x + 0.055)/(1.055), 2.4 );
  }
  return x;
}

float lrgb2srgb(float x)
{
  if (x < 0.0031308)
  {
    x = x*12.92;
  } else {
    x = (1.055)*pow(x, 1.0/2.4) - 0.055;
  }
  return x;
}

void decompand(IplImage* src, IplImage* dst)
{

  // TODO: type checking that src is type 8U or else "companding" makes less sense

  int channels = dst->nChannels;
  int depth = dst->depth;

  static float compandmap[1024];
  static bool has_map = false;

  if (!has_map)
  {
    int compandinc = 1;
    int j = 0;
    int i = 0;
    while (i < 4096)
    {
      compandmap[j] = float(i)/4096.0;

      if (i > 2047)
        compandinc = 8;
      else if (i > 511)
        compandinc = 4;
      else if (i > 255)
        compandinc = 2;
      else 
        compandinc = 1;

      i += compandinc;
      j += 1;
    }
    has_map = true;
  }
  
  if (depth == IPL_DEPTH_32F)
  {
    for (int i = 0; i < src->height; i++)
      for (int j = 0; j < src->width; j++)
        for (int k = 0; k < channels; k++)
          ((float *)(dst->imageData + i*dst->widthStep))[j*dst->nChannels + k] = 
            compandmap[((uchar *)(src->imageData + i*src->widthStep))[j*src->nChannels + k] << 2];
  } else {
    for (int i = 0; i < src->height; i++)
      for (int j = 0; j < src->width; j++)
        for (int k = 0; k < channels; k++)
          ((uchar *)(dst->imageData + i*dst->widthStep))[j*dst->nChannels + k] = 
            compandmap[((uchar *)(src->imageData + i*src->widthStep))[j*src->nChannels + k] << 2]*255;
  }
}

//Mouse event handler
void on_mouse(int event, int x, int y, int flags, void *params)
{
  switch(event) {
  case CV_EVENT_LBUTTONDOWN:
    cvCircle(g_disp_img, cvPoint(x,y), 5, cvScalar(0,0,255));

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
       CvPoint poly[4];
       int minx=1000000;
       int miny=1000000;
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

       cvSetZero(mask);
       cvFillConvexPoly(mask, poly, 4, cvScalar(1));

       CvScalar color = cvAvg(g_img, mask);

       cvReleaseImage(&mask);

       cvmSet(g_meas_colors, i, 0, color.val[0]);
       cvmSet(g_meas_colors, i, 1, color.val[1]);
       cvmSet(g_meas_colors, i, 2, color.val[2]);
     }

     cvReleaseMat(&reproj);

     cvSolve(g_meas_colors, g_real_colors, g_color_cal, CV_SVD);

     cvMatMul(g_meas_colors, g_color_cal, g_reproj_colors);

     for (int k = 0; k < 24; k++)
     {
       printf("%d) Meas: %f %f %f\n   Reproj: %f %f %f\n  Real: %f %f %f\n",
              k,
              cvmGet(g_meas_colors,k,0),
              cvmGet(g_meas_colors,k,1),
              cvmGet(g_meas_colors,k,2),
              cvmGet(g_reproj_colors,k,0),
              cvmGet(g_reproj_colors,k,1),
              cvmGet(g_reproj_colors,k,2),
              cvmGet(g_real_colors,k,0),
              cvmGet(g_real_colors,k,1),
              cvmGet(g_real_colors,k,2));
     }

     cvTranspose(g_color_cal, g_color_cal);
    
     g_ind++;
   }
   cvShowImage("macbeth image", g_disp_img);
   break;
  }
}

bool find_calib(IplImage* img, CvMat* mat, int flags)
{

  bool use_bgr = flags & COLOR_CAL_BGR;
  bool use_float = (img->depth == IPL_DEPTH_32F);

  float mult;
  if (use_float)
    mult = 1;
  else
    mult = 255;

  g_img = cvCloneImage(img);
  g_disp_img = cvCloneImage(g_img);

  if (!use_bgr)
    cvCvtColor(g_disp_img, g_disp_img, CV_RGB2BGR);
 
  CvScalar a = cvAvg(g_img);
  printf("Avg value of g: %f %f %f\n", a.val[0], a.val[1], a.val[2]); 

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

    g_real_colors = cvCreateMat( 24, 3, CV_32FC1);

    // Set to lbgr colors for opencv
    for (int i = 0; i < 24; i++)
    {

      float r = srgb2lrgb(g_srgb_colors_dat[3*i + 0])*mult;
      float g = srgb2lrgb(g_srgb_colors_dat[3*i + 1])*mult;
      float b = srgb2lrgb(g_srgb_colors_dat[3*i + 2])*mult;

      if (use_bgr)
      {
        cvmSet(g_real_colors, i, 0, b);
        cvmSet(g_real_colors, i, 1, g);
        cvmSet(g_real_colors, i, 2, r);
      } else {
        cvmSet(g_real_colors, i, 0, r);
        cvmSet(g_real_colors, i, 1, g);
        cvmSet(g_real_colors, i, 2, b);
      }
    }

    g_meas_colors = cvCreateMat( 24, 3, CV_32FC1);
    g_reproj_colors = cvCreateMat( 24, 3, CV_32FC1);
  
    g_color_cal = mat;

    // Create display
    cvNamedWindow("macbeth image", CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback("macbeth image", on_mouse, 0);
    cvShowImage("macbeth image", g_disp_img);

    while (g_ind < 5)
    {
      cvWaitKey(3);
      usleep(1000);
    }

    cvDestroyWindow("macbeth image");

    cvReleaseImage(&g_img);
    cvReleaseImage(&g_disp_img);
    cvReleaseMat(&g_real_corners);
    cvReleaseMat(&g_meas_corners);
    cvReleaseMat(&g_real_colors);
    cvReleaseMat(&g_meas_colors);
    cvReleaseMat(&g_reproj_colors);
    cvReleaseMat(&g_colors_pos);
    cvReleaseMat(&g_hom);

    return true;
  } else {
    return false;
  }
}
