/*********************************************************************
 *
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc
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

#include <stdio.h>
#include <iostream>
#include <stdint.h>

#include "ros/node.h"
#include "ros/console.h"
#include "color_calib.h"
#include "sensor_msgs/Image.h"
#include "opencv_latest/CvBridge.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <boost/thread/mutex.hpp>

#include "fh/misc.h"
#include "fh/filter.h"
#include "fh/segment-graph.h"


namespace image_segmentation
{

  bool DEBUG_DISPLAY = false;

  using namespace std;

  // MSFHNode - A node wrapper around the Felzenszwalb-Huttenlocher graph-based segmentation algorithm described here: http://people.cs.uchicago.edu/~pff/segment/
  // with the additional option of performing mean shift filtering as preprocessing.

  class MSFHSeg{
    public:
      // ROS
      ros::Node *node_;

      // Images and conversion
      sensor_msgs::Image limage_;
      sensor_msgs::CvBridge lbridge_;
      color_calib::Calibration *lcolor_cal_;

      bool calib_color_;
      bool do_display_;

      double K_;
      int min_clust_size_;
      double sigma_;

      bool do_mean_shift_;
      double spatial_radius_pix_;
      double color_radius_pix_;

      /////////////////////////////////////////////////////////////////////
      MSFHSeg(ros::Node *node) : 
        node_(node)
    { 

      cvNamedWindow("Image, Preprocessed Image, Segmentation");

      node_->param("/image_segmentation/do_display", do_display_, true);
      node_->param("/image_segmentation/do_calib_color", calib_color_, false);
      // FH params
      node_->param("/image_segmentation/sigma",sigma_, 0.5);
      node_->param("/image_segmentation/K", K_, 500.0);
      node_->param("/image_segmentation/min_cluster_size", min_clust_size_, 20);
      // MS params
      node_->param("/image_segmentation/do_mean_shift", do_mean_shift_, false);
      node_->param("/image_segmentation/spatial_radius_pix", spatial_radius_pix_, 2.0);
      node_->param("/image_segmentation/color_radius_pix", color_radius_pix_, 40.0);

      if (calib_color_) 
        lcolor_cal_ = new color_calib::Calibration(node_);

      node_->subscribe("stereo/left/image_rect_color",limage_,&MSFHSeg::imageCB,this,1);


    }

      /////////////////////////////////////////////////////////////////////
      ~MSFHSeg()
      {
      }




      /////////////////////////////////////////////////////////////////////
      void imageCB()
      {

        // Convert images to OpenCV and do apply color calibration if requested.
        IplImage *cv_image_left;
        if (lbridge_.fromImage(limage_,"bgr")) {
          cv_image_left = lbridge_.toIpl();
          // Color calibration
          if (calib_color_) {
            if ( lcolor_cal_->getFromParam("stereo/left/image_rect_color")) {
              lcolor_cal_->correctColor(cv_image_left, cv_image_left, true, true, COLOR_CAL_BGR);
            }
            else {
              ROS_DEBUG_STREAM_NAMED("cv_mean_shift","Color calibration not available.");
              return;
            }
          }
        }


        int width = cv_image_left->width;
        int height = cv_image_left->height;


        // Timing
        struct timeval timeofday;
        gettimeofday(&timeofday,NULL);
        ros::Time startt = ros::Time().fromNSec(1e9*timeofday.tv_sec + 1e3*timeofday.tv_usec);


        //////////////// Mean shift filtering /////////////////
        // If requested, do mean shift filtering. Otherwise, just smooth the image.
        IplImage *cv_image_smooth = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 3);
        if (do_mean_shift_) {
          cvPyrMeanShiftFiltering( cv_image_left, cv_image_smooth, spatial_radius_pix_, color_radius_pix_);
        }
        else {
          cvSmooth(cv_image_left, cv_image_smooth, CV_GAUSSIAN, 0, 0, sigma_, sigma_);
        }
        //////////////////////////////////////////////////////

        ////////////// FH graph-based segmentation ///////////
        // Build the graph
        edge *edges = new edge[width*height*4];
        int num = 0;
        int x, y;
        uint8_t *ptr, *ptrup, *ptrdown;
        for (y=0; y<height; ++y) {

          // This row, the row above, and the row below.
          ptr = (uint8_t*)(cv_image_smooth->imageData + y*cv_image_smooth->widthStep);
          ptrup = 0;
          ptrdown = 0;
          if (y > 0)
            ptrup = (uint8_t*)(cv_image_smooth->imageData + (y-1)*cv_image_smooth->widthStep);
          if (y < height-1)
            ptrdown = (uint8_t*)(cv_image_smooth->imageData + (y+1)*cv_image_smooth->widthStep);

          for (x=0; x<width; ++x) {

            int txy = y*width + x;

            if (x < width-1) {
              // Right edge
              edges[num].a = txy;
              edges[num].b = y*width + (x+1);
              edges[num].w = col_diff(ptr, ptr+3);
              num++;

              if (y < height-1) {
                // Down-right edge
                edges[num].a = txy;
                edges[num].b = (y+1)*width + (x+1);
                edges[num].w = col_diff(ptr, ptrdown+3);
                num++;
              }

              if (y > 0) {
                // Up-right edge
                edges[num].a = txy;
                edges[num].b = (y-1)*width + (x+1);
                edges[num].w = col_diff(ptr, ptrup+3);
                num++;
              }
            }

            if (y < height-1) {
              // Down edge
              edges[num].a = txy;
              edges[num].b = (y+1)*width + x;
              edges[num].w = col_diff(ptr, ptrdown);
              num++;
            }

            ptr += 3;
            ptrup += 3;
            ptrdown += 3;
          }
        }

        // Segment
        universe *u = segment_graph(width*height, num, edges, K_);

        // Merge clusters that are too small
        for (int i=0; i<num; ++i) {
          int a = u->find(edges[i].a);
          int b = u->find(edges[i].b);
          if ((a != b) && ((u->size(a) < min_clust_size_) || (u->size(b) < min_clust_size_)))
            u->join(a, b);
        }
        delete [] edges;	     
        //////////////////////////////////////////////////////


        // Timing
        gettimeofday(&timeofday,NULL);
        ros::Time endt = ros::Time().fromNSec(1e9*timeofday.tv_sec + 1e3*timeofday.tv_usec);
        ros::Duration diff = endt-startt;
        ROS_DEBUG_STREAM_NAMED("msfhseg_node","Segmentation duration " << diff.toSec() );



        // Make the display image and copy the original image into it.
        IplImage *display_image = cvCreateImage( cvSize(3*width, height), IPL_DEPTH_8U, 3);
        cvSetImageROI(display_image, cvRect(0,0,width,height));
        cvCopy(cv_image_left, display_image);
        cvResetImageROI(display_image);

        // Copy the preprocessed image into the display image
        cvSetImageROI(display_image, cvRect(width,0,width,height));
        cvCopy(cv_image_smooth, display_image);
        cvResetImageROI(display_image);


        // Randomly colorize the output clusters and draw them into the output image
        IplImage *colormap = cvCreateImage(cvSize(3, width*height), IPL_DEPTH_8U,1);
        for (y=0; y<width*height; ++y) {
          ptr = (uint8_t*)(colormap->imageData + y*colormap->widthStep);
          *ptr = (uint8_t)(rand()%(256));  ptr++; 
          *ptr = (uint8_t)(rand()%(256));  ptr++; 
          *ptr = (uint8_t)(rand()%(256));  
        }

        int c;
        uint8_t* cptr;
        for (y=0; y<height; ++y) {
          ptr = (uint8_t*)(display_image->imageData + y*display_image->widthStep + 2*cv_image_left->widthStep);
          for (x=0; x<width; ++x) {
            c = u->find(y*width + x);
            cptr = (uint8_t*)(colormap->imageData + c*colormap->widthStep);
            *ptr = *cptr; ptr++; cptr++;
            *ptr = *cptr; ptr++; cptr++;
            *ptr = *cptr; ptr++; cptr++;
          }
        }

        // Display
        cvShowImage("Image, Preprocessed Image, Segmentation", display_image);
        cvWaitKey(3);

        // Clean up
        cvReleaseImage(&cv_image_smooth);
        cvReleaseImage(&display_image);
        cvReleaseImage(&colormap);
        delete u;
      }

    private:

      float col_diff(uint8_t* p1, uint8_t* p2) {
        return sqrt( std::pow((float)(*p1) - (float)(*p2), 2) + std::pow((float)(*(p1+1)) - (float)(*(p2+1)), 2) + std::pow((float)(*(p1+2)) - (float)(*(p2+2)), 2) );
      }


  }; // class
}; // namespace


/////////////////////////////////////////////////////////////////////

  int
main (int argc, char** argv)
{
  ros::init (argc, argv);

  ros::Node fhn("cv_mean_shift");

  image_segmentation::MSFHSeg fh(&fhn);

  fhn.spin ();
  return (0);
}
