/*********************************************************************
* A GUI for clicking on points which are then published as track initialization points.
*
**********************************************************************
*
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Caroline Pantofaru
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
#include <vector>

#include "ros/node.h"
#include "image_msgs/StereoInfo.h"
#include "image_msgs/CamInfo.h"
#include "image_msgs/Image.h"
#include "image_msgs/CvBridge.h"
#include "CvStereoCamModel.h"
#include <robot_msgs/PositionMeasurement.h>
#include "color_calib.h"
#include "topic_synchronizer.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

using namespace std;

struct PublishedPoint {
  CvPoint xy;
  bool published;
};

vector<PublishedPoint> gxys;
ros::thread::mutex g_selection_mutex;
ros::Time g_last_image_time;
bool g_do_cb;

using namespace std;


void on_mouse(int event, int x, int y, int flags, void *params){
  
  switch(event){
  case CV_EVENT_LBUTTONUP:
    // Add a clicked-on point to the list of points, to be published by the image callback on the next image.
    g_selection_mutex.lock();
    PublishedPoint p;
    p.xy = cvPoint(x,y);
    p.published = false;
    gxys.push_back(p);
    g_do_cb = true;
    g_selection_mutex.unlock();

    break;
    
  default:
    break;    
  }
}


class TrackStarterGUI: public ros::node
{
public:
  image_msgs::Image limage_;
  image_msgs::Image dimage_;
  image_msgs::StereoInfo stinfo_;
  image_msgs::CamInfo rcinfo_;
  image_msgs::CvBridge lbridge_;
  image_msgs::CvBridge dbridge_;
  color_calib::Calibration lcolor_cal_;
  bool quit_;
  bool calib_color_;
  CvStereoCamModel *cam_model_;
  CvMat *uvd_, *xyz_;
  IplImage *cv_image_;
  IplImage *cv_disp_image_;
  //IplImage *cv_image_cpy_;
  //IplImage *cv_disp_image_cpy_;
  ros::thread::mutex cv_mutex_;
  robot_msgs::PositionMeasurement pos;
  TopicSynchronizer<TrackStarterGUI> sync_;
  

  TrackStarterGUI():
    //node("track_starter_gui",ros::node::ANONYMOUS_NAME),
    ros::node("track_starter_gui"),
    lcolor_cal_(this),
    quit_(false),
    calib_color_(true),
    cam_model_(NULL),
    uvd_(NULL),
    xyz_(NULL),
    cv_image_(NULL),
    cv_disp_image_(NULL),
    //cv_image_cpy_(NULL),
    //cv_disp_image_cpy_(NULL),
    sync_(this, &TrackStarterGUI::image_cb_all, ros::Duration(0.05), &TrackStarterGUI::image_cb_timeout)
  {
    
    cvNamedWindow("Track Starter: Left Image",CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Track Starter: Disparity",CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback("Track Starter: Left Image", on_mouse, 0);

    uvd_ = cvCreateMat(1,3,CV_32FC1);
    xyz_ = cvCreateMat(1,3,CV_32FC1);

    advertise<robot_msgs::PositionMeasurement>("/track_starter_gui/position_measurement",1);
    sync_.subscribe("dcam/left/image_rect_color",limage_,1);
    sync_.subscribe("dcam/disparity",dimage_,1);
    sync_.subscribe("dcam/stereo_info", stinfo_,1);
    sync_.subscribe("dcam/right/cam_info",rcinfo_,1);
    subscribe("track_starter_gui/position_measurement",pos,&TrackStarterGUI::point_cb,1);
    
  }

  ~TrackStarterGUI() 
  {
    cvDestroyWindow("Track Starter: Left Image");
    cvDestroyWindow("Track Starter: Disparity");
    
    cvReleaseImage(&cv_image_);
    cvReleaseImage(&cv_disp_image_);
    //cvReleaseImage(&cv_image_cpy_);
    //cvReleaseImage(&cv_disp_image_cpy_);
    cvReleaseMat(&uvd_);
    cvReleaseMat(&xyz_);

    if (cam_model_) {
      delete cam_model_;
    }
  }

  // Sanity check, print the published point.
  void point_cb() {
    printf("pos %f %f %f\n",pos.pos.x,pos.pos.y,pos.pos.z);

  }

  // Image callback. Draws selected points on images, publishes the point messages, and copies the images to be displayed.
  void image_cb_all(ros::Time t){

    cv_mutex_.lock();
    if (!g_do_cb) {
      cv_mutex_.unlock();
      return;
    }

    // Set color calibration.
    bool do_calib = false;
    if (calib_color_ && has_param("dcam/left/image_rect_color")) {
      // Exit if color calibration hasn't been performed.
      do_calib = true;
      lcolor_cal_.getFromParam("dcam/left/image_rect_color");
    }

    // Convert the images to OpenCV format.
    if (lbridge_.fromImage(limage_,"bgr")) {
      if (do_calib) {
	lbridge_.reallocIfNeeded(&cv_image_, IPL_DEPTH_8U);
	lcolor_cal_.correctColor(lbridge_.toIpl(), cv_image_, true, true, COLOR_CAL_BGR);
      }
    }
    if (dbridge_.fromImage(dimage_)) {
      dbridge_.reallocIfNeeded(&cv_disp_image_, IPL_DEPTH_16U);
      dbridge_.toIpl();
      //cvCvtScale(cv_disp_image_, cv_disp_image_, 4.0/stinfo.dpp);
    }
    
    // Convert the stereo calibration into a camera model.
    if (cam_model_) {
      delete cam_model_;
    }
    double Fx = rcinfo_.P[0];
    double Fy = rcinfo_.P[5];
    double Clx = rcinfo_.P[2];
    double Crx = Clx;
    double Cy = rcinfo_.P[6];
    double Tx = -rcinfo_.P[3]/Fx;
    cam_model_ = new CvStereoCamModel(Fx,Fy,Tx,Clx,Crx,Cy,stinfo_.dpp);

    CvSize im_size = cvGetSize(cv_image_);

    for (uint i = 0; i<gxys.size(); i++) {

      if (cam_model_ && !gxys[i].published) {
	bool search = true;
  
	int d = cvGetReal2D(cv_disp_image_,gxys[i].xy.y,gxys[i].xy.x);
	if (d==0.0) {
	  search = true;
	  int x1 = gxys[i].xy.x;	
	  int y1 = gxys[i].xy.y;
	  int x2 = gxys[i].xy.x;	
	  int y2 = gxys[i].xy.y;
	  while (d==0.0 && search) {
	    x1--; y1--;
	    x2++; y2++;
	    search = false;
	    if (x1 > 0) {
	      d = cvGetReal2D(cv_disp_image_,gxys[i].xy.y,x1);
	      search = true;
	    }
	    if (d==0.0 && x2 < cv_disp_image_->width) {     
	      d = cvGetReal2D(cv_disp_image_,gxys[i].xy.y,x2);
	      search = true;
	    }
	    if (d==0.0 && y1 > 0) {
	      d = cvGetReal2D(cv_disp_image_,y1,gxys[i].xy.x);
	      search = true;
	    }
	    if (d==0.0 && y2 < cv_disp_image_->height) {
	      d = cvGetReal2D(cv_disp_image_,y2,gxys[i].xy.x);
	      search = true;
	    }
	      
	  }
	}
	if (search) {  
	  robot_msgs::PositionMeasurement pm;
	  pm.header.stamp = g_last_image_time;
	  pm.name = "track_starter_gui";
	  pm.object_id = gxys.size();   
	  cvmSet(uvd_,0,0,gxys[i].xy.x);
	  cvmSet(uvd_,0,1,gxys[i].xy.y);
	  cvmSet(uvd_,0,2,d);
	  cam_model_->dispToCart(uvd_, xyz_);
	  pm.pos.x = cvmGet(xyz_,0,2);
	  pm.pos.y = -1.0*cvmGet(xyz_,0,0);
	  pm.pos.z = -1.0*cvmGet(xyz_,0,1);
	  pm.header.frame_id = "stereo_link";
	  pm.reliability = 1;
	  pm.initialization = 1;
	  publish("track_starter_gui/position_measurement",pm);
	  gxys[i].published = true;
	}	
      }

      cvCircle(cv_image_, gxys[i].xy, 2 , cvScalar(255,0,0) ,4);
    }

    cvShowImage("Track Starter: Left Image", cv_image_);
    cvShowImage("Track Starter: Disparity", cv_disp_image_);
    g_last_image_time = limage_.header.stamp;
//     if (cv_image_cpy_ == NULL) {
//       cv_image_cpy_ = cvCreateImage(im_size,IPL_DEPTH_8U,3);
//     }
//     cvCopy(cv_image_, cv_image_cpy_);

//     if (cv_disp_image_cpy_==NULL) {
//       cv_disp_image_cpy_ = cvCreateImage(im_size,IPL_DEPTH_8U,1);
//     }
//     cvCopy(cv_disp_image_, cv_disp_image_cpy_);

//     g_last_image_time = image_msg_.header.stamp;

    cv_mutex_.unlock();
    
  }

  void image_cb_timeout(ros::Time t) {
    if (limage_.header.stamp != t)
      printf("Timed out waiting for left image\n");
    if (dimage_.header.stamp != t)
      printf("Timed out waiting for disparity image\n");
  }

  // Wait for thread to exit.
  bool spin() {
    while (ok() && !quit_) {
      // Display the image
      cv_mutex_.lock();

      int c = cvWaitKey(2);
      c &= 0xFF;
      // Quit on ESC, "q" or "Q"
      if((c == 27)||(c == 'q')||(c == 'Q')){
	quit_ = true;
      }
      // Pause playback for point selection on "p" or "P"
      else if ((c=='p') || (c=='P')){
	g_do_cb = 1-g_do_cb;
      }
      cv_mutex_.unlock();
      usleep(10000);
	
    }
    return true;
  }
};

// Main 
int main(int argc, char**argv) 
{
  ros::init(argc,argv);
 
  g_do_cb = true;
  TrackStarterGUI tsgui;
  tsgui.spin();
  ros::fini();
  return 0;

}
