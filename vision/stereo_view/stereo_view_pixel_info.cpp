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


/********************************************************************
 * This is an extension to stereo_view.cpp that allows you to click on a point in the left image and display
 * its image coords, color, and stereo frame coords (the vision-version of real-world coords with z forwards.)
 * Best used while playback is stopped because the selected point is only drawn on one frame, 
 * but it works even during live playback.
 ********************************************************************/


#include <vector>

#include "image_msgs/CvBridge.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "ros/node.h"
#include "image_msgs/DisparityInfo.h"
#include "image_msgs/StereoInfo.h"
#include "image_msgs/Image.h"
#include "image_msgs/CamInfo.h"
#include "image_msgs/ColoredLines.h"
#include "image_msgs/ColoredLine.h"

#include "CvStereoCamModel.h"

#include "color_calib.h"

#include "topic_synchronizer.h"

#include <boost/thread.hpp>

using namespace std;

struct MouseCallbackParams {
  IplImage* limage;
  IplImage* disp;
  image_msgs::DisparityInfo* dispinfo;
  image_msgs::StereoInfo* stinfo;
  image_msgs::CamInfo* rcaminfo;
};

//boost::mutex cv_mutex;
boost::mutex g_cv_mutex;

/*!
 * \brief Click on a point in the left image to get 2d, color and 3d information.
 *
 *
 * Click on a point in the left image to 
 * 1) Draw a red dot;
 * 2) Get the (u,v,d) position;
 * 3) Get the (R,G,B) value;
 * 4) Get the (x,y,z) value in the stereo frame (the vision-version of x,y,z with z pointing forwards.)
 */
void on_mouse(int event, int x, int y, int flags, void *params) {
  
  MouseCallbackParams* mcbparams = (MouseCallbackParams*)params;
  CvScalar rgb;
  
  switch(event) {

  case CV_EVENT_MOUSEMOVE:
    break;

  case CV_EVENT_LBUTTONUP:
    g_cv_mutex.lock(); 
    rgb = cvGet2D(mcbparams->limage,y,x);
    if (mcbparams->disp) {
      double d = ((mcbparams->dispinfo->dpp)/4.0)*cvGetReal2D(mcbparams->disp,y,x);    
      printf("Image (u,v,d): (%d,%d,%f); (R/gray,G,B): (%f,%f,%f); ", x,y,d/(mcbparams->dispinfo->dpp), rgb.val[2],rgb.val[1],rgb.val[0] );
      if (mcbparams->rcaminfo) {
	CvStereoCamModel* cam_model = new CvStereoCamModel(mcbparams->rcaminfo->P[0], mcbparams->rcaminfo->P[5], 
							   -(mcbparams->rcaminfo->P[3])/(mcbparams->rcaminfo->P[0]), 
							   mcbparams->rcaminfo->P[2], mcbparams->rcaminfo->P[2], 
							   mcbparams->rcaminfo->P[6], 1.0/mcbparams->dispinfo->dpp);
	CvMat* uvd = cvCreateMat(1,3,CV_32FC1);
	cvmSet(uvd,0,0,x);
	cvmSet(uvd,0,1,y);
	cvmSet(uvd,0,2,d);
	CvMat* xyz = cvCreateMat(1,3,CV_32FC1);  
	cam_model->dispToCart(uvd,xyz);
	printf("Stereo (x,y,z): (%f, %f, %f)\n", cvmGet(xyz,0,0), cvmGet(xyz,0,1), cvmGet(xyz,0,2));
	delete cam_model;
	cvReleaseMat(&uvd); uvd = 0;
	cvReleaseMat(&xyz); xyz = 0;
      }
      cvCircle(mcbparams->disp, cvPoint(x,y), 2, cvScalar(255,255,255), 4);
      cvShowImage("disparity",mcbparams->disp);
    }
    else {
      printf("Image (u,v,d): (%d,%d,0.0); (R/gray,G,B): (%f,%f,%f); Stereo (x,y,z): (?,?,?)\n", x,y,rgb.val[2],rgb.val[1],rgb.val[0] );
    }
    cvCircle(mcbparams->limage, cvPoint(x,y), 2, cvScalar(0,0,255), 4);
    cvShowImage("left",mcbparams->limage);
    cvWaitKey(3);
    g_cv_mutex.unlock();
    break;

  default:
    break;
  }

}

/** StereoView class.
 *
 * This is an extension to stereo_view.cpp that allows you to click on a point in the left image and display
 * its image coords, color, and stereo frame coords (the vision-version of real-world coords with z forwards.)
 * Best used while playback is stopped because the selected point is only drawn on one frame, 
 * but it works even during live playback.
 */
class StereoView : public ros::Node
{
public:

  image_msgs::Image limage; /**< Left camera image msg. */
  image_msgs::Image rimage; /**< Right camera image msg. */
  image_msgs::Image dimage; /**< Disparity camera image msg. */
  image_msgs::DisparityInfo dispinfo; /**< Stereo info msg. */
  image_msgs::StereoInfo stinfo; /**< Stereo info msg. */
  image_msgs::CamInfo rcaminfo; /**< Right camera info msg. */

  image_msgs::CvBridge lbridge; /**< CvBridge for the left camera. */
  image_msgs::CvBridge rbridge; /**< CvBridge for the right camera. */
  image_msgs::CvBridge dbridge; /**< CvBridge for the disparity image. */
  image_msgs::ColoredLines cls;
  map<string,image_msgs::ColoredLines> map_cls;

  color_calib::Calibration lcal; /**< Color calibration for the left image. */
  color_calib::Calibration rcal; /**< Color calibration for the right image. */

  IplImage* lcalimage; /**< Color calibrated left image for display. */
  IplImage* rcalimage; /**< Color calibrated right image for display. */
  IplImage* disp; /**< Disparity image for display. */

  TopicSynchronizer<StereoView> sync; /**< Topic synchronizer for the stereo msgs. */

  bool calib_color_; /**< True/false do color calibration. */
  bool recompand_; /**< True/false recompand the image (gamma correction) for display. */

  MouseCallbackParams mcbparams_; /**< Parameters for the mouse callback. */

  boost::mutex cv_mutex_, lines_mutex_; 

  StereoView() : ros::Node("stereo_view"), 
                 lcal(this), rcal(this), lcalimage(NULL), rcalimage(NULL),
                 sync(this, &StereoView::image_cb_all, ros::Duration().fromSec(0.05), &StereoView::image_cb_timeout),
                 calib_color_(false), recompand_(false)
  { 
    mcbparams_.limage = NULL;
    mcbparams_.disp = NULL;
    mcbparams_.rcaminfo = NULL;
    mcbparams_.dispinfo = NULL;
    mcbparams_.stinfo = NULL;
    disp = NULL;
    cvNamedWindow("left", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("right", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("disparity", CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback("left", on_mouse, &mcbparams_);

    std::list<std::string> left_list;
    left_list.push_back(std::string("stereo/left/image_rect_color"));
    left_list.push_back(std::string("stereo/left/image_rect"));


    std::list<std::string> right_list;
    right_list.push_back(std::string("stereo/right/image_rect_color"));
    right_list.push_back(std::string("stereo/right/image_rect"));


    sync.subscribe(left_list,  limage, 1);
    sync.subscribe(right_list, rimage, 1);

    sync.subscribe("stereo/disparity", dimage, 1);
    sync.subscribe("stereo/disparity_info", dispinfo, 1);
    sync.subscribe("stereo/stereo_info", stinfo, 1);
    sync.subscribe("stereo/right/cam_info", rcaminfo, 1);
    sync.ready();

    subscribe<image_msgs::ColoredLines>("lines_to_draw",cls,&StereoView::line_cb,1);
  }

  ~StereoView()
  {
    if (lcalimage) {
      mcbparams_.limage = 0; cvReleaseImage(&lcalimage); lcalimage = 0;
    }
    if (rcalimage) {
      cvReleaseImage(&rcalimage);  rcalimage = 0;
    }
    if (disp) {
       mcbparams_.disp = 0; cvReleaseImage(&disp);  disp = 0;
    }
  }

 void line_cb() 
  {

    boost::mutex::scoped_lock linelock(lines_mutex_);
    // Find if there's already a message with this label.
    map<string,image_msgs::ColoredLines>::iterator it = map_cls.find(cls.label);
    if (it == map_cls.end()) {
      // If not, inset this new message into the map.
      map_cls.insert(pair<string,image_msgs::ColoredLines>(cls.label,cls));
    }
    else {
      // If so, overwrite the old list of lines.
      (*it).second.lines.clear();
      (*it).second = cls;
    }
  }

  /*! 
   * \brief Image callback for synced msgs. 
   *
   * Image callback for a set of synced stereo msgs.
   * Displays the (possibly color-calibrated, recompanded) images and
   * copies the necessary pointers for the mouse callback. 
   */
  void image_cb_all(ros::Time t)
  {
    boost::mutex::scoped_lock cvlock(cv_mutex_);
    mcbparams_.disp = 0;
    mcbparams_.limage = 0;


    if (lbridge.fromImage(limage, "bgr")) //LEFT IMAGE PROCESSING
    {
      if (calib_color_)
      {
        lbridge.reallocIfNeeded(&lcalimage, IPL_DEPTH_32F);

        lcal.correctColor(lbridge.toIpl(), lcalimage, true, recompand_, COLOR_CAL_BGR);

      } else { 
			lcalimage = lbridge.toIpl();
      }
      mcbparams_.limage = lcalimage;

      boost::mutex::scoped_lock linelock(lines_mutex_);
      // Draw the lines on the image.
      map<string,image_msgs::ColoredLines>::iterator it;
      for (it = map_cls.begin(); it != map_cls.end(); it++) {
			for (uint il=0; il<it->second.lines.size(); il++) {
			  cvLine(lcalimage, 
				 cvPoint(it->second.lines[il].x0,it->second.lines[il].y0),
				 cvPoint(it->second.lines[il].x1,it->second.lines[il].y1), 
				 cvScalar(it->second.lines[il].b,it->second.lines[il].g,it->second.lines[il].r),3);
			}
      }
      linelock.unlock();
      
      cvShowImage("left", lcalimage);
    }  //end left image

    if (dbridge.fromImage(dimage)) //DISPARITY
    {
      // Disparity has to be scaled to be be nicely displayable
      mcbparams_.disp = NULL;
      dbridge.reallocIfNeeded(&disp, IPL_DEPTH_8U);
      cvCvtScale(dbridge.toIpl(), disp, 4.0/dispinfo.dpp);
      cvShowImage("disparity", disp);
      mcbparams_.disp = disp;
      mcbparams_.dispinfo = &dispinfo;
      mcbparams_.stinfo = &stinfo;
      mcbparams_.rcaminfo = &rcaminfo;
    } //END DISPARITY

    if (rbridge.fromImage(rimage, "bgr")) //RIGHT IMAGE
    {
      if (calib_color_)
      {
        rbridge.reallocIfNeeded(&rcalimage, IPL_DEPTH_32F);

        rcal.correctColor(rbridge.toIpl(), rcalimage, true, recompand_, COLOR_CAL_BGR);

      cvShowImage("right", rcalimage);
      } else {
        cvShowImage("right", rbridge.toIpl());
      }
    }

    cvlock.unlock();

  }


  /*! 
   * \brief Image callback for unsynced msgs.
   *
   * Image callback for a set of unsynced messages.
   * Reports which msg is missing and proceeds to call 
   * image_cb_all() on the remaining msgs.
   */
  void image_cb_timeout(ros::Time t)
  {
    //Proceed to show images anyways
    image_cb_all(t);
  }
  
  /*!
   * Spin function. 
   *
   * This function waits while the other threads execute.
   * It also allows OpenCV to display the images and 
   * obtain user input.
   */
  bool spin()
  {
    while (ok())
    {
      boost::mutex::scoped_lock cvlock(cv_mutex_);
      int key = cvWaitKey(3);
      
      switch (key) {
      case 10:
        calib_color_ = !calib_color_;
        break;
      case 32:
        recompand_ = !recompand_;
      }

      // Fetch color calibration parameters as necessary
      if (calib_color_)
      {
        lcal.getFromParam("stereo/left/image_rect_color");
        rcal.getFromParam("stereo/right/image_rect_color");
      }

      cvlock.unlock();
      usleep(10000);
    }

    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  StereoView view;
  view.spin();
  
  return 0;
}

