#include <stdio.h>
#include <iostream>

#include "ros/node.h"
#include "std_msgs/Image.h"
#include "std_msgs/PointStamped.h"
#include "axis_cam/PTZActuatorCmd.h"
#include "axis_cam/PTZActuatorState.h"
#include "image_utils/cv_bridge.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"


// added for the videre tracker
#include <vector>
#include <map>
using namespace std;
#include "std_msgs/ImageArray.h"
#include "std_msgs/String.h"
typedef signed char schar;

#include "CvStereoCamModel.h"
#include "CvMatUtils.h"
using namespace cv::willow;
// end added for the videre tracker


#include "blob_tracker.h"

#define DISPLAY true
#define DISPLAYFREQ 30
#define BLOBNEARCENTER false


/*****************************************
 * Globals. Mainly for the mouse callback.
 *****************************************/

int g_pt_lx, g_pt_ty, g_pt_rx, g_pt_by;
CvRect g_selection;
CvRect g_new_selection;
bool g_start_track;
bool g_get_rect;
bool g_is_new_blob;

int g_iframe;

/*****************************************
 * Helper functions and mouse callback.
 *****************************************/

/*!
 * \brief Resets all of the global variables.
 */
void resetGlobals() {

  g_start_track = false;
  g_get_rect = false;
  g_selection = cvRect(-1,-1,-1,-1);
  g_new_selection = cvRect(-1,-1,-1,-1);
  g_is_new_blob = false;

  g_pt_lx = 0;
  g_pt_ty = 0;
  g_pt_rx = 0;
  g_pt_by = 0;
}

/*!
 * Swap the values of two integers.
 */ 
void swap(int *int1, int *int2) {
  int temp;
  temp = *int1;
  *int1 = *int2;
  *int2 = temp;
}

// Choose a rectangular window to start tracking
void on_mouse(int event, int x, int y, int flags, void *params)
{

  switch(event) {

  case CV_EVENT_LBUTTONDOWN:
    // Get one corner
    g_start_track = false;
    g_get_rect = true;
    g_pt_lx = x;
    g_pt_ty = y;
    g_pt_rx = x;
    g_pt_by = y;
    break;

  case CV_EVENT_MOUSEMOVE:
    // Record points drawn through if you pressed the mouse button
    if (g_get_rect) {
      // Get the other corner.
      g_pt_rx = x;
      g_pt_by = y;
    }
    break;

  case CV_EVENT_LBUTTONUP:

      // Get the other corner.
      g_pt_rx = x;
      g_pt_by = y;

    // Get the other corner. Make sure the corners are in the right order.
    if (g_pt_lx > g_pt_rx) {
      swap(&g_pt_lx,&g_pt_rx);
    }
    if (g_pt_ty > g_pt_by) {
      swap(&g_pt_ty,&g_pt_by);
    }

    // Compute the width and height and enforce they're at least 3 pixels each
    g_selection = cvRect(g_pt_lx, g_pt_ty, g_pt_rx - g_pt_lx + 1, g_pt_by - g_pt_ty + 1);

    // Done getting the rectangle.
    g_get_rect = false;

    if (g_selection.width < 3 || g_selection.height < 3) {
      printf("Selection has either width or height of <3 pixels. Try again.\n");
      //     g_selection = cvRect(-1,-1,-1,-1);
      //g_start_track = false;
      resetGlobals();
      break;
    }

    // Everything is ok, go ahead and track.
    g_start_track = true;
    g_is_new_blob = true;
    break;

  default:
    break;
  }    

} 

/*****************************************************
 * Axis cam blob tracker class
 *****************************************************/

class AxisBlobTracker : public ros::node
{
public:
  std_msgs::Image image_;
  CvBridge<std_msgs::Image> cv_bridge_;

  IplImage *cv_image_;
  IplImage *cv_image_cpy_;

  IplImage *cv_feat_image_cpy_;
  IplImage *cv_backproject_image_cpy_;
  IplImage *cv_mask_image_cpy_;

  bool quit;

  ros::thread::mutex cv_mutex_;

  BTracker *btracker_;

  bool display_;


  // Constructor
  AxisBlobTracker(bool d) : ros::node("axis_blob_tracker"), cv_bridge_(&image_, CvBridge<std_msgs::Image>::CORRECT_BGR), cv_image_(NULL), cv_image_cpy_(NULL), cv_feat_image_cpy_(NULL), cv_backproject_image_cpy_(NULL), cv_mask_image_cpy_(NULL), quit(false)
  {
    btracker_ = new BTracker();    

    g_iframe = 0;

    resetGlobals();

    // OpenCV: pop up an OpenCV highgui window and put in a mouse call back 
    cvNamedWindow("Tracker", CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback("Tracker", on_mouse, 0);

    // Ros: subscribe to image 
    subscribe("image", image_, &AxisBlobTracker::image_cb, 3);

    // OpenCV: pop up other windows for features, etc.
    display_ = d;
    if (display_) {
      cvNamedWindow("Features",CV_WINDOW_AUTOSIZE);
      cvNamedWindow("Backprojection",CV_WINDOW_AUTOSIZE);
      cvNamedWindow("Mask",CV_WINDOW_AUTOSIZE);
    }

  }

  // Destructor. 
  ~AxisBlobTracker()
  {
    if (cv_image_)
      cvReleaseImage(&cv_image_);

    if (cv_image_cpy_)
      cvReleaseImage(&cv_image_cpy_);

    if (cv_feat_image_cpy_)
      cvReleaseImage(&cv_feat_image_cpy_);

    if (cv_backproject_image_cpy_) 
      cvReleaseImage(&cv_backproject_image_cpy_);

    if (cv_mask_image_cpy_)
      cvReleaseImage(&cv_mask_image_cpy_);

    cvDestroyWindow("Tracker");

    if (display_) {
      cvDestroyWindow("Features");
      cvDestroyWindow("Backprojection");
      cvDestroyWindow("Mask");
    }

    delete(btracker_);

  }


  // The image callback. For each new image, copy it, find the new blob window, and draw the new window.
  void image_cb()
  {

    bool oktrack;
    CvSize im_size;

    g_iframe++;

    if (cv_image_)
      cvReleaseImage(&cv_image_);

    if (cv_bridge_.to_cv(&cv_image_)){

      im_size = cvGetSize(cv_image_);

      // Track and draw the new window.
      if (g_start_track) {

	oktrack = btracker_->processFrame(cv_image_, g_is_new_blob, g_selection, &g_new_selection);

	g_is_new_blob = false;
	if (!oktrack) {
	  // Lost track, reset everything.
	  resetGlobals();
	  printf("Track failed, waiting for new window.\n");
	  return;
	}
	else {
	  // Tracking succeeded, copy the new window and draw it.
	  g_selection = g_new_selection;	
	  cvRectangle(cv_image_,
		      cvPoint(g_selection.x,g_selection.y),
		      cvPoint(g_selection.x+g_selection.width,g_selection.y+g_selection.height),
		      cvScalar(0,0,255), 4);

	}

      }

      // Draw a rectangle which is still being defined.
      if (g_get_rect) {
     
	cvRectangle(cv_image_,
		    cvPoint(g_pt_lx, g_pt_ty),
		    cvPoint(g_pt_rx, g_pt_by),
		    cvScalar(0,0, 255), 4);
      }

      // Copy all of the images you might want to display.
      // This is necessary because OpenCV doesn't like multiple threads.
      cv_mutex_.lock();

      if (cv_image_cpy_ == NULL) {
	cv_image_cpy_ = cvCreateImage(im_size,IPL_DEPTH_8U,3);
      }
      cvCopy(cv_image_, cv_image_cpy_);

      if (display_ && g_start_track) {

	if (cv_feat_image_cpy_ == NULL) {
	  cv_feat_image_cpy_ = cvCreateImage(im_size,IPL_DEPTH_8U,1);
	}
	cvCopy(btracker_->feat_image_ptr_, cv_feat_image_cpy_);

	if (cv_backproject_image_cpy_ == NULL) {
	  cv_backproject_image_cpy_ = cvCreateImage(im_size,IPL_DEPTH_8U,1);
	}
	cvCopy(btracker_->backproject_ptr_, cv_backproject_image_cpy_);

	if (cv_mask_image_cpy_ == NULL) {
	  cv_mask_image_cpy_ = cvCreateImage(im_size,IPL_DEPTH_8U,1);
	}
	cvCopy(btracker_->mask_ptr_, cv_mask_image_cpy_);

      }

      cv_mutex_.unlock();
      
    }
  }


  // Wait for completion, wait for user input, display images.
  bool spin()
  {
    while (ok() && !quit)
      {

	// Display all of the images.
	cv_mutex_.lock();
	if (cv_image_cpy_)
	  cvShowImage("Tracker",cv_image_cpy_);

	if (display_) {

	  if (cv_feat_image_cpy_)
	    cvShowImage("Features",cv_feat_image_cpy_);

	  if (cv_backproject_image_cpy_)
	    cvShowImage("Backprojection",cv_backproject_image_cpy_);

	  if (cv_mask_image_cpy_)
	    cvShowImage("Mask",cv_mask_image_cpy_);
	}
	cv_mutex_.unlock();

	// Get user input and allow OpenCV to refresh windows.
	int c = cvWaitKey(2);
	c &= 0xFF;
	// Quit on ESC, "q" or "Q"
	if((c == 27)||(c == 'q')||(c == 'Q'))
	  quit = true;

      }
    return true;
  }

};

// Main
int main(int argc, char **argv)
{
  ros::init(argc, argv);

  AxisBlobTracker a(DISPLAY);

  a.spin();

  ros::fini();
  return 0;
}
