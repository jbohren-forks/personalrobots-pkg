#include <stdio.h>
#include <iostream>

#include "ros/node.h"
#include "image_utils/cv_bridge.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"


#include <vector>
#include <map>
using namespace std;
#include "std_msgs/ImageArray.h"
#include "std_msgs/String.h"

#include "stereo_blob_tracker/Rect2DStamped.h"
using namespace stereo_blob_tracker;

#include "blob_tracker.h"

#define USE_AXIS_CAM 0
#define DISPLAY true
#define DISPLAYFREQ 30

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

struct imgData
{
  string label;
  IplImage *cv_image;
  CvBridge<std_msgs::Image> *bridge;
};

/// BlobTrackerGUI subscribes to the tracker images, display them
/// wait for mouse selection of a retangle area, and publish it (intended for
/// the tracker).
class BlobTrackerGUI: public ros::node {
public:
  bool display_;
  // for subscription
  // images from the tracker
  std_msgs::ImageArray image_msg_;
  std_msgs::ImageArray aux_image_msg_;
  Rect2DStamped trackedBox_msg_;

  // for publishing. diagonal line of the rectangular selected area.
  static string selectionBoxTopic;
  

  /// a copy of the current image for display
  IplImage *cv_image_cpy_;
  /// a copy of the disparity image of the current image
  IplImage *cv_dispImg_cpy_;
  
  IplImage *cv_feat_image_cpy_;
  IplImage *cv_backproject_image_cpy_;
  IplImage *cv_mask_image_cpy_;
  IplImage *cv_depthmask_image_cpy_;


  bool quit;

  ros::thread::mutex cv_mutex_;

  map<string, imgData> images;

  int  numFrames_;

  BlobTrackerGUI(bool display):
    node("blob_tracker_gui", ros::node::ANONYMOUS_NAME),
    display_(display),
    cv_image_cpy_(NULL), 
    cv_dispImg_cpy_(NULL),
    cv_feat_image_cpy_(NULL), 
    cv_backproject_image_cpy_(NULL), 
    cv_mask_image_cpy_(NULL),
    cv_depthmask_image_cpy_(NULL),
    quit(false),
    numFrames_(0)
  { 

    g_iframe = 0;

    resetGlobals();

    // OpenCV: pop up an OpenCV highgui window and put in a mouse call back 
    cvNamedWindow("Tracker", CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback("Tracker", on_mouse, 0);

    // Ros: subscribe to images, with a call back
    subscribe("images",     image_msg_, &BlobTrackerGUI::image_cb, 1);

    // Ros: subscribe to aux images, with a callback
    subscribe("auximages",  aux_image_msg_, &BlobTrackerGUI::aux_image_cb, 1);

    // Ros: subscribe to bounding box of the tracked object
    subscribe("trackedbox", trackedBox_msg_, &BlobTrackerGUI::trackedBox_cb, 1);

    // Ros: advertise a topic.
    advertise<Rect2DStamped>(selectionBoxTopic, 1000);

    // OpenCV: pop up other windows for features, etc.
    if (display_) {
      cvNamedWindow("Features", 0);
      cvNamedWindow("Backprojection",0);
      cvNamedWindow("Mask",0);
      cvNamedWindow("DepthMask", 0);
      cvNamedWindow("left_disparity", 0);
    }

  }

  ~BlobTrackerGUI()
  {
    for (map<string, imgData>::iterator i = images.begin(); i != images.end(); i++)
    {
      if (i->second.cv_image)
        cvReleaseImage(&i->second.cv_image);
    }
    // TODO: lots of image copies need to be deleted here
  }

  void check_keys() 
  {
    cv_mutex_.lock();
    if (cvWaitKey(3) == 10)
    { }
      //      save_image();
    cv_mutex_.unlock();
  }

  void aux_image_cb() {
    printf("got aux images\n");
  }

  /// The image callback. For each new image, copy it, find the new blob 
  /// window, report the 3D location, and draw the new window.
  void image_cb()
  {

    CvSize im_size;

    g_iframe++;

    for (uint32_t i = 0; i < image_msg_.get_images_size(); i++)
    {
      string l = image_msg_.images[i].label;
      map<string, imgData>::iterator j = images.find(l);

      if (j == images.end())
      {
        images[l].label = image_msg_.images[i].label;
        images[l].bridge = new CvBridge<std_msgs::Image>(&image_msg_.images[i], CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U);
        // cvNamedWindow(l.c_str(), 0 /*manually resizable */);
	images[l].cv_image = 0;
      } else {

        if (j->second.cv_image)
          cvReleaseImage(&j->second.cv_image);

        if (j->second.bridge->to_cv(&j->second.cv_image) == false) {
	  // somehow there is no image
	  std::cerr << "no image from cvbridge"<<std::endl;
	  return;
        }
      }
      
    }

    string labelLeft("left");
    string labelDisp("left_disparity");
    if ( images[labelLeft].cv_image != NULL && 
	 images[labelDisp].cv_image != NULL )
    {
      IplImage *leftImg = images[labelLeft].cv_image;
      IplImage *dispImg = images[labelDisp].cv_image;
      im_size = cvGetSize(leftImg);

      // Track and draw the new window.
      if (g_start_track) {


	if (g_is_new_blob) {
	  // publish it
	  publishSelection(g_selection);

	  g_is_new_blob = false;
	}

	  
	// draw it
	cvRectangle(leftImg,
		    cvPoint(g_selection.x,g_selection.y),
		    cvPoint(g_selection.x+g_selection.width,
			    g_selection.y+g_selection.height),
		    cvScalar(0,0,255), 4);
	

      }

      // Draw a rectangle which is still being defined.
      if (g_get_rect) {
     
	cvRectangle(leftImg,
		    cvPoint(g_pt_lx, g_pt_ty),
		    cvPoint(g_pt_rx, g_pt_by),
		    cvScalar(0,0, 255), 4);
      }

      // Copy all of the images you might want to display.
      // This is necessary because OpenCV doesn't like multiple threads.
      // and we only do display every so often (30 frames)
      if ((numFrames_) % DISPLAYFREQ == 0) 
      {
	cv_mutex_.lock();

	if (cv_image_cpy_ == NULL) {
	  cv_image_cpy_ = cvCreateImage(im_size,IPL_DEPTH_8U,3);
	}
	cvCopy(leftImg, cv_image_cpy_);
      

	cv_mutex_.unlock();
      }
      
    }
  }


  // Wait for completion, wait for user input, display images.
  bool spin()
  {
    while (ok() && (numFrames_ % DISPLAYFREQ == 0) && !quit)
      {

	// Display all of the images.
	cv_mutex_.lock();
	if (cv_image_cpy_)
	  cvShowImage("Tracker",cv_image_cpy_);

	if (display_) {

	  if (cv_dispImg_cpy_) {
	    cvShowImage("left_disparity", cv_dispImg_cpy_);
	  }
	  if (cv_feat_image_cpy_)
	    cvShowImage("Features",cv_feat_image_cpy_);

	  if (cv_backproject_image_cpy_)
	    cvShowImage("Backprojection",cv_backproject_image_cpy_);

	  if (cv_mask_image_cpy_)
	    cvShowImage("Mask",cv_mask_image_cpy_);
	  if (cv_depthmask_image_cpy_)
	    cvShowImage("DepthMask",cv_depthmask_image_cpy_);
	}

	cv_mutex_.unlock();

	// Get user input and allow OpenCV to refresh windows.
	int c = cvWaitKey(2);
	c &= 0xFF;
	// Quit on ESC, "q" or "Q"
	if((c == 27)||(c == 'q')||(c == 'Q'))
	  quit = true;

      }

    numFrames_++;
    numFrames_ %= DISPLAYFREQ;

    return true;
  }

  void trackedBox_cb() {

    printf("received a tracked box\n");

    g_selection.x = trackedBox_msg_.rect.x;
    g_selection.y = trackedBox_msg_.rect.y;
    g_selection.width  = trackedBox_msg_.rect.w;
    g_selection.height = trackedBox_msg_.rect.h;

  }

  void publishSelection(const CvRect& selectedBox) {
    Rect2DStamped selectedBox_msg;
    selectedBox_msg.rect.x = selectedBox.x;
    selectedBox_msg.rect.y = selectedBox.y;
    selectedBox_msg.rect.w = selectedBox.width;
    selectedBox_msg.rect.h = selectedBox.height;
    printf("publishing selection box [%f, %f, %f, %f]\n",
	   selectedBox_msg.rect.x,
	   selectedBox_msg.rect.y,
	   selectedBox_msg.rect.w,
	   selectedBox_msg.rect.h);

    publish(selectionBoxTopic, selectedBox_msg);
  }

};

string BlobTrackerGUI::selectionBoxTopic("selectionbox");

// Main
int main(int argc, char **argv)
{
  ros::init(argc, argv);

  BlobTrackerGUI gui(true);

  gui.spin();


  ros::fini();

  return 0;
}
