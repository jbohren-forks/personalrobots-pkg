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

#define USE_AXIS_CAM 0
#define DISPLAY true
#define DISPLAYFREQ 1
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
  AxisBlobTracker(bool d) : ros::node("bt_demo"), cv_bridge_(&image_, CvBridge<std_msgs::Image>::CORRECT_BGR), cv_image_(NULL), cv_image_cpy_(NULL), cv_feat_image_cpy_(NULL), cv_backproject_image_cpy_(NULL), cv_mask_image_cpy_(NULL), quit(false)
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

/// VidereBlobTracker - blob tracking stereo images (left and disparity) from 
/// a videre

struct imgData
{
  string label;
  IplImage *cv_image;
  CvBridge<std_msgs::Image> *bridge;
};

// this class is originally copied from CvView in cv_view_array.cpp in vision/cv_view package.
class VidereBlobTracker: public ros::node {
public:
  // for subscription
  std_msgs::ImageArray image_msg_;
  std_msgs::String cal_params_;

  // for publishing
  std_msgs::PointStamped point_stamped_;

  /// camara model
  CvStereoCamModel *camModel_;

  /// a copy of the current image for display
  IplImage *cv_image_cpy_;
  /// a copy of the disparity image of the current image
  IplImage *cv_dispImg_cpy_;
  
  IplImage *cv_feat_image_cpy_;
  IplImage *cv_backproject_image_cpy_;
  IplImage *cv_mask_image_cpy_;

  bool quit;

  ros::thread::mutex cv_mutex_;

  BTracker *btracker_;

  map<string, imgData> images;

  bool display_;
  bool blobNearCenter_;
  int  numFrames_;

  VidereBlobTracker(/// if true, the blob is always near the center
		    /// of the image
		    bool blobNearCenter,
		    /// if true, display several images of intermediate
		    /// steps. e.g. feature map, back projection. etc..
		    bool display) : 
    node("bt_demo", ros::node::ANONYMOUS_NAME),
    camModel_(NULL),
    cv_image_cpy_(NULL), 
    cv_dispImg_cpy_(NULL),
    cv_feat_image_cpy_(NULL), 
    cv_backproject_image_cpy_(NULL), 
    cv_mask_image_cpy_(NULL),
    quit(false),
    blobNearCenter_(blobNearCenter),
    numFrames_(0)
  { 
    btracker_ = new BTracker();    

    g_iframe = 0;

    resetGlobals();

    // OpenCV: pop up an OpenCV highgui window and put in a mouse call back 
    cvNamedWindow("Tracker", CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback("Tracker", on_mouse, 0);

    // Ros: subscribe to image, with a call back
    subscribe("images", image_msg_, &VidereBlobTracker::image_cb, 1);

    // Ros: subscribe to calibration parameters
    subscribe("calparams", cal_params_, &VidereBlobTracker::cal_params_cb, 1000);

    // Ros: advertise a topic
    advertise<std_msgs::PointStamped>("points", 1000);

    // OpenCV: pop up other windows for features, etc.
    display_ = display;
    if (display_) {
      cvNamedWindow("Features", 0);
      cvNamedWindow("Backprojection",0);
      cvNamedWindow("Mask",0);
      cvNamedWindow("left_disparity", 0);
    }

  }

  ~VidereBlobTracker()
  {
    for (map<string, imgData>::iterator i = images.begin(); i != images.end(); i++)
    {
      if (i->second.cv_image)
        cvReleaseImage(&i->second.cv_image);
    }
    // TODO: lots of other things needs to be release here
    delete camModel_;
  }

  void check_keys() 
  {
    cv_mutex_.lock();
    if (cvWaitKey(3) == 10)
    { }
      //      save_image();
    cv_mutex_.unlock();
  }

  /// The image callback. For each new image, copy it, find the new blob 
  /// window, report the 3D location, and draw the new window.
  void image_cb()
  {

    bool oktrack;
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

	if (blobNearCenter_ == true) {
#if 0
	  printf("old window %d, %d, %d, %d\n", 
		 g_selection.x, g_selection.y, g_selection.width,
		 g_selection.height);
#endif
	  // move the last window to the center regardless.
	  // we may need to make the box larger than this
	  g_selection.x = im_size.width/2  - g_selection.width /2;
	  g_selection.y = im_size.height/2 - g_selection.height/2; 
#if 0
	  printf("center window %d, %d, %d, %d\n", 
		 g_selection.x, g_selection.y, g_selection.width,
		 g_selection.height);
#endif
	}

	oktrack = btracker_->processFrame(leftImg, g_is_new_blob, g_selection, 
					  &g_new_selection);

	g_is_new_blob = false;
	if (!oktrack) {
	  // Lost track, reset everything.
	  resetGlobals();
	  printf("Track failed, waiting for new window.\n");
	  return;
	}
	else {
	  //
	  // Tracking succeeded, copy the new window, compute 3D point
	  // and draw it.
	  //
	  
	  // copy the new window
	  g_selection = g_new_selection;

	  // compute the 3D point
	  if (camModel_) {
	    // get a smaller rectangle from the current window
	    CvRect centerBox = cvRect(g_selection.x+g_selection.width/4,
				      g_selection.y+g_selection.height/4,
				      g_selection.width/2,
				      g_selection.height/2);
	    // check the boundary just to make sure.
	    if (centerBox.x + centerBox.width > im_size.width) {
	      centerBox.width = im_size.width - centerBox.x;
	    }
	    if (centerBox.y + centerBox.height > im_size.height) {
	      centerBox.height = im_size.height - centerBox.y;
	    }
	    // get all the non zero disparity inside this box
	    // construct a list uvd points, in disparity space.
	    // convert each of them into cartesian space, and 
	    // compute the centroid of them.
	    double _uvds[3*centerBox.width*centerBox.height];
	    int numGoodPoints=0;
	    for (int x = centerBox.x; x < centerBox.x+centerBox.width; x++) {
	      for (int y = centerBox.y; y < centerBox.y+centerBox.height; y++) {
		double disp = cvGetReal2D(dispImg, y, x);
		if (disp>0) {
		  // The disparity map we get from the camera is in raw form.
		  // In raw form, the unit in the disparity is 1/4 of a pixel.
		  disp /=4.0;
		  _uvds[numGoodPoints*3    ] = x;
		  _uvds[numGoodPoints*3 + 1] = y;
		  _uvds[numGoodPoints*3 + 2] = disp;
		  numGoodPoints++;
		}
	      }
	    }
	    if (numGoodPoints>0) {
	      CvMat uvds = cvMat( numGoodPoints, 3, CV_64FC1, _uvds );
	      double _xyzs[numGoodPoints*3];
	      CvMat xyzs = cvMat( numGoodPoints, 3, CV_64FC1, _xyzs );
	      camModel_->dispToCart(uvds, xyzs);
	      CvMat xyzsC3;
	      cvReshape( &xyzs, &xyzsC3, 3, 0);
	      CvScalar centroid = cvAvg( &xyzsC3 );
#if 0
	      printf("centroid %f, %f, %f\n", 
		     centroid.val[0], centroid.val[1], centroid.val[2]);
#endif
	      // convert the centroid into meters
	      point_stamped_.point.x = centroid.val[0]/1000.;
	      point_stamped_.point.y = centroid.val[1]/1000.;
	      point_stamped_.point.z = centroid.val[2]/1000.;
	      // publish the centroid of the tracked object
	      publish("points", point_stamped_);
	    }
	  }
	  
	  // draw it
	  cvRectangle(leftImg,
		      cvPoint(g_selection.x,g_selection.y),
		      cvPoint(g_selection.x+g_selection.width,
			      g_selection.y+g_selection.height),
		      cvScalar(0,0,255), 4);
	}

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
      if ((numFrames_++) % DISPLAYFREQ == 0) 
      {
	numFrames_ %= DISPLAYFREQ;

	cv_mutex_.lock();

	if (cv_image_cpy_ == NULL) {
	  cv_image_cpy_ = cvCreateImage(im_size,IPL_DEPTH_8U,3);
	}
	cvCopy(leftImg, cv_image_cpy_);
      

	if (display_ && g_start_track) {

	  if (cv_dispImg_cpy_ == NULL) {
	    cv_dispImg_cpy_ = cvCreateImage(im_size,IPL_DEPTH_8U,1);
	  }
	  cvCopy(dispImg, cv_dispImg_cpy_);

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

	  if (cv_dispImg_cpy_) {
	    cvShowImage("left_disparity", cv_dispImg_cpy_);
	  }
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

  void cal_params_cb() {
    // printf("Calibration Parameter from Videre:\n");
    //printf("%s\n", cal_params_.data.c_str());
    parseCaliParams(cal_params_.data);
  }
  /// a small parser to pick up the projection matrix from 
  /// calibration message. 
  /// TODO: one should turn those calibration parameters into
  /// xml format
  void parseCaliParams(const string& cal_param_str){
    const string labelRightCamera("[right camera]");
    const string labelRightCamProj("proj");
    const string labelRightCamRect("rect");
    // move the current position to the section of "[right camera]"
    size_t rightCamSection = cal_param_str.find(labelRightCamera);
    // move the current position to part of proj in the section of "[right camera]"
    size_t rightCamProj = cal_param_str.find(labelRightCamProj, rightCamSection);
    // get the position of the word "rect", which is also the end of
    // the projection matrix
    size_t rightCamRect = cal_param_str.find(labelRightCamRect, rightCamProj);
    // the string after the word "proj" is the starting of the matrix
    size_t matrix_start = rightCamProj + labelRightCamProj.length();
    // get the sub string that contains the matrix
    string mat_str = cal_param_str.substr(matrix_start, rightCamRect-matrix_start);
    // printf("%s\n", mat_str.c_str());
    // convert the string to a double array of 12
    stringstream sstr(mat_str);
    double matdata[12];
    for (int i=0; i<12; i++) {
      sstr >> matdata[i];
    }
#if 0
    std::cout << mat_str.c_str() << std::endl;
    for (int i=0; i<12; i++) {
      cout << matdata[i] << " "<<std::endl;
    }
#endif
    if (camModel_ == NULL) {
      double Fx  = matdata[0]; // 0,0
      double Fy  = matdata[5]; // 1,1
      double Crx = matdata[2]; // 0,2
      double Cy  = matdata[6]; // 1,2
      double Clx = Crx; // the same
      double Tx  = - matdata[3]/Fx;
      std::cout << "base length "<< Tx << std::endl;
      camModel_ = new CvStereoCamModel(Fx, Fy, Tx, Clx, Crx, Cy);
#if 0
      {
	double _cartToDisp[16], _dispToCart[16];
	CvMat cartToDisp = cvMat(4, 4, CV_64FC1, _cartToDisp);
	CvMat dispToCart = cvMat(4, 4, CV_64FC1, _dispToCart);
	camModel_->getProjectionMatrices(cartToDisp, dispToCart);
	cout << "projection (cart to disp) matrix"<<endl;
	CvMatUtils::printMat(&cartToDisp);
	cout << "reprojection (disp to cart) matrix" << endl;
	CvMatUtils::printMat(&dispToCart);
      }
#endif
    }
  }

};


// Main
int main(int argc, char **argv)
{
  ros::init(argc, argv);

#if USE_AXIS_CAM
  AxisBlobTracker a(DISPLAY);

  a.spin();

#else
  VidereBlobTracker v(BLOBNEARCENTER, DISPLAY);
  v.spin();
#endif

  ros::fini();
  return 0;
}
