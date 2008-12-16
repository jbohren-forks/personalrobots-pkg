#include <stdio.h>
#include <iostream>

#include "ros/node.h"
#include "std_msgs/Image.h"

#include "image_utils/cv_bridge.h"
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"


#include <vector>
#include <map>
using namespace std;
#include "std_msgs/ImageArray.h"
#include "std_msgs/String.h"
#include "std_msgs/PointStamped.h"
#include "stereo_blob_tracker/Rect2DStamped.h"
using namespace stereo_blob_tracker;


typedef signed char schar;

#include "CvStereoCamModel.h"

#include "blob_tracker.h"

#define DISPLAY false
#define DISPLAYFREQ 30
// set the following to be a number between 0.0 and 1.0
// to specify how fast the head moves to track the blob.
// 0.0 for no movemont; 1.0 for putting the blob at the center
// in next frame.
#define BLOBNEARCENTER .01
#define GIVEUPONFAILURE false
// set the following to be true, if you would like
// to run the gui in a separate process
#define REMOTE_GUI true


/*****************************************
 * Globals. Mainly for the mouse callback.
 *****************************************/

int g_pt_lx, g_pt_ty, g_pt_rx, g_pt_by;
CvRect g_selection;
bool g_start_track;
bool g_get_rect;
bool g_is_new_blob;

ros::thread::mutex g_selection_mutex;

int g_iframe;

/*****************************************
 * Helper functions and mouse callback.
 *****************************************/

bool setSelection(CvRect& box, bool newBlob) {
  bool status;
  g_selection_mutex.lock();
  if ((newBlob == true && g_is_new_blob == true) || g_is_new_blob == false) {
    g_selection = box;
    g_is_new_blob = false;
    status = true;
  } else {
    status = false;
  }
  g_selection_mutex.unlock();
  return status;
}

void setSelection(CvRect& box) {
  g_selection_mutex.lock();
  g_selection = box;
  g_start_track = true;
  g_is_new_blob = true;
  g_selection_mutex.unlock();
}

void setSelection(int x, int y, int w, int h, bool newBlob) {
  CvRect box = cvRect(x, y, w, h);
  setSelection(box, newBlob);
}

void resetSelection() {
  g_selection_mutex.lock();
  g_selection = cvRect(-1, -1, -1, -1);
  g_start_track = false;
  g_is_new_blob = false;
  g_selection_mutex.unlock();
}


CvRect getSelection() {
  return g_selection;
}


/*!
 * \brief Resets all of the global variables.
 */
void resetGlobals() {

  resetSelection();

  g_get_rect = false;

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

  case CV_EVENT_LBUTTONUP:    {
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
    CvRect selection = cvRect(g_pt_lx, g_pt_ty, g_pt_rx - g_pt_lx + 1, g_pt_by - g_pt_ty + 1);

    // Done getting the rectangle.
    g_get_rect = false;

    if (selection.width < 3 || selection.height < 3) {
      printf("Selection has either width or height of <3 pixels. Try again.\n");
      resetGlobals();
      break;
    }

    // Everything is ok, go ahead and track.
    setSelection(selection);
    break;
  }

  default:
    break;
  }

}

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
  // configuration consts
  /// true if need to display debugging windows
  const bool display_;
  /// true if the gui is running remotely (not in the process)
  const bool remote_gui_;


  // for subscription
  const string imagesTopic_; // ("images");
  std_msgs::ImageArray image_msg_;
  const string calParamTopic_; // "calparam"
  std_msgs::String cal_params_;
  const string selectionBoxTopic_; // selectionbox
  Rect2DStamped selection_box_;

  // for publishing
  std_msgs::PointStamped point_stamped_;
  const string trackedBoxTopic_; // trackedbox

  /// camara model
  CvStereoCamModel *camModel_;

  /// a copy of the current image for display
  IplImage *cv_image_cpy_;
  /// a copy of the disparity image of the current image
  IplImage *cv_dispImg_cpy_;

  IplImage *cv_feat_image_cpy_;
  IplImage *cv_backproject_image_cpy_;
  IplImage *cv_mask_image_cpy_;
  IplImage *cv_depthmask_image_cpy_;
  IplImage *cv_histogram_image_cpy_;

  IplImage *depthmask_;
  IplImage *temp_;

  /// the 3d blob
  CvMat    *blobPts_;
  /// statistics of the current blob
  CvScalar blobCentroid_;
  double minZ_;
  double maxZ_;

  bool quit;

  ros::thread::mutex cv_mutex_;

  BTracker *btracker_;

  map<string, imgData> images;

  /// weight of next position towards to center
  float blobNearCenter_;
  int  numFrames_;

  bool waitForReEntry_;
  int  num_frames_failed_;

  VidereBlobTracker(/// if true, the blob is always near the center
		    /// of the image
		    float blobNearCenter,
		    /// if true, display several images of intermediate
		    /// steps. e.g. feature map, back projection. etc..
		    bool display,
		    /// if true, remote gui is used to selection windows and
		    /// display tracking results
		    bool remote_gui) :
    node("videre_blob_tracker", ros::node::ANONYMOUS_NAME),
    display_(display),
    remote_gui_(remote_gui),
    imagesTopic_("images"),
    calParamTopic_("calparams"),
    selectionBoxTopic_("selectionbox"),
    trackedBoxTopic_("trackedbox"),
    camModel_(NULL),
    cv_image_cpy_(NULL),
    cv_dispImg_cpy_(NULL),
    cv_feat_image_cpy_(NULL),
    cv_backproject_image_cpy_(NULL),
    cv_mask_image_cpy_(NULL),
    cv_depthmask_image_cpy_(NULL),
    cv_histogram_image_cpy_(NULL),
    depthmask_(NULL),
    temp_(NULL),
    blobPts_(NULL),
    quit(false),
    blobNearCenter_(min(1.0f, max(0.f, blobNearCenter))),
    numFrames_(0),
    waitForReEntry_(false),
    num_frames_failed_(0)
  {
    btracker_ = new BTracker();

    g_iframe = 0;

    resetGlobals();

    if (remote_gui_ == false ) {
      // OpenCV: pop up an OpenCV highgui window and put in a mouse call back
      cvNamedWindow("Tracker", CV_WINDOW_AUTOSIZE);
      cvSetMouseCallback("Tracker", on_mouse, 0);
    }

    // Ros: subscribe to image, with a call back
    subscribe(imagesTopic_, image_msg_, &VidereBlobTracker::image_cb, 1);

    // Ros: subscribe to calibration parameters
    subscribe(calParamTopic_, cal_params_, &VidereBlobTracker::cal_params_cb, 1000);

    if (remote_gui_) {
      // Ros: subscribe to gui selection
      subscribe(selectionBoxTopic_, selection_box_, &VidereBlobTracker::selection_box_cb, 1000);
    }

    // Ros: advertise a topic
    advertise<std_msgs::PointStamped>("points", 1000);
    advertise<std_msgs::PointStamped>("head_controller/frame_track_point", 1000);
    // tracked box
    advertise<Rect2DStamped>(trackedBoxTopic_, 1000);

    // OpenCV: pop up other windows for features, etc.
    if (display_) {
      cvNamedWindow("Features", 0);
      cvNamedWindow("Backprojection",0);
      cvNamedWindow("Mask",0);
      cvNamedWindow("DepthMask", 0);
      cvNamedWindow("left_disparity", 0);
      cvNamedWindow("Histogram", 0);
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
    if (depthmask_)   cvReleaseImage(&depthmask_);
    if (temp_)        cvReleaseImage(&temp_);
    if (blobPts_)     cvReleaseMat(&blobPts_);
    delete camModel_;

    if (cv_feat_image_cpy_)        cvReleaseImage(&cv_feat_image_cpy_);
    if (cv_backproject_image_cpy_) cvReleaseImage(&cv_backproject_image_cpy_);
    if (cv_mask_image_cpy_)        cvReleaseImage(&cv_mask_image_cpy_);
    if (cv_depthmask_image_cpy_)   cvReleaseImage(&cv_depthmask_image_cpy_);
    if (cv_histogram_image_cpy_)   cvReleaseImage(&cv_histogram_image_cpy_);

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
    CvRect new_selection = cvRect(-1, -1, -1, -1);

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

      CvSize im_size = cvGetSize(leftImg);

      if (depthmask_ == NULL) {
	depthmask_ =  cvCreateImage(im_size,IPL_DEPTH_8U,1);
      }


      // Track and draw the new window.
      if (g_start_track) {

	g_selection_mutex.lock();
	CvRect selection = getSelection();
	bool isNewBlob   = g_is_new_blob;
	g_selection_mutex.unlock();

	if (blobNearCenter_> 0.0) {
	  // move the last window position to be somewhere between the last
	  // tracked position and the center of the window, weighted by
	  // blobNearCenter
	  double center_x = im_size.width/2  - selection.width /2;
	  double center_y = im_size.height/2 - selection.height/2;
	  selection.x +=  (center_x - selection.x) * blobNearCenter_;
	  selection.y +=  (center_y - selection.y) * blobNearCenter_;

	}

	IplImage *mask = NULL;

	if (isNewBlob == true || blobPts_ == NULL) {
	  compute3dBlob(dispImg, selection);
	}

	if (blobPts_ != NULL) {
	  int margin;
	  if (waitForReEntry_) {
	    // enlarge the range by 1. meter
	    margin = 1000;
	  } else {
	    // enlarge the range by .5 meter, or 500mm
	    margin = 500;
	  }

	  double minZ, maxZ;
	  minZ = minZ_ - margin;
	  maxZ = maxZ_ + margin;
	  // make sure minZ is at least 100 mm away
	  if (minZ < 100) minZ = 100;

	  // set it down to a hard range
	  //minZ = 500;
	  //maxZ = 2000;

	  // the unit of the raw disparity image is 1/4 pixel
	  //double dispUnitScale = .25;
	  //computeDepthMask(dispImg, depthmask_, dispUnitScale, minZ, maxZ);
	  camModel_->getDepthMask(dispImg, depthmask_, minZ, maxZ, CvStereoCamModel::NOISE_REMOVAL);
	  mask = depthmask_;
	}

	// attemp some poor man's white balancing
	balanceColor(leftImg);

	bool use_cam_shift = true;
	oktrack = btracker_->processFrame(leftImg, isNewBlob, selection,
					  &new_selection, mask, use_cam_shift);

	// filter out blob that are too small or too skining. Based on
	// empirical setting
	if (new_selection.width < 5 || new_selection.height < 5 ||
	    new_selection.width/new_selection.height > 5 ||
	    new_selection.height/new_selection.width > 5 ) {
	  oktrack = false;
	}
	if (!oktrack) {
	  if (GIVEUPONFAILURE == true) {
	    // Lost track, reset everything.
	    resetGlobals();
	    printf("Track failed, waiting for new window.\n");
	    return;
	  } else {
	    // log warning message

	    printf("Track failed, waiting for new window or blob to come back.\n");
	    // remove the depth mask and the last blotpts,
	    // as the blob may enter in a very different depth
	    //if (depthmask_) cvReleaseImage(&depthmask_);
	    //if (blobPts_  ) cvReleaseMat(&blobPts_);
	    // set the selection blob to be centered of the screen with a
	    // reasonable size (quarter of the window dimensions)
	    int width  = im_size.width/4;
	    int height = im_size.height/4;
	    int cx, cy;
	    if (num_frames_failed_ < 20) {
	      // search for two cycles 5 fixed position, and fixed
	      // window size for the blob.
	      int code = num_frames_failed_%5;
	      switch (code) {
	      case 0:
		cx = 0+im_size.width/8; cy = 0+im_size.height/8;
		break;
	      case 1:
		cx = 0+im_size.width/8;
		cy = im_size.height/2+im_size.height/8;
		break;
	      case 2:
		cx = im_size.width/2+im_size.width/8;
		cy = im_size.height/2+im_size.height/8;
		break;
	      case 3:
		cx = im_size.width/2 + im_size.width/8;
		cy = 0 + im_size.width/8;
		break;
	      case 4:
		cx = im_size.width/2 - width/2;
		cy = im_size.height/2 - height/2;
		break;
	      }
	    } else {
	      cx = im_size.width/2 - width/2;
	      cy = im_size.height/2 - height/2;
	    }
	    new_selection = cvRect(cx, cy,
				   width, height);
	    waitForReEntry_ = true;
	    num_frames_failed_++;
	  }
	} else {
	  waitForReEntry_ = false;
	  num_frames_failed_ = 0;
	}
	//
	// Tracking succeeded, copy the new window, compute 3D point
	// and draw it.
	//

	// copy the new window
	if (setSelection(new_selection, isNewBlob) == false ) {
	  // this iteration is out of sync with the new selection from gui
	  // discard the result
	  printf(" discarding out of sync result\n");
	  return;
	}

	// compute the 3D point
	compute3dBlob(dispImg, new_selection);

	if (oktrack == true)
	  publish3dBlobCentroid();

	if (remote_gui_) {
	  // publish tracked box
	  Rect2DStamped box;
	  box.rect.x = new_selection.x;
	  box.rect.y = new_selection.y;
	  box.rect.w = new_selection.width;
	  box.rect.h = new_selection.height;
	  publish(trackedBoxTopic_, box);
	} else {
	  // draw it
	  cvRectangle(leftImg,
		      cvPoint(new_selection.x,new_selection.y),
		      cvPoint(new_selection.x+new_selection.width,
			      new_selection.y+new_selection.height),
		      cvScalar(0,0,255), 4);
	}
      }

      // Draw a rectangle which is still being defined.
      if (remote_gui_ == false && g_get_rect) {

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


	if (display_ && g_start_track) {

	  if (cv_dispImg_cpy_ == NULL) {
	    cv_dispImg_cpy_ = cvCreateImage(im_size,IPL_DEPTH_8U,1);
	  }
	  cvCopy(dispImg, cv_dispImg_cpy_);

	  if (btracker_->feat_image_ptr_) {
	    if (cv_feat_image_cpy_ == NULL) {
	      cv_feat_image_cpy_ = cvCreateImage(im_size,IPL_DEPTH_8U,1);
	    }
	    cvCopy(btracker_->feat_image_ptr_, cv_feat_image_cpy_);
	  }

	  if (btracker_->backproject_ptr_) {
	    if (cv_backproject_image_cpy_ == NULL) {
	      cv_backproject_image_cpy_ = cvCreateImage(im_size,IPL_DEPTH_8U,1);
	    }
	    cvCopy(btracker_->backproject_ptr_, cv_backproject_image_cpy_);
	  }

	  if (btracker_->mask_ptr_) {
	    if (cv_mask_image_cpy_ == NULL) {
	      cv_mask_image_cpy_ = cvCreateImage(im_size,IPL_DEPTH_8U,1);
	    }
	    cvCopy(btracker_->mask_ptr_, cv_mask_image_cpy_);
	  }

	  if (depthmask_) {
	    if (cv_depthmask_image_cpy_ == NULL) {
	      cv_depthmask_image_cpy_ = cvCreateImage(im_size,IPL_DEPTH_8U,1);
	    }
	    cvCopy(depthmask_, cv_depthmask_image_cpy_);
	  }

	  btracker_->drawHistogram(NULL);
	  if (btracker_->hist_img_) {
	    if (cv_histogram_image_cpy_ == NULL ) {
	      cv_histogram_image_cpy_ = cvCloneImage(btracker_->hist_img_);
	    } else {
	      cvCopy(btracker_->hist_img_, cv_histogram_image_cpy_);
	    }
	  }

	}

	cv_mutex_.unlock();
      }

    }
  }

  /// a temporary fixed for color balancing
  void balanceColor(IplImage* img) {
  // try some adjustment on blue channel
    double _colorTransform[] = {1.5, 0., 0.,
				0.,  1., 0.,
				0.,  0., 1.};
    CvMat colorTransform = cvMat(3, 3, CV_64FC1, _colorTransform);
    cvTransform(img, img, &colorTransform);
  }

  void compute3dBlob(IplImage *dispImg, CvRect& selection) {
    if (camModel_ == NULL ) {
      return;
    }
    // compute the 3D point
    CvSize im_size = cvGetSize(dispImg);
    // get a smaller rectangle from the current window
    CvRect centerBox = cvRect(selection.x + selection.width/4,
			      selection.y + selection.height/4,
			      selection.width/2,
			      selection.height/2);
    // check the boundary just to make sure.
    if (centerBox.x + centerBox.width  > im_size.width) {
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
          // disp /=4.0;
          _uvds[numGoodPoints*3    ] = x;
          _uvds[numGoodPoints*3 + 1] = y;
          _uvds[numGoodPoints*3 + 2] = disp;
          numGoodPoints++;
        }
      }
    }
    if (blobPts_) {
      cvReleaseMat(&blobPts_);
    }
    if (numGoodPoints>0) {
      CvMat uvds = cvMat( numGoodPoints, 3, CV_64FC1, _uvds );
      blobPts_ = cvCreateMat(numGoodPoints, 3, CV_64FC1);
      //double _xyzs[numGoodPoints*3];
      //CvMat xyzs = cvMat( numGoodPoints, 3, CV_64FC1, _xyzs );
      camModel_->dispToCart(&uvds, blobPts_);

      CvMat xyzsC3;
      cvReshape( blobPts_, &xyzsC3, 3, 0);
      blobCentroid_ = cvAvg( &xyzsC3 );

#if 0
      printf("centroid %f, %f, %f\n",
          blobCentroid.val[0], blobCentroid.val[1], blobCentroid.val[2]);
#endif

      // compute the min and max if the blob points in 3D
      CvMat blobPtsZ;
      cvGetCol(blobPts_, &blobPtsZ, 2);

      cvMinMaxLoc(&blobPtsZ, &minZ_, &maxZ_);


    }

  }

  void publish3dBlobCentroid() {
    if (blobPts_ == NULL) {
      return;
    }

    // convert the centroid into meters
    point_stamped_.point.x = blobCentroid_.val[0]/1000.;
    point_stamped_.point.y = blobCentroid_.val[1]/1000.;
    point_stamped_.point.z = blobCentroid_.val[2]/1000.;
    // publish the centroid of the tracked object
    publish("points", point_stamped_);

    string topic = "head_controller/frame_track_point" ;

    std_msgs::PointStamped target;

    target.point.x =  point_stamped_.point.z ;
    target.point.y = -point_stamped_.point.x ;
    target.point.z = -point_stamped_.point.y;

    target.header.frame_id = "stereo" ;
    target.header.stamp=image_msg_.header.stamp;

    // copy over the time stamp of the input images, as this
    // point is derived from that input.
    target.header.stamp = image_msg_.header.stamp;
#if 0
    cout << "Requesting to move to: " << point_stamped_.point.x << ","
	 << point_stamped_.point.y << ","
	 << point_stamped_.point.z << endl ;
#endif

#if 1
    cout << " publishing to head_controller/track_point ";
    cout << target.point.x<<"," <<
    target.point.y<<"," <<
    target.point.z<< endl;
#endif

    static int count = 0;
    if (++count % 3== 0)
      publish(topic, target);

  }


  // Wait for completion, wait for user input, display images.
  bool spin()
  {
    while (ok() && (numFrames_ % DISPLAYFREQ == 0) && !quit)
      {

	// Display all of the images.
	cv_mutex_.lock();
	if (remote_gui_ == false && cv_image_cpy_)
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
	  if (cv_histogram_image_cpy_) {
	    cvShowImage("Histogram", cv_histogram_image_cpy_);
	  }
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
    if (camModel_ == NULL) {
      double Fx  = matdata[0]; // 0,0
      double Fy  = matdata[5]; // 1,1
      double Crx = matdata[2]; // 0,2
      double Cy  = matdata[6]; // 1,2
      double Clx = Crx; // the same
      double Tx  = - matdata[3]/Fx;
      std::cout << "base length "<< Tx << std::endl;
      // The disparity value unit scale is set to .25. Namely, a unit is
      // a quarter of a pixel, which is the current convention for
      // raw disparity map from the videre stereo head
      camModel_ = new CvStereoCamModel(Fx, Fy, Tx, Clx, Crx, Cy, .25);
    }
  }

  void selection_box_cb(){
    CvRect selection;
    selection.x      = selection_box_.rect.x;
    selection.y      = selection_box_.rect.y;
    selection.width  = selection_box_.rect.w;
    selection.height = selection_box_.rect.h;

    printf("receiving a selection box from gui [%d, %d, %d, %d]\n",
	   selection.x, selection.y,
	   selection.width, selection.height);

    if (selection.width < 3 || selection.height < 3) {
      printf("Selection has either width or height of <3 pixels. Try again.\n");
      resetGlobals();
    } else {
      setSelection(selection);
    }
  }
};

// Main
int main(int argc, char **argv)
{
  ros::init(argc, argv);

  VidereBlobTracker v(BLOBNEARCENTER, DISPLAY, REMOTE_GUI);
  v.spin();

  ros::fini();
  return 0;
}

