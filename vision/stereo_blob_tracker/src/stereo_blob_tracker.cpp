#include <stdio.h>
#include <iostream>

#include "ros/node.h"
#include "std_msgs/Image.h"
#include "std_msgs/PointStamped.h"
#include "axis_cam/PTZActuatorCmd.h"
#include "axis_cam/PTZActuatorState.h"
#include "image_utils/cv_bridge.h"
#include "pr2_mechanism_controllers/TrackPoint.h"
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
#define GIVEUPONFAILURE false


//Just some convienience macros
#define CV_CVX_WHITE	CV_RGB(0xff,0xff,0xff)
#define CV_CVX_BLACK	CV_RGB(0x00,0x00,0x00)



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
  IplImage *cv_depthmask_image_cpy_;

  IplImage *depthmask_;
  IplImage *dispImgGT_;
  IplImage *dispImgLT_;
  IplImage *temp_;
  IplConvKernel* dilateKernel_;
  IplConvKernel* openKernel_;
  IplConvKernel* closeKernel_;

  CvMemStorage*	mem_storage_;
  /// approx threshold - the bigger it is, the simpler is the boundary
  static const int CVCONTOUR_APPROX_LEVEL = 2;
  /// how many iterations of erosion and/or dilation
  static const int CVCLOSE_ITR = 1; 

  CvMat    *blobPts_;

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
    node("videre_blob_tracker", ros::node::ANONYMOUS_NAME),
    camModel_(NULL),
    cv_image_cpy_(NULL), 
    cv_dispImg_cpy_(NULL),
    cv_feat_image_cpy_(NULL), 
    cv_backproject_image_cpy_(NULL), 
    cv_mask_image_cpy_(NULL),
    cv_depthmask_image_cpy_(NULL),
    depthmask_(NULL),
    dispImgGT_(NULL),
    dispImgLT_(NULL),
    temp_(NULL),
    dilateKernel_(cvCreateStructuringElementEx(15, 15, 7, 7, CV_SHAPE_RECT)),
    //    structElem2_(cvCreateStructuringElementEx(5, 5, 2, 2, CV_SHAPE_RECT)),
    openKernel_(cvCreateStructuringElementEx(3, 3, 1, 1, CV_SHAPE_RECT)),
    closeKernel_(cvCreateStructuringElementEx(7, 7, 3, 3, CV_SHAPE_RECT)),
    mem_storage_(NULL),
    blobPts_(NULL),
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
    advertise<std_msgs::PointStamped>("head_controller/track_point", 1000);
    // OpenCV: pop up other windows for features, etc.
    display_ = display;
    if (display_) {
      cvNamedWindow("Features", 0);
      cvNamedWindow("Backprojection",0);
      cvNamedWindow("Mask",0);
      cvNamedWindow("DepthMask", 0);
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
    if (depthmask_)   cvReleaseImage(&depthmask_);
    if (dispImgGT_)   cvReleaseImage(&dispImgGT_);
    if (dispImgLT_)   cvReleaseImage(&dispImgLT_);
    if (temp_)        cvReleaseImage(&temp_);
    if (blobPts_)     cvReleaseMat(&blobPts_);
    delete camModel_;

    if (cv_feat_image_cpy_)        cvReleaseImage(&cv_feat_image_cpy_);
    if (cv_backproject_image_cpy_) cvReleaseImage(&cv_backproject_image_cpy_);
    if (cv_mask_image_cpy_)        cvReleaseImage(&cv_mask_image_cpy_);
    if (cv_depthmask_image_cpy_)   cvReleaseImage(&cv_depthmask_image_cpy_);

    if (mem_storage_) cvReleaseMemStorage(&mem_storage_);

    //TODO: how do I release contours_

    cvReleaseStructuringElement(&dilateKernel_);
    cvReleaseStructuringElement(&openKernel_);
    cvReleaseStructuringElement(&closeKernel_);
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

      if (depthmask_ == NULL) {
	depthmask_ =  cvCreateImage(im_size,IPL_DEPTH_8U,1);
      }
      

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

	IplImage *mask = NULL;
	if (blobPts_ != NULL) {
	  // compute the min and max if the blob points in 3D
	  CvMat blobPtsZ;
	  cvGetCol(blobPts_, &blobPtsZ, 2);
	  double minZ, maxZ;
	  cvMinMaxLoc(&blobPtsZ, &minZ, &maxZ);

	  // enlarge the range by .5 meter, or 500mm
	  minZ -= 500;
	  // make sure minZ is at least 100 mm away
	  if (minZ < 100) minZ = 100;
	  maxZ += 500;

	  // the unit of the raw disparity image is 1/4 pixel
	  double dispUnitScale = .25;
	  computeDepthMask(dispImg, depthmask_, dispUnitScale, minZ, maxZ);
	  mask = depthmask_;
	}


	oktrack = btracker_->processFrame(leftImg, g_is_new_blob, g_selection, 
					  &g_new_selection, mask);

	g_is_new_blob = false;
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
	    // as the blob may enter in a very different
	    // depth
	    if (depthmask_) cvReleaseImage(&depthmask_);
	    if (blobPts_  ) cvReleaseMat(&blobPts_);
	    // set the selection blob to be centered of the screen with a
	    // reasonable size (quarter of the window dimensions)
	    int width = im_size.width/4;
	    int height = im_size.height/4;
	    g_new_selection = cvRect(im_size.width/2  - width/2, 
				     im_size.height/2 - height/2,
				     width, height);
	  }
	}
	{
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
	      if (blobPts_) {
		cvReleaseMat(&blobPts_);
	      }
	      blobPts_ = cvCreateMat(numGoodPoints, 3, CV_64FC1);
	      //double _xyzs[numGoodPoints*3];
	      //CvMat xyzs = cvMat( numGoodPoints, 3, CV_64FC1, _xyzs );
	      camModel_->dispToCart(uvds, *blobPts_);
	      CvMat xyzsC3;
	      cvReshape( blobPts_, &xyzsC3, 3, 0);
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
	      
	      string topic = "head_controller/track_point" ;
      
	      std_msgs::PointStamped target;

	      target.point.x = point_stamped_.point.z ;
	      target.point.y = -point_stamped_.point.x ;
	      target.point.z = -point_stamped_.point.y;
        
	      target.header.frame_id = "stereo_block" ;
        
#if 0
	      cout << "Requesting to move to: " << point_stamped_.point.x << ","
                                        << point_stamped_.point.y << ","
                                        << point_stamped_.point.z << endl ;
#endif
        
	      static int count = 0;
	      if (++count % 1 == 0)
		publish(topic, target);
	      
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

	  if (depthmask_) {
	    if (cv_depthmask_image_cpy_ == NULL) {
	      cv_depthmask_image_cpy_ = cvCreateImage(im_size,IPL_DEPTH_8U,1);
	    }
	    cvCopy(depthmask_, cv_depthmask_image_cpy_);
	  }

	}

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
      camModel_ = new CvStereoCamModel(Fx, Fy, Tx, Clx, Crx, Cy);
    }
  }


  /// The following function is modified based on Gary Bradsky's function cvconnectedComponents in cv_yuv_codebook.cpp
  ///////////////////////////////////////////////////////////////////////////////////////////
  //void cvconnectedComponents(IplImage *mask, int poly1_hull0, float perimScale, int *num, CvRect *bbs, CvPoint *centers)
  // This cleans up the forground segmentation mask derived from calls to cvbackgroundDiff
  //
  // mask			Is a grayscale (8 bit depth) "raw" mask image which will be cleaned up
  //
  // OPTIONAL PARAMETERS:
  // @param poly1_hull0	If set, approximate connected component by (DEFAULT) polygon, or else convex hull (0)
  // @param perimScale 	Len = image (width+height)/perimScale.  If contour len < this, delete that contour (DEFAULT: 4)
  // @param openKernel   the kernel used to remove noise in raw mask. If NULL, 3x3 rectangle is used
  // @param closeKernel  the kernel used to merge blobs in raw mask. If NULL, 3x3 rectangle is used
  // @param num			Maximum number of rectangles and/or centers to return, on return, will contain number filled (DEFAULT: NULL)
  // @param bbs			Pointer to bounding box rectangle vector of length num.  (DEFAULT SETTING: NULL)
  // @param centers		Pointer to contour centers vectors of length num (DEFAULT: NULL)
  //
  void connectedComponents(IplImage *mask, int poly1_hull0, 
			   IplConvKernel* openKernel,
			   IplConvKernel* closeKernel,
			   float perimScale, int *num, CvRect *bbs, CvPoint *centers)
  {
    //CLEAN UP RAW MASK
    // open followed by a close
    cvMorphologyEx( mask, mask, NULL, openKernel,  CV_MOP_OPEN,  CVCLOSE_ITR );
    cvMorphologyEx( mask, mask, NULL, closeKernel, CV_MOP_CLOSE, CVCLOSE_ITR );

    //FIND CONTOURS AROUND ONLY BIGGER REGIONS
    if( mem_storage_==NULL ) mem_storage_ = cvCreateMemStorage(0);
    else cvClearMemStorage(mem_storage_);

    CvContourScanner scanner = cvStartFindContours(mask,mem_storage_,sizeof(CvContour),CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    CvSeq* c;
    int numCont = 0;
    while( (c = cvFindNextContour( scanner )) != NULL )
      {
	double len = cvContourPerimeter( c );
	double q = (mask->height + mask->width) /perimScale;   //calculate perimeter len threshold
	if( len < q ) //Get rid of blob if it's perimeter is too small
	  {
	    cvSubstituteContour( scanner, NULL );
	  }
	else //Smooth it's edges if it's large enough
	  {
	    CvSeq* c_new;
	    if(poly1_hull0) //Polygonal approximation of the segmentation
	      c_new = cvApproxPoly(c,sizeof(CvContour),mem_storage_,CV_POLY_APPROX_DP, CVCONTOUR_APPROX_LEVEL,0);
	    else //Convex Hull of the segmentation
	      c_new = cvConvexHull2(c,mem_storage_,CV_CLOCKWISE,1);
            cvSubstituteContour( scanner, c_new );
	    numCont++;
	  }
      }
    CvSeq* contours = cvEndFindContours( &scanner );

    // PAINT THE FOUND REGIONS BACK INTO THE IMAGE
    cvZero( mask );
    IplImage *maskTemp;
    //CALC CENTER OF MASS AND OR BOUNDING RECTANGLES
    if(num != NULL)
      {
	int N = *num, numFilled = 0, i=0;
	CvMoments moments;
	double M00, M01, M10;
	maskTemp = cvCloneImage(mask);
	for(i=0, c=contours; c != NULL; c = c->h_next,i++ )
	  {
	    if(i < N) //Only process up to *num of them
	      {
		cvDrawContours(maskTemp,c,CV_CVX_WHITE, CV_CVX_WHITE,-1,CV_FILLED,8);
		//Find the center of each contour
		if(centers != NULL)
		  {
		    cvMoments(maskTemp,&moments,1);
		    M00 = cvGetSpatialMoment(&moments,0,0);
		    M10 = cvGetSpatialMoment(&moments,1,0);
		    M01 = cvGetSpatialMoment(&moments,0,1);
		    centers[i].x = (int)(M10/M00);
		    centers[i].y = (int)(M01/M00);
		  }
		//Bounding rectangles around blobs
		if(bbs != NULL)
		  {
		    bbs[i] = cvBoundingRect(c);
		  }
		cvZero(maskTemp);
		numFilled++;
	      }
	    //Draw filled contours into mask
	    cvDrawContours(mask,c,CV_CVX_WHITE,CV_CVX_WHITE,-1,CV_FILLED,8); //draw to central mask
	  } //end looping over contours
	*num = numFilled;
	cvReleaseImage( &maskTemp);
      }
    //ELSE JUST DRAW PROCESSED CONTOURS INTO THE MASK
    else
      {
	for( c=contours; c != NULL; c = c->h_next )
	  {
	    cvDrawContours(mask,c,CV_CVX_WHITE, CV_CVX_BLACK,-1,CV_FILLED,8);
	  }
      }
  }

  void computeDepthMask(IplImage* dispImg, IplImage* depthMask, 
			double dispUnitScale,
			double minZ, double maxZ){
    double maxDisp = camModel_->getDisparity(minZ);
    double minDisp = camModel_->getDisparity(maxZ);

    maxDisp /= dispUnitScale;
    minDisp /= dispUnitScale;

    // printf("range mask [%f, %f] => [%f, %f]\n", minZ, maxZ, minDisp, maxDisp);

    // fill in the mask according to disparity or depth
    cvInRangeS(dispImg, cvScalar(minDisp), cvScalar(maxDisp), depthMask);

#if 1 
    // two simple morphology operation seem to be good enough. But
    // but connected component analysis provides blob with better shape
    cvMorphologyEx(depthMask, depthMask, NULL, openKernel_, CV_MOP_OPEN, 1);
    //cvMorphologyEx(depthMask, depthMask, NULL, dilateKernel_, CV_MOP_CLOSE, 1);
    cvDilate(depthMask, depthMask, dilateKernel_, 1);
#else

    const float perimScale = 16;
    const int maxNumBBoxes = 25;
    CvRect bboxes[maxNumBBoxes];
    int numCComp = maxNumBBoxes;
    connectedComponents(depthMask, 0, 
			openKernel_,
			closeKernel_,
			perimScale, &numCComp, 
			(CvRect *)NULL, // bboxes, 
			(CvPoint *)NULL);

    //printf("found %d blobs\n", numCComp);

    cvDilate(depthMask, depthMask, dilateKernel_, 1);
#endif
  }
    
};


// Main
int main(int argc, char **argv)
{
  ros::init(argc, argv);

  VidereBlobTracker v(BLOBNEARCENTER, DISPLAY);
  v.spin();

  ros::fini();
  return 0;
}
