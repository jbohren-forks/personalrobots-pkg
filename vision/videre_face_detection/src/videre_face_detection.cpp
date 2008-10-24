#include <stdio.h>
#include <iostream>

#include "ros/node.h"
#include "std_msgs/ImageArray.h"
#include "std_msgs/String.h"
#include "image_utils/cv_bridge.h"
#include "CvStereoCamModel.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "people.h"

//typedef signed char schar;
using namespace std;
//using namespace cv::willow;

// VidereFaceDetector - Face detection in the videre cams. A wrapper around OpenCV's face detection, plus some usage of depth to restrict the search.

class VidereFaceDetector: public ros::node {
public:
  // Images and conversion
  std_msgs::ImageArray image_msg_;
  std_msgs::String cal_params_;
  CvBridge<std_msgs::Image> *cv_bridge_left_;
  CvBridge<std_msgs::Image> *cv_bridge_disp_;
  bool built_bridge_;

  // The left and disparity images.
  IplImage *cv_image_left_;
  IplImage *cv_image_disp_;

  // Display copies of the images.
  IplImage *cv_image_left_cpy_;
  IplImage *cv_image_disp_cpy_;

  bool use_depth_;
  CvStereoCamModel *cam_model_;

  People *people_;
  const char *haar_filename_;

  bool quit_;
  int detect_;

  ros::thread::mutex cv_mutex_;

  VidereFaceDetector(const char *haar_filename, bool use_depth) : 
    node("videre_face_detector", ros::node::ANONYMOUS_NAME),
    cv_bridge_left_(NULL),
    cv_bridge_disp_(NULL),
    built_bridge_(false),
    cv_image_left_(NULL),
    cv_image_disp_(NULL),
    cv_image_left_cpy_(NULL), 
    cv_image_disp_cpy_(NULL),
    use_depth_(use_depth),
    cam_model_(NULL),
    people_(NULL),
    haar_filename_(haar_filename),
    quit_(false),
    detect_(0)
  { 

    // OpenCV: pop up an OpenCV highgui window
    cvNamedWindow("Disparity",CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Face Detection", CV_WINDOW_AUTOSIZE);

    people_ = new People();

    // Subscribe to image
    subscribe("videre/images", image_msg_, &VidereFaceDetector::image_cb, 1);


    // Subscribe to calibration parameters
    subscribe("videre/cal_params", cal_params_, &VidereFaceDetector::cal_params_cb, 1);

  }

  ~VidereFaceDetector()
  {

    cvReleaseImage(&cv_image_left_);
    cvReleaseImage(&cv_image_disp_);
    cvReleaseImage(&cv_image_left_cpy_);
    cvReleaseImage(&cv_image_disp_cpy_);
    //    cvReleaseImage(&cv_image_gray_);
    
    //  cvReleaseMemStorage( &storage_ );
    //cvReleaseHaarClassifierCascade(&cascade_);

    cvDestroyWindow("Face Detection");

    delete cam_model_;

    delete people_;

    if (built_bridge_) {
      delete cv_bridge_left_;
      delete cv_bridge_disp_;
    }


  }

  /// The image callback. For each new image, copy it, perform face detection, and draw the rectangles on the image.
  void image_cb()
  {

    detect_++;
    if (detect_ % 2) {
      return;
    }
 
    CvSize im_size;

    // Set up the cv bridges, should only run once.
    if (!built_bridge_) {
      cv_bridge_left_ = new CvBridge<std_msgs::Image>(&image_msg_.images[1],  CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U);
      cv_bridge_disp_ = new CvBridge<std_msgs::Image>(&image_msg_.images[0],  CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U);
      built_bridge_ = true;
    }
 
    // Convert the images to opencv format.
    if (cv_image_left_) {
      cvReleaseImage(&cv_image_left_);
      cvReleaseImage(&cv_image_disp_);
    }
    cv_bridge_left_->to_cv(&cv_image_left_);
    cv_bridge_disp_->to_cv(&cv_image_disp_);
 
    if ( cv_image_left_ )  {
      im_size = cvGetSize(cv_image_left_);

      vector<CvRect> faces_vector = people_->detectAllFaces(cv_image_left_, haar_filename_, 1.0, cv_image_disp_, cam_model_, true);
 
      // Copy all of the images you might want to display.
      // This is necessary because OpenCV doesn't like multiple threads.
      cv_mutex_.lock();

      if (cv_image_left_cpy_ == NULL) {
	cv_image_left_cpy_ = cvCreateImage(im_size,IPL_DEPTH_8U,3);
      }
      cvCopy(cv_image_left_, cv_image_left_cpy_);

      if (cv_image_disp_cpy_==NULL) {
	cv_image_disp_cpy_ = cvCreateImage(im_size,IPL_DEPTH_8U,1);
      }
      cvCopy(cv_image_disp_, cv_image_disp_cpy_);

      cv_mutex_.unlock();
    }
  }

  // Calibration parameters callback
  void cal_params_cb() {
    parseCaliParams(cal_params_.data);
  }

  /// JD's small parser to pick up the projection matrix from the
  /// calibration message. This should really be somewhere else, this code is copied in multiple files.
  void parseCaliParams(const string& cal_param_str){

    if (cam_model_==NULL) {
      const string labelRightCamera("[right camera]");
      const string labelRightCamProj("proj");
      const string labelRightCamRect("rect");
      // move the current position to the section of "[right camera]"
      size_t rightCamSection = cal_param_str.find(labelRightCamera);
      // move the current position to part of proj in the section of "[right camera]"
      size_t rightCamProj = cal_param_str.find(labelRightCamProj, rightCamSection);
      // get the position of the word "rect", which is also the end of the projection matrix
      size_t rightCamRect = cal_param_str.find(labelRightCamRect, rightCamProj);
      // the string after the word "proj" is the starting of the matrix
      size_t matrix_start = rightCamProj + labelRightCamProj.length();
      // get the sub string that contains the matrix
      string mat_str = cal_param_str.substr(matrix_start, rightCamRect-matrix_start);
      // convert the string to a double array of 12
      stringstream sstr(mat_str);
      double matdata[12];
      for (int i=0; i<12; i++) {
	sstr >> matdata[i];
      }

      //if (cam_model_ == NULL) {
      double Fx  = matdata[0]; // 0,0
      double Fy  = matdata[5]; // 1,1
      double Crx = matdata[2]; // 0,2
      double Cy  = matdata[6]; // 1,2
      double Clx = Crx; // the same
      double Tx  = - matdata[3]/Fx;
      std::cout << "base length "<< Tx << std::endl;
      cam_model_ = new CvStereoCamModel(Fx, Fy, Tx, Clx, Crx, Cy);
    }
  }


  // Wait for completion, wait for user input, display images.
  bool spin() {
    while (ok() && !quit_) {

	// Display all of the images.
	cv_mutex_.lock();
	if (cv_image_left_cpy_)
	  cvShowImage("Face Detection",cv_image_left_cpy_);
	if (cv_image_disp_cpy_)
	  cvShowImage("Disparity",cv_image_disp_cpy_);

	cv_mutex_.unlock();

	// Get user input and allow OpenCV to refresh windows.
	int c = cvWaitKey(2);
	c &= 0xFF;
	// Quit on ESC, "q" or "Q"
	if((c == 27)||(c == 'q')||(c == 'Q'))
	  quit_ = true;

    }
    return true;
  } 

};


// Main
int main(int argc, char **argv)
{
  ros::init(argc, argv);
  bool use_depth = true;

  if (argc < 2) {
    cerr << "Path to cascade file required.\n" << endl;
    return 0;
  }
  char *haar_filename = argv[1]; //"./cascades/haarcascade_frontalface_alt.xml";
  //char haar_filename[] = "./cascades/haarcascade_profileface.xml";
  //char haar_filename[] = "./cascades/haarcascade_upperbody.xml";
  VidereFaceDetector fd(haar_filename, use_depth);
  //if (!fd.cascade_) {
  //  ros::fini();
  //  return 1;
  //}
  fd.spin();


  ros::fini();
  return 0;
}


