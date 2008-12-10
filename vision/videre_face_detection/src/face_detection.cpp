/*********************************************************************
* A ros node to run face detection with images from the videre cameras.
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

#include "ros/node.h"
#include "std_msgs/ImageArray.h"
#include "std_msgs/String.h"
#include "image_utils/cv_bridge.h"
#include "CvStereoCamModel.h"
#include <robot_msgs/PositionMeasurement.h>

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "people.h"


using namespace std;

// FaceDetector - Face detection using the videre cams. A wrapper around OpenCV's face detection, plus some usage of depth to restrict the search.

class FaceDetector: public ros::node {
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

  FaceDetector(const char *haar_filename, bool use_depth) : 
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
    subscribe("videre/images", image_msg_, &FaceDetector::image_cb, 1);


    // Subscribe to calibration parameters
    subscribe("videre/cal_params", cal_params_, &FaceDetector::cal_params_cb, 1);

    // Advertise a position measure message.
    advertise<robot_msgs::PositionMeasurement>("face_detection/position_measurement",1);
    //subscribe<robot_msgs::PositionMeasurement>("face_detection",pos,&FaceDetector::pos_cb,1);

  }

  ~FaceDetector()
  {

    cvReleaseImage(&cv_image_left_);
    cvReleaseImage(&cv_image_disp_);
    cvReleaseImage(&cv_image_left_cpy_);
    cvReleaseImage(&cv_image_disp_cpy_);

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
      

      // Get the average disparity in the middle half of the bounding box, and compute the face center in 3d. Publish the face center as a track point.
      if (cv_image_disp_ && cam_model_) {
	int r, c, good_pix;
	uchar* ptr;
	double avg_disp;
	CvRect *one_face;
	robot_msgs::PositionMeasurement pos;
	CvMat *uvd = cvCreateMat(1,3,CV_32FC1);
	CvMat *xyz = cvCreateMat(1,3,CV_32FC1);
	for (uint iface = 0; iface < faces_vector.size(); iface++) {
	  one_face = &faces_vector[iface];
	  good_pix = 0;
	  avg_disp = 0;
	  for (r = floor(one_face->y+0.25*one_face->height); r < floor(one_face->y+0.75*one_face->height); r++) {
	    ptr = (uchar*)(cv_image_disp_->imageData + r*cv_image_disp_->widthStep);
	    for (c = floor(one_face->x+0.25*one_face->width); c < floor(one_face->x+0.75*one_face->width); c++) {
	      if (ptr[c] > 0) {
		avg_disp += ptr[c];
		good_pix++;
	      }
	    }
	  }
	  avg_disp /= (double)good_pix; // Take the average.
	  cvmSet(uvd,0,0,one_face->x+one_face->width/2.0);
	  cvmSet(uvd,0,1,one_face->y+one_face->height/2.0);
	  cvmSet(uvd,0,2,avg_disp);
	  cam_model_->dispToCart(uvd,xyz);
	  pos.header.stamp = image_msg_.header.stamp;
	  pos.name = "face_detection";
	  pos.object_id = -1;
	  pos.pos.x = cvmGet(xyz,0,2);
	  pos.pos.y = -1.0*cvmGet(xyz,0,0);
	  pos.pos.z = -1.0*cvmGet(xyz,0,1);
	  pos.header.frame_id = "stereo_link";
	  pos.reliability = 0.8;
	  pos.initialization = 0;
	  //pos.covariance = ;
	  publish("face_detection/position_measurement",pos);
	}
	cvReleaseMat(&uvd);
	cvReleaseMat(&xyz);
      }
 
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
      cam_model_ = new CvStereoCamModel(Fx, Fy, Tx, Clx, Crx, Cy, 0.25);
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
  FaceDetector fd(haar_filename, use_depth);
 
  fd.spin();


  ros::fini();
  return 0;
}


