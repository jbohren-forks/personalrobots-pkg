/*********************************************************************
* A ros node to run face detection and colour-based face tracking.
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
#include "colorcalib.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "people.h"

#define MYDEBUG 0
#define DISPLAY 1

using namespace std;

// StereoFaceColorTracker - Face detection and color-based tracking using the videre cams.

class StereoFaceColorTracker: public ros::node {
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
  
  bool calib_color_;
  CvMat* color_calib_mat_;

  People *people_;
  const char *haar_filename_;

  bool quit_;
  int detect_;

  IplImage *X_, *Y_, *Z_;
  CvMat *uvd_, *xyz_;

  ros::thread::mutex cv_mutex_;

  StereoFaceColorTracker(const char *haar_filename, bool use_depth, bool calib_color) : 
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
    calib_color_(false),
    color_calib_mat_(NULL),
    people_(NULL),
    haar_filename_(haar_filename),
    quit_(false),
    detect_(0),
    X_(NULL),
    Y_(NULL),
    Z_(NULL),
    uvd_(NULL),
    xyz_(NULL)
  { 

#if DISPLAY
    // OpenCV: pop up an OpenCV highgui window
    cvNamedWindow("Disparity",CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Face Detection", CV_WINDOW_AUTOSIZE);
#endif

    calib_color_ = calib_color;

    people_ = new People();

    // Subscribe to image
    subscribe("videre/images", image_msg_, &StereoFaceColorTracker::image_cb, 1);


    // Subscribe to calibration parameters
    subscribe("videre/cal_params", cal_params_, &StereoFaceColorTracker::cal_params_cb, 1);

  }

  ~StereoFaceColorTracker()
  {

    cvReleaseImage(&cv_image_left_);
    cvReleaseImage(&cv_image_disp_);
    cvReleaseImage(&cv_image_left_cpy_);
    cvReleaseImage(&cv_image_disp_cpy_);

#if DISPLAY
    cvDestroyWindow("Face Detection");
    cvDestroyWindow("Disparity");
#endif

    delete cam_model_;

    cvReleaseMat(&color_calib_mat_);

    cvReleaseImage(&X_);
    cvReleaseImage(&Y_);
    cvReleaseImage(&Z_);
    cvReleaseMat(&uvd_);
    cvReleaseMat(&xyz_);

    delete people_;

    if (built_bridge_) {
      delete cv_bridge_left_;
      delete cv_bridge_disp_;
    }


  }

  /// The image callback. For each new image, copy it, perform face detection or track it, and draw the rectangles on the image.
  void image_cb()
  {

    // Set color calibration.
    if (calib_color_) {
      // Exit if color calibration hasn't been performed.
      std::string color_cal_str = std::string("videre/images/") + image_msg_.images[1].label + std::string("/color_cal");
      cout << color_cal_str;
      if (!has_param(color_cal_str)) {
	printf("No params\n");
	return;
      }
      // Otherwise, set the color calibration matrix.
      if (!color_calib_mat_) {
	color_calib_mat_ = cvCreateMat(3,3,CV_32FC1);
	cvSetZero(color_calib_mat_);
      }
      XmlRpc::XmlRpcValue xml_color_cal;
      get_param(color_cal_str, xml_color_cal);
      for (int i=0; i<3; i++) {
	for (int j=0; j<3; j++) {
	  cvmSet(color_calib_mat_,i,j,(double)(xml_color_cal[3*i+j]));
	}
      }
    }

    // Exit if the camera model params haven't yet arrived.
    if (cam_model_==NULL) {
      return;
    }

    CvSize im_size;

    // Set up the cv bridges, should only run once.
    if (!built_bridge_) {
      cv_bridge_left_ = new CvBridge<std_msgs::Image>(&image_msg_.images[1],  
						      CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U);
      cv_bridge_disp_ = new CvBridge<std_msgs::Image>(&image_msg_.images[0], 
						      CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U);
      built_bridge_ = true;
    }
 
    // Convert the images to opencv format.
    if (cv_image_left_) {
      cvReleaseImage(&cv_image_left_);
      cvReleaseImage(&cv_image_disp_);
    }
    cv_bridge_left_->to_cv(&cv_image_left_);
    cv_bridge_disp_->to_cv(&cv_image_disp_);

    if ( !cv_image_left_ )  {
      return;
    }
    // Check that the image is in color.
    if (cv_image_left_->nChannels!=3) {
      printf("The left image is not a 3-channel color image.\n");
      return;
    }

    im_size = cvGetSize(cv_image_left_);

    // Calibrate color if requested.
    if (calib_color_) {
      IplImage* t_img = cvCreateImage(im_size, IPL_DEPTH_32F, 3);
      decompand(cv_image_left_, t_img);
      cvTransform(t_img, t_img, color_calib_mat_);
      compand(t_img, cv_image_left_);
      cvReleaseImage(&t_img);
    }

    int npeople = people_->getNumPeople();
    if (!X_) {
      X_ = cvCreateImage( im_size, IPL_DEPTH_32F, 1);
      Y_ = cvCreateImage( im_size, IPL_DEPTH_32F, 1);
      Z_ = cvCreateImage( im_size, IPL_DEPTH_32F, 1);	
      xyz_ = cvCreateMat(im_size.width*im_size.height,3,CV_32FC1);
      uvd_ = cvCreateMat(im_size.width*im_size.height,3,CV_32FC1);
    }

    CvSize roi_size;
    double x_size, y_size,d;
    CvScalar avgz;
    CvMat *xyzpts = cvCreateMat(2,3,CV_32FC1), *uvdpts = cvCreateMat(2,3,CV_32FC1) ;
    if (npeople == 0) {
      vector<CvRect> faces_vector = people_->detectAllFaces(cv_image_left_, haar_filename_, 1.0, cv_image_disp_, cam_model_, true);
#if MYDEBUG
      printf("Detected faces\n");
#endif

#if 0
      cvResetImageROI(cv_image_disp_);
      cvSetZero(X_);
      cvSetZero(Y_);
      cvSetZero(Z_);
      cam_model_->disp8UToCart32F(cv_image_disp_, (float)1.0, (float)MAX_Z_MM, Z_, X_, Y_); ///
      float *fptrz = (float*)(Z_->imageData + 300*Z_->width);
      float *fptrx = (float*)(X_->imageData + 300*X_->width);
      float *fptry = (float*)(Y_->imageData + 300*Y_->width);
      while (*fptrz == 0.0) {
	fptrz++; fptrx++; fptry++;
      }
      printf("\n%f %f %f\n", *fptrx, *fptry, *fptrz);
#endif
#if 1
      float* fptr = (float*)(uvd_->data.ptr);
      uchar* cptr = (uchar*)(cv_image_disp_->imageData);
      for (int v =0; v < im_size.height; v++) {
	for (int u=0; u<im_size.width; u++) {
	  *fptr = u; fptr++;
	  *fptr = v; fptr++;
	  *fptr = *cptr; cptr++; fptr++;
	}
      }
      fptr = NULL;
      cam_model_->dispToCart(uvd_,xyz_);

      float *fptrx = (float*)(X_->imageData);
      float *fptry = (float*)(Y_->imageData);
      float *fptrz = (float*)(Z_->imageData);
      fptr = (float*)(xyz_->data.ptr);
      for (int v =0; v < im_size.height; v++) {
	for (int u=0; u<im_size.width; u++) {
	  *fptrx = *fptr; fptrx++; fptr++;
	  *fptry = *fptr; fptry++; fptr++;
	  *fptrz = *fptr; fptrz++; fptr++;
	}
      }

#endif
      for (unsigned int iface = 0; iface < faces_vector.size(); iface++) {
	people_->addPerson();
	people_->setFaceBbox2D(faces_vector[iface],iface);

	// Take the average valid Z within the face bounding box. Invalid values are 0.
	CvRect tface = cvRect(faces_vector[iface].x+4, faces_vector[iface].y+4, faces_vector[iface].width-8, faces_vector[iface].height-8);
	cvSetImageROI(Z_,tface);
	avgz = cvSum(Z_);
	avgz.val[0] /= cvCountNonZero(Z_);
	cvResetImageROI(Z_);

	// Get the two diagonal corners of the bounding box in the camera frame.
	// Since not all pts will have x,y,z values, we'll take the average z, convert it to d,
	// and the real u,v values to approximate the corners.
	d = cam_model_->getDisparity(avgz.val[0]);
	cvmSet(uvdpts,0,0, faces_vector[iface].x);
	cvmSet(uvdpts,0,1, faces_vector[iface].y);
	cvmSet(uvdpts,0,2, d);
	cvmSet(uvdpts,1,0, faces_vector[iface].x+faces_vector[iface].width-1);
	cvmSet(uvdpts,1,1, faces_vector[iface].y+faces_vector[iface].height-1);
	cvmSet(uvdpts,1,2, d);
	cam_model_->dispToCart(uvdpts,xyzpts);

	x_size = (cvmGet(xyzpts,1,0)-cvmGet(xyzpts,0,0))/2.0;
	y_size = (cvmGet(xyzpts,1,1)-cvmGet(xyzpts,0,1))/2.0;
	people_->setFaceCenter3D((cvmGet(xyzpts,1,0)+cvmGet(xyzpts,0,0))/2.0,
				 (cvmGet(xyzpts,1,1)+cvmGet(xyzpts,0,1))/2.0,
				 cvmGet(xyzpts,0,2), iface);
	people_->setFaceSize3D((x_size>y_size) ? x_size : y_size , iface);      

#if MYDEBUG
	printf("face opp corners 2d %d %d %d %d\n",
	       faces_vector[iface].x,faces_vector[iface].y,
	       faces_vector[iface].x+faces_vector[iface].width-1, 
	       faces_vector[iface].y+faces_vector[iface].height-1);
	printf("3d center %f %f %f\n", (cvmGet(xyzpts,1,0)+cvmGet(xyzpts,0,0))/2.0, (cvmGet(xyzpts,1,1)+cvmGet(xyzpts,0,1))/2.0,cvmGet(xyzpts,0,2));
	printf("3d size %f\n",people_->getFaceSize3D(iface));
	printf("Z within the face box:\n");
	  
	for (int v=faces_vector[iface].y; v<=faces_vector[iface].y+faces_vector[iface].height; v++) {
	  for (int u=faces_vector[iface].x; u<=faces_vector[iface].x+faces_vector[iface].width; u++) {
	    printf("%4.0f ",cvGetReal2D(Z,v,u));
	  }
	  printf("\n");
	}
#endif
      }
    }  

    npeople = people_->getNumPeople();

    if (npeople==0) {
      // No people to track, try again later.
      return;
    }

    CvMat *end_points = cvCreateMat(npeople,3,CV_32FC1);
    bool did_track = people_->track_color_3d_bhattacharya(cv_image_left_, cv_image_disp_, cam_model_, 0, NULL, NULL, end_points);
    if (!did_track) {
      // If tracking failed, just return.
      return;
    }
    // Copy endpoints to the people. This is temporary, eventually you might have something else sending a new location as well.
    // Also copy the new 2d bbox to the person.
    // And draw the points on the image.
    CvMat *four_corners_3d = cvCreateMat(4,3,CV_32FC1);
    CvMat *four_corners_2d = cvCreateMat(4,3,CV_32FC1); 
    CvMat *my_end_point = cvCreateMat(1,3,CV_32FC1);
    CvMat *my_size = cvCreateMat(1,2,CV_32FC1);

    for (int iperson = 0; iperson < npeople; iperson++) {
      people_->setFaceCenter3D(cvmGet(end_points,iperson,0),cvmGet(end_points,iperson,1),cvmGet(end_points,iperson,2),iperson);
      // Convert the new center and size to a 3d rect that is parallel with the camera plane.
      cvSet(my_size,cvScalar(people_->getFaceSize3D(iperson)));
      cvGetRow(end_points,my_end_point,iperson);
      people_->centerSizeToFourCorners(my_end_point,my_size, four_corners_3d);
      for (int icorner = 0; icorner < 4; icorner++) {
	cvmSet(four_corners_3d,icorner,2,cvmGet(my_end_point,0,2));
      }
      cam_model_->cartToDisp(four_corners_3d, four_corners_2d);
      people_->setFaceBbox2D(cvRect(cvmGet(four_corners_2d,0,0),cvmGet(four_corners_2d,0,1),
				    cvmGet(four_corners_2d,1,0)-cvmGet(four_corners_2d,0,0),
				    cvmGet(four_corners_2d,2,1)-cvmGet(four_corners_2d,0,1)),
			     iperson);
      cvRectangle(cv_image_left_, 
		  cvPoint(cvmGet(four_corners_2d,0,0),cvmGet(four_corners_2d,0,1)), 
		  cvPoint(cvmGet(four_corners_2d,3,0),cvmGet(four_corners_2d,3,1)),
		  cvScalar(255,255,255));      
    }
	  
    cvReleaseMat(&end_points);
    cvReleaseMat(&four_corners_3d);
    cvReleaseMat(&four_corners_2d);
    cvReleaseMat(&my_end_point);
    cvReleaseMat(&my_size);     

#if DISPLAY
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
#endif
          
      
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
#if DISPLAY
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
#endif
    }
    return true;
  } 

};


// Main
int main(int argc, char **argv)
{
  ros::init(argc, argv);
  bool use_depth = true;
  bool color_calib = true;

#if 0
  if (argc < 2) {
    cerr << "Path to cascade file required.\n" << endl;
    return 0;
  }
#endif
  //char *haar_filename = argv[1]; 
  char haar_filename[] = "./cascades/haarcascade_frontalface_alt.xml";
  //char haar_filename[] = "./cascades/haarcascade_profileface.xml";
  //char haar_filename[] = "./cascades/haarcascade_upperbody.xml";
  StereoFaceColorTracker sfct(haar_filename, use_depth, color_calib);
 
  sfct.spin();


  ros::fini();
  return 0;
}


