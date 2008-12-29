/*********************************************************************
 * A ros node to run face detection and colour-based face tracking.
 *
 **********************************************************************
 *
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc
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
#include "CvStereoCamModel.h"
#include "color_calib.h"
#include <robot_msgs/PositionMeasurement.h>
#include "image_msgs/StereoInfo.h"
#include "image_msgs/CamInfo.h"
#include "image_msgs/Image.h"
#include "image_msgs/CvBridge.h"
#include "topic_synchronizer.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "people.h"
#include "utils.h"

#define __FACE_COLOR_TRACKER_DEBUG__ 0
#define __FACE_COLOR_TRACKER_DISPLAY__ 1

using namespace std;

// StereoFaceColorTracker - Color histogram-based tracking from stereo cameras.

class StereoFaceColorTracker: public ros::node {
public:
  // Images and conversion
  image_msgs::Image limage_;
  image_msgs::Image dimage_;
  image_msgs::StereoInfo stinfo_;
  image_msgs::CamInfo rcinfo_;
  image_msgs::CvBridge lbridge_;
  image_msgs::CvBridge dbridge_;
  color_calib::Calibration lcolor_cal_;
  TopicSynchronizer<StereoFaceColorTracker> sync_;
  ros::Time last_image_time_;

  // The left and disparity images.
  IplImage *cv_image_left_;
  IplImage *cv_image_disp_;
  IplImage *cv_image_disp_out_;

  bool use_depth_;
  CvStereoCamModel *cam_model_;
  
  bool calib_color_;

  People *people_;
  bool detect_faces_;
  robot_msgs::PositionMeasurement init_pos_;
  const char *haar_filename_;

  bool quit_;
  int detect_;

  IplImage *X_, *Y_, *Z_;
  CvMat *uvd_, *xyz_;

  ros::thread::mutex cv_mutex_;

  StereoFaceColorTracker(bool detect_faces, const char *haar_filename, bool use_depth, bool calib_color) : 
    ros::node("stereo_face_color_tracker",ros::node::ANONYMOUS_NAME),
    lcolor_cal_(this),
    sync_(this, &StereoFaceColorTracker::image_cb_all, ros::Duration().fromSec(0.05), &StereoFaceColorTracker::image_cb_timeout),
    last_image_time_(ros::Time().fromSec(0)),
    cv_image_left_(NULL),
    cv_image_disp_(NULL),
    cv_image_disp_out_(NULL),
    use_depth_(use_depth),
    cam_model_(NULL),
    calib_color_(false),
    people_(NULL),
    detect_faces_(detect_faces),
    haar_filename_(haar_filename),
    quit_(false),
    detect_(0),
    X_(NULL),
    Y_(NULL),
    Z_(NULL),
    uvd_(NULL),
    xyz_(NULL)
  { 

#if  __FACE_COLOR_TRACKER_DISPLAY__
    // OpenCV: pop up an OpenCV highgui window
    cvNamedWindow("Face color tracker: Disparity",CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Face color tracker: Face Detection", CV_WINDOW_AUTOSIZE);
#endif

    calib_color_ = calib_color;

    people_ = new People();

    // Subscribe to the images and parameters
    sync_.subscribe("stereodcam/left/image_rect_color",limage_,1);
    sync_.subscribe("stereodcam/disparity",dimage_,1);
    sync_.subscribe("stereodcam/stereo_info",stinfo_,1);
    sync_.subscribe("stereodcam/right/cam_info",rcinfo_,1);

    // Advertise a 3d position measurement for each head.
    advertise<robot_msgs::PositionMeasurement>("person_measurement",1);

    if (!detect_faces_) {
      subscribe<robot_msgs::PositionMeasurement>("person_measurement",init_pos_, &StereoFaceColorTracker::init_pos_cb,1);
    }

  }

  ~StereoFaceColorTracker()
  {

    cvReleaseImage(&cv_image_disp_out_); cv_image_disp_out_ = 0;

#if  __FACE_COLOR_TRACKER_DISPLAY__
    cvDestroyWindow("Face color tracker: Face Detection");
    cvDestroyWindow("Face color tracker: Disparity");
#endif

    delete cam_model_;

    cvReleaseImage(&X_); X_ = 0;
    cvReleaseImage(&Y_); Y_ = 0;
    cvReleaseImage(&Z_); Z_ = 0;
    cvReleaseMat(&uvd_); uvd_ = 0;
    cvReleaseMat(&xyz_); xyz_ = 0;

    delete people_;
  }


  // Callback to (re)initialize the location of a face.
  void init_pos_cb() {

    // Check that this is the right message, there are multiple people publishing "person_measurement"s.
    if (init_pos_.name != "track_starter_gui") {
      return;
    }

    cv_mutex_.lock();

    printf("got init pos\n");

    // Check that the timestamp is close to the current time. If not, just wait for another message.
    // (The time is in nanosecs, right?)
    if ((init_pos_.header.stamp - last_image_time_) < ros::Duration().fromSec(-10.0)) {
      cv_mutex_.unlock();
      return;
    }

    // Check that we have the camera parameters. Otherwise, we can't convert the position
    // to image coords anyway, so we might as well ignore this message.
    if (cam_model_ == NULL) {
      cv_mutex_.unlock();
      return;
    }

    // Find the person this position message refers to.
    int iperson = people_->findID(init_pos_.object_id);
    CvMat *four_corners_3d = cvCreateMat(4,3,CV_32FC1);
    CvMat *four_corners_2d = cvCreateMat(4,3,CV_32FC1);
    CvMat *center_3d = cvCreateMat(1,3,CV_32FC1);
    CvMat *size_3d = cvCreateMat(1,3,CV_32FC1);
    if (iperson == -1) {
      // This is a new person, create them. Their histogram will be sent on the first tracking frame.
      people_->addPerson();
      iperson = people_->getNumPeople()-1;
      people_->setID(init_pos_.object_id,iperson);
      people_->setFaceSize3D(0, iperson);
    }
    else {
      // This is a person we've seen before.
    }
    people_->setFaceCenter3D(-init_pos_.pos.y, -init_pos_.pos.z, init_pos_.pos.x, iperson);
    cvmSet(center_3d,0,0,-init_pos_.pos.y);
    cvmSet(center_3d,0,1,-init_pos_.pos.z);
    cvmSet(center_3d,0,2,init_pos_.pos.x);
    cvSet(size_3d,cvScalar(people_->getFaceSize3D(iperson)));
    people_->centerSizeToFourCorners(center_3d,size_3d,four_corners_3d);
    cam_model_->cartToDisp(four_corners_3d, four_corners_2d);
    people_->setFaceBbox2D(cvRect(cvmGet(four_corners_2d,0,0),cvmGet(four_corners_2d,0,1),
				  cvmGet(four_corners_2d,1,0)-cvmGet(four_corners_2d,0,0),
				  cvmGet(four_corners_2d,2,1)-cvmGet(four_corners_2d,0,1)),
			   iperson);
    
    cvReleaseMat(&four_corners_3d);
    cvReleaseMat(&four_corners_2d);
    cvReleaseMat(&center_3d);
    cvReleaseMat(&size_3d);
    cv_mutex_.unlock();
  }

  /// The image callback when not all topics are sync'ed. Don't do anything, just wait for sync.
  void image_cb_timeout(ros::Time t) {
  }

  /// The image callback. For each new image, copy it, perform face detection or track it, and draw the rectangles on the image.
  void image_cb_all(ros::Time t)
  {

    //double startt, endt;
    //startt = t.now().toSec();

    cv_mutex_.lock();
 
    last_image_time_ = limage_.header.stamp;

    CvSize im_size;

    if (limage_.encoding=="mono") {
      printf("The left image is not a color image.\n");
      cv_mutex_.unlock();
      return;
    }

    // Set color calibration.
    bool do_calib = false;
    if (!calib_color_) {
      if (lbridge_.fromImage(limage_,"bgr")) {
	cv_image_left_ = lbridge_.toIpl();
      }
    }
    else if (calib_color_ && lcolor_cal_.getFromParam("dcam/left/image_rect_color")) {
      // Exit if color calibration hasn't been performed.
      do_calib = true;
      if (lbridge_.fromImage(limage_,"bgr")) {
	cv_image_left_ = lbridge_.toIpl();
	lcolor_cal_.correctColor(cv_image_left_, cv_image_left_, true, true, COLOR_CAL_BGR);
	
      }
    }
    else {
      cv_mutex_.unlock();
      return;
    }

    if (dbridge_.fromImage(dimage_)) {
      cv_image_disp_ = dbridge_.toIpl();
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
    cam_model_ = new CvStereoCamModel(Fx,Fy,Tx,Clx,Crx,Cy,1.0/stinfo_.dpp);

    im_size = cvGetSize(cv_image_left_);

    if (detect_faces_) {

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
#if __FACE_COLOR_TRACKER_DEBUG__
	printf("Detected faces\n");
#endif

	float* fptr = (float*)(uvd_->data.ptr);
	ushort* cptr = (ushort*)(cv_image_disp_->imageData);
	for (int v =0; v < im_size.height; v++) {
	  for (int u=0; u<im_size.width; u++) {
	    *fptr = (float)u; fptr++;
	    *fptr = (float)v; fptr++;
	    *fptr = (float)(*cptr); cptr++; fptr++;
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

	for (unsigned int iface = 0; iface < faces_vector.size(); iface++) {
	  people_->addPerson();
	  people_->setFaceBbox2D(faces_vector[iface],iface);

	  // Take the average valid Z within the face bounding box. Invalid values are 0.
	  avgz.val[0] = cvMedianNonZeroElIn2DArr(Z_,
						 faces_vector[iface].y+4,faces_vector[iface].y+faces_vector[iface].height-4,
						 faces_vector[iface].x+4,faces_vector[iface].x+faces_vector[iface].width-4); 
	  //CvRect tface = cvRect(faces_vector[iface].x+4, faces_vector[iface].y+4, faces_vector[iface].width-8, faces_vector[iface].height-8);
	  //cvSetImageROI(Z_,tface);
	  //avgz = cvSum(Z_);
	  //avgz.val[0] /= cvCountNonZero(Z_);
	  //cvResetImageROI(Z_);

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

#if __FACE_COLOR_TRACKER_DEBUG_
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

    }

    int npeople = people_->getNumPeople();

    if (npeople==0) {
      // No people to track, try again later.
      cv_mutex_.unlock();
      return;
    }

    CvMat *end_points = cvCreateMat(npeople,3,CV_32FC1);
    double kernel_size_m = 0.4; 
    bool did_track = people_->track_color_3d_bhattacharya(cv_image_left_, cv_image_disp_, cam_model_, kernel_size_m, 0, NULL, NULL, end_points);//0.3
    if (!did_track) {
      // If tracking failed, just return.
      cv_mutex_.unlock();
      return;
    }
    // Copy endpoints and the new 2d bbox to the people structure.
    // Draw the points on the image.
    CvMat *four_corners_3d = cvCreateMat(4,3,CV_32FC1);
    CvMat *four_corners_2d = cvCreateMat(4,3,CV_32FC1); 
    CvMat *my_end_point = cvCreateMat(1,3,CV_32FC1);
    CvMat *my_size = cvCreateMat(1,2,CV_32FC1);
    robot_msgs::PositionMeasurement pos;

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

      // Publish the 3d head center for this person.
      pos.header.stamp = limage_.header.stamp;
      pos.name = "stereo_face_color_tracker";
      pos.object_id = people_->getID(iperson);
      pos.pos.x = cvmGet(end_points,iperson,2);
      pos.pos.y = -1.0*cvmGet(end_points,iperson,0);
      pos.pos.z = -1.0*cvmGet(end_points,iperson,1);
      pos.header.frame_id = "stereo_link";
      pos.reliability = 0.5;
      pos.initialization = 0;
      //pos.covariance
      publish("person_measurement",pos);
    }
	  
    cvReleaseMat(&end_points);
    cvReleaseMat(&four_corners_3d);
    cvReleaseMat(&four_corners_2d);
    cvReleaseMat(&my_end_point);
    cvReleaseMat(&my_size);     

#if  __FACE_COLOR_TRACKER_DISPLAY__
    if (!cv_image_disp_out_) {
      cv_image_disp_out_ = cvCreateImage(im_size,IPL_DEPTH_8U,1);
    }
    cvCvtScale(cv_image_disp_,cv_image_disp_out_,4.0/stinfo_.dpp);
    cvShowImage("Face color tracker: Face Detection", cv_image_left_);
    cvShowImage("Face color tracker: Disparity", cv_image_disp_out_);
#endif
    cv_mutex_.unlock();       

    //endt = t.now().toSec();
    //printf("Start %f End %f Duration %f\n", startt, endt, endt-startt);
  }


  // Wait for completion, wait for user input, display images.
  bool spin() {
    while (ok() && !quit_) {
#if  __FACE_COLOR_TRACKER_DISPLAY__
      cv_mutex_.lock(); 
      // Get user input and allow OpenCV to refresh windows.
      int c = cvWaitKey(2);
      c &= 0xFF;
      // Quit on ESC, "q" or "Q"
      if((c == 27)||(c == 'q')||(c == 'Q'))
	quit_ = true;
      cv_mutex_.unlock(); 
#endif
      usleep(10000);
    }
    return true;
  } 

};


// Main
int main(int argc, char **argv)
{
  ros::init(argc, argv);
  bool use_depth = true;
  bool color_calib = false;
  bool detect_faces = false;

  if (argc > 1) {
    detect_faces = atoi(argv[1]);
  }
  char haar_filename[] = "./cascades/haarcascade_frontalface_alt.xml";
  StereoFaceColorTracker sfct(detect_faces, haar_filename, use_depth, color_calib);
 
  sfct.spin();


  ros::fini();
  return 0;
}


