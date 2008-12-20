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
#include <vector>

#include "ros/node.h"
#include "rosthread/mutex.h"

#include "CvStereoCamModel.h"
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

using namespace std;

// FaceDetector - A wrapper around OpenCV's face detection, plus some usage of depth to restrict the search.

class FaceDetector: public ros::node {
public:
  // Images and conversion
  image_msgs::Image limage_;
  image_msgs::Image dimage_;
  image_msgs::StereoInfo stinfo_;
  image_msgs::CamInfo rcinfo_;
  image_msgs::CvBridge lbridge_;
  image_msgs::CvBridge dbridge_;
  TopicSynchronizer<FaceDetector> sync_;

  // The left and disparity images.
  IplImage *cv_image_left_;
  IplImage *cv_image_disp_;
  bool do_display_;
  IplImage *cv_image_disp_out_;

  bool use_depth_;
  CvStereoCamModel *cam_model_;

  People *people_;
  const char *haar_filename_;

  bool external_init_;
  robot_msgs::PositionMeasurement pos_;

  bool quit_;
  int detect_;

  ros::thread::mutex cv_mutex_;

  // ros::node("face_detector", ros::node::ANONYMOUS_NAME ),
  FaceDetector(std::string node_name, const char *haar_filename, bool use_depth, bool do_display, bool external_init) : 
    ros::node(node_name ),
    sync_(this, &FaceDetector::image_cb_all, ros::Duration().fromSec(0.05), &FaceDetector::image_cb_timeout),
    cv_image_left_(NULL),
    cv_image_disp_(NULL),
    do_display_(do_display),
    cv_image_disp_out_(NULL),
    use_depth_(use_depth),
    cam_model_(NULL),
    people_(NULL),
    haar_filename_(haar_filename),
    external_init_(external_init),
    quit_(false),
    detect_(0)
  { 
    
    if (do_display_) {
      // OpenCV: pop up an OpenCV highgui window
      cvNamedWindow("Face detector: Disparity",CV_WINDOW_AUTOSIZE);
      cvNamedWindow("Face detector: Face Detection", CV_WINDOW_AUTOSIZE);
    }

    people_ = new People();

    // Subscribe to the images and parameters
    std::list<std::string> left_list;
    left_list.push_back(std::string("dcam/left/image_rect"));
    left_list.push_back(std::string("dcam/left/image_rect_color"));
    sync_.subscribe(left_list,limage_,1);
    sync_.subscribe("dcam/disparity",dimage_,1);
    sync_.subscribe("dcam/stereo_info",stinfo_,1);
    sync_.subscribe("dcam/right/cam_info",rcinfo_,1);

    // Advertise a position measure message.
    advertise<robot_msgs::PositionMeasurement>("person_measurement",1);
    if (external_init_) {
      subscribe<robot_msgs::PositionMeasurement>("person_measurement",pos_,&FaceDetector::pos_cb,1);
    }

  }

  ~FaceDetector()
  {

    if (cv_image_disp_out_) 
      cvReleaseImage(&cv_image_disp_out_);
    cv_image_disp_out_ = NULL;

    if (do_display_) {
      cvDestroyWindow("Face detector: Face Detection");
      cvDestroyWindow("Face detector: Disparity");
    }

    if (cam_model_)
      delete cam_model_;

    delete people_;

  }

  // Position message callback.
  void pos_cb() {

    // Check that the message came from the person filter, not one of the individual trackers.
    if (pos_.name != "person_filter") {
      return;
    }

    cv_mutex_.lock();
    // Find the person in my list. If they don't exist, or this is an initialization, create them.
    // Otherwise, reset the position.
    int iperson = people_->findID(pos_.object_id);
    if (pos_.initialization == 1 || iperson < 0) {
      // Create a person with this id.
      people_->addPerson();
      iperson = people_->getNumPeople()-1;
      people_->setID(pos_.object_id, iperson);
    }
    else {
      // Person already exists.
    }
    // Set the position.
    people_->setFaceCenter3D(-pos_.pos.y, -pos_.pos.z, pos_.pos.x, iperson);
    cv_mutex_.unlock();

    ///// I'm not currently checking the time of this positioning message, which is a little worrying.

  }

  /// The image callback when not all topics are sync'ed. Don't do anything, just wait for sync.
  void image_cb_timeout(ros::Time t) {
  }

  /// The image callback. For each new image, copy it, perform face detection, and draw the rectangles on the image.
  void image_cb_all(ros::Time t)
  {

    detect_++;
    if (detect_ % 2) {
      return;
    }

    cv_mutex_.lock();
 
    CvSize im_size;

    // Convert the images to OpenCV format
    if (lbridge_.fromImage(limage_,"bgr")) {
      cv_image_left_ = lbridge_.toIpl();
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
    cam_model_ = new CvStereoCamModel(Fx,Fy,Tx,Clx,Crx,Cy,1.0/(double)stinfo_.dpp);
 
    if ( cv_image_left_ )  {
      im_size = cvGetSize(cv_image_left_);
      vector<CvRect> faces_vector = people_->detectAllFaces(cv_image_left_, haar_filename_, 1.0, cv_image_disp_, cam_model_, true);

      // Get the average (median) disparity in the middle half of the bounding box, and compute the face center in 3d. 
      // Publish the face center as a track point.
      if (cv_image_disp_ && cam_model_) {
	int r, c, good_pix;
	ushort* ptr;
	double avg_disp;
	CvRect *one_face;
	robot_msgs::PositionMeasurement pos;
	CvMat *uvd = cvCreateMat(1,3,CV_32FC1);
	CvMat *xyz = cvCreateMat(1,3,CV_32FC1);
	for (uint iface = 0; iface < faces_vector.size(); iface++) {
	  one_face = &faces_vector[iface];
	  good_pix = 0;
	  avg_disp = 0;

	  // Get the median disparity in the middle half of the bounding box.
	  avg_disp = cvMedianNonZeroElIn2DArr(cv_image_disp_,
					      floor(one_face->y+0.25*one_face->height),floor(one_face->y+0.75*one_face->height),
					      floor(one_face->x+0.25*one_face->width), floor(one_face->x+0.75*one_face->width)); 

	  cvmSet(uvd,0,0,one_face->x+one_face->width/2.0);
	  cvmSet(uvd,0,1,one_face->y+one_face->height/2.0);
	  cvmSet(uvd,0,2,avg_disp);
	  cam_model_->dispToCart(uvd,xyz);
	  
	  bool do_publish = true;
	  std::string id = "-1";
	  if (external_init_) {
	    // Check if this person's face is close enough to one of the previously known faces and associate it.
	    // If not, don't publish it.
	    // If yes, set it's new position and publish it.
	    int close_person = people_->findPersonFaceLTDist3D(FACE_DIST, cvmGet(xyz,0,0), cvmGet(xyz,0,1), cvmGet(xyz,0,2));
	    if (close_person < 0) {
	      do_publish = false;
	    }
	    else {
	      id = people_->getID(close_person);
	      people_->setFaceCenter3D(cvmGet(xyz,0,0), cvmGet(xyz,0,1), cvmGet(xyz,0,2), close_person);
	    }
	  }

	  if (do_publish) {
	    pos.header.stamp = limage_.header.stamp;
	    pos.name = "face_detection";
	    pos.object_id = id;
	    pos.pos.x = cvmGet(xyz,0,2);
	    pos.pos.y = -1.0*cvmGet(xyz,0,0);
	    pos.pos.z = -1.0*cvmGet(xyz,0,1);
	    pos.header.frame_id = "stereo_link";
	    pos.reliability = 0.8;
	    pos.initialization = 0;
	    //pos.covariance = ;
	    publish("person_measurement",pos);
	  }
	}

	cvReleaseMat(&uvd);
	cvReleaseMat(&xyz);	
      
	if (do_display_) {
	  if (!cv_image_disp_out_) {
	    cv_image_disp_out_ = cvCreateImage(im_size,IPL_DEPTH_8U,1);
	  }
	  cvCvtScale(cv_image_disp_,cv_image_disp_out_,4.0/stinfo_.dpp);

	  cvShowImage("Face detector: Face Detection",cv_image_left_);
	  cvShowImage("Face detector: Disparity",cv_image_disp_out_);
	}
      }
      
    }
    cv_mutex_.unlock();
  }


  // Wait for completion, wait for user input, display images.
  bool spin() {
    while (ok() && !quit_) {

	// Display all of the images.
	cv_mutex_.lock();

	// Get user input and allow OpenCV to refresh windows.
	int c = cvWaitKey(2);
	c &= 0xFF;
	// Quit on ESC, "q" or "Q"
	if((c == 27)||(c == 'q')||(c == 'Q'))
	  quit_ = true;

	cv_mutex_.unlock();
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
  bool do_display = true;
  bool external_init = false;

  if (argc < 3) {
    cerr << "Node name ending and path to cascade file required.\n" << endl;
    return 0;
  }
  char *haar_filename = argv[2];
  if (argc >= 4) {
    do_display = atoi(argv[3]);
    if (argc >= 5) {
      external_init = atoi(argv[4]);
    }
  }
  ostringstream node_name;
  node_name << "face_detection_" << argv[1];
  FaceDetector fd(node_name.str(), haar_filename, use_depth, do_display, external_init);
  

  fd.spin();
  ros::fini();
  return 0;
}


