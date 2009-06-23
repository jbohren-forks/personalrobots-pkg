/**********************************************************************
 *
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
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
#include <fstream>

#include "ros/node.h"
#include "ros/console.h"
#include <boost/thread/mutex.hpp>

#include "CvStereoCamModel.h"
#include <people/PositionMeasurement.h>
#include "image_msgs/StereoInfo.h"
#include "image_msgs/DisparityInfo.h"
#include "image_msgs/CamInfo.h"
#include "image_msgs/Image.h"
#include "opencv_latest/CvBridge.h"
#include "image_msgs/ColoredLine.h"
#include "image_msgs/ColoredLines.h"
#include "topic_synchronizer2/topic_synchronizer.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"
#include "people/StartDetection.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "people/people.h"
#include "utils.h"

using namespace std;

namespace people {

/** FaceDetector - A wrapper around OpenCV's face detection, plus some usage of depth to restrict the search.
 *
 * This class provides a ROS node wrapper around OpenCV's face detection, plus some use of depth from stereo to restrict the
 * results presented to plausible face sizes. 
 * Displayed face detections are colored by:
 * red - not a plausible face size;
 * blue - no stereo information available;
 * green - plausible face size. 
 */
class FaceDetector {
public:
  // Constants
  const double BIGDIST_M;// = 1000000.0;

  // Node handle
  ros::NodeHandle nh_;

  // Images and conversion
  image_msgs::ImageConstPtr limage_; /**< Left image msg. */
  image_msgs::ImageConstPtr dimage_; /**< Disparity image msg. */
  image_msgs::DisparityInfoConstPtr dispinfo_; /**< Disparity info msg. */
  image_msgs::CamInfoConstPtr rcinfo_; /**< Right camera info msg. */
  image_msgs::CvBridge lbridge_; /**< ROS->OpenCV bridge for the left image. */
  image_msgs::CvBridge dbridge_; /**< ROS->OpenCV bridge for the disparity image. */

  IplImage *cv_image_left_; /**< Left image. */
  IplImage *cv_image_disp_; /**< Disparity image. */

  // Subscribers
  ros::Subscriber limage_sub_;
  ros::Subscriber dimage_sub_;
  ros::Subscriber rcinfo_sub_;
  ros::Subscriber dispinfo_sub_;
  ros::Subscriber pos_sub_;
  TopicSynchronizer sync_; /**< Stereo topic synchronizer. */

  // Publishers
  ros::Publisher pos_pub_;
  ros::Publisher vis_pub_;
  ros::Publisher clines_pub_;

  // Service
  ros::ServiceServer service_server_;

  string do_display_; /**< Type of display, none/local/remote */
  IplImage *cv_image_disp_out_; /**< Display image. */

  bool use_depth_; /**< True/false use depth information. */
  CvStereoCamModel *cam_model_; /**< A model of the stereo cameras. */
 
  People *people_; /**< List of people and associated fcns. */
  int num_filenames_;
  string *names_; /**< The name of each detector. Ie frontalface, profileface. These will be the names in the published face location msgs. */
  string *haar_filenames_; /**< Training file for the haar cascade classifier. */
  double *reliabilities_; /**< Reliability of the predictions. This should depend on the training file used. */

  bool external_init_; 

  struct RestampedPositionMeasurement {
    ros::Time restamp;
    people::PositionMeasurement pos;
    double dist;
  };
  map<string, RestampedPositionMeasurement> pos_list_; /**< Queue of updated people positions from the filter. */

  bool quit_;

  tf::TransformListener tf_;

  boost::mutex cv_mutex_, pos_mutex_;

  bool do_continuous_;
  bool run_detector_;
  bool do_publish_unknown_;

  FaceDetector(int num_filenames, string *names, string *haar_filenames, double *reliabilities) : 
    BIGDIST_M(1000000.0),
    cv_image_left_(NULL),
    cv_image_disp_(NULL),
    cv_image_disp_out_(NULL),
    sync_(&FaceDetector::imageCBAll,this),
    cam_model_(0),
    people_(0),
    num_filenames_(num_filenames),
    names_(names),
    haar_filenames_(haar_filenames),
    reliabilities_(reliabilities),
    quit_(false)
  { 
    
    if (do_display_ == "local") {
      // OpenCV: pop up an OpenCV highgui window
      cvNamedWindow("Face detector: Disparity",CV_WINDOW_AUTOSIZE);
      cvNamedWindow("Face detector: Face Detection", CV_WINDOW_AUTOSIZE);
    }

    // Parameters
    nh_.param("/people/face_detector/do_display",do_display_,std::string("none"));
    nh_.param("/people/face_detector/do_continuous",do_continuous_,true);
    nh_.param("/people/face_detector/do_publish_faces_of_unknown_size",do_publish_unknown_,false);
    nh_.param("/people/face_detector/use_depth",use_depth_,true);
    nh_.param("/people/face_detector/use_external_init",external_init_,true);
    run_detector_ = do_continuous_;

    people_ = new People();
    people_->initFaceDetection(num_filenames_, haar_filenames_);

    // Subscribe to the images and parameters
    limage_sub_ = nh_.subscribe("stereo/left/image_rect", 1, sync_.synchronize(&FaceDetector::leftImageCallback,this));
    dimage_sub_ = nh_.subscribe("stereo/disparity", 1, sync_.synchronize(&FaceDetector::dispImageCallback,this));
    dispinfo_sub_ = nh_.subscribe("stereo/disparity_info", 1, sync_.synchronize(&FaceDetector::dispInfoCallback,this));
    rcinfo_sub_ = nh_.subscribe("stereo/right/cam_info", 1, sync_.synchronize(&FaceDetector::rcamInfoCallback,this));

    ROS_INFO_STREAM_NAMED("face_detector","Subscribed to images");

    // Advertise a position measure message.
    pos_pub_ = nh_.advertise<people::PositionMeasurement>("~people_tracker_measurements",1);

    ROS_INFO_STREAM_NAMED("face_detector","Advertised people_tracker_measurements");

    vis_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker",0);

    // Advertise the rectangles to draw if stereo_view is running.
    if (do_display_ == "remote") {
      clines_pub_ = nh_.advertise<image_msgs::ColoredLines>("lines_to_draw",1);
      ROS_INFO_STREAM_NAMED("face_detector","Advertising colored lines to draw remotely.");
    }
    // Subscribe to filter measurements.
    if (external_init_) {
      pos_sub_ = nh_.subscribe("people_tracker_filter",1,&FaceDetector::posCallback,this);
      ROS_INFO_STREAM_NAMED("face_detector","Subscribed to the person filter messages.");
    }

    service_server_ = nh_.advertiseService("start_detection",&FaceDetector::startDetection,this);

    ros::MultiThreadedSpinner s(2);
    ros::spin(s);

  }

  ~FaceDetector()
  {

    if (cv_image_disp_out_) {cvReleaseImage(&cv_image_disp_out_); cv_image_disp_out_ = 0;}

    if (do_display_ == "local") {
      cvDestroyWindow("Face detector: Face Detection");
      cvDestroyWindow("Face detector: Disparity");
    }

    if (cam_model_) {delete cam_model_; cam_model_ = 0;}

    if (people_) {delete people_; people_ = 0;}
  }


  // Start the detector running. It will automatically stop running when at least one face is found.
  bool startDetection(people::StartDetection::Request &req, people::StartDetection::Response &resp)
  {
    run_detector_ = true;
    return true;
  }

  /*!
   * \brief Position message callback. 
   *
   * When hooked into the person tracking filter, this callback will listen to messages 
   * from the filter with a person id and 3D position and adjust the person's face position accordingly.
   */ 
  void posCallback(const people::PositionMeasurementConstPtr& pos_ptr) {

    // Put the incoming position into the position queue. It'll be processed in the next image callback.
    boost::mutex::scoped_lock lock(pos_mutex_);
    map<string, RestampedPositionMeasurement>::iterator it;
    it = pos_list_.find(pos_ptr->object_id);
    RestampedPositionMeasurement rpm;
    rpm.pos = *pos_ptr;
    rpm.restamp = pos_ptr->header.stamp;
    rpm.dist = BIGDIST_M;
    if (it == pos_list_.end()) {
      pos_list_.insert(pair<string, RestampedPositionMeasurement>(pos_ptr->object_id, rpm));
    }
    else if ((pos_ptr->header.stamp - (*it).second.pos.header.stamp) > ros::Duration().fromSec(-1.0) ){
      (*it).second = rpm;
    }
    lock.unlock();

  }

  void leftImageCallback(const image_msgs::ImageConstPtr& image_ptr) 
  {
    limage_ = image_ptr;
  }

  void dispImageCallback(const image_msgs::ImageConstPtr& image_ptr)
  {
    dimage_ = image_ptr;
  }

  void dispInfoCallback(const image_msgs::DisparityInfoConstPtr &info_ptr)
  {
    dispinfo_ = info_ptr;
  }

  void rcamInfoCallback(const image_msgs::CamInfoConstPtr &info_ptr)
  {
    rcinfo_ = info_ptr;
  }

  /*! 
   * \brief Image callback for synced messages. 
   *
   * For each new image:
   * convert it to OpenCV format, perform face detection using OpenCV's haar filter cascade classifier, and
   * (if requested) draw rectangles around the found faces.
   * Can also compute which faces are associated (by proximity, currently) with faces it already has in its list of people.
   */
  void imageCBAll()
  {

    // Only run the detector if in continuous mode or a service call was made.
    if (!run_detector_) 
      return;

    if (do_display_ == "local") {
      cv_mutex_.lock();
    }
 
    CvSize im_size;

    if (lbridge_.fromImage(*limage_,"bgr") && dbridge_.fromImage(*dimage_)) {
      cv_image_left_ = lbridge_.toIpl();
      cv_image_disp_ = dbridge_.toIpl();
    }
    else {
      return;
    }

    // Convert the stereo calibration into a camera model.
    if (cam_model_) delete cam_model_;
    double Fx = rcinfo_->P[0];  double Fy = rcinfo_->P[5];
    double Clx = rcinfo_->P[2]; double Crx = Clx;
    double Cy = rcinfo_->P[6];
    double Tx = -rcinfo_->P[3]/Fx;
    cam_model_ = new CvStereoCamModel(Fx,Fy,Tx,Clx,Crx,Cy,1.0/(double)dispinfo_->dpp);
 
    im_size = cvGetSize(cv_image_left_);
    struct timeval timeofday;
    gettimeofday(&timeofday,NULL);
    ros::Time starttdetect = ros::Time().fromNSec(1e9*timeofday.tv_sec + 1e3*timeofday.tv_usec);

    vector<Box2D3D> faces_vector = people_->detectAllFaces(cv_image_left_, 1.0, cv_image_disp_, cam_model_);
    gettimeofday(&timeofday,NULL);
    ros::Time endtdetect = ros::Time().fromNSec(1e9*timeofday.tv_sec + 1e3*timeofday.tv_usec);
    ros::Duration diffdetect = endtdetect - starttdetect;
    ROS_DEBUG_STREAM_NAMED("face_detector","Detection duration = " << diffdetect.toSec() << "sec");   

    bool published = false;

    image_msgs::ColoredLines all_cls;
    vector<image_msgs::ColoredLine> lines;

    if (faces_vector.size() > 0 ) {

      // Transform the positions of the known faces and remove anyone who hasn't had an update in a long time.
      boost::mutex::scoped_lock pos_lock(pos_mutex_);
      map<string, RestampedPositionMeasurement>::iterator it;
      for (it = pos_list_.begin(); it != pos_list_.end(); it++) {
	if ((limage_->header.stamp - (*it).second.restamp) > ros::Duration().fromSec(5.0)) {
	  // Position is too old, kill the person.
	  pos_list_.erase(it);
	}
	else {
	  // Transform the person to this time. Note that the pos time is updated but not the restamp. 
	  tf::Point pt;
	  tf::PointMsgToTF((*it).second.pos.pos, pt);
	  tf::Stamped<tf::Point> loc(pt, (*it).second.pos.header.stamp, (*it).second.pos.header.frame_id);
	  try {
     	    tf_.transformPoint(limage_->header.frame_id, limage_->header.stamp, loc, "odom_combined", loc);
	    (*it).second.pos.header.stamp = limage_->header.stamp;
	    (*it).second.pos.pos.x = loc[0];
            (*it).second.pos.pos.y = loc[1];
            (*it).second.pos.pos.z = loc[2];
	  } 
	  catch (tf::TransformException& ex) {
	  }
	}
      } 
      // End filter face position update

      // Associate the found faces with previously seen faces, and publish all good face centers.
      Box2D3D *one_face;
      people::PositionMeasurement pos;
      
      for (uint iface = 0; iface < faces_vector.size(); iface++) {
	one_face = &faces_vector[iface];
	  
	if (one_face->status=="good" || (one_face->status=="unknown" && do_publish_unknown_)) {

	  std::string id = "";

	  // Convert the face format to a PositionMeasurement msg.
	  pos.header.stamp = limage_->header.stamp;
	  pos.name = names_[0]; 
	  pos.pos.x = one_face->center3d.val[0]; 
	  pos.pos.y = one_face->center3d.val[1];
	  pos.pos.z = one_face->center3d.val[2]; 
	  pos.header.frame_id = limage_->header.frame_id;//"stereo_optical_frame";
	  pos.reliability = reliabilities_[0];
	  pos.initialization = 1;//0;
	  pos.covariance[0] = 0.04; pos.covariance[1] = 0.0;  pos.covariance[2] = 0.0;
	  pos.covariance[3] = 0.0;  pos.covariance[4] = 0.04; pos.covariance[5] = 0.0;
	  pos.covariance[6] = 0.0;  pos.covariance[7] = 0.0;  pos.covariance[8] = 0.04;

	  // Check if this person's face is close enough to one of the previously known faces and associate it with the closest one.
	  // Otherwise publish it with an empty id.
	  // Note that multiple face positions can be published with the same id, but ids in the pos_list_ are unique. The position of a face in the list is updated with the closest found face.
	  double dist, mindist = BIGDIST_M;
	  map<string, RestampedPositionMeasurement>::iterator close_it = pos_list_.end();
	  for (it = pos_list_.begin(); it != pos_list_.end(); it++) {
	    dist = pow((*it).second.pos.pos.x - pos.pos.x, 2.0) + pow((*it).second.pos.pos.y - pos.pos.y, 2.0) + pow((*it).second.pos.pos.z - pos.pos.z, 2.0);
	    if (dist <= FACE_DIST && dist < mindist) {
	      mindist = dist;
	      close_it = it;
	    }
	  }
	  if (close_it != pos_list_.end()) {
	    if (mindist < (*close_it).second.dist) {
	      (*close_it).second.restamp = limage_->header.stamp;
	      (*close_it).second.dist = mindist;
	      (*close_it).second.pos = pos;
	    }
	    pos.object_id = (*close_it).second.pos.object_id;
	  }
	  else {
	    pos.object_id = "";
	  }
	  ROS_INFO_STREAM_NAMED("face_detector","Closest person: " << pos.object_id);
	  pos_pub_.publish(pos);
	  published = true;

	}

      } // end for iface
      pos_lock.unlock();

      // Clean out all of the distances in the pos_list_
      for (it = pos_list_.begin(); it != pos_list_.end(); it++) {
	(*it).second.dist = BIGDIST_M;
      }
      // Done associating faces.


      // If you don't want continuous processing and you've found at least one face, turn off the detector.
      if (!do_continuous_ && published) run_detector_ = false;

      /******** Everything from here until the end of the function is for display *********/

      // Draw an appropriately colored rectangle on the display image and in the visualizer.

      if (do_display_ == "remote"){
	lines.resize(4*faces_vector.size());
      }

      for (uint iface = 0; iface < faces_vector.size(); iface++) {
	one_face = &faces_vector[iface];	
	
	// Visualization markers
	if (one_face->status == "good") {
	  visualization_msgs::Marker marker;
	  marker.header.frame_id = limage_->header.frame_id;
	  marker.header.stamp = limage_->header.stamp;
	  marker.ns = "people";
	  marker.id = 0;
	  marker.type = visualization_msgs::Marker::SPHERE;
	  marker.action = visualization_msgs::Marker::ADD;
	  marker.pose.position.x = one_face->center3d.val[0];
	  marker.pose.position.y = one_face->center3d.val[1];
	  marker.pose.position.z = one_face->center3d.val[2];
	  marker.pose.orientation.x = 0.0;
	  marker.pose.orientation.y = 0.0;
	  marker.pose.orientation.z = 0.0;
	  marker.pose.orientation.w = 1.0;
	  marker.scale.x = 0.1;
	  marker.scale.y = 0.1;
	  marker.scale.z = 0.1;
	  marker.color.a = 1.0;
	  marker.color.r = 0.0;
	  marker.color.g = 1.0;
	  marker.color.b = 0.0;
	  vis_pub_.publish(marker);
	}
	else {
	  ROS_DEBUG_STREAM_NAMED("face_detector","The detection didn't have a valid size, so it wasn't visualized.");
	}

	// Image display 

	if (do_display_ != "none") {
	  CvScalar color;
	  if (one_face->status == "good") {
	    color = cvScalar(0,255,0);
	  }
	  else if (one_face->status == "unknown") {
	    color = cvScalar(255,0,0);
	  }
	  else {
	    color = cvScalar(0,0,255);
	  }

	  if (do_display_ == "local") {
	    cvRectangle(cv_image_left_, 
			cvPoint(one_face->box2d.x,one_face->box2d.y), 
			cvPoint(one_face->box2d.x+one_face->box2d.width, one_face->box2d.y+one_face->box2d.height), color, 4);
	  }
	  else if (do_display_ == "remote") {
	    
	    lines[4*iface].r = color.val[2]; lines[4*iface+1].r = lines[4*iface].r; 
	    lines[4*iface+2].r = lines[4*iface].r; lines[4*iface+3].r = lines[4*iface].r;
	    lines[4*iface].g = color.val[1]; lines[4*iface+1].g = lines[4*iface].g; 
	    lines[4*iface+2].g = lines[4*iface].g; lines[4*iface+3].g = lines[4*iface].g;
	    lines[4*iface].b = color.val[0]; lines[4*iface+1].b = lines[4*iface].b; 
	    lines[4*iface+2].b = lines[4*iface].b; lines[4*iface+3].b = lines[4*iface].b;

	    lines[4*iface].x0 = one_face->box2d.x; 
	    lines[4*iface].x1 = one_face->box2d.x + one_face->box2d.width;
	    lines[4*iface].y0 = one_face->box2d.y; 
	    lines[4*iface].y1 = one_face->box2d.y;

	    lines[4*iface+1].x0 = one_face->box2d.x; 
	    lines[4*iface+1].x1 = one_face->box2d.x;
	    lines[4*iface+1].y0 = one_face->box2d.y; 
	    lines[4*iface+1].y1 = one_face->box2d.y + one_face->box2d.height;

	    lines[4*iface+2].x0 = one_face->box2d.x; 
	    lines[4*iface+2].x1 = one_face->box2d.x + one_face->box2d.width;
	    lines[4*iface+2].y0 = one_face->box2d.y + one_face->box2d.height; 
	    lines[4*iface+2].y1 = one_face->box2d.y + one_face->box2d.height;

	    lines[4*iface+3].x0 = one_face->box2d.x + one_face->box2d.width; 
	    lines[4*iface+3].x1 = one_face->box2d.x + one_face->box2d.width;
	    lines[4*iface+3].y0 = one_face->box2d.y; 
	    lines[4*iface+3].y1 = one_face->box2d.y + one_face->box2d.height;

	    lines[4*iface].header.stamp = limage_->header.stamp;
	    lines[4*iface+1].header.stamp = limage_->header.stamp;
	    lines[4*iface+2].header.stamp = limage_->header.stamp;
	    lines[4*iface+3].header.stamp = limage_->header.stamp;
	    lines[4*iface].header.frame_id = limage_->header.frame_id;
	    lines[4*iface+1].header.frame_id = limage_->header.frame_id;
	    lines[4*iface+2].header.frame_id = limage_->header.frame_id;
	    lines[4*iface+3].header.frame_id = limage_->header.frame_id;
	  
	  }
	} // End if do_display_
      } // End for iface
    } // End if faces_vector.size()>0

    // Display
    if (do_display_ == "remote") {
      all_cls.header.stamp = limage_->header.stamp;
      all_cls.label = "face_detection";
      all_cls.header.frame_id = limage_->header.frame_id;
      all_cls.lines = lines;
      clines_pub_.publish(all_cls);
    }
    else if (do_display_ == "local") {
      if (!cv_image_disp_out_) {
	cv_image_disp_out_ = cvCreateImage(im_size,IPL_DEPTH_8U,1);
      }
      cvCvtScale(cv_image_disp_,cv_image_disp_out_,4.0/dispinfo_->dpp);
      cvShowImage("Face detector: Face Detection",cv_image_left_);
      cvShowImage("Face detector: Disparity",cv_image_disp_out_);
      cvWaitKey(2);
 
      cv_mutex_.unlock();
    }
    // Done display

  }


  /*!
   *\brief Wait for completion, wait for user input, display images.
   */
  bool spin2() {


    while (nh_.ok() && !quit_) {

      if (do_display_ == "local") {
	// Display all of the images.
	boost::mutex::scoped_lock lock(cv_mutex_);

	// Get user input and allow OpenCV to refresh windows.
	int c = cvWaitKey(2);
	c &= 0xFF;
	// Quit on ESC, "q" or "Q"
	if((c == 27)||(c == 'q')||(c == 'Q'))
	  quit_ = true;

	lock.unlock(); 
      }
      ros::spinOnce();
      //usleep(10000);
    }
    return true;
  } 

}; // end class
 
}; // end namespace people

// Main
int main(int argc, char **argv)
{

  if (argc < 2) {
    cerr << "A path to the file containing the experiment list is required.\n" << endl;
    return 0;
  }

  ifstream expfile(argv[1]);
  if (!expfile.is_open()) {
    ROS_ERROR_STREAM_NAMED("face_detector","File " << argv[1] << " cannot be opened.");
    return 0;
  }
  int numlines;
  expfile >> numlines;
  double reliabilities[numlines];
  string haar_filenames[numlines];
  string names[numlines];
  for (int i=0; i<numlines; i++) {
    if (expfile.eof()) {
      ROS_ERROR_STREAM_NAMED("face_detector","File " << argv[1] << " has an incorrect number of lines.");
      return 0;
    }
    expfile >> names[i];
    expfile >> haar_filenames[i];
    expfile >> reliabilities[i];
  }

  ros::init(argc,argv,"face_detection");

  people::FaceDetector fd(numlines, names, haar_filenames, reliabilities);

  //fd.spin();
  
  return 0;
}


