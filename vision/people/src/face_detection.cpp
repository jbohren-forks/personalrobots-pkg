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
#include <boost/thread/mutex.hpp>

#include "CvStereoCamModel.h"
#include <robot_msgs/PositionMeasurement.h>
#include "image_msgs/StereoInfo.h"
#include "image_msgs/DisparityInfo.h"
#include "image_msgs/CamInfo.h"
#include "image_msgs/Image.h"
#include "image_msgs/CvBridge.h"
#include "image_msgs/ColoredLine.h"
#include "image_msgs/ColoredLines.h"
#include "topic_synchronizer.h"
#include "tf/transform_listener.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "people.h"
#include "utils.h"

using namespace std;

/** FaceDetector - A wrapper around OpenCV's face detection, plus some usage of depth to restrict the search.
 *
 * This class provides a ROS node wrapper around OpenCV's face detection, plus some use of depth from stereo to restrict the
 * results presented to plausible face sizes. 
 * Displayed face detections are colored by:
 * red - not a plausible face size;
 * blue - no stereo information available;
 * green - plausible face size. 
 */
class FaceDetector: public ros::node {
public:
  // Images and conversion
  image_msgs::Image limage_; /**< Left image msg. */
  image_msgs::Image dimage_; /**< Disparity image msg. */
  image_msgs::StereoInfo stinfo_; /**< Stereo info msg. */
  image_msgs::DisparityInfo dispinfo_; /**< Disparity info msg. */
  image_msgs::CamInfo rcinfo_; /**< Right camera info msg. */
  image_msgs::CvBridge lbridge_; /**< ROS->OpenCV bridge for the left image. */
  image_msgs::CvBridge dbridge_; /**< ROS->OpenCV bridge for the disparity image. */
  TopicSynchronizer<FaceDetector> sync_; /**< Stereo topic synchronizer. */

  // The left and disparity images.
  IplImage *cv_image_left_; /**< Left image. */
  IplImage *cv_image_disp_; /**< Disparity image. */
  string do_display_; /**< True/false display face detections. */
  IplImage *cv_image_disp_out_; /**< Display image. */

  bool use_depth_; /**< True/false use depth information. */
  CvStereoCamModel *cam_model_; /**< A model of the stereo cameras. */
 
  People *people_; /**< List of people and associated fcns. */
  int num_filenames_;
  string *names_; /**< The name of each detector. Ie frontalface, profileface. These will be the names in the published face location msgs. */
  string *haar_filenames_; /**< Training file for the haar cascade classifier. */
  double *reliabilities_; /**< Reliability of the predictions. This should depend on the training file used. */

  bool external_init_; 
  robot_msgs::PositionMeasurement pos_; /**< A person position update from the filter. */
  struct RestampedPositionMeasurement {
    ros::Time restamp;
    robot_msgs::PositionMeasurement pos;
  };
  map<string, RestampedPositionMeasurement> pos_list_; /**< Queue of updated people positions from the filter. */

  bool quit_;

  std::string node_name_;

  tf::TransformListener tf;

  boost::mutex cv_mutex_, pos_mutex_;

  // ros::node("face_detector", ros::node::ANONYMOUS_NAME ),
  FaceDetector(string node_name, int num_filenames, string *names, string *haar_filenames, double *reliabilities, bool use_depth, string do_display, bool external_init) : 
    ros::node("face_detector", ros::node::ANONYMOUS_NAME),
    sync_(this, &FaceDetector::image_cb_all, ros::Duration().fromSec(0.05), &FaceDetector::image_cb_timeout),
    cv_image_left_(0),
    cv_image_disp_(0),
    do_display_(do_display),
    cv_image_disp_out_(0),
    use_depth_(use_depth),
    cam_model_(0),
    people_(0),
    num_filenames_(num_filenames),
    names_(names),
    haar_filenames_(haar_filenames),
    reliabilities_(reliabilities),
    external_init_(external_init),
    quit_(false),
    node_name_(node_name),
    tf(*this)
  { 
    
    if (do_display_ == "local") {
      // OpenCV: pop up an OpenCV highgui window
      cvNamedWindow("Face detector: Disparity",CV_WINDOW_AUTOSIZE);
      cvNamedWindow("Face detector: Face Detection", CV_WINDOW_AUTOSIZE);
    }

    people_ = new People();

    // Subscribe to the images and parameters
    std::list<std::string> left_list;
    left_list.push_back(std::string("stereo/left/image_rect"));
    //left_list.push_back(std::string("stereodcam/left/image_rect_color"));
    sync_.subscribe(left_list,limage_,1);
    sync_.subscribe("stereo/disparity",dimage_,1);
    sync_.subscribe("stereo/stereo_info",stinfo_,1);
    sync_.subscribe("stereo/disparity_info",dispinfo_,1);
    sync_.subscribe("stereo/right/cam_info",rcinfo_,1);
    sync_.ready();

    // Advertise a position measure message.
    advertise<robot_msgs::PositionMeasurement>("people_tracker_measurements",1);
    // Advertise the rectangles to draw if stereo_view is running.
    if (do_display_ == "remote") {
      advertise<image_msgs::ColoredLines>("lines_to_draw",1);
    }
    // Subscribe to filter measurements.
    if (external_init_) {
      subscribe<robot_msgs::PositionMeasurement>("people_tracker_filter",pos_,&FaceDetector::pos_cb,1);
    }

  }

  ~FaceDetector()
  {

    if (cv_image_disp_out_) {
      cvReleaseImage(&cv_image_disp_out_); cv_image_disp_out_ = 0;
    }

    if (do_display_ == "local") {
      cvDestroyWindow("Face detector: Face Detection");
      cvDestroyWindow("Face detector: Disparity");
    }

    if (cam_model_) {
      delete cam_model_; cam_model_ = 0;
    }

    if (people_) {
      delete people_; people_ = 0;
    }
  }

  /*!
   * \brief Position message callback. 
   *
   * When hooked into the person tracking filter, this callback will listen to messages 
   * from the filter with a person id and 3D position and adjust the person's face position accordingly.
   */ 
  void pos_cb() {

    // Put the incoming position into the position queue. It'll be processed in the next image callback.
    boost::mutex::scoped_lock lock(pos_mutex_);
    map<string, RestampedPositionMeasurement>::iterator it;
    it = pos_list_.find(pos_.object_id);
    RestampedPositionMeasurement rpm;
    rpm.pos = pos_;
    rpm.restamp = pos_.header.stamp;
    if (it == pos_list_.end()) {
      pos_list_.insert(pair<string, RestampedPositionMeasurement>(pos_.object_id, rpm));
    }
    else if (pos_.header.stamp - (*it).second.pos.header.stamp > -1.0 ){
      (*it).second = rpm;
    }
    lock.unlock();

  }

  /*!
   * \brief Image callback for unsynced messages.
   *
   * If unsynced stereo msgs are received, do nothing. 
   */
  void image_cb_timeout(ros::Time t) {
    cout << "In timeout" << endl;
  }

  /*! 
   * \brief Image callback for synced messages. 
   *
   * For each new image:
   * convert it to OpenCV format, perform face detection using OpenCV's haar filter cascade classifier, and
   * (if requested) draw rectangles around the found faces.
   * Only publishes faces which are associated (by proximity, currently) with faces it already has in its list of people.
   */
  void image_cb_all(ros::Time t)
  {
    ros::Time startt, endt;
    startt = t.now();

    if (do_display_ == "local") {
      cv_mutex_.lock();
    }
 
    CvSize im_size;

    // Convert the images to OpenCV format
    if (lbridge_.fromImage(limage_,"bgr") && dbridge_.fromImage(dimage_)) {
      cv_image_left_ = lbridge_.toIpl();
      cv_image_disp_ = dbridge_.toIpl();
    }
    else {
      return;
    }

    // Convert the stereo calibration into a camera model.
    if (cam_model_) delete cam_model_;
    double Fx = rcinfo_.P[0];  double Fy = rcinfo_.P[5];
    double Clx = rcinfo_.P[2]; double Crx = Clx;
    double Cy = rcinfo_.P[6];
    double Tx = -rcinfo_.P[3]/Fx;
    cam_model_ = new CvStereoCamModel(Fx,Fy,Tx,Clx,Crx,Cy,1.0/(double)dispinfo_.dpp);
 
    im_size = cvGetSize(cv_image_left_);
    vector<Box2D3D> faces_vector = people_->detectAllFaces(cv_image_left_, num_filenames_, haar_filenames_, 1.0, cv_image_disp_, cam_model_);
 
    image_msgs::ColoredLines all_cls;
    vector<image_msgs::ColoredLine> lines;   

    if (faces_vector.size() > 0 ) {

      // Transform the positions of the known faces and remove anyone who hasn't had an update in a long time.
      boost::mutex::scoped_lock pos_lock(pos_mutex_);
      map<string, RestampedPositionMeasurement>::iterator it;
      for (it = pos_list_.begin(); it != pos_list_.end(); it++) {
	if ((limage_.header.stamp - (*it).second.restamp) > ros::Duration().fromSec(5.0)) {
	  // Position is too old, kill the person.
	  pos_list_.erase(it);
	}
	else {
	  // Transform the person to this time. Note that their pos time is updated but not their restamp. 
	  tf::Point pt;
	  tf::PointMsgToTF((*it).second.pos.pos, pt);
	  tf::Stamped<tf::Point> loc(pt, (*it).second.pos.header.stamp, (*it).second.pos.header.frame_id);
	  try {
	    tf.transformPoint(limage_.header.frame_id, limage_.header.stamp, loc, "odom", loc);
	    (*it).second.pos.header.stamp = limage_.header.stamp;
	  } 
	  catch (tf::TransformException& ex) {
	  }
	}
      } 
      // End filter face position update

      if (do_display_ == "remote"){
	lines.resize(4*faces_vector.size());
      }

      // Associate the found faces with previously seen faces, and publish all good face centers.
      Box2D3D *one_face;
      robot_msgs::PositionMeasurement pos;
      for (uint iface = 0; iface < faces_vector.size(); iface++) {
	one_face = &faces_vector[iface];
	  
	if (one_face->status != "bad") {
	  std::string id = "";

	  // Convert the face format to a PositionMeasurement msg.
	  pos.header.stamp = limage_.header.stamp;
	  pos.name = names_[0];
	  pos.pos.x = one_face->center3d.val[2]; 
	  pos.pos.y = -1.0*one_face->center3d.val[0];
	  pos.pos.z = -1.0*one_face->center3d.val[1]; 
	  pos.header.frame_id = limage_.header.frame_id;//"stereo_link";
	  pos.reliability = reliabilities_[0];
	  pos.initialization = 0;
	  pos.covariance[0] = 0.04; pos.covariance[1] = 0.0;  pos.covariance[2] = 0.0;
	  pos.covariance[3] = 0.0;  pos.covariance[4] = 0.04; pos.covariance[5] = 0.0;
	  pos.covariance[6] = 0.0;  pos.covariance[7] = 0.0;  pos.covariance[8] = 0.04;

	  // Check if this person's face is close enough to one of the previously known faces and associate it.
	  // Otherwise publish it with an empty id.
	  double dist, mindist = 1000000.0;
	  map<string, RestampedPositionMeasurement>::iterator close_it = pos_list_.end();
	  for (it = pos_list_.begin(); it != pos_list_.end(); it++) {
	    dist = pow((*it).second.pos.pos.x - pos.pos.x, 2.0) + pow((*it).second.pos.pos.y - pos.pos.y, 2.0) + pow((*it).second.pos.pos.z - pos.pos.z, 2.0);
	    if (dist < FACE_DIST && dist < mindist) {
	      mindist = dist;
	      close_it = it;
	    }
	  }
	  if (close_it != pos_list_.end()) {
	    (*close_it).second.restamp = limage_.header.stamp;
	    pos.object_id = (*close_it).second.pos.object_id;
	  }
	  else {
	    pos.object_id = "";
	  }
	  cout << "Closest person: " << pos.object_id << endl;
	  publish("people_tracker_measurements",pos);
	}

      } // end for iface
      pos_lock.unlock();
      // Done associating faces.

      // Draw an appropriately colored rectangle on the display image.
      if (do_display_ != "none") {
	for (uint iface = 0; iface < faces_vector.size(); iface++) {	
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

	      lines[4*iface].header.stamp = limage_.header.stamp;
	      lines[4*iface+1].header.stamp = limage_.header.stamp;
	      lines[4*iface+2].header.stamp = limage_.header.stamp;
	      lines[4*iface+3].header.stamp = limage_.header.stamp;
	      lines[4*iface].header.frame_id = limage_.header.frame_id;
	      lines[4*iface+1].header.frame_id = limage_.header.frame_id;
	      lines[4*iface+2].header.frame_id = limage_.header.frame_id;
	      lines[4*iface+3].header.frame_id = limage_.header.frame_id;
	  
	    }
	  }	
	} // End for iface
      }

    }

    // Display
    if (do_display_ == "remote") {
      all_cls.header.stamp = limage_.header.stamp;
      all_cls.label = node_name_;
      all_cls.header.frame_id = limage_.header.frame_id;
      all_cls.lines = lines;
      publish("lines_to_draw",all_cls);
    }
    else if (do_display_ == "local") {
      if (!cv_image_disp_out_) {
	cv_image_disp_out_ = cvCreateImage(im_size,IPL_DEPTH_8U,1);
      }
      cvCvtScale(cv_image_disp_,cv_image_disp_out_,4.0/dispinfo_.dpp);
      cvShowImage("Face detector: Face Detection",cv_image_left_);
      cvShowImage("Face detector: Disparity",cv_image_disp_out_);

      cv_mutex_.unlock();
    }
    // Done display

    endt = t.now();
    ros::Duration diff = endt-startt;
    printf("Image callback duration = %fsec\n", diff.toSec());
  }


  /*!
   *\brief Wait for completion, wait for user input, display images.
   */
  bool spin() {
    while (ok() && !quit_) {

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
  string do_display = "none";
  bool external_init = true;

  if (argc < 2) {
    cerr << "A path to the file containing the experiment list is required.\n" << endl;
    return 0;
  }
  if (argc >= 3) {
    do_display = argv[2];
    if (argc >= 4) {
      external_init = atoi(argv[3]);
    }
  }

  ifstream expfile(argv[1]);
  if (!expfile.is_open()) {
    cerr << "File " << argv[1] << " cannot be opened" << endl;
    return 0;
  }
  int numlines;
  expfile >> numlines;
  double reliabilities[numlines];
  string haar_filenames[numlines];
  string names[numlines];
  for (int i=0; i<numlines; i++) {
    if (expfile.eof()) {
      cerr << "File " << argv[1] << " has an incorrect number of lines." << endl;
      return 0;
    }
    expfile >> names[i];
    expfile >> haar_filenames[i];
    expfile >> reliabilities[i];
  }
  

  ostringstream node_name;
  node_name << "face_detection"; //_" << argv[1];
  FaceDetector fd(node_name.str(), numlines, names, haar_filenames, reliabilities, use_depth, do_display, external_init);

  fd.spin();
  ros::fini();
  return 0;
}


