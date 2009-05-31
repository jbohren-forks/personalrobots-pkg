/*********************************************************************
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
#include <stdint.h>

#include "ros/node.h"
#include "ros/console.h"
#include "CvStereoCamModel.h"
#include <people/PositionMeasurement.h>
#include "image_msgs/StereoInfo.h"
#include "image_msgs/DisparityInfo.h"
#include "image_msgs/CamInfo.h"
#include "image_msgs/Image.h"
#include "opencv_latest/CvBridge.h"
#include "image_msgs/ColoredLines.h"
#include "topic_synchronizer/topic_synchronizer.h"
#include "tf/transform_listener.h"
#include <tf/message_notifier.h>

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <boost/thread/mutex.hpp>

#include "utils.h"

namespace people
{

  bool DEBUG_DISPLAY = false;

  using namespace std;

  // PedestrianDetectorHOG - a wrapper around the Dalal-Triggs HOG pedestrian detector in OpenCV, plus the use of 3d information.

  class PedestrianDetectorHOG{
  public:

    // ROS
    ros::Node *node_;

    // Images and conversion
    image_msgs::Image limage_;
    image_msgs::Image dimage_;
    image_msgs::StereoInfo stinfo_;
    image_msgs::DisparityInfo dispinfo_;
    image_msgs::CamInfo rcinfo_;
    image_msgs::CvBridge lbridge_;
    image_msgs::CvBridge dbridge_;
    TopicSynchronizer<PedestrianDetectorHOG> *sync_;

    tf::TransformListener *tf_;
    tf::MessageNotifier<people::PositionMeasurement>* message_notifier_person_;
    string fixed_frame_;

    double hit_threshold_;
    int group_threshold_;

    bool use_depth_;
  
    bool do_display_;

    /////////////////////////////////////////////////////////////////
    // Constructor
    PedestrianDetectorHOG(ros::Node *node): node_(node) {
      
      // Get parameters from the server
      node_->param("/people/pedestrian_detector_HOG/do_display", do_display_, true);
      node_->param("/people/pedestrian_detector_HOG/use_depth", use_depth_, false);
      node_->param("/people/pedestrian_detector_HOG/hit_threshold",hit_threshold_,0.0);
      node_->param("/people/pedestrian_detector_HOG/group_threshold",group_threshold_,2);
       
      // TODO: Initialize OpenCV structures.

      // Advertise a 3d position measurement for each head.
      node_->advertise<people::PositionMeasurement>("people_tracker_measurements",1);

      // Advertise the display boxes.
      if (do_display_) {
	node_->advertise<image_msgs::ColoredLines>("lines_to_draw",1);
	ROS_INFO_STREAM_NAMED("pedestrian_detector_HOG","Advertising colored lines to draw remotely.");
	cv::namedWindow("people detector", 1);
      }

      // Subscribe to the images  
      if (use_depth_) {  
	sync_ = new TopicSynchronizer<PedestrianDetectorHOG>(node_, this, &people::PedestrianDetectorHOG::imageCBAll, ros::Duration().fromSec(0.05), &PedestrianDetectorHOG::imageCBTimeout);
	sync_->subscribe("stereo/left/image_rect",limage_,1);
	sync_->subscribe("stereo/disparity",dimage_,1);
	sync_->subscribe("stereo/stereo_info",stinfo_,1);
	sync_->subscribe("stereo/disparity_info",dispinfo_,1);
	sync_->subscribe("stereo/right/cam_info",rcinfo_,1);
	sync_->ready();
      }
      else {
	node_->subscribe("stereo/left/image_rect",limage_,&people::PedestrianDetectorHOG::imageCB,this,1);
      }

    }
    

    /////////////////////////////////////////////////////////////////
    // Destructor
    ~PedestrianDetectorHOG(){
    }

    
    /////////////////////////////////////////////////////////////////////
    // The image callback when not all topics are sync'ed. Don't do anything, just wait for sync.
    void imageCBTimeout(ros::Time t) {
      ROS_DEBUG_STREAM_NAMED("pedestrian_detector_HOG","Message timeout");
    }

    /////////////////////////////////////////////////////////////////////
    // Image callback for an image without stereo, etc.
    void imageCB() {
      imageCBAll(limage_.header.stamp);
    }


    /////////////////////////////////////////////////////////////////////
    // Image callback
    void imageCBAll(ros::Time t)
    {
      // Convert the image to OpenCV
      if (!lbridge_.fromImage(limage_,"mono")) 
	return;
      IplImage *cv_image_left = lbridge_.toIpl();

      IplImage *cv_image_disp = NULL;
      if (use_depth_) {
	if (!dbridge_.fromImage(dimage_)) 
	  return;
	cv_image_disp = dbridge_.toIpl();
      }

      // TODO: Finish this function
      
      // TODO: Preprocess the image for the detector

      cv::Mat img(cv_image_left,false);
      cv::Vector<cv::Rect> found;
      if (!use_depth_) {
	// Run the HOG detector. Similar to samples/peopledetect.cpp.
	cv::HOGDescriptor hog;
	hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
	double t = (double)cv::getTickCount();
	hog.detectMultiScale(img, found, hit_threshold_, cv::Size(8,8), cv::Size(24,16), 1.05, group_threshold_);
	t = (double)cv::getTickCount() - t;
	ROS_DEBUG_STREAM_NAMED("pedestrian_detector", "Detection time = "<< t*1000./cv::getTickFrequency() <<"ms\n");

      }
      else {
	// TODO: Get the places to run the detector.
	// TODO: Run the HOG detector at certain locations.
      }
      
      // TODO: Publish the found people.

      for( int i = 0; i < (int)found.size(); i++ )
      {
	cv::Rect r = found[i];
	cv::rectangle(img, r.tl(), r.br(), cv::Scalar(0,255,0), 1);
      }
      //ostringstream fname;
      //fname << "/tmp/left_HOG_OpenCV_results/" << setfill('0') << setw(6) << counter << "L.jpg";
      //counter ++;
      //cv::imwrite(fname.str(), img);
      cv::imshow("people detector", img);
      cv::waitKey(3);

      // TODO: Release the sequence and related memory.

    }


  private:

    //int counter;

    /////////////////////////////////////////////////////////////////////
    // TODO: Get possible person positions and scales by considering the floor plane.
    // - Listen to the floor plane
    // - Project the floor plane into the image
    // - If the floor plane is not horizontal to the image, the image should probably be rotated to make the next step just use horizontal bands.
    // - At each pixel in the image, given the depth on the floor plane at that pixel, set the predicted height(s) and position for detection.
    // - Detect and return the people.


    /////////////////////////////////////////////////////////////////////
    // TODO: Get possible person positions and scales from leg detections.
    // - Listen to leg detections.
    // - Synchronize leg detections with this image frame.
    // - Transform the leg detections to the stereo_optical_frame.
    // - Project into the image.
    // - Extract a portion of the image around the detection. 
    // -- Note that the people are only 3/4 of the bbox size (96pix person --> 128pix box)
    // -- How to define the box? Big enough so leg can be at left-most or right-most edge?
    // -- How does this interact with the multiple detections needed at each position? If the box is too tight, will detection fail?
    // - Detect people at that location. 
    // - Return all of the detected people.


    /////////////////////////////////////////////////////////////////////
    // TODO: Get possible person positions and scales from stereo depth.
    // - I'm not sure this is possible with our current stereo setup. There are so many holes that detection will still be expensive.


    /////////////////////////////////////////////////////////////////////
    // TODO: Get possible person positions and scales from laser 3d information.

  }; // Class
}; // Namespace




/////////////////////////////////////////////////////////////////////

int main (int argc, char** argv)
{
  ros::init (argc, argv);

  ros::Node pd("pedestrian_detector_HOG");

  people::PedestrianDetectorHOG pdhog(&pd);

  pd.spin ();
  return (0);
}


