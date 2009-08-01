/*********************************************************************
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

#include "ros/node.h"
#include "boost/thread/mutex.hpp"
#include "mechanism_msgs/MechanismState.h"

#include "topic_synchronizer/topic_synchronizer.h"

#include "std_msgs/Empty.h"
#include "sensor_msgs/PointCloud.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

#include "kinematic_calibration/CalSnapshot.h"

#include <unistd.h>
#include <termios.h>

using namespace std ;

#define ADD_MSG(type, name)   \
  type name ;              \
  type safe_##name ;

#define TO_SAFE(var) safe_##var = var

namespace kinematic_calibration
{


/**
 * This node is kind of a hack, currently being used to grab stereo and tilt laser
 * data for doing the laser head calibration.
 */
class LaserHeadGrabber
{

public:

  ros::Node* node_ ;

  TopicSynchronizer<LaserHeadGrabber> stereo_sync_ ;
  TopicSynchronizer<LaserHeadGrabber> laser_sync_ ;

  // Define the output message
  CalSnapshot snapshot_msg_ ;

  // Mechanism State
  boost::mutex mech_state_lock_ ;
  ADD_MSG(mechanism_msgs::MechanismState, mech_state_) ;

  // Stereo Messages
  boost::mutex stereo_lock_ ;
  ADD_MSG(sensor_msgs::PointCloud, corners_left_) ;
  ADD_MSG(sensor_msgs::PointCloud, corners_right_) ;
  ADD_MSG(sensor_msgs::Image, left_debug_) ;
  ADD_MSG(sensor_msgs::Image, right_debug_) ;
  ADD_MSG(sensor_msgs::CameraInfo, left_info_) ;
  ADD_MSG(sensor_msgs::CameraInfo, right_info_) ;

  // Laser Messages
  boost::mutex laser_lock_ ;
  ADD_MSG(sensor_msgs::PointCloud, laser_measurement_) ;
  ADD_MSG(sensor_msgs::Image, laser_debug_) ;

  // Empty message used for callbacks
  std_msgs::Empty capture_msg_ ;

  unsigned int capture_count_ ;
  bool waiting_for_keypress_ ;
  boost::mutex keypress_lock_ ;

  LaserHeadGrabber(ros::Node* node) : node_(node),
                   stereo_sync_(node_, this, &LaserHeadGrabber::stereoCallback,
                                ros::Duration().fromSec(2.0),
                                &LaserHeadGrabber::stereoTimeout),
                   laser_sync_(node_, this, &LaserHeadGrabber::laserCallback,
                               ros::Duration().fromSec(2.0),
                               &LaserHeadGrabber::laserTimeout)

  {
    capture_count_ = 0 ;
    waiting_for_keypress_ = false ;

    node_->advertise<CalSnapshot>("~snapshots", 1) ;
    node_->advertise<sensor_msgs::Image>("~left", 1) ;
    node_->advertise<sensor_msgs::Image>("~right", 1) ;
    node_->advertise<sensor_msgs::Image>("~laser", 1) ;

    node_->subscribe("mechanism_state", mech_state_, &LaserHeadGrabber::mechStateCallback, this, 1) ;

    stereo_sync_.subscribe("stereo/left/cam_info",  left_info_, 1) ;
    stereo_sync_.subscribe("stereo/right/cam_info", right_info_, 1) ;
    stereo_sync_.subscribe("cb_corners_left",  corners_left_, 1) ;
    stereo_sync_.subscribe("cb_corners_right", corners_right_, 1) ;
    stereo_sync_.subscribe("left_cb_debug",  left_debug_, 1) ;
    stereo_sync_.subscribe("right_cb_debug", right_debug_, 1) ;

    laser_sync_.subscribe("checkerboard_corners_node/debug_image", laser_debug_, 1) ;
    laser_sync_.subscribe("/dense_tilt_scan/measured_corners", laser_measurement_, 1) ;

    stereo_sync_.ready() ;
    laser_sync_.ready() ;

    node_->subscribe("~capture", capture_msg_, &LaserHeadGrabber::captureCallback, this, 1) ;
  }

  ~LaserHeadGrabber()
  {

  }

  void captureCallback()
  {
/*
    CalibrationData2 all_data ;
    printf("\n") ;
    capture_count_++ ;
    printf("Capturing Data #%u...\n", capture_count_) ;
    raw_stereo_lock_.lock() ;
    all_data.raw_stereo = safe_raw_stereo_ ;
    raw_stereo_lock_.unlock() ;

    laser_cloud_lock_.lock() ;
    all_data.laser_cloud = safe_laser_cloud_ ;
    laser_cloud_lock_.unlock() ;

    mech_state_lock_.lock() ;
    all_data.mechanism_state = safe_mech_state_ ;
    mech_state_lock_.unlock() ;

    hi_res_lock_.lock() ;
    all_data.set_image_size(1) ;
    all_data.set_cam_info_size(1) ;
    all_data.image[0] = safe_hi_res_image_ ;
    all_data.cam_info[0] = safe_hi_res_info_ ;
    hi_res_lock_.unlock() ;



    displayAllInfo(all_data) ;

    publish("~calibration_data", all_data) ;*/
  }

  void mechStateCallback()
  {
    mech_state_lock_.lock() ;
    TO_SAFE(mech_state_) ;
    mech_state_lock_.unlock() ;
  }

  bool spin()
  {
    // Setup terminal settings for getchar
    const int fd = fileno(stdin);
    termios prev_flags ;
    tcgetattr(fd, &prev_flags) ;
    termios flags ;
    tcgetattr(fd,&flags);
    flags.c_lflag &= ~ICANON;  // set raw (unset canonical modes)
    flags.c_cc[VMIN]  = 0;     // i.e. min 1 char for blocking, 0 chars for non-blocking
    flags.c_cc[VTIME] = 0;     // block if waiting for char
    tcsetattr(fd,TCSANOW,&flags);

    bool need_to_quit = false ;

    while (node_->ok() && !need_to_quit)
    {
      char c = getchar() ;

      switch (c)
      {
        case 'y':
        case 'Y':
        {
          printf("\n") ;
          triggerCapture(true) ;
          break ;
        }
        case 'n':
        case 'N':
        {
          printf("\n") ;
          triggerCapture(false) ;
          break ;
        }
        case EOF:               // Means that we didn't catch any keyboard hits
        {
          usleep(1000) ;
          break ;
        }
        default:
        {
          printf("\nPress <Ctrl>-C to Exit\n") ;
          break ;
        }
      }
    } ;

    tcsetattr(fd,TCSANOW, &prev_flags) ;         // Undo any terminal changes that we made
    return true ;
  }


  void stereoCallback(ros::Time t)
  {
    //ROS_INFO("Stereo Callback - %f", t.toSec()) ;
    stereo_lock_.lock() ;

    TO_SAFE(left_info_) ;
    TO_SAFE(right_info_) ;
    TO_SAFE(corners_left_) ;
    TO_SAFE(corners_right_) ;
    TO_SAFE(left_debug_) ;
    TO_SAFE(right_debug_) ;

    stereo_lock_.unlock() ;
  }

  void stereoTimeout(ros::Time t)
  {
    ROS_WARN("Stereo Timeout - %f", t.toSec()) ;

    if (left_info_.header.stamp != t)
      printf("- left info      %f\n", left_info_.header.stamp.toSec()) ;
    else
      printf("+ left info      %f\n", left_info_.header.stamp.toSec()) ;

    if (right_info_.header.stamp != t)
      printf("- right info     %f\n", right_info_.header.stamp.toSec()) ;
    else
      printf("+ right info     %f\n", right_info_.header.stamp.toSec()) ;

    if (corners_left_.header.stamp != t)
      printf("- corners left   %f\n", corners_left_.header.stamp.toSec()) ;
    else
      printf("+ corners left   %f\n", corners_left_.header.stamp.toSec()) ;

    if (corners_right_.header.stamp != t)
      printf("- corners right  %f\n", corners_right_.header.stamp.toSec()) ;
    else
      printf("+ corners right  %f\n", corners_right_.header.stamp.toSec()) ;

    if (left_debug_.header.stamp != t)
      printf("- left debug     %f\n", left_debug_.header.stamp.toSec()) ;
    else
      printf("+ left debug     %f\n", left_debug_.header.stamp.toSec()) ;

    if (right_debug_.header.stamp != t)
      printf("- right debug    %f\n", right_debug_.header.stamp.toSec()) ;
    else
      printf("+ right debug    %f\n", right_debug_.header.stamp.toSec()) ;
  }

  void laserCallback(ros::Time t)
  {
    //ROS_INFO("Laser Callback - %f", t.toSec()) ;
    laser_lock_.lock() ;
    TO_SAFE(laser_measurement_) ;
    TO_SAFE(laser_debug_) ;
    laser_lock_.unlock() ;

    keypress_lock_.lock() ;
    if (waiting_for_keypress_)
    {
      printf("Throwing away new data!\n");
      printf("Still waiting for response (Y/N)...\n");
    }
    else
    {
      buildSnapshotMessage() ;
      waiting_for_keypress_ = true ;
      printf("Press Y to save data. Press N to discard...\n") ;
    }
    keypress_lock_.unlock() ;
  }

  void buildSnapshotMessage()
  {
    mech_state_lock_.lock() ;
    stereo_lock_.lock() ;
    laser_lock_.lock() ;
    snapshot_msg_.header.stamp = safe_laser_measurement_.header.stamp ;
    snapshot_msg_.mech_state = safe_mech_state_ ;

    // Build Laser Measurement Vector (Tilt angle, pointing angle, dist)
    SensorSample laser_sample ;
    laser_sample.m.resize(safe_laser_measurement_.pts.size()*3) ;
    for(unsigned int i=0; i<safe_laser_measurement_.pts.size(); i++)
    {
      laser_sample.m[3*i+0] = safe_laser_measurement_.pts[i].x ;
      laser_sample.m[3*i+1] = safe_laser_measurement_.pts[i].y ;
      laser_sample.m[3*i+2] = safe_laser_measurement_.pts[i].z ;
    }
    laser_sample.sensor = "tilt_laser" ;
    laser_sample.target = "6x8_cb" ;

    SensorSample left_cb_sample    = buildCamCornersSample(safe_corners_left_,  "stereo_left",  "6x8_cb") ;
    SensorSample right_cb_sample   = buildCamCornersSample(safe_corners_right_, "stereo_right", "6x8_cb") ;
    SensorSample left_info_sample  = buildCameraInfoSample(safe_left_info_,  "stereo_left_info","6x8_cb") ;
    SensorSample right_info_sample = buildCameraInfoSample(safe_right_info_, "stereo_right_info","6x8_cb") ;

    snapshot_msg_.samples.resize(5) ;
    snapshot_msg_.samples[0] = laser_sample ;
    snapshot_msg_.samples[1] = left_cb_sample ;
    snapshot_msg_.samples[2] = right_cb_sample ;
    snapshot_msg_.samples[3] = left_info_sample ;
    snapshot_msg_.samples[4] = right_info_sample ;

    node_->publish("~left", safe_left_debug_) ;
    node_->publish("~right", safe_right_debug_) ;
    node_->publish("~laser", safe_laser_debug_) ;

    laser_lock_.unlock() ;
    stereo_lock_.unlock() ;
    mech_state_lock_.unlock() ;
  }

  void triggerCapture(bool should_capture)
  {
    keypress_lock_.lock() ;
    if (waiting_for_keypress_)
    {
      if (should_capture)
      {
        capture_count_++ ;
        printf("%u - Publishing Sample\n", capture_count_) ;
        node_->publish("~snapshots", snapshot_msg_) ;
      }
      else
        printf("Throwing away sample\n") ;

      // No longer have fresh data, so we're no longer waiting for a user keypress
      waiting_for_keypress_ = false ;
    }
    else
      printf("No data needs review yet. Wait until new data is ready...\n") ;


    keypress_lock_.unlock() ;
  }


  SensorSample buildCamCornersSample(const sensor_msgs::PointCloud& corners, const string& sensor, const string& target)
  {
    SensorSample sample ;
    sample.sensor = sensor ;
    sample.target = target ;
    sample.m.resize(2*corners.pts.size()) ;
    for(unsigned int i=0; i<corners.pts.size(); i++)
    {
      sample.m[2*i+0] = corners.pts[i].x ;
      sample.m[2*i+1] = corners.pts[i].y ;
    }
    return sample ;
  }

  SensorSample buildCameraInfoSample(const sensor_msgs::CameraInfo& info, const string& sensor, const string& target)
  {
    SensorSample sample ;
    sample.sensor = sensor ;
    sample.target = target ;
    sample.m.resize(info.P.size()) ;
    for(unsigned int i=0; i<info.P.size(); i++)
      sample.m[i] = info.P[i] ;
    return sample ;
  }


  void laserTimeout(ros::Time t)
  {
    ROS_WARN("Laser Timeout - %f", t.toSec()) ;

    if (laser_measurement_.header.stamp != t)
      printf("- Laser Measurement %f\n", laser_measurement_.header.stamp.toSec()) ;
    else
      printf("+ Laser Measurement %f\n", laser_measurement_.header.stamp.toSec()) ;

    if (laser_debug_.header.stamp != t)
      printf("- Laser Debug       %f\n", laser_debug_.header.stamp.toSec()) ;
    else
      printf("+ Laser Debug       %f\n", laser_debug_.header.stamp.toSec()) ;
  }

} ;

}

using namespace kinematic_calibration ;

int main(int argc, char **argv)
{
  ros::init(argc, argv) ;

  ros::Node node("laser_head_grabber") ;

  LaserHeadGrabber grabber(&node) ;
  grabber.spin() ;

  return 0 ;
}
