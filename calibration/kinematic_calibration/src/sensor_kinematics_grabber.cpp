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
#include "robot_msgs/MechanismState.h"
#include "robot_msgs/MocapSnapshot.h"

#include "std_msgs/Empty.h"
#include "std_msgs/PointCloud.h"

#include "image_msgs/RawStereo.h"
#include "kinematic_calibration/CalibrationData.h"

#include <unistd.h>
#include <termios.h>

using namespace std ;

namespace kinematic_calibration
{


/**
 * This node is used to collect bagfiles that can then be used offline
 * for kinematic calibration. It collects point clouds, stereocam data, and
 * mechanism state. When the <spacebar> is hit, this node published a
 * "meta-packet" that contains the latest information from all of these data
 * sources.
 *
 * And standard use case would be to capture sensor data of still checkerboard
 * in the environment and then hit spacebar to record all the sensor data at once
 * to a bag file.  We can then move the checkerboard and robot, and capture a new
 * datapoint.
 */
class SensorKinematicsGrabber : public ros::Node
{

public:

  // Mechanism State Messages
  robot_msgs::MechanismState mech_state_ ;
  robot_msgs::MechanismState safe_mech_state_ ;
  boost::mutex mech_state_lock_ ;

  // Empty message used for callbacks
  std_msgs::Empty capture_msg_ ;

  // Dcam messages
  image_msgs::RawStereo raw_stereo_ ;
  image_msgs::RawStereo safe_raw_stereo_ ;
  boost::mutex raw_stereo_lock_ ;

  // Point Cloud Messages
  std_msgs::PointCloud laser_cloud_ ;
  std_msgs::PointCloud safe_laser_cloud_ ;
  boost::mutex laser_cloud_lock_ ;

  unsigned int capture_count_ ;

  SensorKinematicsGrabber() : ros::Node("grabber")
  {
    capture_count_ = 0 ;

    subscribe("stereo/raw_stereo", raw_stereo_, &SensorKinematicsGrabber::rawStereoCallback, 1) ;
    subscribe("mechanism_state", mech_state_, &SensorKinematicsGrabber::mechStateCallback, 1) ;
    subscribe("tilt_laser_cloud", laser_cloud_, &SensorKinematicsGrabber::laserCloudCallback, 1) ;

    advertise<CalibrationData>("~calibration_data", 1) ;
    subscribe("~capture", capture_msg_, &SensorKinematicsGrabber::captureCallback, 1) ;
  }

  ~SensorKinematicsGrabber()
  {
    unsubscribe("~capture") ;
    unadvertise("~calibration_data") ;
    unsubscribe("tilt_laser_cloud") ;
    unsubscribe("mechanism_state") ;
    unsubscribe("stereo/raw_stereo") ;
  }

  void captureCallback()
  {
    CalibrationData all_data ;
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

    displayAllInfo(all_data) ;

    publish("~calibration_data", all_data) ;
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

    while (ok() && !need_to_quit)
    {
      char c = getchar() ;

      switch (c)
      {
        case ' ':
        {
          captureCallback() ;
          break ;
        }
        case EOF:               // Means that we didn't catch any keyboard hits
        {
          usleep(1000) ;
          break ;
        }
        default:
        {
          printf("Press <spacebar> to capture data. Press <Ctrl>-C to Exit\n") ;
          break ;
        }
      }
    } ;

    tcsetattr(fd,TCSANOW, &prev_flags) ;         // Undo any terminal changes that we made
    return true ;
  }

  void mechStateCallback()
  {
    mech_state_lock_.lock() ;
    safe_mech_state_ = mech_state_ ;
    mech_state_lock_.unlock() ;
  }

  void laserCloudCallback()
  {
    laser_cloud_lock_.lock() ;
    safe_laser_cloud_ = laser_cloud_ ;
    laser_cloud_lock_.unlock() ;
  }

  void rawStereoCallback()
  {
    raw_stereo_lock_.lock() ;
    safe_raw_stereo_ = raw_stereo_ ;
    raw_stereo_lock_.unlock() ;
  }

  /**
   * Display some basic information about all the different data types that we
   *  just captured. This lets us check if we're running into any serious
   *  timing issues and if we're actually receiving reasonable data from all of
   *  out sensors.
   */
  void displayAllInfo(const CalibrationData& data)
  {
    ros::Time cur_time = ros::Time::now() ;

    ros::Duration lag ;

    lag = data.raw_stereo.header.stamp-cur_time ;
    if (data.raw_stereo.left_image.uint8_data.layout.get_dim_size() < 2)
      printf("     Left Image:       %lf [NO DATA]\n", lag.toSec()) ;
    else
      printf("     Left Image:       %lf (%u, %u)\n", lag.toSec(), data.raw_stereo.left_image.uint8_data.layout.dim[0].size,
                                                                       data.raw_stereo.left_image.uint8_data.layout.dim[1].size) ;
    if (data.raw_stereo.right_image.uint8_data.layout.get_dim_size() < 2)
      printf("     Right Image:      %lf [NO DATA]\n", lag.toSec()) ;
    else
      printf("     Right Image:      %lf (%u, %u)\n", lag.toSec(), data.raw_stereo.right_image.uint8_data.layout.dim[0].size,
                                                                       data.raw_stereo.right_image.uint8_data.layout.dim[1].size) ;
    if (data.raw_stereo.disparity_image.uint8_data.layout.get_dim_size() < 2)
      printf("     Disparity Image:  %lf [NO DATA]\n", lag.toSec()) ;
    else
      printf("     Disparity Image:  %lf (%u, %u)\n", lag.toSec(), data.raw_stereo.disparity_image.uint8_data.layout.dim[0].size,
                                                                       data.raw_stereo.disparity_image.uint8_data.layout.dim[1].size) ;

    lag = data.mechanism_state.header.stamp-cur_time ;
    printf("     Mechanism State:  %lf   %u Joints\n", lag.toSec(), data.mechanism_state.get_joint_states_size()) ;

    lag = data.laser_cloud.header.stamp-cur_time ;
    printf("     Laser Cloud:      %lf   %u points\n", lag.toSec(), data.laser_cloud.get_pts_size() ) ;

    lag = data.mocap_snapshot.header.stamp-cur_time ;
    printf("     MoCap:       %lf   %u Markers | %u Bodies\n", lag.toSec(), data.mocap_snapshot.get_markers_size(),
                                                                                data.mocap_snapshot.get_bodies_size()) ;
  }

} ;

}

using namespace kinematic_calibration ;

int main(int argc, char **argv)
{
  ros::init(argc, argv) ;
  SensorKinematicsGrabber grabber ;
  grabber.spin() ;
  ros::fini() ;
  return 0 ;
}
