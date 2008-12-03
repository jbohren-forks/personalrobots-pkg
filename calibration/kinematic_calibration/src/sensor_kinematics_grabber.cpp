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
#include "robot_msgs/MechanismState.h"
#include "phase_space/PhaseSpaceSnapshot.h"

#include "std_msgs/PointCloud.h"

#include "image_msgs/Image.h"
#include "image_msgs/StereoInfo.h"
#include "image_msgs/CamInfo.h"
#include "kinematic_calibration/SensorKinematics.h"

#include "topic_synchronizer.h"         // From package topic_synchronizer

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
class SensorKinematicsGrabber : public ros::node
{

public:

  TopicSynchronizer<SensorKinematicsGrabber> dcam_sync_ ;

  robot_msgs::MechanismState mech_state_ ;
  robot_msgs::MechanismState safe_mech_state_ ;
  ros::thread::mutex mech_state_lock_ ;

  // dcam subscription callback messages
  image_msgs::Image left_image_ ;
  image_msgs::Image right_image_ ;
  image_msgs::Image disparity_image_ ;
  image_msgs::StereoInfo stereo_info_ ;
  image_msgs::CamInfo left_info_ ;
  image_msgs::CamInfo right_info_ ;

  // Mutexed dcam messages
  image_msgs::Image safe_left_image_ ;
  image_msgs::Image safe_right_image_ ;
  image_msgs::Image safe_disparity_image_ ;
  image_msgs::StereoInfo safe_stereo_info_ ;
  image_msgs::CamInfo safe_left_info_ ;
  image_msgs::CamInfo safe_right_info_ ;
  ros::thread::mutex dcam_lock_ ;

  std_msgs::PointCloud laser_cloud_ ;
  std_msgs::PointCloud safe_laser_cloud_ ;
  ros::thread::mutex laser_cloud_lock_ ;
  
  // Parameters
  bool subscribe_color_ ;
  string output_topic_ ;                // Topic name for publishing our SensorKinematics metapacket
  
  SensorKinematicsGrabber() : ros::node("sensor_kinematics_grabber"),
                              dcam_sync_(this, &SensorKinematicsGrabber::dcamCallback, ros::Duration(0.05), &SensorKinematicsGrabber::dcamCallbackTimeout)
  {
    param("~subscribe_color", subscribe_color_, false);
    param("~output_topic", output_topic_, string("sensor_kinematics"));
    
    // Stereo Cam Synchronized Subscriptions
    if (subscribe_color_)
    {
      dcam_sync_.subscribe("dcam/left/image_rect_color", left_image_, 1) ;
      dcam_sync_.subscribe("dcam/right/image_rect_color", right_image_, 1) ;
    }
    else
    {
      dcam_sync_.subscribe("dcam/left/image_rect", left_image_, 1) ;
      dcam_sync_.subscribe("dcam/right/image_rect", right_image_, 1) ;
    }
    dcam_sync_.subscribe("dcam/disparity", disparity_image_, 1) ;
    dcam_sync_.subscribe("dcam/stereo_info", stereo_info_, 1) ;
    dcam_sync_.subscribe("dcam/right/cam_info", right_info_, 1) ;
    dcam_sync_.subscribe("dcam/left/cam_info", left_info_, 1) ;

    subscribe("mechanism_state", mech_state_, &SensorKinematicsGrabber::mechStateCallback, 2) ;
    subscribe("tilt_laser_cloud", laser_cloud_, &SensorKinematicsGrabber::laserCloudCallback, 1) ;
    
    advertise<SensorKinematics>(output_topic_, 1) ;
  }

  ~SensorKinematicsGrabber()
  {

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
    unsigned int capture_count = 0 ;
    
    while (ok() && !need_to_quit)
    {
      char c = getchar() ;

      switch (c)
      {
        case ' ':
        {
          SensorKinematics all_data ;
          printf("\n") ;
          capture_count++ ;
          printf("Capturing Data #%u...\n", capture_count) ;
          dcam_lock_.lock() ;
          all_data.left_image = safe_left_image_ ;
          all_data.right_image = safe_right_image_ ;
          all_data.disparity_image = safe_disparity_image_ ;
          all_data.stereo_info = safe_stereo_info_ ;
          all_data.left_info = safe_left_info_ ;
          all_data.right_info = safe_right_info_ ;
          all_data.left_info = safe_left_info_ ;
          dcam_lock_.unlock() ;
          
          laser_cloud_lock_.lock() ;
          all_data.laser_cloud = safe_laser_cloud_ ;
          laser_cloud_lock_.unlock() ;
          
          mech_state_lock_.lock() ;
          all_data.mechanism_state = safe_mech_state_ ;
          mech_state_lock_.unlock() ;
          
          displayAllInfo(all_data) ;
          
          publish(output_topic_, all_data) ;
          
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

  void dcamCallback(ros::Time t)
  {
    dcam_lock_.lock() ;

    safe_left_image_ = left_image_ ;
    safe_right_image_ = right_image_ ;
    safe_disparity_image_ = disparity_image_ ;
    safe_stereo_info_ = stereo_info_ ;
    safe_left_info_ = left_info_ ;
    safe_right_info_ = right_info_ ;
    
    dcam_lock_.unlock() ;
  }
  
  void dcamCallbackTimeout(ros::Time t)
  {
    ROS_WARN("SensorKinematicsGrabber::DCam Synchronizer Timeout") ;
  }
  
  
  /**
   * Display some basic information about all the different data types that we
   *  just captured. This lets us check if we're running into any serious
   *  timing issues and if we're actually receiving reasonable data from all of
   *  out sensors.
   */
  void displayAllInfo(const SensorKinematics& data)
  {
    ros::Time cur_time = ros::Time::now() ;

    ros::Duration lag ;
    
    lag = data.left_image.header.stamp-cur_time ;
    if (data.left_image.byte_data.layout.get_dim_size() < 2)
      printf("     Left Image:       %lf [NO DATA]\n", lag.to_double()) ;
    else
      printf("     Left Image:       %lf (%u, %u)\n", lag.to_double(), data.left_image.byte_data.layout.dim[0].size,
                                                                  data.left_image.byte_data.layout.dim[1].size) ;

    lag = data.right_image.header.stamp-cur_time ;
    if (data.right_image.byte_data.layout.get_dim_size() < 2)
      printf("     Right Image:      %lf [NO DATA]\n", lag.to_double()) ;
    else
      printf("     Right Image:      %lf (%u, %u)\n", lag.to_double(), data.right_image.byte_data.layout.dim[0].size,
                                                                  data.right_image.byte_data.layout.dim[1].size) ;
    
    lag = data.disparity_image.header.stamp-cur_time ;
    if (data.disparity_image.byte_data.layout.get_dim_size() < 2)
      printf("     Disparity Image:  %lf [NO DATA]\n", lag.to_double()) ;
    else
      printf("     Disparity Image:  %lf (%u, %u)\n", lag.to_double(), data.disparity_image.byte_data.layout.dim[0].size,
                                                                  data.disparity_image.byte_data.layout.dim[1].size) ;
    
    lag = data.stereo_info.header.stamp-cur_time ;
    printf("     Stereo Info:      %lf\n", lag.to_double()) ;

    lag = data.left_info.header.stamp-cur_time ;
    printf("     Left Info:        %lf\n", lag.to_double()) ;

    lag = data.right_info.header.stamp-cur_time ;
    printf("     Right Info:       %lf\n", lag.to_double()) ;
    
    lag = data.mechanism_state.header.stamp-cur_time ;
    printf("     Mechanism State:  %lf   %u Joints\n", lag.to_double(), data.mechanism_state.get_joint_states_size()) ;
    
    lag = data.laser_cloud.header.stamp-cur_time ;
    printf("     Laser Cloud:      %lf   %u points\n", lag.to_double(), data.laser_cloud.get_pts_size() ) ;
    
    lag = data.phase_space_snapshot.header.stamp-cur_time ;
    printf("     PhaseSpace:       %lf   %u Markers | %u Bodies\n", lag.to_double(), data.phase_space_snapshot.get_markers_size(),
                                                                                     data.phase_space_snapshot.get_bodies_size()) ;
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
