/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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

//! \author Vijay Pradeep

#include <ros/ros.h>
#include <laser_cb_detector/laser_cb_detector.h>
#include <sstream>

using namespace laser_cb_detector;
using namespace std;


#define ROS_INFO_CONFIG(name) \
{\
  ostringstream ss;\
  ss << "[" << #name << "] -> " << config.name;\
  ROS_INFO(ss.str().c_str());\
}



laser_cb_detector::ConfigGoal getParamConfig(ros::NodeHandle &n)
{
  laser_cb_detector::ConfigGoal config;

  int num_x;
  n.param("~num_x", num_x, 3);
  config.num_x = (unsigned int) num_x;

  int num_y;
  n.param("~num_y", num_y, 3);
  config.num_y = (unsigned int) num_y;

  double spacing_x;
  n.param("~spacing_x", spacing_x, 1.0);
  config.spacing_x = spacing_x;

  double spacing_y;
  n.param("~spacing_y", spacing_y, 1.0);
  config.spacing_y = spacing_y;

  double width_scaling;
  n.param("~width_scaling",  width_scaling,  1.0);
  config.width_scaling = width_scaling;

  double height_scaling;
  n.param("~height_scaling", height_scaling, 1.0);
  config.height_scaling = height_scaling;

  double min_intensity;
  n.param("~min_intensity", min_intensity, 500.0);
  config.min_intensity = min_intensity;

  double max_intensity;
  n.param("~max_intensity", max_intensity, 5000.0);
  config.max_intensity = max_intensity;

  int subpixel_window;
  n.param("~subpixel_window", subpixel_window, 2);
  config.subpixel_window = config.subpixel_window;

  n.param("~subpixel_zero_zone", config.subpixel_zero_zone, -1);

  ROS_INFO_CONFIG(num_x);
  ROS_INFO_CONFIG(num_y);
  ROS_INFO_CONFIG(spacing_x);
  ROS_INFO_CONFIG(spacing_y);
  ROS_INFO_CONFIG(width_scaling);
  ROS_INFO_CONFIG(height_scaling);
  ROS_INFO_CONFIG(min_intensity);
  ROS_INFO_CONFIG(max_intensity);
  ROS_INFO_CONFIG(subpixel_window);
  ROS_INFO_CONFIG(subpixel_zero_zone);

  return config;
}

void snapshotCallback(ros::Publisher* pub, LaserCbDetector* detector, const calibration_msgs::DenseLaserSnapshotConstPtr& msg)
{
  bool detect_result;
  camera_calibration::CalibrationPattern result;
  detect_result = detector->detect(*msg, result);

  if (!detect_result)
    ROS_ERROR("Error during checkerboard detection. (This error is worse than simply not seeing a checkerboard");
  else
  {
    pub->publish(result);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_cb_detector");

  ros::NodeHandle n;

  // Set up the LaserCbDetector
  laser_cb_detector::ConfigGoal config = getParamConfig(n);
  LaserCbDetector detector;
  detector.configure(config);

  // Output
  ros::Publisher pub = n.advertise<camera_calibration::CalibrationPattern>("laser_checkerboard", 1);

  // Input
  boost::function<void (const calibration_msgs::DenseLaserSnapshotConstPtr&)> cb
      = boost::bind(&snapshotCallback, &pub, &detector, _1);

  ros::Subscriber sub = n.subscribe(std::string("snapshot"), 1, cb);

  ros::spin();
}
