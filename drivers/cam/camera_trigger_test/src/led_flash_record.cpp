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

#include "ros/node_handle.h"
#include "sensor_msgs/Image.h"
#include "opencv_latest/CvBridge.h"
#include <stdio.h>
#include <signal.h>
#include <robot_mechanism_controllers/SetWaveform.h>
#include <realtime_tools/realtime_tools.h>
#include <robot_mechanism_controllers/trigger_controller.h>
#include <algorithm>
#include <boost/format.hpp>

class LedFlashTest
{
private:
  sensor_msgs::CvBridge img_bridge_;
  std::string window_name_;
  ros::NodeHandle &node_handle_;
  double rate_;
  std::string led_set_waveform_;
  controller::trigger_configuration led_config_;
  robot_mechanism_controllers::SetWaveform::Response dummy_resp_;
  int frame_;
  int skip_frames_;
  ros::Subscriber img_sub_;
  double avgintensity_;
  FILE *outfile_;

public:
  LedFlashTest(ros::NodeHandle &n) : node_handle_(n)
  {
    // Open the output file.

    // Get names for waveform generator.
  
    sleep(5); // Otherwise the others aren't ready. Plus let some frames go by...

    node_handle_.param("led_controller", led_set_waveform_, (std::string) "led_controller");    
    led_set_waveform_ += "/set_waveform";

    // Initialize waveform generators.
    
    node_handle_.param("~rate", rate_, 0.);
    
    led_config_.running = 1;
    led_config_.rep_rate = rate_;
    led_config_.phase = 0;
    led_config_.active_low = 1;
    led_config_.pulsed = 1;
    led_config_.duty_cycle = .5;

    SetWaveform(led_set_waveform_, led_config_);

    // Subscribe to image stream.
    img_sub_ = node_handle_.subscribe("image", 10, &LedFlashTest::image_cb, this);
  
    // Other parameters.
    node_handle_.param("~skip", skip_frames_, 10);
    frame_ = 0;
    if (skip_frames_ < 1)
      skip_frames_ = 1;

    avgintensity_ = 0;

    // File name
    std::string outfilename;
    if (!node_handle_.getParam("~file", outfilename))
    {
      ROS_FATAL("Need to specify ~file parameter");
      exit(-1);
    }
    outfile_ = fopen(outfilename.c_str(), "w");
    if (!outfile_)
    {
      ROS_FATAL("Error opening %s for writing.", outfilename.c_str());
      exit(-1);
    }
    ROS_INFO("Opened %s for writing.", outfilename.c_str());
  }

  ~LedFlashTest()
  {
  }

  void SetWaveform(std::string s, robot_mechanism_controllers::SetWaveform::Request req)
  {
    ROS_DEBUG("Calling \"%s\"", s.c_str());
    if (!ros::service::call(s, req, dummy_resp_))
    {
      ROS_FATAL("Error calling \"%s\"", s.c_str());
      node_handle_.shutdown();
    }
  }

  void image_cb(const sensor_msgs::Image::ConstPtr &img_msg_orig)
  {
    sensor_msgs::Image img_msg = *img_msg_orig; // Because we will be changing the encoding.
    frame_++;
    
    // Compute image intensity.

    if (img_msg.encoding.find("bayer") != std::string::npos)
      img_msg.encoding = "mono";
    
    long long sum = 0;
    
    if (img_bridge_.fromImage(img_msg, "mono"))
    {
      std::vector<unsigned char> data = img_msg.uint8_data.data;
      int pixels = img_msg.uint8_data.layout.dim[0].size * img_msg.uint8_data.layout.dim[1].size;

      for (int i = 0; i < pixels; i++)
      {
        sum += data[i];
      }
      
      //ROS_INFO("Sum: %f", sum / 7e6);
    }
    
    double intensity = sum / 7e6;

    if (frame_ <= skip_frames_)
    {
      avgintensity_ += intensity;
      return;
    }
    if (frame_ == skip_frames_ + 1)
    {
      ROS_INFO("Started recording.");
      avgintensity_ /= skip_frames_;
    }

    // Control logic
    
    double exp_time = img_msg.header.stamp.toSec();
    double led_time = controller::TriggerController::getTickStartTimeSec(exp_time, led_config_);
    avgintensity_ = avgintensity_ * .99 + intensity * .01;
    fprintf(outfile_, "delta: %f exp: %f led: %f int: %f Avgint: %f Filtint: %f\n", exp_time - led_time, exp_time, led_time, intensity, avgintensity_, intensity - avgintensity_);
    fflush(outfile_);
  }                 
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "timestamp_test");
  ros::NodeHandle n;
  LedFlashTest tt(n);
  ros::spin();
  
  return 0;
}
