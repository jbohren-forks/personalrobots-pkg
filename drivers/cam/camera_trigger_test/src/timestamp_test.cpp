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

#include "ros/node.h"
#include "image_msgs/Image.h"
#include "image_msgs/CvBridge.h"
#include <stdio.h>
#include <signal.h>
#include <robot_mechanism_controllers/SetWaveform.h>
#include <realtime_tools/realtime_tools.h>
#include <robot_mechanism_controllers/trigger_controller.h>

class TimestampTest
{
private:
  image_msgs::Image img_msg_;
  image_msgs::CvBridge img_bridge_;
  std::string window_name_;
  ros::Node &node_;
  double rate_;
  FILE *outfile_;
  std::string led_set_waveform_;
  controller::trigger_configuration led_config_;
  robot_mechanism_controllers::SetWaveform::Response dummy_resp_;

public:
  TimestampTest(ros::Node &node) : node_(node), outfile_(NULL)
  {
    // Open the output file.

    std::string filename;
    if (node_.hasParam("~file_name"))
      node_.getParam("~file_name", filename);
    else
    {
      ROS_FATAL("file_name parameter must be specified.");
      node_.shutdown();
      return;
    }

    outfile_ = fopen(filename.c_str(), "w");
    if (!outfile_)
    {
      ROS_FATAL("Error opening file \"%s\".", filename.c_str());
      node_.shutdown();
      return;
    }

    // Get names for waveform generator.
  
    sleep(2); // Otherwise the others aren't ready.

    node_.param("led_controller", led_set_waveform_, (std::string) "led_controller");    
    led_set_waveform_ += "/set_waveform";

    // Initialize waveform generators.
    
    node_.param("~rate", rate_, 0.);
    
    led_config_.running = 1;
    led_config_.rep_rate = rate_;
    led_config_.phase = 0;
    led_config_.active_low = 1;
    led_config_.pulsed = 1;
    led_config_.duty_cycle = .5;

    SetWaveform(led_set_waveform_, led_config_);

    // Subscribe to image stream.

    node_.subscribe("image", img_msg_, &TimestampTest::image_cb, this, 1);
  }

  ~TimestampTest()
  {
    if (outfile_) 
      fclose(outfile_);
  }

  void SetWaveform(std::string s, robot_mechanism_controllers::SetWaveform::Request req)
  {
    ROS_DEBUG("Calling \"%s\"", s.c_str());
    if (!ros::service::call(s, req, dummy_resp_))
    {
      ROS_FATAL("Error calling \"%s\"", s.c_str());
      node_.shutdown();
    }
  }

  void image_cb()
  {
    // Compute image intensity.
    
    if (img_msg_.encoding.find("bayer") != std::string::npos)
      img_msg_.encoding = "mono";
    
    long long sum = 0;
    
    if (img_bridge_.fromImage(img_msg_, "mono"))
    {
      std::vector<unsigned char> data = img_msg_.uint8_data.data;
      int pixels = img_msg_.uint8_data.layout.dim[0].size * img_msg_.uint8_data.layout.dim[1].size;

      for (int i = 0; i < pixels; i++)
      {
        sum += data[i];
      }
      
      //ROS_INFO("Sum: %f", sum / 7e6);
    }
    
    double intensity = sum / 7e6;

    // Control logic
    
    static double prevtime = 0;
    double curtime = img_msg_.header.stamp.toSec();
    double tickTime = controller::TriggerController::getTickStartTimeSec(curtime, led_config_);

    static int skip = 100;
    if (skip)
      skip--;
    else
      fprintf(outfile_, "time: %f intensity: %f led_time: %f delta: %f\n", curtime, intensity, tickTime, curtime - prevtime); 
    fflush(outfile_);
    prevtime = img_msg_.header.stamp.toSec();
  }                 
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node n("timestamp_test");
  TimestampTest tt(n);
  n.spin();
  
  return 0;
}
