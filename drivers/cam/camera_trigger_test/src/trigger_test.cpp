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

class TriggerTest
{
private:
  image_msgs::Image img_msg_;
  image_msgs::CvBridge img_bridge_;
  std::string window_name_;
  ros::Node &node_;
  int mode_;
  FILE *outfile_;
  std::string led_set_waveform_;
  std::string cam_set_waveform_;
  robot_mechanism_controllers::SetWaveform::Request led_config_, cam_config_;
  robot_mechanism_controllers::SetWaveform::Response dummy_resp_;
  int ignore_count_start_;
  int ignore_count_;
  int num_repetitions_;
  int reps_;

public:
  TriggerTest(ros::Node &node) : node_(node), outfile_(NULL)
  {
    // Subscribe to image stream.

    node_.subscribe("image", img_msg_, &TriggerTest::image_cb, this, 1);
    
    // Read operating mode.
    
    node_.param("~mode", mode_, 0);
    if (mode_ < 0 || mode_ >= 2)
    {
      ROS_FATAL("mode parameter must be 0 or 1.");
      node_.shutdown();
      return;
    }

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

    // Get names for waveform generators.
  
    sleep(2); // Otherwise the others aren't ready.

    node_.param("led_controller", led_set_waveform_, (std::string) "led_controller");    
    node_.param("cam_controller", cam_set_waveform_, (std::string) "cam_controller");    
    led_set_waveform_ += "/set_waveform";
    cam_set_waveform_ += "/set_waveform";

    // Initialize waveform generators.
    
    led_config_.running = 1;
    led_config_.rep_rate = mode_ ? .5 : 10; 
    led_config_.phase = 0;
    led_config_.active_low = 1;
    led_config_.pulsed = mode_ ? 0 : 1;
    led_config_.duty_cycle = .5;

    cam_config_.running = 1;
    cam_config_.rep_rate = mode_ ? 5 : 10; 
    cam_config_.phase = 0;
    cam_config_.active_low = 0;
    cam_config_.pulsed = 1;
    cam_config_.duty_cycle = .5;
    
    SetWaveforms();

    // Various remaining initializations.
    node_.param("~ignore_count", ignore_count_start_, 5);
    ignore_count_ = 5;
    node_.param("~repetitions", num_repetitions_, 10);
    reps_ = 0;
  }

  ~TriggerTest()
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

  void SetWaveforms()
  {
    SetWaveform(cam_set_waveform_, cam_config_);
    SetWaveform(led_set_waveform_, led_config_);
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
    
    ROS_INFO("intensity: %f, ignore: %i, reps: %i, mode: %i", intensity, ignore_count_, reps_, mode_);

    if (ignore_count_) // Ignore first frames after a waveform change.
      ignore_count_--;
    else
    {
      switch (mode_)
      {
        case 0: // Steadily increase phase of LED. Record intensity.
          if (reps_ > num_repetitions_)
          {
            reps_ = 0;
            ignore_count_ = ignore_count_start_;
            led_config_.phase += 0.001 * led_config_.rep_rate;
            SetWaveforms();
            if (led_config_.phase >= 1)
            {
              ROS_INFO("Finished collecting data.");
              node_.shutdown();
              break;
            }
            ROS_INFO("delay: %f", led_config_.phase / led_config_.rep_rate);
          }
          reps_++;
          if (led_config_.phase < 1)
            fprintf(outfile_, "delay: %f intensity: %f\n", 
                led_config_.phase / led_config_.rep_rate, intensity);
          fflush(outfile_);
          break;

        case 1:
          double curtime = realtime_gettime();
          double curphase = fmod(curtime * led_config_.rep_rate - led_config_.phase, 1);
          fprintf(outfile_, "delay: %f intensity: %f led: %i\n", curphase / led_config_.rep_rate, 
              intensity, curphase < led_config_.duty_cycle);
          fflush(outfile_);
          break;
      }
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node n("trigger_test");
  TriggerTest tt(n);
  n.spin();
  
  return 0;
}
