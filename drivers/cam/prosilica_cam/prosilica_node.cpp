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

// TODO: doxygen mainpage

#include "ros/node.h"
#include "image_msgs/Image.h"
#include "image_msgs/FillImage.h"
#include "diagnostic_updater/diagnostic_updater.h"

#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>

#include "prosilica.h"

// TODO: don't inherit from Node?
class ProsilicaNode : public ros::Node
{
private:
  boost::scoped_ptr<prosilica::Camera> cam_;
  image_msgs::Image img_;
  bool running_;
  int count_;
  DiagnosticUpdater<ProsilicaNode> diagnostic_;

public:
  ProsilicaNode() : ros::Node("prosilica"), cam_(NULL),
                    running_(false), count_(0), diagnostic_(this)
  {
    prosilica::init();
    //diagnostic_.addUpdater( &ProsilicaNode::freqStatus );

    int num_cams = prosilica::numCameras();
    if (num_cams > 0)
    {
      uint64_t guid;
      if (hasParam("~guid"))
      {
        std::string guid_str;
        getParam("~guid", guid_str);
        guid = strtoll(guid_str.c_str(), NULL, 16);
      } else {
        guid = prosilica::getGuid(0);
      }

      cam_.reset( new prosilica::Camera(guid) );
      cam_->setFrameCallback(boost::bind(&ProsilicaNode::publishImage, this, _1));

      std::string auto_setting;

      param("~exposure_auto", auto_setting, std::string("Auto"));
      if (auto_setting == std::string("Auto"))
        cam_->setExposure(0, prosilica::Auto);
      else if (auto_setting == std::string("AutoOnce"))
        cam_->setExposure(0, prosilica::AutoOnce);
      else if (auto_setting == std::string("Manual"))
      {
        int val;
        getParam("~exposure", val);
        cam_->setExposure(val, prosilica::Manual);
      } else {
        ROS_FATAL("Unknown setting\n");
        shutdown();
      }

      param("~gain_auto", auto_setting, std::string("Auto"));
      if (auto_setting == std::string("Auto"))
        cam_->setGain(0, prosilica::Auto);
      else if (auto_setting == std::string("AutoOnce"))
        cam_->setGain(0, prosilica::AutoOnce);
      else if (auto_setting == std::string("Manual"))
      {
        int val;
        getParam("~gain", val);
        cam_->setGain(val, prosilica::Manual);
      } else {
        ROS_FATAL("Unknown setting\n");
        shutdown();
      }

      param("~whitebal_auto", auto_setting, std::string("Auto"));
      if (auto_setting == std::string("Auto"))
        cam_->setWhiteBalance(0, 0, prosilica::Auto);
      else if (auto_setting == std::string("AutoOnce"))
        cam_->setWhiteBalance(0, 0, prosilica::AutoOnce);
      else if (auto_setting == std::string("Manual"))
      {
        int blue, red;
        getParam("~whitebal_blue", blue);
        getParam("~whitebal_red", red);
        cam_->setWhiteBalance(blue, red, prosilica::Manual);
      } else {
        ROS_FATAL("Unknown setting\n");
        shutdown();
      }
      
      // TODO: other params
      // TODO: check any diagnostics?

      ROS_FATAL("Found camera, guid = %lu\n", guid);
      advertise<image_msgs::Image>("~image", 1);
    } else {
      ROS_FATAL("Found no Prosilica cameras\n");
      shutdown();
    }
  }

  ~ProsilicaNode()
  {
    stop();
    prosilica::fini();
  }

  // TODO: catch exceptions, warn, etc.
  int start()
  {
    if (running_)
      return 0;

    cam_->start();
    
    running_ = true;

    return 0;
  }

  int stop()
  {
    if (!running_)
      return 0;

    cam_->stop();
    
    running_ = false;

    return 0;
  }

  bool serviceCam()
  {
    return true;
  }
  
  void freqStatus(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Frequency Status";
    // TODO: complete this
  }

  // TODO: more like Hokuyo node or dcam node?
  bool spin()
  {
    // Start up the camera
    start();
    
    while (ok())
    {
      /*
      if (serviceCam())
        publishImage();
      */
      usleep(1000000);
      diagnostic_.update();
    }

    stop();

    return true;
  }

private:
  void publishImage(tPvFrame* frame)
  {
    //ROS_FATAL("Received frame, format = %d\n", frame->Format);

    if (frame->Format == ePvFmtRgb24)
    {
      fillImage(img_, "image", frame->Height, frame->Width, 3,
                "rgb", "uint8", frame->ImageBuffer);
    } else {
      ROS_WARN("Received frame with unsupported pixel format %d\n", frame->Format);
      return;
    }

    //img_.header.stamp = ros::Time().fromNSec(??);
    publish("~image", img_);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);

  ProsilicaNode node;
  node.spin();

  return 0;
}
