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
#include "image_msgs/CamInfo.h"
#include "diagnostic_updater/diagnostic_updater.h"

#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>

#include "prosilica.h"
#include "prosilica_cam/PolledImage.h"

// TODO: don't inherit from Node
class ProsilicaNode : public ros::Node
{
private:
  boost::scoped_ptr<prosilica::Camera> cam_;
  prosilica::AcquisitionMode mode_;
  image_msgs::Image img_;
  image_msgs::CamInfo cam_info_;
  bool running_;
  int count_;
  DiagnosticUpdater<ProsilicaNode> diagnostic_;

public:
  ProsilicaNode() : ros::Node("prosilica"), cam_(NULL),
                    running_(false), count_(0), diagnostic_(this)
  {
    prosilica::init();
    //diagnostic_.addUpdater( &ProsilicaNode::freqStatus );

    if (prosilica::numCameras() == 0)
      ROS_WARN("Found no cameras on local subnet\n");

    unsigned long guid = 0;

    if (hasParam("~guid"))
    {
      std::string guid_str;
      getParam("~guid", guid_str);
      guid = strtol(guid_str.c_str(), NULL, 0);
    }
      
    if (hasParam("~ip_address"))
    {
      std::string ip_str;
      getParam("~ip_address", ip_str);
      cam_.reset( new prosilica::Camera(ip_str.c_str()) );
      
      // Verify Guid is the one expected
      if (guid != 0 && guid != cam_->guid())
        throw prosilica::ProsilicaException("Guid does not match expected");
    }
    else if (guid != 0)
    {
      cam_.reset( new prosilica::Camera(guid) );
    }
    else {
      guid = prosilica::getGuid(0);
      cam_.reset( new prosilica::Camera(guid) );
    }

    cam_->setFrameCallback(boost::bind(&ProsilicaNode::publishImage, this, _1));

    // Acquisition control
    std::string mode_str;
    param("~acquisition_mode", mode_str, std::string("Continuous"));
    if (mode_str == std::string("Continuous"))
      mode_ = prosilica::Continuous;
    else if (mode_str == std::string("Triggered"))
      mode_ = prosilica::Triggered;
    else {
      ROS_FATAL("Unknown setting\n");
      shutdown();
    }
    
    // Feature control
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
    
    ROS_INFO("Found camera, guid = %lu\n", guid);
    advertise<image_msgs::Image>("~image", 1);
    advertise<image_msgs::CamInfo>("~cam_info", 1);
  }

  ~ProsilicaNode()
  {
    stop();
    prosilica::fini();
  }

  int start()
  {
    if (running_)
      return 0;

    cam_->start(mode_);
    if (mode_ == prosilica::Triggered)
      advertiseService("~poll", &ProsilicaNode::triggeredGrab);
    
    running_ = true;

    return 0;
  }

  int stop()
  {
    if (!running_)
      return 0;

    if (mode_ == prosilica::Triggered)
      unadvertiseService("~poll");
    cam_->stop();
    
    running_ = false;

    return 0;
  }
  
  void freqStatus(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Frequency Status";
    // TODO: complete this
  }

  bool spin()
  {
    // Start up the camera
    start();
    
    while (ok())
    {
      usleep(1000000);
      diagnostic_.update();
    }

    stop();

    return true;
  }

  bool triggeredGrab(prosilica_cam::PolledImage::Request &req,
                     prosilica_cam::PolledImage::Response &res)
  {
    tPvFrame* frame = cam_->grab(req.timeout_ms);
    if (!frame)
      return false;

    return frameToImage(frame, res.image);
    // TODO: publish while we're at it?
  }

private:
  static std::string bayerPatternString(tPvBayerPattern pattern)
  {
    static const char* patternStrings[] = { "bayer_rggb", "bayer_gbrg",
                                            "bayer_grbg", "bayer_bggr" };
    return patternStrings[pattern];
  }
  
  static bool frameToImage(tPvFrame* frame, image_msgs::Image &image)
  {
    // NOTE: 16-bit formats and Yuv444 not supported yet
    switch (frame->Format)
    {
      case ePvFmtMono8:
        fillImage(image, "image", frame->Height, frame->Width, 1,
                  "mono", "uint8", frame->ImageBuffer);
        break;
      case ePvFmtBayer8:
        /*
        fillImage(image, "image", frame->Height, frame->Width, 1,
                  bayerPatternString(frame->BayerPattern), "uint8",
                  frame->ImageBuffer);
        */
        
        // Debayer to bgr format so CvBridge can handle it
        image.label = "image";
        image.encoding = "bgr";
        image.depth = "uint8";
        image.uint8_data.layout.dim.resize(3);
        image.uint8_data.layout.dim[0].label = "height";
        image.uint8_data.layout.dim[0].size = frame->Height;
        image.uint8_data.layout.dim[0].stride = frame->Height * (frame->Width * 3);
        image.uint8_data.layout.dim[1].label = "width";
        image.uint8_data.layout.dim[1].size = frame->Width;
        image.uint8_data.layout.dim[1].stride = frame->Width * 3;
        image.uint8_data.layout.dim[2].label = "channel";
        image.uint8_data.layout.dim[2].size = 3;
        image.uint8_data.layout.dim[2].stride = 3;
        image.uint8_data.data.resize(frame->Height * (frame->Width * 3));

        PvUtilityColorInterpolate(frame, &image.uint8_data.data[2],
                                  &image.uint8_data.data[1], &image.uint8_data.data[0],
                                  2, 0);

        break;
      case ePvFmtRgb24:
        fillImage(image, "image", frame->Height, frame->Width, 3,
                  "rgb", "uint8", frame->ImageBuffer);
        break;
      case ePvFmtYuv411:
        fillImage(image, "image", frame->Height, frame->Width, 6,
                  "yuv411", "uint8", frame->ImageBuffer);
        break;
      case ePvFmtYuv422:
        fillImage(image, "image", frame->Height, frame->Width, 4,
                  "yuv422", "uint8", frame->ImageBuffer);
        break;
      case ePvFmtBgr24:
        fillImage(image, "image", frame->Height, frame->Width, 3,
                  "bgr", "uint8", frame->ImageBuffer);
        break;
      case ePvFmtRgba32:
        fillImage(image, "image", frame->Height, frame->Width, 4,
                  "rgba", "uint8", frame->ImageBuffer);
        break;
      case ePvFmtBgra32:
        fillImage(image, "image", frame->Height, frame->Width, 4,
                  "bgra", "uint8", frame->ImageBuffer);
        break;
      default:        
        ROS_WARN("Received frame with unsupported pixel format %d\n", frame->Format);
        return false;
    }

    //img_.header.stamp = ros::Time().fromNSec(??);
    return true;
  }
  
  void publishImage(tPvFrame* frame)
  {
    if (frameToImage(frame, img_))
      publish("~image", img_);

    //cam_info_.header.stamp =
    cam_info_.height = frame->Height;
    cam_info_.width = frame->Width;

    // TODO: read intrinsics out of camera user memory
    static const double D[5] = { -0.2991980605722184, -0.1027444035896514,
                                 5.7068998039573127e-03, -4.6529841471001079e-03, 0 };
    static const double K[9] = { 3.0829229234340064e+03, 0., 812.2004403609867040,
                                 0., 3.1068998579451186e+03, 821.9381780675055325,
                                 0., 0., 1. };
    static const double R[9] = { 1., 0., 0.,
                                 0., 1., 0.,
                                 0., 0., 1. };
    static const double P[12] = { 3.0829229234340064e+03, 0., 812.2004403609867040, 0.,
                                  0., 3.1068998579451186e+03, 821.9381780675055325, 0.,
                                  0., 0., 1., 0. };
    memcpy((char*)(&cam_info_.D[0]), (char*)D, sizeof(D));
    memcpy((char*)(&cam_info_.K[0]), (char*)K, sizeof(K));
    memcpy((char*)(&cam_info_.R[0]), (char*)R, sizeof(R));
    memcpy((char*)(&cam_info_.P[0]), (char*)P, sizeof(P));

    publish("~cam_info", cam_info_);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);

  ProsilicaNode node;
  node.spin();

  return 0;
}
