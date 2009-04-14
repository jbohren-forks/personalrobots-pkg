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

#include <ros/node.h>
#include <image_msgs/Image.h>
#include <image_msgs/CamInfo.h>
#include <image_msgs/FillImage.h>
#include <image_msgs/CvBridge.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <cv.h>
#include <cvwimage.h>

#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/tokenizer.hpp>
#include <boost/thread.hpp>
#include <string>

#include "prosilica/prosilica.h"
#include "prosilica_cam/PolledImage.h"

class ProsilicaNode
{
private:
  ros::Node &node_;
  
  boost::scoped_ptr<prosilica::Camera> cam_;
  prosilica::AcquisitionMode mode_;
  image_msgs::Image img_, rect_img_;
  image_msgs::CvBridge img_bridge_, rect_img_bridge_;
  image_msgs::CamInfo cam_info_;
  bool running_;
  DiagnosticUpdater<ProsilicaNode> diagnostic_;
  int count_;
  double desired_freq_;
  
  boost::mutex grab_mutex_;

  // calibration matrices
  double D_[5], K_[9], R_[9], P_[12];
  CvMat Dmat_, Kmat_, Rmat_, Pmat_;
  cv::WImageBuffer1_f undistortX_, undistortY_;
  cv::WImageBuffer1_f roiUndistortX_, roiUndistortY_;
  unsigned int region_x_, region_y_;
  bool calibrated_;

public:
  ProsilicaNode(ros::Node &node)
    : node_(node), cam_(NULL), running_(false), diagnostic_(this, &node_), count_(0)
  {
    prosilica::init();

    if (prosilica::numCameras() == 0)
      ROS_WARN("Found no cameras on local subnet");

    unsigned long guid = 0;

    if (node_.hasParam("~guid"))
    {
      std::string guid_str;
      node_.getParam("~guid", guid_str);
      guid = strtol(guid_str.c_str(), NULL, 0);
    }
      
    if (node_.hasParam("~ip_address"))
    {
      std::string ip_str;
      node_.getParam("~ip_address", ip_str);
      cam_.reset( new prosilica::Camera(ip_str.c_str()) );
      
      // Verify Guid is the one expected
      unsigned long cam_guid = cam_->guid();
      if (guid != 0 && guid != cam_guid)
        throw prosilica::ProsilicaException("Guid does not match expected");
      guid = cam_guid;
    }
    else if (guid != 0)
    {
      cam_.reset( new prosilica::Camera(guid) );
    }
    else {
      guid = prosilica::getGuid(0);
      cam_.reset( new prosilica::Camera(guid) );
    }
    ROS_INFO("Found camera, guid = %lu", guid);

    cam_->setFrameCallback(boost::bind(&ProsilicaNode::publishImage, this, _1));

    // Acquisition control
    std::string mode_str;
    node_.param("~acquisition_mode", mode_str, std::string("Continuous"));
    if (mode_str == std::string("Continuous")) {
      mode_ = prosilica::Continuous;
      desired_freq_ = 1; // make sure we get _something_
    }
    else if (mode_str == std::string("Triggered")) {
      mode_ = prosilica::Triggered;
      desired_freq_ = 0;
    }
    else {
      ROS_FATAL("Unknown setting");
      node_.shutdown();
      return;
    }
    
    // Feature control
    std::string auto_setting;
    node_.param("~exposure_auto", auto_setting, std::string("Auto"));
    if (auto_setting == std::string("Auto"))
      cam_->setExposure(0, prosilica::Auto);
    else if (auto_setting == std::string("AutoOnce"))
      cam_->setExposure(0, prosilica::AutoOnce);
    else if (auto_setting == std::string("Manual"))
    {
      int val;
      node_.getParam("~exposure", val);
      cam_->setExposure(val, prosilica::Manual);
    } else {
      ROS_FATAL("Unknown setting");
      node_.shutdown();
      return;
    }
    
    node_.param("~gain_auto", auto_setting, std::string("Auto"));
    if (auto_setting == std::string("Auto"))
      cam_->setGain(0, prosilica::Auto);
    else if (auto_setting == std::string("AutoOnce"))
      cam_->setGain(0, prosilica::AutoOnce);
    else if (auto_setting == std::string("Manual"))
    {
      int val;
      node_.getParam("~gain", val);
      cam_->setGain(val, prosilica::Manual);
    } else {
      ROS_FATAL("Unknown setting");
      node_.shutdown();
      return;
    }
    
    node_.param("~whitebal_auto", auto_setting, std::string("Auto"));
    if (auto_setting == std::string("Auto"))
      cam_->setWhiteBalance(0, 0, prosilica::Auto);
    else if (auto_setting == std::string("AutoOnce"))
      cam_->setWhiteBalance(0, 0, prosilica::AutoOnce);
    else if (auto_setting == std::string("Manual"))
    {
      int blue, red;
      node_.getParam("~whitebal_blue", blue);
      node_.getParam("~whitebal_red", red);
      cam_->setWhiteBalance(blue, red, prosilica::Manual);
    } else {
      ROS_FATAL("Unknown setting");
      node_.shutdown();
      return;
    }

    // Try to load intrinsics from on-camera memory
    loadIntrinsics();
    if (calibrated_)
      ROS_INFO("Loaded intrinsics from camera");
    else
      ROS_WARN("Failed to load intrinsics from camera");

    cam_->setRoiToWholeFrame();
    
    node_.advertise<image_msgs::Image>("~image", 1);
    if (calibrated_) {
      node_.advertise<image_msgs::Image>("~image_rect", 1);
      node_.advertise<image_msgs::CamInfo>("~cam_info", 1);
    }

    diagnostic_.addUpdater( &ProsilicaNode::freqStatus );
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
      node_.advertiseService("~poll", &ProsilicaNode::triggeredGrab, this);
    
    running_ = true;

    return 0;
  }

  int stop()
  {
    if (!running_)
      return 0;

    if (mode_ == prosilica::Triggered)
      node_.unadvertiseService("~poll");
    cam_->stop();
    
    running_ = false;

    return 0;
  }
  
  void freqStatus(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Frequency Status";

    double freq = (double)(count_)/diagnostic_.getPeriod();

    if (freq < (.9*desired_freq_))
    {
      status.level = 2;
      status.message = "Desired frequency not met";
    }
    else
    {
      status.level = 0;
      status.message = "Desired frequency met";
    }

    status.set_values_size(3);
    status.values[0].label = "Images in interval";
    status.values[0].value = count_;
    status.values[1].label = "Desired frequency";
    status.values[1].value = desired_freq_;
    status.values[2].label = "Actual frequency";
    status.values[2].value = freq;

    count_ = 0;
  }

  bool spin()
  {
    // Start up the camera
    start();
    
    while (node_.ok())
    {
      usleep(100000);
      diagnostic_.update();
    }

    stop();

    return true;
  }

  bool triggeredGrab(prosilica_cam::PolledImage::Request &req,
                     prosilica_cam::PolledImage::Response &res)
  {
    if (mode_ != prosilica::Triggered)
      return false;

    // TODO: need/don't need my own lock here?
    boost::mutex::scoped_lock guard(grab_mutex_);
    if (req.region_x || req.region_y || req.width || req.height) {
      cam_->setRoi(req.region_x, req.region_y, req.width, req.height);
    } else {
      cam_->setRoiToWholeFrame();
    }

    tPvFrame* frame = cam_->grab(req.timeout_ms);
    if (!frame)
      return false;

    if (calibrated_)
      return processFrame(frame, img_, res.image, res.cam_info);
    else
      return processFrame(frame, res.image, img_, res.cam_info);
  }

private:
  static std::string bayerPatternString(tPvBayerPattern pattern)
  {
    static const char* patternStrings[] = { "bayer_rggb", "bayer_gbrg",
                                            "bayer_grbg", "bayer_bggr" };
    return patternStrings[pattern];
  }

  static void setBgrLayout(image_msgs::Image &image, int width, int height)
  {
    image.label = "image";
    image.encoding = "bgr";
    image.depth = "uint8";
    image.uint8_data.layout.dim.resize(3);
    image.uint8_data.layout.dim[0].label = "height";
    image.uint8_data.layout.dim[0].size = height;
    image.uint8_data.layout.dim[0].stride = height * (width * 3);
    image.uint8_data.layout.dim[1].label = "width";
    image.uint8_data.layout.dim[1].size = width;
    image.uint8_data.layout.dim[1].stride = width * 3;
    image.uint8_data.layout.dim[2].label = "channel";
    image.uint8_data.layout.dim[2].size = 3;
    image.uint8_data.layout.dim[2].stride = 3;
    image.uint8_data.data.resize(height * (width * 3));
  }
  
  static bool frameToImage(tPvFrame* frame, image_msgs::Image &image)
  {
    // NOTE: 16-bit formats and Yuv444 not supported
    switch (frame->Format)
    {
      case ePvFmtMono8:
        fillImage(image, "image", frame->Height, frame->Width, 1,
                  "mono", "uint8", frame->ImageBuffer);
        break;
      case ePvFmtBayer8:
        // Debayer to bgr format so CvBridge can handle it
        setBgrLayout(image, frame->Width, frame->Height);
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
        ROS_WARN("Received frame with unsupported pixel format %d", frame->Format);
        return false;
    }

    return true;
  }

  bool rectifyFrame(tPvFrame* frame, image_msgs::Image &img, image_msgs::Image &rect_img)
  {
    // Currently assume BGR format so bridge.toIpl() image points to msg data buffer
    if (img.encoding != "bgr") {
      ROS_WARN("Couldn't rectify frame, unsupported encoding %s", img.encoding.c_str());
      return false;
    }
    
    // Prepare image buffer
    setBgrLayout(rect_img, frame->Width, frame->Height);
    if (!img_bridge_.fromImage(img, "bgr") ||
        !rect_img_bridge_.fromImage(rect_img, "bgr")) {
      ROS_WARN("Couldn't rectify frame, failed to convert");
      return false;
    }
    IplImage *xMap = NULL, *yMap = NULL;

    // If whole frame captured, use the full undistort maps
    if (frame->Width  == (unsigned long)undistortX_.Width() &&
        frame->Height == (unsigned long)undistortX_.Height()) {
      assert(frame->RegionX == 0 && frame->RegionY == 0);
      xMap = undistortX_.Ipl();
      yMap = undistortY_.Ipl();
    }
    // Try to reuse cached ROI undistort maps
    else if (!roiUndistortX_.IsNull() && !roiUndistortY_.IsNull() &&
             frame->Width  == (unsigned long)roiUndistortX_.Width() &&
             frame->Height == (unsigned long)roiUndistortX_.Height() &&
             frame->RegionX == region_x_ && frame->RegionY == region_y_) {
      xMap = roiUndistortX_.Ipl();
      yMap = roiUndistortY_.Ipl();
    }
    // Compute new ROI undistort maps
    else {
      region_x_ = frame->RegionX;
      region_y_ = frame->RegionY;
      roiUndistortX_.Allocate(frame->Width, frame->Height);
      cvSubS(undistortX_.View(region_x_, region_y_, frame->Width, frame->Height).Ipl(),
             cvScalar(region_x_), roiUndistortX_.Ipl());
      roiUndistortY_.Allocate(frame->Width, frame->Height);
      cvSubS(undistortY_.View(region_x_, region_y_, frame->Width, frame->Height).Ipl(),
             cvScalar(region_y_), roiUndistortY_.Ipl());

      xMap = roiUndistortX_.Ipl();
      yMap = roiUndistortY_.Ipl();
    }

    // Rectify image
    cvRemap( img_bridge_.toIpl(), rect_img_bridge_.toIpl(),
             xMap, yMap, CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS );
    return true;
  }
  
  bool processFrame(tPvFrame* frame, image_msgs::Image &img, image_msgs::Image &rect_img,
                    image_msgs::CamInfo &cam_info)
  {
    if (!frameToImage(frame, img))
      return false;

    if (calibrated_) {
      if (!rectifyFrame(frame, img, rect_img))
        return false;
      
      // Camera info (uses full-frame width/height)
      cam_info.width = undistortY_.Width();
      cam_info.height = undistortX_.Height();

      memcpy((char*)(&cam_info.D[0]), (char*)D_, sizeof(D_));
      memcpy((char*)(&cam_info.K[0]), (char*)K_, sizeof(K_));
      memcpy((char*)(&cam_info.R[0]), (char*)R_, sizeof(R_));
      memcpy((char*)(&cam_info.P[0]), (char*)P_, sizeof(P_));

      // If using ROI, need to translate principal point
      if (frame->RegionX != 0 || frame->RegionY != 0) {
        cam_info.K[2] -= frame->RegionX; //cx
        cam_info.K[5] -= frame->RegionY; //cy
        // TODO: cam_info.R, cam_info.P
      }
    }

    count_++;
    return true;
  }
  
  void publishImage(tPvFrame* frame)
  {
    ros::Time time = ros::Time::now();
    
    if (!processFrame(frame, img_, rect_img_, cam_info_))
      return;

    img_.header.stamp = time;
    node_.publish("~image", img_);
    if (calibrated_) {
      rect_img_.header.stamp = time;
      node_.publish("~image_rect", rect_img_);
      cam_info_.header.stamp = time;
      node_.publish("~cam_info", cam_info_);
    }
  }

  void loadIntrinsics()
  {
    calibrated_ = false;
    
    // Retrieve contents of user memory
    std::string buffer(prosilica::Camera::USER_MEMORY_SIZE, '\0');
    cam_->readUserMemory(&buffer[0], prosilica::Camera::USER_MEMORY_SIZE);

    // Separate into lines
    typedef boost::tokenizer<boost::char_separator<char> > Tok;
    boost::char_separator<char> sep("\n");
    Tok tok(buffer, sep);

    // Check "header"
    Tok::iterator iter = tok.begin();
    if (*iter++ != "# Prosilica camera intrinsics")
      return;

    // Read calibration matrices
    int width = 0, height = 0;
    int items_read = 0;
    static const int EXPECTED_ITEMS = 9 + 5 + 9 + 12;
    for (Tok::iterator ie = tok.end(); iter != ie; ++iter) {
      if (*iter == "width") {
        ++iter;
        width = atoi(iter->c_str());
      }
      else if (*iter == "height") {
        ++iter;
        height = atoi(iter->c_str());
      }
      else if (*iter == "camera matrix")
        for (int i = 0; i < 3; ++i) {
          ++iter;
          items_read += sscanf(iter->c_str(), "%lf %lf %lf",
                               &K_[3*i], &K_[3*i+1], &K_[3*i+2]);
        }
      else if (*iter == "distortion") {
        ++iter;
        items_read += sscanf(iter->c_str(), "%lf %lf %lf %lf %lf",
                             D_, D_+1, D_+2, D_+3, D_+4);
      }
      else if (*iter == "rectification")
        for (int i = 0; i < 3; ++i) {
          ++iter;
          items_read += sscanf(iter->c_str(), "%lf %lf %lf",
                               &R_[3*i], &R_[3*i+1], &R_[3*i+2]);
        }
      else if (*iter == "projection")
        for (int i = 0; i < 3; ++i) {
          ++iter;
          items_read += sscanf(iter->c_str(), "%lf %lf %lf %lf",
                               &P_[4*i], &P_[4*i+1], &P_[4*i+2], &P_[4*i+3]);
        }
    }

    // Check we got everything
    if (items_read != EXPECTED_ITEMS || width == 0 || height == 0)
      return;

    // Set up OpenCV structures
    cvInitMatHeader(&Kmat_, 3, 3, CV_64FC1, K_);
    cvInitMatHeader(&Dmat_, 1, 5, CV_64FC1, D_);
    cvInitMatHeader(&Rmat_, 3, 3, CV_64FC1, R_);
    cvInitMatHeader(&Pmat_, 3, 4, CV_64FC1, P_);

    undistortX_.Allocate(width, height);
    undistortY_.Allocate(width, height);
    cvInitUndistortMap( &Kmat_, &Dmat_, undistortX_.Ipl(), undistortY_.Ipl() );

    calibrated_ = true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node n("prosilica");
  ProsilicaNode pn(n);
  pn.spin();

  return 0;
}
