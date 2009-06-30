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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CamInfo.h>
#include <sensor_msgs/FillImage.h>
#include <opencv_latest/CvBridge.h>
#include <camera_calibration/pinhole.h>
#include <image_publisher/image_publisher.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

#include <cv.h>
#include <cvwimage.h>

#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/tokenizer.hpp>
#include <boost/thread.hpp>
#include <string>

#include "prosilica/prosilica.h"
#include "prosilica/rolling_sum.h"
#include "prosilica_cam/PolledImage.h"
#include "prosilica_cam/CamInfo.h"

class ProsilicaNode
{
private:
  ros::Node &node_;

  // Camera
  boost::scoped_ptr<prosilica::Camera> cam_;
  prosilica::AcquisitionMode mode_;
  bool running_;

  // ROS messages
  sensor_msgs::Image img_, rect_img_;
  sensor_msgs::CvBridge img_bridge_, rect_img_bridge_;
  sensor_msgs::CamInfo cam_info_;
  sensor_msgs::Image thumbnail_;
  int thumbnail_size_;
  std::string frame_id_;
  
  // Calibration data
  camera_calibration::PinholeCameraModel model_;
  //camera_calibration::PinholeCameraModel roi_model_;
  //unsigned int region_x_, region_y_;
  bool calibrated_;

  // Diagnostics
  DiagnosticUpdater<ProsilicaNode> diagnostic_;
  int count_;
  double desired_freq_;
  static const int WINDOW_SIZE = 5; // remember previous 5s
  unsigned long frames_dropped_total_, frames_completed_total_;
  RollingSum<unsigned long> frames_dropped_acc_, frames_completed_acc_;
  unsigned long packets_missed_total_, packets_received_total_;
  RollingSum<unsigned long> packets_missed_acc_, packets_received_acc_;

  // So we don't get burned by auto-exposure
  unsigned long last_exposure_value_;
  int consecutive_stable_exposures_;

public:
  ProsilicaNode(ros::Node &node)
    : node_(node), cam_(NULL), running_(false),
      diagnostic_(this, node_), count_(0),
      frames_dropped_total_(0), frames_completed_total_(0),
      frames_dropped_acc_(WINDOW_SIZE),
      frames_completed_acc_(WINDOW_SIZE),
      packets_missed_total_(0), packets_received_total_(0),
      packets_missed_acc_(WINDOW_SIZE),
      packets_received_acc_(WINDOW_SIZE)
  {
    prosilica::init();

    if (prosilica::numCameras() == 0)
      ROS_WARN("Found no cameras on local subnet");

    unsigned long guid = 0;

    // Specify which frame to add to message header
    node_.param("~frame_id", frame_id_, std::string("NO_FRAME")) ;

    // Acquisition control
    size_t buffer_size;
    std::string mode_str;
    node_.param("~acquisition_mode", mode_str, std::string("Continuous"));
    if (mode_str == std::string("Continuous")) {
      mode_ = prosilica::Continuous;
      // TODO: tighter bound than this minimal check
      desired_freq_ = 1; // make sure we get _something_
      buffer_size = prosilica::Camera::DEFAULT_BUFFER_SIZE;
    }
    else if (mode_str == std::string("Triggered")) {
      mode_ = prosilica::Triggered;
      desired_freq_ = 0;
      buffer_size = 1;
    }
    else {
      ROS_FATAL("Unknown setting");
      node_.shutdown();
      return;
    }

    // Determine which camera to use
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
      cam_.reset( new prosilica::Camera(ip_str.c_str(), buffer_size) );
      
      // Verify Guid is the one expected
      unsigned long cam_guid = cam_->guid();
      if (guid != 0 && guid != cam_guid)
        throw prosilica::ProsilicaException(ePvErrBadParameter,
                                            "Guid does not match expected");
      guid = cam_guid;
    }
    else if (guid != 0)
    {
      cam_.reset( new prosilica::Camera(guid, buffer_size) );
    }
    else {
      guid = prosilica::getGuid(0);
      cam_.reset( new prosilica::Camera(guid, buffer_size) );
    }
    ROS_INFO("Found camera, guid = %lu", guid);

    // Set appropriate frame callback
    if (mode_ == prosilica::Continuous)
      cam_->setFrameCallback(boost::bind(&ProsilicaNode::publishImage, this, _1));
    else
      cam_->setFrameCallback(boost::bind(&ProsilicaNode::normalizeCallback, this, _1));

    // Feature control
    bool auto_expose = true;
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
      auto_expose = false;
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
    
    node_.advertise<sensor_msgs::Image>("~image", 1);
    if (calibrated_) {
      node_.advertise<sensor_msgs::Image>("~image_rect", 1);
      node_.advertise<sensor_msgs::CamInfo>("~cam_info", 1);
      node_.advertiseService("~cam_info_service", &ProsilicaNode::camInfoService, this, 0);
    }

    node_.param("~thumbnail_size", thumbnail_size_, 128);
    node_.advertise<sensor_msgs::Image>("~thumbnail", 1);

    diagnostic_.addUpdater( &ProsilicaNode::freqStatus );
    diagnostic_.addUpdater( &ProsilicaNode::frameStatistics );
    diagnostic_.addUpdater( &ProsilicaNode::packetStatistics );
    diagnostic_.addUpdater( &ProsilicaNode::packetErrorStatus );

    // Auto-exposure tends to go wild the first few frames after startup
    if (mode_ == prosilica::Triggered && auto_expose)
      normalizeExposure();
  }

  ~ProsilicaNode()
  {
    stop();
    cam_.reset(); // must destroy Camera before calling prosilica::fini
    prosilica::fini();
  }

  int start()
  {
    if (running_)
      return 0;

    cam_->start(mode_);
    if (mode_ == prosilica::Triggered)
      node_.advertiseService("~poll", &ProsilicaNode::triggeredGrab, this, 0);
    
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
  
  void freqStatus(diagnostic_msgs::DiagnosticStatus& status)
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

  void frameStatistics(diagnostic_msgs::DiagnosticStatus& status)
  {
    status.name = "Frame Statistics";

    // Get stats from camera driver
    float frame_rate;
    unsigned long completed, dropped;
    cam_->getAttribute("StatFrameRate", frame_rate);
    cam_->getAttribute("StatFramesCompleted", completed);
    cam_->getAttribute("StatFramesDropped", dropped);

    // Compute rolling totals, percentages
    frames_completed_acc_.add(completed - frames_completed_total_);
    frames_completed_total_ = completed;
    unsigned long completed_recent = frames_completed_acc_.sum();
    
    frames_dropped_acc_.add(dropped - frames_dropped_total_);
    frames_dropped_total_ = dropped;
    unsigned long dropped_recent = frames_dropped_acc_.sum();

    float recent_ratio = float(completed_recent) / (completed_recent + dropped_recent);
    float total_ratio = float(completed) / (completed + dropped);

    // Set level based on recent % completed frames
    if (dropped_recent == 0) {
      status.level = 0;
      status.message = "No dropped frames";
    }
    else if (recent_ratio > 0.8f) {
      status.level = 1;
      status.message = "Some dropped frames";
    }
    else {
      status.level = 2;
      status.message = "Excessive proportion of dropped frames";
    }

    status.set_values_size(5);
    status.values[0].label = "Camera Frame Rate";
    status.values[0].value = frame_rate;
    status.values[1].label = "Recent % Frames Completed";
    status.values[1].value = recent_ratio * 100.0f;
    status.values[2].label = "Overall % Frames Completed";
    status.values[2].value = total_ratio * 100.0f;
    status.values[3].label = "Frames Completed";
    status.values[3].value = completed;
    status.values[4].label = "Frames Dropped";
    status.values[4].value = dropped;
  }

  void packetStatistics(diagnostic_msgs::DiagnosticStatus& status)
  {
    status.name = "Packet Statistics";

    // Get stats from camera driver
    unsigned long received, missed, requested, resent;
    cam_->getAttribute("StatPacketsReceived", received);
    cam_->getAttribute("StatPacketsMissed", missed);
    cam_->getAttribute("StatPacketsRequested", requested);
    cam_->getAttribute("StatPacketsResent", resent);

    // Compute rolling totals, percentages
    packets_received_acc_.add(received - packets_received_total_);
    packets_received_total_ = received;
    unsigned long received_recent = packets_received_acc_.sum();
    
    packets_missed_acc_.add(missed - packets_missed_total_);
    packets_missed_total_ = missed;
    unsigned long missed_recent = packets_missed_acc_.sum();

    float recent_ratio = float(received_recent) / (received_recent + missed_recent);
    float total_ratio = float(received) / (received + missed);

    if (missed_recent == 0) {
      status.level = 0;
      status.message = "No missed packets";
    }
    else if (recent_ratio > 0.99f) {
      status.level = 1;
      status.message = "Some missed packets";
    }
    else {
      status.level = 2;
      status.message = "Excessive proportion of missed packets";
    }

    unsigned long data_rate;
    cam_->getAttribute("StreamBytesPerSecond", data_rate);
#if 1
    // Adjust data rate
    static const unsigned long MAX_DATA_RATE = 115000000; // for typical GigE port
    float multiplier = 1.0f;
    if (recent_ratio == 1.0f) {
      multiplier = 1.1f;
    } else if (recent_ratio < 0.99f) {
      multiplier = 0.9f;
    }
    if (multiplier != 1.0f) {
      unsigned long new_data_rate = std::min((unsigned long)(multiplier * data_rate + 0.5), MAX_DATA_RATE);
      new_data_rate = std::max(new_data_rate, MAX_DATA_RATE/1000);
      if (data_rate != new_data_rate) {
        data_rate = new_data_rate;
        cam_->setAttribute("StreamBytesPerSecond", data_rate);
        ROS_WARN("Changed data rate to %lu bytes per second", data_rate);
      }
    }
#endif
    
    status.set_values_size(7);
    status.values[0].label = "Recent % Packets Received";
    status.values[0].value = recent_ratio * 100.0f;
    status.values[1].label = "Overall % Packets Received";
    status.values[1].value = total_ratio * 100.0f;
    status.values[2].label = "Received Packets";
    status.values[2].value = received;
    status.values[3].label = "Missed Packets";
    status.values[3].value = missed;
    status.values[4].label = "Requested Packets";
    status.values[4].value = requested;
    status.values[5].label = "Resent Packets";
    status.values[5].value = resent;
    status.values[6].label = "Data Rate (bytes/s)";
    status.values[6].value = data_rate;
  }

  void packetErrorStatus(diagnostic_msgs::DiagnosticStatus& status)
  {
    status.name = "Packet Error Status";
    
    unsigned long erroneous;
    cam_->getAttribute("StatPacketsErroneous", erroneous);

    if (erroneous == 0) {
      status.level = 0;
      status.message = "No erroneous packets";
    } else {
      status.level = 2;
      status.message = "Possible camera hardware failure";
    }

    status.set_values_size(1);
    status.values[0].label = "Erroneous Packets";
    status.values[0].value = erroneous;
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

  bool camInfoService(prosilica_cam::CamInfo::Request &req,
                      prosilica_cam::CamInfo::Response &res)
  {
    res.cam_info = cam_info_;
    res.cam_info.header.stamp = ros::Time::now();
    res.cam_info.header.frame_id = frame_id_;
    return true;
  }

  bool triggeredGrab(prosilica_cam::PolledImage::Request &req,
                     prosilica_cam::PolledImage::Response &res)
  {
    if (mode_ != prosilica::Triggered)
      return false;

    tPvFrame* frame = NULL;

    try {
      if (req.region_x || req.region_y || req.width || req.height) {
        cam_->setRoi(req.region_x, req.region_y, req.width, req.height);
      } else {
        cam_->setRoiToWholeFrame();
      }

      frame = cam_->grab(req.timeout_ms);
    }
    catch (prosilica::ProsilicaException &e) {
      if (e.error_code == ePvErrBadSequence)
        throw; // not easily recoverable
      
      ROS_ERROR("Prosilica exception: %s\n\tx = %d, y = %d, width = %d, height = %d",
                e.what(), req.region_x, req.region_y, req.width, req.height);
      return false;
    }
    
    if (!frame)
      return false;

    sensor_msgs::Image &image = calibrated_ ? img_ : res.image;
    sensor_msgs::Image &rect_image = calibrated_ ? res.image : rect_img_;
    bool success = processFrame(frame, image, rect_image, res.cam_info);
    if (success)
      publishTopics(image, rect_image, res.cam_info);

    return success;
  }

private:
  static std::string bayerPatternString(tPvBayerPattern pattern)
  {
    static const char* patternStrings[] = { "bayer_rggb", "bayer_gbrg",
                                            "bayer_grbg", "bayer_bggr" };
    return patternStrings[pattern];
  }

  static void setBgrLayout(sensor_msgs::Image &image, int width, int height)
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
  
  static bool frameToImage(tPvFrame* frame, sensor_msgs::Image &image)
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

  bool rectifyFrame(tPvFrame* frame, sensor_msgs::Image &img, sensor_msgs::Image &rect_img,
                    sensor_msgs::CamInfo &cam_info)
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
    IplImage* raw = img_bridge_.toIpl();
    IplImage* rect = rect_img_bridge_.toIpl();

    // Undistort using full frame model or ROI model
    if (frame->Width  == (unsigned long)model_.width() &&
        frame->Height == (unsigned long)model_.height()) {
      assert(frame->RegionX == 0 && frame->RegionY == 0);
      
      model_.undistort(raw, rect);
      model_.fillCamInfo(cam_info);
    }
    else {
      camera_calibration::PinholeCameraModel roi_model =
        model_.withRoi(frame->RegionX, frame->RegionY, frame->Width, frame->Height);
      roi_model.undistort(raw, rect);
      roi_model.fillCamInfo(cam_info);
    }

    return true;
  }
  
  bool processFrame(tPvFrame* frame, sensor_msgs::Image &img, sensor_msgs::Image &rect_img,
                    sensor_msgs::CamInfo &cam_info)
  {
    ros::Time time = ros::Time::now();
    
    if (!frameToImage(frame, img))
      return false;

    img.header.stamp = time;
    img.header.frame_id = frame_id_;
    
    if (calibrated_) {
      if (!rectifyFrame(frame, img, rect_img, cam_info))
        return false;

      rect_img.header.stamp = time;
      rect_img.header.frame_id = frame_id_;
      cam_info.header.stamp = time;
      cam_info.header.frame_id = frame_id_;
    }

    count_++;
    return true;
  }

  void publishTopics(sensor_msgs::Image &img, sensor_msgs::Image &rect_img,
                     sensor_msgs::CamInfo &cam_info)
  {
    node_.publish("~image", img);
    if (calibrated_) {
      node_.publish("~image_rect", rect_img);
      node_.publish("~cam_info", cam_info);
    }

    if (node_.numSubscribers("~thumbnail") > 0) {
      int width  = img.uint8_data.layout.dim[1].size;
      int height = img.uint8_data.layout.dim[0].size;
      float aspect = std::sqrt((float)width / height);
      int scaled_width  = thumbnail_size_ * aspect + 0.5;
      int scaled_height = thumbnail_size_ / aspect + 0.5;
      
      setBgrLayout(thumbnail_, scaled_width, scaled_height);
      if (!rect_img_bridge_.fromImage(thumbnail_, "bgr"))
        return;
      cvResize(img_bridge_.toIpl(), rect_img_bridge_.toIpl());

      node_.publish("~thumbnail", thumbnail_);
    }
  }
  
  void publishImage(tPvFrame* frame)
  {
    if (!processFrame(frame, img_, rect_img_, cam_info_))
      return;

    publishTopics(img_, rect_img_, cam_info_);
  }

  void loadIntrinsics()
  {
    // Retrieve contents of user memory
    std::string buffer(prosilica::Camera::USER_MEMORY_SIZE, '\0');
    cam_->readUserMemory(&buffer[0], prosilica::Camera::USER_MEMORY_SIZE);

    // Parse calibration file
    calibrated_ = model_.parse(buffer);
  }

  void normalizeCallback(tPvFrame* frame)
  {
    unsigned long exposure;
    cam_->getAttribute("ExposureValue", exposure);
    //ROS_WARN("Exposure value = %u", exposure);

    if (exposure == last_exposure_value_)
      consecutive_stable_exposures_++;
    else {
      last_exposure_value_ = exposure;
      consecutive_stable_exposures_ = 0;
    }
  }
  
  void normalizeExposure()
  {
    ROS_INFO("Normalizing exposure");
    //cam_->stop();

    last_exposure_value_ = 0;
    consecutive_stable_exposures_ = 0;
    cam_->start(prosilica::Continuous);

    // TODO: thread safety
    while (consecutive_stable_exposures_ < 3)
      boost::this_thread::sleep(boost::posix_time::millisec(250));

    cam_->stop();
    //cam_->start(mode_);
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
