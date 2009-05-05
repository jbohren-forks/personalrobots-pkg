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

//#define SIM_TEST
// TODO: Timeout receiving packet.
// TODO: Check that partial EOF missing frames get caught.

#include <ros/node.h>
#include <image_msgs/Image.h>
#include <image_msgs/CamInfo.h>
#include <image_msgs/FillImage.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <robot_mechanism_controllers/SetWaveform.h>
#include <robot_mechanism_controllers/trigger_controller.h>
#include <stdlib.h>
#include <limits>
#include <math.h>

#include <boost/tokenizer.hpp>
#include <boost/format.hpp>

#include "pr2lib.h"
#include "host_netutil.h"

static const double MODE_FPS[] = {15, 12.5, 30, 25, 15, 12.5, 60, 50, 30, 25};

// The FrameTimeFilter class takes a stream of image arrival times that
// include time due to system load and network asynchrony, and generates a
// (hopefully) more steady stream of arrival times. The filtering is based
// on the assumption that the frame rate is known, and that often the time
// stamp is correct. The general idea of the algorithm is:
//
// anticipated_time_ = previous time stamp + frame_period_
// is a good estimate of the current frame time stamp.
//
// Take the incoming time stamp, or the anticipated_time_, whichever one is
// lower. The rationale here is that when the latency is low or zero, the
// incoming time stamp is correct and will dominate. If latency occurs, the
// anticipated_time_ will be used.
//
// To avoid problems with clock skew, max_skew indicates the maximum
// expected clock skew. frame_period_ is set to correspond to the 
// slowest expected rate when clock skew is taken into account. If
// frame_period_ was less than the actual frame rate, then
// anticipated_time_ would always dominate, and the output time stamps
// would slowly diverge from the actual time stamps.
//
// Because the frame rate may sometimes skip around in an unanticipated way
// the filter detects if anticipated_time_ has dominated by more than
// locked_threshold_ more than max_recovery_frames_ in a row. In that case,
// it picks the lowest latency input value that occurs during the
// max_recovery_frames_ to reset anticipated_time_.
//
// Finally, if the filter misses too many frames in a row, it assumes that
// its anticipated_time is invalid and immediately resets the filter.

class FrameTimeFilter
{
public:  
  FrameTimeFilter(double frame_rate = 1., double late_locked_threshold = 1e-3, double early_locked_threshold = 3e-3, int early_recovery_frames = 5, int max_recovery_frames = 10, double max_skew = 1.001, double max_skipped_frames = 100)
  {
    frame_period_ = max_skew / frame_rate;
    max_recovery_frames_ = max_recovery_frames;
    early_recovery_frames_ = early_recovery_frames;
    late_locked_threshold_ = late_locked_threshold;
    early_locked_threshold_ = early_locked_threshold;
    max_skipped_frames_ = max_skipped_frames;
    last_frame_number_ = 0; // Don't really care about this value.
    reset_filter();
  }
  
  void reset_filter()
  {
    relock_time_ = anticipated_time_ = std::numeric_limits<double>::infinity();
    unlocked_count_ = late_locked_threshold_;
  }

  double run(double in_time, int frame_number)
  {
    double out_time = in_time;
    int delta = (frame_number - last_frame_number_) & 0xFFFF; // Hack because the frame rate currently wraps around too early.

    if (delta > max_skipped_frames_)
    {
      ROS_WARN("FrameTimeFilter missed too many frames from #%u to #%u. Resetting.", last_frame_number_, frame_number);
      reset_filter();
    }

    anticipated_time_ += frame_period_ * delta; 
    relock_time_ += frame_period_ * delta;

    if (out_time < relock_time_)
      relock_time_ = out_time;
      
    //ROS_DEBUG("in: %f ant: %f", in_time, anticipated_time_);

    bool early_trigger = false;
    if (out_time < anticipated_time_ - early_locked_threshold_ && finite(anticipated_time_))
    {
      ROS_WARN("FrameTimeFilter saw frame #%u was early by %f.", frame_number, anticipated_time_ - out_time);
      early_trigger = true;
    }
    if (out_time < anticipated_time_)
    {
      anticipated_time_ = out_time;
      relock_time_ = std::numeric_limits<double>::infinity();
      unlocked_count_ = 0;
    }
    else if (out_time > anticipated_time_ + late_locked_threshold_)
    {
      unlocked_count_++;
      if (unlocked_count_ > max_recovery_frames_)
      {
        ROS_WARN("FrameTimeFilter lost lock at frame #%u, shifting by %f.", frame_number, relock_time_ - anticipated_time_);
        anticipated_time_ = relock_time_;
        relock_time_ = std::numeric_limits<double>::infinity();
      }
      //else 
      //  ROS_DEBUG("FrameTimeFilter losing lock at frame #%u, lock %i/%i, off by %f.", frame_number, unlocked_count_, max_recovery_frames_, anticipated_time_ - out_time);
      out_time = anticipated_time_;
    }
    
    last_frame_number_ = frame_number;

    return early_trigger ? -out_time : out_time;
  }
  
private:  
  int max_recovery_frames_;
  int unlocked_count_;
  int last_frame_number_;
  int max_skipped_frames_;
  int early_recovery_frames_;
  double early_locked_threshold_;
  double late_locked_threshold_;
  double anticipated_time_;
  double relock_time_;
  double frame_period_;
};

class ForearmNode
{
private:
  ros::Node &node_;
  IpCamList* camera_;
  image_msgs::Image image_;
  image_msgs::CamInfo cam_info_;
  bool calibrated_;
  std::string frame_id_;

  int video_mode_;
  int width_;
  int height_;
  double desired_freq_;
  double imager_freq_;
  double trig_rate_;
  double trig_phase_;
  double first_packet_offset_;
  bool started_video_;
  bool ext_trigger_;
  int missed_eof_count_;
  int missed_line_count_;
  double last_image_time_;
  unsigned int last_frame_number_;
  int misfire_blank_;
  sockaddr localMac_;
  in_addr localIp_;
  int port_;
  
  boost::mutex diagnostics_lock_;

  std::string mac_;
  std::string trig_controller_;
  std::string trig_controller_cmd_;
  std::string ip_address_;
  std::string if_name_;
  std::string serial_number_;
  std::string hwinfo_;
  std::string mode_name_;
  controller::trigger_configuration trig_req_;
  robot_mechanism_controllers::SetWaveform::Response trig_rsp_;

  DiagnosticUpdater<ForearmNode> diagnostic_;
  int count_;

  FrameTimeFilter frameTimeFilter_;
  
  boost::thread *image_thread_;
  boost::thread *diagnostic_thread_;

public:
  int exit_status_;

  ForearmNode(ros::Node &node)
    : node_(node), camera_(NULL), started_video_(false),
      diagnostic_(this, &node_), count_(0)
  {
    exit_status_ = 0;
    misfire_blank_ = 0;
    image_thread_ = NULL;
    diagnostic_thread_ = NULL;

    // Clear statistics
    last_image_time_ = 0;
    missed_line_count_ = 0;
    missed_eof_count_ = 0;
    
    // Read parameters
    node_.param("~if_name", if_name_, std::string("eth0"));

    if (node_.hasParam("~ip_address"))
      node_.getParam("~ip_address", ip_address_);
    else {
      ROS_FATAL("IP address not specified");
      exit_status_ = 1;
      node_.shutdown();
      return;
    }

    node_.param("~port", port_, 9090);

    node_.param("~ext_trigger", ext_trigger_, false);

    node_.param("~video_mode", mode_name_, std::string("752x480x15"));
    if (mode_name_.compare("752x480x15") == 0)
      video_mode_ = MT9VMODE_752x480x15b1;
    else if (mode_name_.compare("752x480x12.5") == 0)
      video_mode_ = MT9VMODE_752x480x12_5b1;
    else if (mode_name_.compare("640x480x30") == 0)
      video_mode_ = MT9VMODE_640x480x30b1;
    else if (mode_name_.compare("640x480x25") == 0)
      video_mode_ = MT9VMODE_640x480x25b1;
    else if (mode_name_.compare("640x480x15") == 0)
      video_mode_ = MT9VMODE_640x480x15b1;
    else if (mode_name_.compare("640x480x12.5") == 0)
      video_mode_ = MT9VMODE_640x480x12_5b1;
    else if (mode_name_.compare("320x240x60") == 0)
      video_mode_ = MT9VMODE_320x240x60b2;
    else if (mode_name_.compare("320x240x50") == 0)
      video_mode_ = MT9VMODE_320x240x50b2;
    else if (mode_name_.compare("320x240x30") == 0)
      video_mode_ = MT9VMODE_320x240x30b2;
    else if (mode_name_.compare("320x240x25") == 0)
      video_mode_ = MT9VMODE_320x240x25b2;
    else {
      ROS_FATAL("Unknown video mode %s", mode_name_.c_str());
      exit_status_ = 1;
      node_.shutdown();
      return;
    }

    if (video_mode_ <= MT9VMODE_752x480x12_5b1)
      width_ = 752;
    else if (video_mode_ <= MT9VMODE_640x480x12_5b1)
      width_ = 640;
    else
      width_ = 320;
    height_ = (video_mode_ <= MT9VMODE_640x480x12_5b1) ? 480 : 240;
    desired_freq_ = imager_freq_ = MODE_FPS[ video_mode_ ];

    // Specify which frame to add to message header
    node_.param("~frame_id", frame_id_, std::string("NO_FRAME"));
    image_.header.frame_id = frame_id_;
    cam_info_.header.frame_id = frame_id_;
    cam_info_.width = width_;
    cam_info_.height = height_;
    
    // Configure camera
    configure(if_name_, ip_address_, port_);

    if (node_.ok())
    {
      diagnostic_.addUpdater( &ForearmNode::freqStatus );
      diagnostic_.addUpdater( &ForearmNode::linkStatus );

      diagnostic_thread_ = new boost::thread( boost::bind(&ForearmNode::diagnosticsLoop, this) );
    }
  }

  void diagnosticsLoop()
  {
    int frameless_updates = 0;
    
    while (node_.ok())
    {
      { 
        boost::mutex::scoped_lock(diagnostics_lock_);
        diagnostic_.update();
      }
      sleep(1);

      if (count_ == 0 && started_video_)
        frameless_updates++;
      else
        frameless_updates = 0;

      if (frameless_updates >= 2)
      {
        ROS_WARN("No frames are arriving. Attempting to restart image stream.");
        pr2Reset(camera_);
        if ( pr2StartVid( camera_, (uint8_t *)&(localMac_.sa_data[0]), inet_ntoa(localIp_), port_) != 0 )
        {
          ROS_ERROR("Failed to restart image stream. Will retry later.");
        }
      }
    }

    ROS_DEBUG("Diagnostic thread exiting.");
  }

  ~ForearmNode()
  {
    // Stop threads
    node_.shutdown();
    if (image_thread_)
    {
      if (image_thread_->timed_join((boost::posix_time::milliseconds) 1000))
        delete image_thread_;
      else
        ROS_DEBUG("image_thread_ did not die after one second. Proceeding with shutdown.");
    }

    if (diagnostic_thread_)
    {
      if (diagnostic_thread_->timed_join((boost::posix_time::milliseconds) 1000))
        delete diagnostic_thread_;
      else
        ROS_DEBUG("diagnostic_thread_ did not die after one second. Proceeding with shutdown.");
    }

    // Stop video
    if ( started_video_ && pr2StopVid(camera_) != 0 )
      ROS_ERROR("Video Stop error");

    // Stop Triggering
    if (!trig_controller_cmd_.empty())
    {   
      ROS_DEBUG("Stopping triggering.");
      trig_req_.running = 0;
      ros::Node shutdown_node("forearm_node", ros::Node::ANONYMOUS_NAME | ros::Node::DONT_ADD_ROSOUT_APPENDER); // Need this because the node has been shutdown already
      if (!ros::service::call(trig_controller_cmd_, trig_req_, trig_rsp_))
      { // This probably means that the trigger controller was turned off,
        // so we don't really care.
        ROS_DEBUG("Was not able to stop triggering.");
      }
    }
  
    ROS_DEBUG("ForearmNode constructor exiting.");
  }

  void configure(const std::string &if_name, const std::string &ip_address, int port_)
  {
    // Create a new IpCamList to hold the camera list
    IpCamList camList;
    pr2CamListInit(&camList);

    // Set anti-spoofing filter off on camera interface. Needed to prevent
    // the first reply from the camera from being filtered out.
    // @todo Should we be setting rp_filter to zero here? This may violate
    // the user's secutity preferences?
    std::string rp_str = "sysctl net.ipv4.conf."+if_name+".rp_filter|grep -q 0||sysctl -q -w net.ipv4.conf."+if_name+".rp_filter=0";
    ROS_DEBUG("Running \"%s\"", rp_str.c_str());
    int retval = system(rp_str.c_str());
    if (retval == -1 || !WIFEXITED(retval) || WEXITSTATUS(retval))
    {
      ROS_WARN("Unable to set rp_filter to 0 on interface. Camera discovery is likely to fail.");
    }

#ifndef SIM_TEST
    // Discover any connected cameras, wait for 0.5 second for replies
    if( pr2Discover(if_name.c_str(), &camList, SEC_TO_USEC(0.5)) == -1) {
      ROS_FATAL("Discover error");
      exit_status_ = 1;
      node_.shutdown();
      return;
    }

    if (pr2CamListNumEntries(&camList) == 0) {
      ROS_FATAL("No cameras found");
      exit_status_ = 1;
      node_.shutdown();
      return;
    }

    // Open camera with requested serial number.
    int index = -1;
    if (node_.hasParam("~serial_number")) {
      int sn;
      node_.getParam("~serial_number", sn);
      index = pr2CamListFind(&camList, sn);
      if (index == -1) {
        ROS_FATAL("Couldn't find camera with S/N %i", sn);
        exit_status_ = 1;
      }
      else
        camera_ = pr2CamListGetEntry(&camList, index);
    }
    else
    {
      ROS_FATAL("No camera serial_number was specified. Specifying a serial number is now mandatory to avoid accidentally configuring a random camera elsewhere in the building.");
      exit_status_ = 1;
    }

    // List found cameras if we were unable to open the requested one or
    // none was specified. 
    if (index == -1)
    {
      for (int i = 0; i < pr2CamListNumEntries(&camList); i++)
      {
        camera_ = pr2CamListGetEntry(&camList, i);
        ROS_FATAL("Found camera with S/N #%u", camera_->serial);
        exit_status_ = 1;
      }
      node_.shutdown();
      return;
    }

    // Configure the camera with its IP address, wait up to 500ms for completion
    retval = pr2Configure(camera_, ip_address.c_str(), SEC_TO_USEC(0.5));
    if (retval != 0) {
      if (retval == ERR_CONFIG_ARPFAIL) {
        ROS_WARN("Unable to create ARP entry (are you root?), continuing anyway");
      } else {
        ROS_FATAL("IP address configuration failed");
        exit_status_ = 1;
        node_.shutdown();
        return;
      }
    }
    ROS_INFO("Configured camera, S/N #%u, IP address %s",
             camera_->serial, ip_address.c_str());
      
    serial_number_ = str(boost::format("%i") % camera_->serial);
    hwinfo_ = camera_->hwinfo;
    mac_ = str(boost::format("%02X:%02X:%02X:%02X:%02X:%02X")% 
        (int) camera_->mac[0] % (int) camera_->mac[1] % (int) camera_->mac[2]% 
        (int) camera_->mac[3] % (int) camera_->mac[4] % (int) camera_->mac[5] );

    // We are going to receive the video on this host, so we need our own MAC address
    if ( wgEthGetLocalMac(camera_->ifName, &localMac_) != 0 ) {
      ROS_FATAL("Unable to get local MAC address for interface %s", camera_->ifName);
      exit_status_ = 1;
      node_.shutdown();
      return;
    }

    // We also need our local IP address
    if ( wgIpGetLocalAddr(camera_->ifName, &localIp_) != 0) {
      ROS_FATAL("Unable to get local IP address for interface %s", camera_->ifName);
      exit_status_ = 1;
      node_.shutdown();
      return;
    }
      
    // Select trigger mode.
    if ( pr2TriggerControl( camera_, ext_trigger_ ? TRIG_STATE_EXTERNAL : TRIG_STATE_INTERNAL ) != 0) {
      ROS_FATAL("Trigger mode set error. Is %s accessible from interface %s? (Try running route to check.)", ip_address.c_str(), if_name.c_str());
      exit_status_ = 1;
      node_.shutdown();
      return;
    }

    if (ext_trigger_)
    {
      // How fast should we be triggering the camera? By default 1 Hz less
      // than nominal.
      node_.param("~trigger_rate", trig_rate_, imager_freq_ - 1.);
      desired_freq_ = trig_rate_;

      // Configure the triggering controller
      if (node_.hasParam("~trigger_controller") )
      {
        node_.getParam("~trigger_controller", trig_controller_);
        ROS_INFO("Configuring controller \"%s\" for triggering.", trig_controller_.c_str());
        trig_controller_cmd_ = trig_controller_ + "/set_waveform";

        double trig_phase_;
        node_.param("~trigger_phase", trig_phase_, 0.);

        ROS_DEBUG("Setting trigger off.");
        trig_req_.running = 0;
        trig_req_.rep_rate = trig_rate_; 
        trig_req_.phase = trig_phase_;
        trig_req_.active_low = 0;
        trig_req_.pulsed = 1;
        trig_req_.duty_cycle = 0; // Unused in pulsed mode.

        if (!ros::service::call(trig_controller_cmd_, trig_req_, trig_rsp_))
        {
          ROS_FATAL("Unable to set trigger controller.");
          exit_status_ = 1;
          node_.shutdown();
          return;
        }

        ROS_DEBUG("Waiting for current frame to be guaranteed complete.");

        // Wait twice the expected time just to be safe.
        ros::Duration(2/imager_freq_).sleep();

        ROS_DEBUG("Starting trigger.");

        trig_req_.running = 1;

        if (!ros::service::call(trig_controller_cmd_, trig_req_, trig_rsp_))
        {
          ROS_FATAL("Unable to set trigger controller.");
          exit_status_ = 1;
          node_.shutdown();
          return;
        }
      }
      else
      {
        ROS_WARN("External triggering is selected, but no \"trigger_controller\" was specified.");
      }
    }

    // First packet offset parameter

    node_.param("~first_packet_offset", first_packet_offset_, 0.0025);
    if (!node_.hasParam("~first_packet_offset") && trig_controller_.empty())
      ROS_WARN("first_packet_offset not specified. Using default value of %f ms.", first_packet_offset_);

    // Select a video mode
    if ( pr2ImagerModeSelect( camera_, video_mode_ ) != 0) {
      ROS_FATAL("Mode select error");
      exit_status_ = 1;
      node_.shutdown();
      return;
    }

    /*
    // Set maximum course shutter width
    if ( pr2SensorWrite( camera_, 0xBD, 240 ) != 0) {
      ROS_FATAL("Sensor write error");
      exit_status_ = 1;
      node_.shutdown();
      return;
    }
    */

    // Try to load camera intrinsics from flash memory
    calibrated_ = loadIntrinsics(&cam_info_.D[0], &cam_info_.K[0],
                                 &cam_info_.R[0], &cam_info_.P[0]);
    if (calibrated_)
      ROS_INFO("Loaded intrinsics from camera");
    else
      ROS_WARN("Failed to load intrinsics from camera");

    frameTimeFilter_ = FrameTimeFilter(desired_freq_, 0.001, 0.5 / imager_freq_); 
    
    // Start video; send it to specified host port
    // @todo TODO: Only start when somebody is listening?
    if ( pr2StartVid( camera_, (uint8_t *)&(localMac_.sa_data[0]),
                      inet_ntoa(localIp_), port_) != 0 ) {
      ROS_FATAL("Video start error");
      exit_status_ = 1;
      node_.shutdown();
      return;
    }
    started_video_ = true;
#endif

    // Receive frames through callback
    // TODO: start this in separate thread?
    node_.advertise<image_msgs::Image>("~image_raw", 1);
    if (calibrated_)
      node_.advertise<image_msgs::CamInfo>("~cam_info", 1);
    

    image_thread_ = new boost::thread(boost::bind(&ForearmNode::imageThread, this, port_));
  }

  void imageThread(int port)
  {
#ifndef SIM_TEST
    pr2VidReceive(camera_->ifName, port, height_, width_, &ForearmNode::frameHandler, this);
#else
    pr2VidReceive(if_name.c_str(), port, height_, width_, &ForearmNode::frameHandler, this);
#endif
    
    ROS_DEBUG("Image thread exiting.");
  }

  void freqStatus(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Frequency Status";

    double freq = count_/diagnostic_.getPeriod();

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

    status.set_values_size(4);
    status.values[0].label = "Images in interval";
    status.values[0].value = count_;
    status.values[1].label = "Desired frequency";
    status.values[1].value = desired_freq_;
    status.values[2].label = "Actual frequency";
    status.values[2].value = freq;
    status.values[3].label = "Free-running frequency";
    status.values[3].value = imager_freq_;
    status.set_strings_size(2);
    status.strings[0].label = "External trigger controller";
    status.strings[0].value = trig_controller_;
    status.strings[1].label = "Trigger mode";
    status.strings[1].value = ext_trigger_ ? "external" : "internal";

    count_ = 0;
  }

  void linkStatus(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Link Status";

    if (ros::Time::now().toSec() - last_image_time_ > 5 / desired_freq_)
    {
      status.level = 2;
      status.message = "Next frame is past due.";
    }
    else
    {
      status.level = 0;
      status.message = "Frames are streaming.";
    }
    
    status.set_values_size(3);
    status.values[0].label = "Missing image line frames";
    status.values[0].value = missed_line_count_;
    status.values[1].label = "Missing EOF frames";
    status.values[1].value = missed_eof_count_;
    status.values[2].label = "First packet offset";
    status.values[2].value = first_packet_offset_;
    status.set_strings_size(8);
    status.strings[0].label = "Interface";
    status.strings[0].value = if_name_;
    status.strings[1].label = "Camera IP";
    status.strings[1].value = ip_address_;
    status.strings[2].label = "Camera Serial #";
    status.strings[2].value = serial_number_;
    status.strings[3].label = "Camera Hardware";
    status.strings[3].value = hwinfo_;
    status.strings[4].label = "Camera MAC";
    status.strings[4].value = mac_;
    status.strings[5].label = "Image mode";
    status.strings[5].value = mode_name_;
    status.strings[6].label = "Latest frame time";
    status.strings[6].value = str(boost::format("%f")%last_image_time_);
    status.strings[6].label = "Latest frame #";
    status.strings[6].value = str(boost::format("%d")%last_frame_number_);
  }

private:
  void publishImage(size_t width, size_t height, uint8_t *frameData, ros::Time t)
  {
    fillImage(image_, "image", height, width, 1, "bayer_bggr", "uint8", frameData);
    
    image_.header.stamp = t;
    node_.publish("~image_raw", image_);
    if (calibrated_) {
      cam_info_.header.stamp = t;
      node_.publish("~cam_info", cam_info_);
    }
  }
  
  double getTriggeredFrameTime(double firstPacketTime)
  {
    // Assuming that there was no delay in receiving the first packet,
    // this should tell us the time at which the first trigger after the
    // image exposure occurred.
    double pulseStartTime = 
      controller::TriggerController::getTickStartTimeSec(firstPacketTime, trig_req_);

    // We can now compute when the exposure ended. By offsetting to the
    // falling edge of the pulse, going back one pulse duration, and going
    // forward one camera frame time.
    double exposeEndTime = pulseStartTime + 
      controller::TriggerController::getTickDurationSec(trig_req_) +
      1 / imager_freq_ -
      1 / trig_rate_;

    return exposeEndTime;
  }

  double getExternallyTriggeredFrameTime(double firstPacketTime)
  {
    // We can now compute when the exposure ended. By offsetting by the
    // empirically determined first packet offset, going back one pulse
    // duration, and going forward one camera frame time.
    
    double exposeEndTime = firstPacketTime - first_packet_offset_ + 
      1 / imager_freq_ - 
      1 / trig_rate_;

    return exposeEndTime;
  }

  double getFreeRunningFrameTime(double firstPacketTime)
  {
    // This offset is empirical, but fits quite nicely most of the time.
    
    return firstPacketTime - first_packet_offset_;
  }
  
//    double nowTime = ros::Time::now().toSec();
//    static double lastExposeEndTime;
//    static double lastStartTime;
//    if (fabs(exposeEndTime - lastExposeEndTime - 1 / trig_rate_) > 1e-4) 
//      ROS_INFO("Mistimed frame #%u first %f first-pulse %f now-first %f pulse-end %f end-last %f first-last %f", eofInfo->header.frame_number, frameStartTime, frameStartTime - pulseStartTime, nowTime - pulseStartTime, pulseStartTime - exposeEndTime, exposeEndTime - lastExposeEndTime, frameStartTime - lastStartTime);
//    lastStartTime = frameStartTime;
//    lastExposeEndTime = exposeEndTime;
//    
//    static double lastImageTime;
//    static double firstFrameTime;
//    if (eofInfo->header.frame_number == 100)
//      firstFrameTime = imageTime;
//    ROS_INFO("Frame #%u time %f ofs %f ms delta %f Hz %f", eofInfo->header.frame_number, imageTime, 1000 * (imageTime - firstFrameTime - 1. / (29.5/* * 0.9999767*/) * (eofInfo->header.frame_number - 100)), imageTime - lastImageTime, 1. / (imageTime - lastImageTime));
//    lastImageTime = imageTime;
  
  int frameHandler(pr2FrameInfo *frame_info)
  {
    boost::mutex::scoped_lock(diagnostics_lock_);
    
    if (!node_.ok())
      return 1;
    
    if (frame_info->eofInfo == NULL) {
      // We no longer use the eofInfo.
      missed_eof_count_++;
      ROS_WARN("Frame %u was missing EOF", frame_info->frame_number);
    }

    // If we are not in triggered mode then use the arrival time of the
    // first packet as the image time.

    double unfilteredImageTime;
    double frameStartTime = frame_info->startTime.tv_sec + frame_info->startTime.tv_usec / 1e6;
    
    if (!trig_controller_.empty())
      unfilteredImageTime = getTriggeredFrameTime(frameStartTime);
    else if (ext_trigger_)
      unfilteredImageTime = getExternallyTriggeredFrameTime(frameStartTime);
    else
      unfilteredImageTime = getFreeRunningFrameTime(frameStartTime);

    //ROS_DEBUG("first_packet %f unfilteredImageTime %f frame_id %u", frameStartTime, unfilteredImageTime, frame_info->frame_number);
    
    double imageTime = frameTimeFilter_.run(unfilteredImageTime, frame_info->frame_number);
    if (imageTime < 0) // Signals an early frame arrival.
    {
      imageTime = -imageTime;
      if (!trig_controller_.empty())
        misfire_blank_ = 1 + imager_freq_ / (imager_freq_ - desired_freq_);
    }

    //ROS_DEBUG("imageTime %f", imageTime);
    
    // Check for short packet (video lines were missing)
    if (frame_info->shortFrame) {
      missed_line_count_++;
      ROS_WARN("Short frame #%u (%i video lines were missing, last was %i)", frame_info->frame_number, 
          frame_info->missingLines, frame_info->lastMissingLine);
      return 0;
    }

    if (misfire_blank_ > 0)
    {
      ROS_WARN("Dropping frame #%u because of mistrigger event.", frame_info->frame_number);
    }

    last_image_time_ = imageTime;
    last_frame_number_ = frame_info->frame_number;
    publishImage(frame_info->width, frame_info->height, frame_info->frameData, ros::Time(imageTime));
    count_++;
  
    return 0;
  }

  static int frameHandler(pr2FrameInfo *frameInfo, void *userData)
  {
    ForearmNode &fa_node = *(ForearmNode*)userData;
    return fa_node.frameHandler(frameInfo);
  }

  // TODO: parsing is basically duplicated in prosilica_node
  bool loadIntrinsics(double* D, double* K, double* R, double* P)
  {
    // FIXME: Hardcoding these until we get a response on flash read/write bug.
    //        These values are good for PRF and possibly OK for the other cameras.
#define FOREARM_FLASH_IS_BUGGY
#ifdef FOREARM_FLASH_IS_BUGGY
    static const double D_[] = {-0.34949, 0.13668, 0.00039, -0.00110, 0.00000};
    static const double K_[] = {427.31441, 0.00000, 275.80804,
                                0.00000, 427.37949, 238.88978,
                                0.00000, 0.00000, 1.00000};
    static const double R_[] = {1.00000, 0.00000, 0.00000,
                                0.00000, 1.00000, 0.00000,
                                0.00000, 0.00000, 1.00000};
    static const double P_[] = {427.31441, 0.00000, 275.80804, 0.00000,
                                0.00000, 427.37949, 238.88978, 0.00000,
                                0.00000, 0.00000, 1.00000, 0.00000};
    memcpy(D, D_, sizeof(D_));
    memcpy(K, K_, sizeof(K_));
    memcpy(R, R_, sizeof(R_));
    memcpy(P, P_, sizeof(P_));
    return true;
#else
    // Retrieve contents of user memory
    static const int CALIBRATION_PAGE = 0;
    std::string buffer(FLASH_PAGE_SIZE, '\0');
    if (pr2FlashRead(camera_, CALIBRATION_PAGE, (uint8_t*)&buffer[0]) != 0) {
      ROS_WARN("Flash read error");
      return false;
    }

    // Separate into lines
    typedef boost::tokenizer<boost::char_separator<char> > Tok;
    boost::char_separator<char> sep("\n");
    Tok tok(buffer, sep);

    // Check "header"
    Tok::iterator iter = tok.begin();
    if (*iter++ != "# Forearm camera intrinsics") {
      ROS_WARN("Header doesn't match");
      return false;
    }

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
                               &K[3*i], &K[3*i+1], &K[3*i+2]);
        }
      else if (*iter == "distortion") {
        ++iter;
        items_read += sscanf(iter->c_str(), "%lf %lf %lf %lf %lf",
                             D, D+1, D+2, D+3, D+4);
      }
      else if (*iter == "rectification")
        for (int i = 0; i < 3; ++i) {
          ++iter;
          items_read += sscanf(iter->c_str(), "%lf %lf %lf",
                               &R[3*i], &R[3*i+1], &R[3*i+2]);
        }
      else if (*iter == "projection")
        for (int i = 0; i < 3; ++i) {
          ++iter;
          items_read += sscanf(iter->c_str(), "%lf %lf %lf %lf",
                               &P[4*i], &P[4*i+1], &P[4*i+2], &P[4*i+3]);
        }
    }

    // Check we got everything
    return items_read == EXPECTED_ITEMS && width != 0 && height != 0;
#endif
  }
};



int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node n("forearm_node");
  ForearmNode fn(n);
  n.spin();
  ROS_DEBUG("Exited from n.spin()");
  
  return fn.exit_status_;
}
