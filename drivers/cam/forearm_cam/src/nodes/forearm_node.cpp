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

// TODO: Timeout receiving packet.
// TODO: Check that partial EOF missing frames get caught.
// @todo Do the triggering based on a stream of incoming timestamps.

#include <ros/node.h>
#include <image_msgs/Image.h>
#include <image_msgs/CamInfo.h>
#include <image_msgs/FillImage.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/publisher.h>
#include <self_test/self_test.h>
#include <robot_mechanism_controllers/SetWaveform.h>
#include <robot_mechanism_controllers/trigger_controller.h>
#include <stdlib.h>
#include <limits>
#include <math.h>
#include <linux/sysctl.h>
#include <iostream>
#include <fstream>

#include <boost/tokenizer.hpp>
#include <boost/format.hpp>

#include "pr2lib.h"
#include "host_netutil.h"
#include "mt9v.h"

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
  ros::NodeHandle &node_handle_;
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
  bool exit_on_fault_;
  
  bool open_;

  diagnostic_updater::Updater diagnostic_;
  
  diagnostic_updater::DiagnosedPublisher<image_msgs::Image> cam_pub_;
  ros::Publisher cam_info_pub_;
  ros::ServiceClient trig_service_;
  
  boost::mutex diagnostics_lock_;

  unsigned char mac_[6];
  std::string trig_controller_;
  std::string trig_controller_cmd_;
  std::string ip_address_;
  std::string if_name_;
  int serial_number_;
  std::string hwinfo_;
  std::string mode_name_;
  controller::trigger_configuration trig_req_;
  robot_mechanism_controllers::SetWaveform::Response trig_rsp_;
  
  int gain_;
  double exposure_;

//  diagnostic_updater::FrequencyStatus freq_diag_;
//  diagnostic_updater::TimeStampStatus timestamp_diag_;

  SelfTest<ForearmNode> self_test_;
  bool was_running_pretest_;
  
  FrameTimeFilter frameTimeFilter_;
  
  boost::thread *image_thread_;
  boost::thread *diagnostic_thread_;

  typedef boost::function<int(size_t, size_t, uint8_t*, ros::Time)> UseFrameFunction;
  UseFrameFunction useFrame_;

public:
  int exit_status_;

  ForearmNode(ros::NodeHandle &nh)
    : node_handle_(nh), camera_(NULL), started_video_(false),
      diagnostic_(ros::NodeHandle()), 
      cam_pub_(node_handle_.advertise<image_msgs::Image>("~image_raw", 1), 
          diagnostic_,
          diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05), 
          diagnostic_updater::TimeStampStatusParam()),
      self_test_(this),
      useFrame_(boost::bind(&ForearmNode::publishImage, this, _1, _2, _3, _4))
  {
    started_video_ = false;
    open_ = false;
    exit_status_ = 0;
    misfire_blank_ = 0;
    image_thread_ = NULL;
    diagnostic_thread_ = NULL;

    // Setup diagnostics
    //diagnostic_.add(timestamp_diag_);
    //diagnostic_.add("Frequency Status", this, &ForearmNode::freqStatus );
    diagnostic_.add("Link Status", this, &ForearmNode::linkStatus );

    // Setup self test
    self_test_.setPretest( &ForearmNode::pretest );
    self_test_.addTest( &ForearmNode::interruptionTest );
    self_test_.addTest( &ForearmNode::connectTest );
    self_test_.addTest( &ForearmNode::startTest );
    self_test_.addTest( &ForearmNode::streamingTest );
    self_test_.addTest( &ForearmNode::disconnectTest );
    for (int i = 0; i < MT9V_NUM_MODES; i++)
    {
      diagnostic_updater::TaskFunction f = boost::bind(&ForearmNode::videoModeTest, this, MT9VModes[i].name, _1);
      self_test_.add( str(boost::format("Test Pattern in mode %s")%MT9VModes[i].name), f );
    }
    self_test_.addTest( &ForearmNode::resumeTest );
    
    // Clear statistics
    last_image_time_ = 0;
    missed_line_count_ = 0;
    missed_eof_count_ = 0;
    
    // Read parameters
    read_config();

    if (node_handle_.ok())
    {
      cam_pub_.clear_window(); // Avoids having an error until the window fills up.
      diagnostic_thread_ = new boost::thread( boost::bind(&ForearmNode::diagnosticsLoop, this) );
    }
  }
 
  void read_config()
  {
    node_handle_.param("~if_name", if_name_, std::string("eth0"));
    
    node_handle_.param("~exit_on_fault", exit_on_fault_, false);
    
    if (node_handle_.hasParam("~ip_address"))
      node_handle_.getParam("~ip_address", ip_address_);
    else {
      ROS_FATAL("IP address not specified");
      exit_status_ = 1;
      //node_handle_.shutdown();
      return;
    }

    node_handle_.param("~gain", gain_, -1);
    if (gain_ != -1)
    {
      if (gain_ < 16)
      {
        ROS_WARN("Gain of %i is too low. Legal range is 16 to 64. Resetting to 16.", gain_);
        gain_ = 16;
      }
      if (gain_ > 64)
      {
        ROS_WARN("Gain of %i is too high. Legal range is 16 to 64. Resetting to 64.", gain_);
        gain_ = 64;
      }
    }

    node_handle_.param("~port", port_, 9090); /// @todo Should get rid of this and let the OS pick a free port.

    node_handle_.param("~ext_trigger", ext_trigger_, false);

    node_handle_.param("~video_mode", mode_name_, std::string("752x480x15"));
    for (video_mode_ = 0; video_mode_ < MT9V_NUM_MODES; video_mode_++)
      if (mode_name_.compare(MT9VModes[video_mode_].name) == 0)
        break;
    if (video_mode_ == MT9V_NUM_MODES) 
    {
      ROS_FATAL("Unknown video mode %s", mode_name_.c_str());
      exit_status_ = 1;
      //node_handle_.shutdown();
      return;
    }

    /// @todo add a check that width_ and height_ are set in cam_info in test bench
    width_ = MT9VModes[video_mode_].width;
    height_ = MT9VModes[video_mode_].height;
    cam_info_.width = width_;
    cam_info_.height = height_;
    imager_freq_ = MT9VModes[video_mode_].fps;

    // Specify which frame to add to message header
    node_handle_.param("~frame_id", frame_id_, std::string("NO_FRAME"));
    image_.header.frame_id = frame_id_;
    cam_info_.header.frame_id = frame_id_;
        
    node_handle_.param("~trigger_controller", trig_controller_, std::string());
    node_handle_.param("~trigger_rate", trig_rate_, imager_freq_ - 1.);
    node_handle_.getParam("~serial_number", serial_number_, -2);
    node_handle_.param("~trigger_phase", trig_phase_, 0.);
    
    desired_freq_ = ext_trigger_ ? trig_rate_ : imager_freq_;
    
    // First packet offset parameter

    node_handle_.param("~first_packet_offset", first_packet_offset_, 0.0025);
    if (!node_handle_.hasParam("~first_packet_offset") && trig_controller_.empty())
      ROS_INFO("first_packet_offset not specified. Using default value of %f ms.", first_packet_offset_);
    
    node_handle_.param("~exposure", exposure_, -1.0);
    if (exposure_ != -1)
    {
      if (exposure_ <= 0)
      {
        ROS_WARN("Exposure is %f, but must be positive. Setting to automatic.", exposure_);
        exposure_ = -1;
      }

      if (exposure_ > 1 / desired_freq_)
      {
        ROS_WARN("Exposure (%f s) is greater frame period (%f s). Setting to 90%% of frame period.", 
            exposure_, 1 / desired_freq_);
        exposure_ = 0.9 * 1 / desired_freq_;
      }

      if (exposure_ > 0.95 / desired_freq_)
      {
        ROS_WARN("Exposure (%f s) is greater than 95%% of frame period (%f s). You may miss frames.", 
            exposure_, 0.95 / desired_freq_);
      }
    }
  }

  void diagnosticsLoop()
  {
    //int frameless_updates = 0;
    
    bool have_started = false;

    while (node_handle_.ok())
    {
      if (!started_video_)
      {
        stop();
        close();
        if (have_started && exit_on_fault_)
        {
          node_handle_.shutdown();
          break;
        }
        open();
        start();
        have_started = true;
      }

      { 
        boost::mutex::scoped_lock(diagnostics_lock_);
        diagnostic_.update();
        self_test_.checkTest();
      }
      sleep(1);

      /*if (count_ == 0 && started_video_)
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
      }*/
    }

    ROS_DEBUG("Diagnostic thread exiting.");
  }

  ~ForearmNode()
  {
    close();

    ROS_DEBUG("ForearmNode destructor exiting.");
    
    node_handle_.shutdown();
  }

  void open()
  {
    ROS_DEBUG("open()");
    if (open_)
      return;
    
    int retval;
    // Create a new IpCamList to hold the camera list
    IpCamList camList;
    pr2CamListInit(&camList);
    
    // Check that rmem_max is large enough.
    int rmem_max;
    {
      std::ifstream f("/proc/sys/net/core/rmem_max");
      f >> rmem_max;
    }
    if (rmem_max < 10000000)
    {
      ROS_WARN("rmem_max is %i. Buffer overflows and packet loss may occur. Minimum recommended value is 10000000. Updates may not take effect until the driver is restarted. See http://pr.willowgarage.com/wiki/errors/Dropped_Frames_and_rmem_max for details.", rmem_max);
    }

    // Discover any connected cameras, wait for 0.5 second for replies
    if( pr2Discover(if_name_.c_str(), &camList, ip_address_.c_str(), SEC_TO_USEC(0.5)) == -1) {
      ROS_FATAL("Discover error");
      exit_status_ = 1;
      //node_handle_.shutdown();
      return;
    }

    if (pr2CamListNumEntries(&camList) == 0) {
      ROS_FATAL("No cameras found");
      exit_status_ = 1;
      //node_handle_.shutdown();
      return;
    }

    // Open camera with requested serial number.
    int index = -1;
    if (serial_number_ == -1) // Auto
    {
      if (pr2CamListNumEntries(&camList) == 1)
      {
        index = 0;
      }
      else
      {
        ROS_FATAL("Camera autodetection only works when exactly one camera is discoverable. Unfortunately, we found %i cameras.", pr2CamListNumEntries(&camList));
      }
    }
    else if (serial_number_ == -2) // Nothing specified
    {
      ROS_FATAL("No camera serial_number was specified. Specifying a serial number is now mandatory to avoid accidentally configuring a random camera elsewhere in the building. You can specify -1 for autodetection.");
      exit_status_ = 1;
    }
    else
    {
      index = pr2CamListFind(&camList, serial_number_);
      if (index == -1) {
        ROS_FATAL("Couldn't find camera with S/N %i", serial_number_);
        exit_status_ = 1;
      }
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
      //node_handle_.shutdown();
      return;
    }

    // Configure the camera with its IP address, wait up to 500ms for completion
    camera_ = pr2CamListGetEntry(&camList, index);
    retval = pr2Configure(camera_, ip_address_.c_str(), SEC_TO_USEC(0.5));
    if (retval != 0) {
      if (retval == ERR_CONFIG_ARPFAIL) {
        ROS_WARN("Unable to create ARP entry (are you root?), continuing anyway");
      } else {
        ROS_FATAL("IP address configuration failed");
        exit_status_ = 1;
        //node_handle_.shutdown();
        return;
      }
    }
    ROS_INFO("Configured camera, S/N #%u, IP address %s",
             camera_->serial, ip_address_.c_str());
      
    serial_number_ = camera_->serial;
    hwinfo_ = camera_->hwinfo;
    memcpy(mac_, camera_->mac, sizeof(mac_));

    // We are going to receive the video on this host, so we need our own MAC address
    if ( wgEthGetLocalMac(camera_->ifName, &localMac_) != 0 ) {
      ROS_FATAL("Unable to get local MAC address for interface %s", camera_->ifName);
      exit_status_ = 1;
      //node_handle_.shutdown();
      return;
    }

    // We also need our local IP address
    if ( wgIpGetLocalAddr(camera_->ifName, &localIp_) != 0) {
      ROS_FATAL("Unable to get local IP address for interface %s", camera_->ifName);
      exit_status_ = 1;
      //node_handle_.shutdown();
      return;
    }
      
    // Select trigger mode.
    if ( pr2TriggerControl( camera_, ext_trigger_ ? TRIG_STATE_EXTERNAL : TRIG_STATE_INTERNAL ) != 0) {
      ROS_FATAL("Trigger mode set error. Is %s accessible from interface %s? (Try running route to check.)", ip_address_.c_str(), if_name_.c_str());
      exit_status_ = 1;
      //node_handle_.shutdown();
      return;
    }

    if (ext_trigger_)
    {
      // Configure the triggering controller
      if (!trig_controller_.empty())
      {
        ROS_INFO("Configuring controller \"%s\" for triggering.", trig_controller_.c_str());
        trig_controller_cmd_ = trig_controller_ + "/set_waveform";

        ROS_DEBUG("Setting trigger.");
        trig_req_.running = 1;
        trig_req_.rep_rate = trig_rate_; 
        trig_req_.phase = trig_phase_;
        trig_req_.active_low = 0;
        trig_req_.pulsed = 1;
        trig_req_.duty_cycle = 0; // Unused in pulsed mode.

        trig_service_ = node_handle_.serviceClient<robot_mechanism_controllers::SetWaveform>(trig_controller_cmd_);
        // Retry a few times in case the service is just coming up.
      }
      else
      {
        ROS_WARN("External triggering is selected, but no \"trigger_controller\" was specified.");
      }
    }

    // Select a video mode
    if ( pr2ImagerModeSelect( camera_, video_mode_ ) != 0) {
      ROS_FATAL("Mode select error");
      exit_status_ = 1;
      //node_handle_.shutdown();
      return;
    }

    // Set horizontal blanking
    if ( pr2SensorWrite( camera_, MT9V_REG_HORIZONTAL_BLANKING, MT9VModes[video_mode_].hblank ) != 0)
    {
      ROS_WARN("Error setting horizontal blanking.");
    }

    if ( pr2SensorWrite( camera_, MT9V_REG_VERTICAL_BLANKING, MT9VModes[video_mode_].vblank) != 0)
    {
      ROS_WARN("Error setting vertical blanking.");
    }

    /*
    // Set maximum course shutter width
    if ( pr2SensorWrite( camera_, 0xBD, 240 ) != 0) {
      ROS_FATAL("Sensor write error");
      exit_status_ = 1;
      node_handle_.shutdown();
      return;
    }
    */

    if (pr2SensorWrite(camera_, MT9V_REG_AGC_AEC_ENABLE, (gain_ == -1) * 2 + (exposure_ == -1)) != 0)
    {
      ROS_WARN("Error setting AGC/AEC mode. Exposure and gain may be incorrect.");
    }

    if (gain_ != -1) // Manual gain
    {
      if ( pr2SensorWrite( camera_, MT9V_REG_ANALOG_GAIN, gain_) != 0)
      {
        ROS_WARN("Error setting analog gain.");
      }
    }

    if (exposure_ != -1) // Manual exposure
    {
      uint16_t hblank = MT9VModes[video_mode_].hblank; 
      uint16_t hpix = width_ > 320 ? width_ : width_ * 2; 
      double line_time = (hpix + hblank) / MT9V_CK_FREQ;
      ROS_DEBUG("Line time is %f microseconds. (hpix = %i, hblank = %i)", line_time * 1e6, hpix, hblank);
      int explines = exposure_ / line_time;
      if (explines < 1) /// @TODO warning here?
        explines = 1;
      if (explines > 32767) /// @TODO warning here?
        explines = 32767;
      ROS_DEBUG("Setting exposure lines to %i.", explines);
      if ( pr2SensorWrite( camera_, MT9V_REG_TOTAL_SHUTTER_WIDTH, explines) != 0)
      {
        ROS_WARN("Error setting exposure.");
      }
    }

    // Try to load camera intrinsics from flash memory
    calibrated_ = loadIntrinsics(&cam_info_.D[0], &cam_info_.K[0],
                                 &cam_info_.R[0], &cam_info_.P[0]);
    if (calibrated_)
      ROS_INFO("Loaded intrinsics from camera");
    else
      ROS_WARN("Failed to load intrinsics from camera");

    frameTimeFilter_ = FrameTimeFilter(desired_freq_, 0.001, 0.5 / imager_freq_); 
    
    // Receive frames through callback
    // TODO: start this in separate thread?
    //cam_pub_ = node_handle_.advertise<image_msgs::Image>("~image_raw", 1);
    if (calibrated_)
      cam_info_pub_ = node_handle_.advertise<image_msgs::CamInfo>("~cam_info", 1);
    
    open_ = true;;
  }

  void close()
  {
    ROS_DEBUG("close()");
    stop();
    open_ = false;
  }

  void start()
  {
    ROS_DEBUG("start()");
    if (!open_)
      open();
    if (open_)
    {
      // Start video; send it to specified host port
      // @todo TODO: Only start when somebody is listening?
      started_video_ = true;
      image_thread_ = new boost::thread(boost::bind(&ForearmNode::imageThread, this, port_));
    }
  }

  void stop()
  {
    ROS_DEBUG("stop()");

    started_video_ = false;
    if (image_thread_)
    {
      if (image_thread_->timed_join((boost::posix_time::milliseconds) 2000))
      {
        delete image_thread_;
        image_thread_ = NULL;
      }
      else
      {
        ROS_DEBUG("image_thread_ did not die after two seconds. Proceeding.");
      }
    }
    
  }
  
private:
  void imageThread(int port)
  {
    // Start video
    if ( pr2StartVid( camera_, (uint8_t *)&(localMac_.sa_data[0]),
                      inet_ntoa(localIp_), port_) != 0 ) {
      ROS_FATAL("Video start error");
      started_video_ = false;
      exit_status_ = 1;
      return;
    }
    if (!trig_controller_cmd_.empty())
    {
      trig_req_.running = 1;
      if (!trig_service_.call(trig_req_, trig_rsp_))
      {
        ROS_DEBUG("Could not start trigger on first attempt. Sleeping and retrying.");
        sleep(3); // Perhaps the trigger is just being brought up.
        if (!trig_service_.call(trig_req_, trig_rsp_))
        {
          ROS_ERROR("Unable to set trigger controller.");
          exit_status_ = 1;
          //node_handle_.shutdown();
          goto stop_video;
        }
      }
    }
    frameTimeFilter_.reset_filter();
    ROS_INFO("Camera running.");
    
    // Receive video
    pr2VidReceive(camera_->ifName, port, height_, width_, &ForearmNode::frameHandler, this);
    
    // Stop Triggering
    if (!trig_controller_cmd_.empty())
    {   
      ROS_DEBUG("Stopping triggering.");
      trig_req_.running = 0;
      /// @todo need to turn on a node in the case where the node is
      //already down.
      //ros::Node shutdown_node("forearm_node", ros::Node::ANONYMOUS_NAME | ros::Node::DONT_ADD_ROSOUT_APPENDER); // Need this because the node has been shutdown already
      if (!trig_service_.call(trig_req_, trig_rsp_))
      { // This probably means that the trigger controller was turned off,
        // so we don't really care.
        ROS_DEBUG("Was not able to stop triggering.");
      }
    }
stop_video:
    // Stop video
    if (started_video_) // Exited unexpectedly.
    {
      started_video_ = false;
      ROS_ERROR("Image thread exited unexpectedly.");
      
      if ( pr2StopVid(camera_) == 0 )
        ROS_ERROR("Video should have been stopped"); /// @todo get rid of this once things have stabilized.
    }
    else // Exited expectedly.
      if ( pr2StopVid(camera_) != 0)
        ROS_ERROR("Video Stop error");
    
    ROS_DEBUG("Image thread exiting.");
  }

/*  void freqStatus(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    freq_diag_(status);

  }*/

  void linkStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    if (ros::Time::now().toSec() - last_image_time_ > 5 / desired_freq_)
    {
      stat.summary(2, "Next frame is past due.");
    }
    else if (started_video_)
    {
      stat.summary(0, "Frames are streaming.");
    }
    else
    {
      stat.summary(1, "Frames are not streaming.");
    }
    
    stat.addv("Missing image line frames", missed_line_count_);
    stat.addv("Missing EOF frames", missed_eof_count_);
    stat.addv("First packet offset", first_packet_offset_);
    stat.adds("Interface", if_name_);
    stat.adds("Camera IP", ip_address_);
    stat.adds("Camera Serial #", serial_number_);
    stat.adds("Camera Hardware", hwinfo_);
    stat.addsf("Camera MAC", "%02X:%02X:%02X:%02X:%02X:%02X", mac_[0], mac_[1], mac_[2], mac_[3], mac_[4], mac_[5]);
    stat.adds("Image mode", mode_name_);
    stat.addsf("Latest frame time", "%f", last_image_time_);
    stat.adds("Latest frame #", last_frame_number_);
    stat.addv("Free-running frequency", imager_freq_);
    stat.adds("External trigger controller", trig_controller_);
    stat.adds("Trigger mode", ext_trigger_ ? "external" : "internal");
  }

  int publishImage(size_t width, size_t height, uint8_t *frameData, ros::Time t)
  {
    fillImage(image_, "image", height, width, 1, "bayer_bggr", "uint8", frameData);
    
    /*static FILE *f = fopen("/tmp/deltas.out", "w");
    std::vector<unsigned char> idat = image_.uint8_data.data;
    int maxdelta = 0;
    for (int i = width_/3; i < 2*width_/3; i++)
    {
      int d1 = (i & 1) * width_;
      int d2 = width_ - d1;
      int h1 = idat[width_ * (height_ / 2) + i + d2] - idat[width_ * (height_ / 2) + i + 1 + d1];
      if (abs(h1) > maxdelta)
        maxdelta = abs(h1);
      h1 += height_ / 2;
      image_.uint8_data.data[h1 * width_ + i] = 255;
      image_.uint8_data.data[(h1 + 1) * width_ + i] = 0;
      image_.uint8_data.data[(h1 + 2) * width_ + i] = 255;
      image_.uint8_data.data[(height_/2 - 1) * width_ + i] = 0;
      image_.uint8_data.data[height_/2 * width_ + i] = 255;
      image_.uint8_data.data[(height_/2 + 1) * width_ + i] = 0;
    }
    fprintf(f, "Maxdelta: %i\n", maxdelta);
    fflush(f);*/

    image_.header.stamp = t;
    //timestamp_diag_.tick(t);
    cam_pub_.publish(image_);
    if (calibrated_) {
      cam_info_.header.stamp = t;
      cam_info_pub_.publish(cam_info_);
    }
    //freq_diag_.tick();

    return 0;
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
    
    if (!node_handle_.ok())
      started_video_ = false;

    if (!started_video_)
    {
      return 1;
    }

    if (frame_info == NULL)
    {
      // The select call in the driver timed out.
      ROS_WARN("No data have arrived for more than one second.");
      started_video_ = false;
      return 1;
    }

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

    //static double lastunfiltered;
    //ROS_DEBUG("first_packet %f unfilteredImageTime %f frame_id %u deltaunf: %f", frameStartTime, unfilteredImageTime, frame_info->frame_number, unfilteredImageTime - lastunfiltered);
    //lastunfiltered = unfilteredImageTime;
    
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
      misfire_blank_--;
    }

    last_image_time_ = imageTime;
    last_frame_number_ = frame_info->frame_number;
    
    if (useFrame_(frame_info->width, frame_info->height, frame_info->frameData, ros::Time(imageTime)))
    {
      started_video_ = false;
      return 1;
    }

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

  void pretest()
  {
    was_running_pretest_ = started_video_;
    stop();
  }

  void interruptionTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.name = "Interruption Test";

    if (node_handle_.getNode()->numSubscribers("~image_raw") == 0)
    {
      status.level = 0;
      status.message = "No operation interrupted.";
    }
    else
    {
      status.level = 1;
      status.message = "There were active subscribers.  Running of self test interrupted operations.";
    }
  }

  void connectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.name = "Connection Test";

    open();

    if (open_)
    {
      status.level = 0;
      status.message = 
        str(boost::format("Connected successfully to camera %i.")%serial_number_);
    }
    else
    {
      status.level = 2;
      status.message = "Failed to connect.";
    }

    self_test_.setID(str(boost::format("FCAM%i")%serial_number_));
  }

  void startTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.name = "Start Test";

    start();

    status.level = 0;
    status.message = "Started successfully.";
  }

  void streamingTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    cam_pub_.clear_window();
    sleep(5);

    cam_pub_.run(status);

    status.name = "Streaming Test";
  }

  void disconnectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.name = "Disconnect Test";

    close();

    status.level = 0;
    status.message = "Disconnected successfully.";
  }
  
  int setTestMode(uint16_t mode, diagnostic_updater::DiagnosticStatusWrapper &status)
  {
    if ( pr2SensorWrite( camera_, 0x7F, mode ) != 0) {
      status.summary(2, "Could not set imager into test mode.");
      status.adds("Writing imager test mode", "Fail");
      return 1;
    }
    else
    {
      status.adds("Writing imager test mode", "Pass");
    }

    usleep(100000);
    uint16_t inmode;
    if ( pr2SensorRead( camera_, 0x7F, &inmode ) != 0) {
      status.summary(2, "Could not read imager mode back.");
      status.adds("Reading imager test mode", "Fail");
      return 1;
    }
    else
    {
      status.adds("Reading imager test mode", "Pass");
    }
    
    if (inmode != mode) {
      status.summary(2, "Imager test mode read back did not match.");
      status.addsf("Comparing read back value", "Fail (%04x != %04x)", inmode, mode);
      return 1;
    }
    else
    {
      status.adds("Comparing read back value", "Pass");
    }
    
    return 0;
  }

  class VideoModeTestFrameHandler
  {
    public:
      VideoModeTestFrameHandler(diagnostic_updater::DiagnosticStatusWrapper &status) : status_(status)
      {}

      int run(size_t width, size_t height, uint8_t *data, ros::Time stamp)
      {
        status_.adds("Got a frame", "Pass");

        for (size_t y = 0; y < height; y++)
          for (size_t x = 0; x < width; x++, data++)
          {
            uint8_t expected;
            
            if (width > 320)
              expected = get_expected(x, y);
            else
              expected = (get_expected(2 * x, 2 * y) + get_expected(2 * x, 2 * y + 1)) / 2;
                                   
            if (*data == expected)
              continue;

            status_.summaryf(2, "Unexpected value in frame at x=%i y=%i expected=%i got=%i.", x, y, (int) expected, (int) *data);
            status_.addsf("Frame content", "Fail: Unexpected value at (x=%i, y=%i, %hhi != %hhi)", x, y, expected, *data);
            return 1;
          }

        status_.addsf("Frame content", "Pass");
        return 1;
      }

    private:
      static uint8_t get_expected(int x, int y)
      {
        if ((x + 1) / 2 + y < 500)
          return 14 + x / 4;
        else
          return 0;
      }
      diagnostic_updater::DiagnosticStatusWrapper &status_;
  };

  void videoModeTest(const std::string mode, diagnostic_updater::DiagnosticStatusWrapper& status) 
  {
    const std::string oldmode = mode_name_;
    UseFrameFunction oldUseFrame = useFrame_;

    mode_name_ = mode;
    VideoModeTestFrameHandler callback(status);
    useFrame_ = boost::bind(&VideoModeTestFrameHandler::run, boost::ref(callback), _1, _2, _3, _4);

    status.name = mode + " Pattern Test";
    status.summary(0, "Passed"); // If nobody else fills this, then the test passed.

    open();

    if (setTestMode(0x3800, status))
      goto reset_state;

    start();  
    if (image_thread_->timed_join((boost::posix_time::milliseconds) 3000))
    {
      delete image_thread_;
      image_thread_ = NULL;
    }
    else
    {
      ROS_ERROR("Lost the image_thread. This should never happen.");
      status.summary(2, "Lost the image_thread. This should never happen.");
    }
    close();

    if (setTestMode(0x0000, status))
      goto reset_state;

reset_state:
    close();
    useFrame_ = oldUseFrame;
    mode_name_ = oldmode;
  }

  void resumeTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.name = "Resume Test";

    if (was_running_pretest_)
    {
      open();
      start();

      if (!started_video_)
      {
        status.level = 2;
        status.message = "Failed to resume previous mode of operation.";
        return;
      }
    }

    status.level = 0;
    status.message = "Previous operation resumed successfully.";
  }

};

#define __CHECK_FD_FREE__
#ifdef __CHECK_FD_FREE__
#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>
#endif

int main(int argc, char **argv)
{
  ros::init(argc, argv, "forearm_node");
  ros::NodeHandle nh;
  ForearmNode fn(nh);
  ros::spin();
  ROS_DEBUG("Exited from nh.spin()");
	
#ifdef __CHECK_FD_FREE__
  int s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  ROS_DEBUG("First free file descriptor is: %i", s); 
  if (s != -1)
    close(s);
#endif  
  return fn.exit_status_;
}
