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
#include <diagnostic_updater/diagnostic_updater.h>

#include "pr2lib.h"
#include "host_netutil.h"

class ForearmNode
{
private:
  ros::Node &node_;
  IpCamList* camera_;
  image_msgs::Image image_;

public:
  ForearmNode(ros::Node &node) : node_(node), camera_(NULL)
  {
    // Create a new IpCamList to hold the camera list
    IpCamList camList;
    pr2CamListInit(&camList);

    // Discover any connected cameras, wait for 0.5 second for replies
    std::string if_name;
    node_.param("~if_name", if_name, std::string("eth0"));
    if( pr2Discover(if_name.c_str(), &camList, SEC_TO_USEC(0.5)) == -1) {
      ROS_FATAL("Discover error");
      node_.shutdown();
      return;
    }

    if (pr2CamListNumEntries(&camList) == 0) {
      ROS_FATAL("No cameras found");
      node_.shutdown();
      return;
    }

    // TODO: look for specific serial number
    camera_ = pr2CamListGetEntry(&camList, 0);

    // Configure the camera with its IP address, wait up to 500ms for completion
    if (node_.hasParam("~ip_address")) {
      std::string ip_address;
      node_.getParam("~ip_address", ip_address);
      
      int retval = pr2Configure(camera_, ip_address.c_str(), SEC_TO_USEC(0.5));
      if (retval != 0) {
        ROS_FATAL("IP address configuration failed");
        if (retval == ERR_CONFIG_ARPFAIL)
          ROS_FATAL("Must be root to make changes to the ARP table");
        node_.shutdown();
        return;
      }
      ROS_INFO("Configured camera #%d, S/N #%u, IP address %s", 0, camera_->serial, ip_address.c_str());
    }
    else {
      ROS_FATAL("IP address not specified");
      node_.shutdown();
      return;
    }

    // We are going to receive the video on this host, so we need our own MAC address
    sockaddr localMac;
    if ( wgEthGetLocalMac(camera_->ifName, &localMac) != 0 ) {
      ROS_FATAL("Unable to get local MAC address for interface %s", camera_->ifName);
      node_.shutdown();
      return;
    }

    // We also need our local IP address
    in_addr localIp;
    if ( wgIpGetLocalAddr(camera_->ifName, &localIp) != 0) {
      ROS_FATAL("Unable to get local IP address for interface %s", camera_->ifName);
      node_.shutdown();
      return;
    }

    // Select a video mode
    // TODO: make this a parameter
    if ( pr2ImagerModeSelect( camera_, MT9VMODE_752x480x15b1 ) != 0) {
      ROS_FATAL("Mode select error");
      node_.shutdown();
      return;
    }

    // Start video; send it to specified host port (defaults to 9090)
    int port;
    node_.param("~port", port, 9090);
    if ( pr2StartVid( camera_, (uint8_t *)&(localMac.sa_data[0]), inet_ntoa(localIp), port) != 0 ) {
      ROS_FATAL("Video start error");
      node_.shutdown();
      return;
    }

    // Receive frames through callback
    // TODO: start this in separate thread?
    node_.advertise<image_msgs::Image>("~raw", 1);
    pr2VidReceive( camera_->ifName, port, 480, 752, &ForearmNode::frameHandler, this );
  }

  ~ForearmNode()
  {
    // Stop video
    if ( pr2StopVid(camera_) != 0 )
      ROS_ERROR("Video Stop error");
  }

  void publishImage(size_t width, size_t height, uint8_t *frameData)
  {
    fillImage(image_, "image", height, width, 1, "bayer_bggr", "uint8", frameData);
    node_.publish("~raw", image_);
  }

private:
  static int frameHandler(size_t width, size_t height, uint8_t *frameData,
                          PacketEOF *eofInfo, void *userData)
  {
    ForearmNode &fa_node = *(ForearmNode*)userData;
    if (!fa_node.node_.ok())
      return 1;
    
    if (eofInfo == NULL) {
      ROS_WARN("Frame was missing EOF, no frame information available");
      return 0;
    }

    // Check for short packet (video lines were missing)
    if (eofInfo->header.line_number == IMAGER_LINENO_SHRT) {
      ROS_WARN("Short frame (video lines were missing)");
      return 0;
    }

    fa_node.publishImage(width, height, frameData);

    return 0;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node n("forearm_node");
  ForearmNode fn(n);
  //n.spin();

  return 0;
}
