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

// Author: Blaise Gassend

#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <sched.h>

#include <ros/console.h>
#include <ros/time.h>

#include <ipcam_packet.h>
#include <host_netutil.h>
#include <fcamlib.h>

class ForearmCameraSimulator
{
public:
  ForearmCameraSimulator(uint32_t serial) : serial_no_(serial)
  {                      
    in_addr in_addr;
    in_addr.s_addr = 0;
    socket_ = wgSocketCreate(&in_addr, WG_CAMCMD_PORT);
    if (socket_ == -1)
    {
      ROS_ERROR("Error creating/binding socket: %s", strerror(errno));
      return;
    }
    
    for (int i = 0; i < NUM_IMAGER_REGISTERS; i++)
      imager_register_flags_[i] = IMAGER_REGISTER_UNSUPPORTED;
    imager_register_flags_[0x7F] &= ~IMAGER_REGISTER_UNSUPPORTED;
    
    signal(SIGTERM, ForearmCameraSimulator::setExiting);
    reset();
  }

  int run()
  {
    if (socket_ == -1)
      return -1;
  
    while (!exiting_)
    {
      ProcessCommands();
      SendData();
    }

    return 0;
  }

  ~ForearmCameraSimulator()
  {
    if (socket_ != -1)
      close(socket_);
  }

private:
  uint32_t serial_no_;
  int socket_;
  int running_;
  int width_;
  int height_;
  int frame_;
  ros::Duration period_;
  ros::Time next_frame_time_;
  sockaddr_in data_addr_;
  static const int NUM_IMAGER_REGISTERS = 256;
  static const int IMAGER_REGISTER_UNDEFINED = 1;
  static const int IMAGER_REGISTER_UNSUPPORTED = 2;
  uint16_t imager_registers_[NUM_IMAGER_REGISTERS]; 
  uint8_t imager_register_flags_[NUM_IMAGER_REGISTERS];

  void reset()
  {
    running_ = 0;
    frame_ = 0;
    width_ = 640;
    height_ = 480;
    period_ = ros::Duration(1/30.);
    next_frame_time_ = ros::Time(0);
    for (int i = 0; i < NUM_IMAGER_REGISTERS; i++)
    {
      imager_registers_[i] = 0;
      imager_register_flags_[i] |= IMAGER_REGISTER_UNDEFINED;
    }
    imager_register_flags_[0x7F] &= ~IMAGER_REGISTER_UNDEFINED;
  }
      
  static inline uint8_t get_diag_test_pattern(int x, int y)
  {
    if ((x + 1) / 2 + y < 500)
      return 14 + x / 4;
    else
      return 0;
  }

  void SendData()
  {
    if (!running_)
    {
      ros::Duration(0.01).sleep();
      return;
    }

    ros::Duration sleeptime = (next_frame_time_ - ros::Time::now());
    if (sleeptime < -period_)
    {
      ROS_WARN("Lost lock on frame #%i", frame_);
      next_frame_time_ = ros::Time::now();
    }
    else
      sleeptime.sleep();
    next_frame_time_ += period_;
    
    PacketVideoLine pvl;
    PacketEOF peof;

    peof.header.frame_number = pvl.header.frame_number = htonl(frame_);
    peof.header.horiz_resolution = pvl.header.horiz_resolution = htons(width_);
    peof.header.vert_resolution = pvl.header.vert_resolution = htons(height_);
  
    for (int y = 0; y < height_; y++)
    {
      if ((imager_registers_[0x7F] & 0x3C00) == 0x3800)
      {
        if (width_ > 320)
          for (int x = 0; x < width_; x++)
            pvl.data[x] = get_diag_test_pattern(x, y);
        else
          for (int x = 0; x < width_; x++)
            pvl.data[x] = (get_diag_test_pattern(2 * x, 2 * y) + get_diag_test_pattern(2 * x, 2 * y + 1)) / 2;
      }
      else
      {
        for (int x = 0; x < width_; x++)
          pvl.data[x] = x + y + frame_;
      }

      pvl.header.line_number = htons(y | (frame_ << 10));
      sendto(socket_, &pvl, sizeof(pvl.header) + width_, 0, 
          (struct sockaddr *) &data_addr_, sizeof(data_addr_));
      if (y == 0)
        sched_yield(); // Make sure that first packet arrives with low jitter
    }

    peof.header.line_number = htons(IMAGER_LINENO_EOF);
    sendto(socket_, &peof, sizeof(peof), 0, 
        (struct sockaddr *) &data_addr_, sizeof(data_addr_));
    
    frame_ = (frame_ + 1) & 0xFFFF;
  }

  void ProcessCommands()
  {
    char buff[100];

    while (true)
    {
      ssize_t len = recv(socket_, buff, 100, MSG_DONTWAIT);
      if (len == -1)
        break;

      PacketGeneric *hdr = (PacketGeneric *) buff;

      if (ntohl(hdr->magic_no) != WG_MAGIC_NO ||
          ntohl(hdr->type) > PKT_MAX_ID)
      {
        ROS_INFO("Got a packet with a bad magic number or type."); \
        continue;
      }

#define CAST_AND_CHK(type) \
if (len != sizeof(type)) \
{ \
   ROS_INFO("Got a "#type" with incorrect length"); \
   continue;\
} \
ROS_INFO("Got a "#type""); \
type *pkt = (type *) buff; 
// End of this horrid macro that makes pkt appear by magic.

      switch (ntohl(hdr->type))
      {
        case PKTT_DISCOVER:
          {
            CAST_AND_CHK(PacketDiscover);
            sendAnnounce(&pkt->hdr);
            break;
          }

        case PKTT_CONFIGURE:
          {
            CAST_AND_CHK(PacketConfigure);
            if (pkt->product_id != htonl(CONFIG_PRODUCT_ID) ||
                pkt->ser_no != htonl(serial_no_))
              break;
            running_ = false;
            sendAnnounce(&pkt->hdr);
            break;
          } 
    
        case PKTT_TRIGCTRL:
          {
            CAST_AND_CHK(PacketTrigControl);
            if (pkt->trig_state == 0 && !running_)
              sendStatus(&pkt->hdr, PKT_STATUST_OK, 0);
            else
              sendStatus(&pkt->hdr, PKT_STATUST_ERROR, PKT_ERROR_INVALID);
            break;
          }

        case PKTT_IMGRMODE:
          {
            CAST_AND_CHK(PacketImagerMode);
            uint32_t mode = htonl(pkt->mode);
            if (mode >= 10 || running_)
            {
              sendStatus(&pkt->hdr, PKT_STATUST_ERROR, PKT_ERROR_INVALID);
              break;
            }
            double fps[10] = {  15,12.5,  30,  25,  15,12.5,  60,  50,  30,  25 };
            int widths[10] = { 752, 752, 640, 640, 640, 640, 320, 320, 320, 320 };
            int heights[10] ={ 480, 480, 480, 480, 480, 480, 240, 240, 240, 240 };
            period_ = ros::Duration(1/fps[mode]);
            width_ = widths[mode];
            height_ = heights[mode];
            sendStatus(&pkt->hdr, PKT_STATUST_OK, 0);
          }
          break;

        case PKTT_VIDSTART:
          {
            CAST_AND_CHK(PacketVidStart);
            if (running_)
              sendStatus(&pkt->hdr, PKT_STATUST_ERROR, PKT_ERROR_INVALID);
            else
            {
              sendStatus(&pkt->hdr, PKT_STATUST_OK, 0);
              bzero(&data_addr_, sizeof(data_addr_)); 
              data_addr_.sin_family = AF_INET; 
              data_addr_.sin_addr.s_addr = pkt->receiver.addr; 
              data_addr_.sin_port = pkt->receiver.port; 
            }
            running_ = true;
          }
          break;

        case PKTT_VIDSTOP:
          {
            CAST_AND_CHK(PacketVidStop);
            if (running_)
              sendStatus(&pkt->hdr, PKT_STATUST_OK, 0);
            else
              sendStatus(&pkt->hdr, PKT_STATUST_ERROR, PKT_ERROR_INVALID);
            running_ = false;
          }
          break;

        case PKTT_RESET:
          {
            reset();
          }
          break;

        case PKTT_SENSORRD:
          {
            CAST_AND_CHK(PacketSensorRequest);
            if (imager_register_flags_[pkt->address] & IMAGER_REGISTER_UNDEFINED)
              ROS_ERROR("Reading uninitialized imager register 0x%02X.", pkt->address);
            if (imager_register_flags_[pkt->address] & IMAGER_REGISTER_UNSUPPORTED)
              ROS_ERROR("Reading unsupported imager register 0x%02X.", pkt->address);
            sendSensorData(&pkt->hdr, pkt->address, imager_registers_[pkt->address]);
          }
          break;

        case PKTT_SENSORWR:
          {
            CAST_AND_CHK(PacketSensorData);
            if (imager_register_flags_[pkt->address] & IMAGER_REGISTER_UNSUPPORTED)
              ROS_ERROR("Writing unsupported imager register 0x%02X.", pkt->address);
            imager_register_flags_[pkt->address] &= ~IMAGER_REGISTER_UNDEFINED;
            imager_registers_[pkt->address] = ntohs(pkt->data);
            sendStatus(&pkt->hdr, PKT_STATUST_OK, 0);
          }
          break;

        default:
          ROS_ERROR("Got an unknown message type: %i", ntohl(hdr->type));
          break;
      }
#undef CAST_AND_CHK
    }
  }
  
#define FILL_HDR(pkttype, pktid) \
ROS_INFO("Sending a "#pkttype" packet."); \
Packet##pkttype rsp; \
rsp.hdr.magic_no = htonl(WG_MAGIC_NO); \
strncpy(rsp.hdr.hrt, #pkttype, sizeof(rsp.hdr.hrt)); \
rsp.hdr.type = htonl(pktid);
// End of this horrid macro that makes resp appear by magic.
    
#define SEND_RSP \
sockaddr_in rsp_addr; \
bzero(&rsp_addr, sizeof(rsp_addr)); \
rsp_addr.sin_family = AF_INET; \
rsp_addr.sin_addr.s_addr = hdr->reply_to.addr; \
rsp_addr.sin_port = hdr->reply_to.port; \
sendto(socket_, &rsp, sizeof(rsp), 0, \
    (struct sockaddr *) &rsp_addr, sizeof(rsp_addr)); 

  void sendAnnounce(PacketGeneric *hdr)
  {        
    FILL_HDR(Announce, PKTT_ANNOUNCE); 
    bzero(rsp.mac, sizeof(rsp.mac)); 
    rsp.product_id = htonl(CONFIG_PRODUCT_ID); 
    rsp.ser_no = htonl(serial_no_); 
    strncpy(rsp.product_name, "Forearm_camera_simulator.", sizeof(rsp.product_name)); 
    rsp.hw_version = htonl(0x2041); 
    rsp.fw_version = htonl(0x0112); 
    strncpy(rsp.camera_name, "Forearm_camera_simulator.", sizeof(rsp.camera_name)); 
    SEND_RSP;
  }

  void sendStatus(PacketGeneric *hdr, uint32_t type, uint32_t code)
  {        
    if (type != PKT_STATUST_OK)
      ROS_ERROR("Error Condition.");
    FILL_HDR(Status, PKTT_STATUS); 
    rsp.status_type = htonl(type);
    rsp.status_code = htonl(code);
    SEND_RSP;
  }

  void sendSensorData(PacketGeneric *hdr, uint8_t addr, uint16_t value)
  {        
    FILL_HDR(SensorData, PKTT_SENSORDATA); 
    rsp.address = addr;
    rsp.data = htons(value);
    SEND_RSP;
  }

#undef FILL_HDR
#undef SEND_RSP
  volatile static bool exiting_;
  
  static void setExiting(int i)
  {
    exiting_ = true;
  }
};

volatile bool ForearmCameraSimulator::exiting_ = false;

int main(int argc, char **argv)
{
  ForearmCameraSimulator fcamsim(12345);
  return fcamsim.run();
}

