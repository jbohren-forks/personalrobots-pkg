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

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv_latest/CvBridge.h>

#include <boost/thread.hpp>
#include <boost/format.hpp>

#include "compressed_video_streaming/packet.h"

#include <stdio.h>

#include <vector>

#define null 0

//#include <ogg/ogg.h>
//#include <theora/theora.h>
#include <theora/codec.h>
#include <theora/theoraenc.h>
#include <theora/theoradec.h>

using std::vector;

class ImageView
{
private:
  ros::NodeHandle node_handle_;
  ros::Subscriber sub_;
  
  sensor_msgs::ImageConstPtr last_msg_;
  sensor_msgs::CvBridge img_bridge_;
  boost::mutex image_mutex_;
  
  std::string window_name_;
  boost::format filename_format_;
  int count_;
  ros::Publisher pub_;
  bool received_header_;

  th_dec_ctx* decoding_context_;
  th_info header_info_;
  th_comment header_comment_;

  //th_setup_info setup_info_struct;
  th_setup_info* setup_info_;

public:
  ImageView(const ros::NodeHandle& node_handle)
    : node_handle_(node_handle), filename_format_(""), count_(0)
  {
    node_handle_.param("~window_name", window_name_, node_handle_.resolveName("image"));

    bool autosize;
    node_handle_.param("~autosize", autosize, false);
    
    std::string format_string;
    //node_handle_.param("~filename_format", format_string, std::string("frame%04i.jpg"));
    //filename_format_.parse(format_string);
    
    cvNamedWindow(window_name_.c_str(), autosize ? CV_WINDOW_AUTOSIZE : 0);
    cvResizeWindow( window_name_.c_str(), 640, 480 );
    //cvSetMouseCallback(window_name_.c_str(), &ImageView::mouse_cb, this);
    cvStartWindowThread();

    sub_ = node_handle_.subscribe("image_stream", 1, &ImageView::packet_cb, this);
    pub_ = node_handle_.advertise<sensor_msgs::Image>("streamed_video", 30);

    decoding_context_ = null;
    received_header_ = false;

    setup_info_ = null;

    th_info_init(&header_info_);
  }

  ~ImageView()
  {
    cvDestroyWindow(window_name_.c_str());
    if(decoding_context_ != null)
    	th_decode_free(decoding_context_);
  }

  //When using this caller is responsible for deleting oggpacket.packet!!
  void msgToOggPacket(const compressed_video_streaming::packet &msg, ogg_packet &oggpacketOutput)
  {
	  oggpacketOutput.bytes = msg.bytes;
	  oggpacketOutput.b_o_s = msg.b_o_s;
	  oggpacketOutput.e_o_s = msg.e_o_s;
	  oggpacketOutput.granulepos = msg.granulepos;
	  oggpacketOutput.packetno = msg.packetno;
	  oggpacketOutput.packet = new unsigned char[msg.bytes];
	  memcpy(oggpacketOutput.packet, &msg.blob[0], msg.bytes);

	  //ROS_DEBUG("Received %d bytes in packet#%d and granule%d (and this is BOS: %d).", oggpacketOutput.bytes, oggpacketOutput.packetno, oggpacketOutput.granulepos, oggpacketOutput.b_o_s);
	  unsigned int i=0;
	  for(int j=0; j<msg.bytes; j++)
		  i=i*2 % 91 + oggpacketOutput.packet[j];
	  ROS_DEBUG("Checksum is: %d", i);
  }

  void packet_cb(const compressed_video_streaming::packet::ConstPtr& msgPtr)
  {
	ROS_DEBUG("Received a packet!");
    boost::lock_guard<boost::mutex> guard(image_mutex_);
    
    const compressed_video_streaming::packet &pkt = *msgPtr;
    ogg_packet oggpacket;
    msgToOggPacket(pkt, oggpacket);

    if(received_header_ == false) //still receiving header info
    {
    	if(oggpacket.packetno != 0)
    	{
    		ROS_DEBUG("Dumping header packet because packet# is non-zero: %d.", oggpacket.packetno);
    		return;
    	}
    	//static th_setup_info* setup_info_ptr = null;
    	//static th_setup_info** setup_info_ = &setup_info_ptr;
    	ROS_DEBUG("Setup_info: %p", setup_info_);
    	int rval = th_decode_headerin(&header_info_, &header_comment_, &setup_info_, &oggpacket);
    	ROS_DEBUG("Setup_info: %p", setup_info_);
    	if(rval == 0)
    	{
    		received_header_=true;
    		decoding_context_ = th_decode_alloc(&header_info_, setup_info_);
    		//th_setup_free(setup_info_);
    	}
    	else if(rval == TH_EFAULT)
    		ROS_DEBUG("EFault when processing header.");
    	else if(rval == TH_EBADHEADER) //Oddly, I seem to always get one of these...
    		ROS_DEBUG("Bad header when processing header.");
    	else if(rval == TH_EVERSION)
    		ROS_DEBUG("Bad version when processing header.");
    	else if(rval == TH_ENOTFORMAT)
    		ROS_DEBUG("Received packet which was not a Theora header.");
    	else if(rval < 0)
    		ROS_DEBUG("Error code when processing header: %d.", rval);

    	if(setup_info_ != null)
    	{
    		received_header_=true;
    		decoding_context_ = th_decode_alloc(&header_info_, setup_info_);
    	}
    }

    if(received_header_ == true)
    {
    	int rval = th_decode_packetin(decoding_context_, &oggpacket, null);

    	if(rval == 0)  	//Successfully got a frame
    	{
    		th_ycbcr_buffer ycbcr_image;
			th_decode_ycbcr_out(decoding_context_, ycbcr_image);

			//convert image to IplImage
			IplImage* img = cvCreateImage(cvSize(ycbcr_image[0].width, ycbcr_image[0].height), IPL_DEPTH_8U, 3);

//			for (int i = 0; i < img->imageSize/3; i++) {
//				*(unsigned char*) (img->imageData + i * 3)     = ycbcr_image[0].data[i];     //Y
//				*(unsigned char*) (img->imageData + i * 3 + 1) = ycbcr_image[2].data[i];     //Cr
//				*(unsigned char*) (img->imageData + i * 3 + 2) = ycbcr_image[1].data[i];     //Cb
//			}
			for (int planeIdx = 0; planeIdx<3; planeIdx++)
			{
				int swappedIdx = planeIdx;
				if(planeIdx == 1)
					swappedIdx = 2;
				else if(planeIdx == 2)
					swappedIdx = 1;
				for(int i=0; i<img->width; i++)
					for(int j=0; j<img->height; j++)
					{
						int ci = planeIdx > 0 ? i/2 : i; //Do simple pixel to 2x2 block scaling
						int cj = planeIdx > 0 ? j/2 : j;
						((uchar*)(img->imageData + img->widthStep*j))[i*3+planeIdx] = ycbcr_image[swappedIdx].data[ci+cj*ycbcr_image[swappedIdx].stride];
					}
			}

			IplImage* img2 = cvCreateImage(cvGetSize(img), img->depth, img->nChannels);
			cvCvtColor(img, img2, CV_YCrCb2BGR);

			cvShowImage(window_name_.c_str(), img2);

			sensor_msgs::Image outputMsg;

			if(pub_.getNumSubscribers() > 0) //No need to convert and publish if no one's listening
			{
				img_bridge_.fromIpltoRosImage(img2, outputMsg);
				pub_.publish(outputMsg);
			}

			cvReleaseImage(&img);
			cvReleaseImage(&img2);
    	}
    	else if(rval == TH_DUPFRAME)
    		ROS_DEBUG("Got a duplicate frame.");
    	else
    		ROS_DEBUG("Error code when decoding packet: %d.", rval);
    }

    delete oggpacket.packet;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stream_decoder", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  //if (n.resolveName("image") == "/image") {
  //  ROS_DEBUG("image_decompression_node: image has not been remapped! Example command-line usage:\n"
  //           "\t$ rosrun image_compression_node image_view image:=/forearm/image_color");
  //}
  
  ImageView view(n);

  ros::spin();
  
  return 0;
}
