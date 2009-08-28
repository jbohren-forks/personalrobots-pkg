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

/* Theora compressed video streaming node for ROS, written by Ethan Dreyfuss*/
// Usage, start a compression node on one side of a slow network link (i.e. wireless to the Robot)
// and start a decompression node on the other side of the link, feed ROS images in one and and get ROS images out the other end,
// The process is lossy but low-bandwidth.

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv_latest/CvBridge.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/format.hpp>

#include "compressed_video_streaming/packet.h"

#include <stdio.h>

#include <vector>

#define null 0

#include <theora/codec.h>
#include <theora/theoraenc.h>
#include <theora/theoradec.h>

//#include <ogg/ogg.h>
//#include <shout/shout.h>

using std::vector;

class ImageCompressionNode {
private:
    ros::NodeHandle node_handle_;
    ros::V_Subscriber subs_;

    sensor_msgs::ImageConstPtr last_msg_;
    sensor_msgs::CvBridge img_bridge_;
    boost::mutex image_mutex_;

    std::string window_name_;
    boost::format filename_format_;
    int count_;
    ros::Publisher pub_;

    th_enc_ctx* encoding_context_;
    vector<ogg_packet> stream_header_;

    int nearestWidth;
    int nearestHeight;
    int nearestXoff;
    int nearestYoff;

public:
    ImageCompressionNode(const ros::NodeHandle& node_handle) :
        node_handle_(node_handle), filename_format_(""), count_(0) {
        node_handle_.param("~window_name", window_name_,
                node_handle_.resolveName("image"));

        bool autosize;
        node_handle_.param("~autosize", autosize, false);

        std::string format_string;
        //node_handle_.param("~filename_format", format_string, std::string("frame%04i.jpg"));
        //filename_format_.parse(format_string);

        //cvNamedWindow(window_name_.c_str(), autosize ? CV_WINDOW_AUTOSIZE : 0);
        //cvSetMouseCallback(window_name_.c_str(), &ImageCompressionNode::mouse_cb, this);
        // cvStartWindowThread();

        subs_.push_back(node_handle_.subscribe("image", 1,
                &ImageCompressionNode::image_cb, this));
        pub_ = node_handle_.advertise<compressed_video_streaming::packet> ("image_stream", 30, boost::bind(&ImageCompressionNode::sendHeader, this, _1));

        do {
            ROS_DEBUG("No subscribers, sleeping for 3 seconds");
            ros::Duration d = ros::Duration(3, 0);
            d.sleep();
            ROS_DEBUG("Done sleeping, flushing packets through system:");
            for (int i = 0; i < 10; i++) {
                ogg_packet oggpacket;
                oggpacket.packetno = 999999;
                oggpacket.bytes = 0;
                compressed_video_streaming::packet msg;
                oggPacketToMsg(oggpacket, msg);
                pub_.publish(msg);
            }
        } while (pub_.getNumSubscribers() == 0);

        encoding_context_ = null;
    }

    ~ImageCompressionNode() {
        //cvDestroyWindow(window_name_.c_str());
        if (encoding_context_ != null)
            th_encode_free(encoding_context_);
    }

    void sendHeader(const ros::SingleSubscriberPublisher& pub)
    {
        compressed_video_streaming::packet msg;
        for(unsigned int i=0; i<stream_header_.size(); i++)
        {
            oggPacketToMsg(stream_header_[i], msg);
            pub.publish(msg);
            ROS_DEBUG("Published header packet, sleeping for 0.1 second");
            ros::Duration d = ros::Duration(0, 100000000);
            d.sleep();
        }
    }

    void ensure_encoding_context(const CvSize &size) {
        if (encoding_context_ == null) {
            th_info encoder_setup;
            th_info_init(&encoder_setup);
            nearestWidth = size.width + size.width % 16 == 0 ? 0 : (16
                    - size.width % 16);
            nearestHeight = size.height + size.height % 16 == 0 ? 0 : (16
                    - size.height % 16);

            /* Theora has a divisible-by-sixteen restriction for the encoded frame size */
            /* scale the picture size up to the nearest /16 and calculate offsets */
            nearestWidth = size.width + 15 & ~0xF;
            nearestHeight = size.height + 15 & ~0xF;
            /*Force the offsets to be even so that chroma samples line up like we
             expect.*/
            nearestXoff = nearestWidth - size.width >> 1 & ~1;
            nearestYoff = nearestHeight - size.height >> 1 & ~1;

            encoder_setup.frame_width = nearestWidth;
            encoder_setup.frame_height = nearestHeight;
            encoder_setup.pic_width = size.width;
            encoder_setup.pic_height = size.height;
            encoder_setup.pic_x = nearestXoff;
            encoder_setup.pic_y = nearestYoff;
            ROS_DEBUG("Creating context with Width: %d, Height: %d", nearestWidth, nearestHeight);
            encoder_setup.colorspace = TH_CS_UNSPECIFIED;
            //encoder_setup.colorspace = TH_CS_ITU_REC_470M;     //TH_CS_ITU_REC_470M     A color space designed for NTSC content.
            //TH_CS_ITU_REC_470BG     A color space designed for PAL/SECAM content.
            encoder_setup.pixel_fmt = TH_PF_420; //see bottom of http://www.theora.org/doc/libtheora-1.1beta1/codec_8h.html
            encoder_setup.target_bitrate = 800000;
            //encoder_setup.quality = 63;    //On a scale of 0 to 63, to use this set target bitrate to 0
            encoder_setup.aspect_numerator = 1;
            encoder_setup.aspect_denominator = 1;
            encoder_setup.fps_numerator = 0;
            encoder_setup.fps_denominator = 0;
            encoder_setup.keyframe_granule_shift = 6; //Apparently a good default

            encoding_context_ = th_encode_alloc(&encoder_setup);

            if (encoding_context_ == null)
                ROS_DEBUG("No encoding context immediately after alloc.");

            th_comment comment;
            th_comment_init(&comment);
            th_comment_add(&comment,
                    (char*) "Compression node written by Ethan.");
            comment.vendor
                    = (char*) "Encoded by Willow Garage image_compression_node.";

            if (encoding_context_ == null)
                ROS_DEBUG("Encoding context not successfully created.");

            ogg_packet oggpacket;
            while (th_encode_flushheader(encoding_context_, &comment,
                    &oggpacket) > 0) {
                stream_header_.push_back(oggpacket);
                //Have to deep copy the packet since theora owns the packet memory, not doing this causes nasty bugs and is very hard to track down!!!
                stream_header_.back().packet = new unsigned char[oggpacket.bytes];
                memcpy(stream_header_.back().packet, oggpacket.packet, oggpacket.bytes);
            }
            //ROS_DEBUG("Published %d header packets.", stream_header_.size());
            //th_comment_clear(&comment);  //TODO: this should happen but is causing crazy seg faults (probably trying to free a string literal)

            //Stream the header in case anyone is already listening
            compressed_video_streaming::packet msg;
            for(unsigned int i=0; i<stream_header_.size(); i++)
            {
                //testProcessHeader(stream_header_[i]);
                oggPacketToMsg(stream_header_[i], msg);
                pub_.publish(msg);
                ROS_DEBUG("Published header packet, sleeping for 0.1 second");
                ros::Duration d = ros::Duration(0, 100000000);
                d.sleep();
            }

            if (encoding_context_ == null)
                ROS_DEBUG("Encoding context killed by header flushing.");
        }
    }

    void oggPacketToMsg(const ogg_packet &oggpacket, compressed_video_streaming::packet &msgOutput) {
        msgOutput.blob.resize(oggpacket.bytes);
        memcpy(&msgOutput.blob[0], oggpacket.packet, oggpacket.bytes);
        msgOutput.bytes = oggpacket.bytes;
        msgOutput.b_o_s = oggpacket.b_o_s;
        msgOutput.e_o_s = oggpacket.e_o_s;
        msgOutput.granulepos = oggpacket.granulepos;
        msgOutput.packetno = oggpacket.packetno;
        //ROS_DEBUG("Ready to send %d bytes in packet#%d and granule%d (and this is BOS: %d).", msgOutput.bytes, msgOutput.packetno, msgOutput.granulepos, msgOutput.b_o_s);
        unsigned int i = 0;
        for (int j = 0; j < msgOutput.bytes; j++)
            i = i * 2 % 91 + msgOutput.blob[j];
        ROS_DEBUG("Checksum is: %d", i);
    }

//This function duplicates the header processing code on the decoding side and can be used
//for testing that the header packets produced are correct.
    void testProcessHeader(ogg_packet &oggpacket)
    {
        static th_info header_info_;
        static th_comment header_comment_;

        //th_setup_info setup_info_struct;
        static th_setup_info* setup_info_ = 0;
        static th_dec_ctx* decoding_context_ = 0;
        static bool inited = false;
        if(!inited)
        {
            inited = true;
            th_info_init(&header_info_);
        }

        ROS_DEBUG("Setup_info: %p", setup_info_);
        int rval = th_decode_headerin(&header_info_, &header_comment_, &setup_info_, &oggpacket);
        ROS_DEBUG("Setup_info: %p", setup_info_);
        if(rval == 0)
        { //This never seems to happen for some reason even though the docs make it sound like it should
            ROS_DEBUG("Processed header properly");
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
            ROS_DEBUG("Processed header improperly");
            decoding_context_ = th_decode_alloc(&header_info_, setup_info_);
        }
    }

    void image_cb(const sensor_msgs::ImageConstPtr& msg) {
        boost::lock_guard<boost::mutex> guard(image_mutex_);

        // Hang on to message pointer for sake of mouse_cb
        last_msg_ = msg;

        /** @todo: restore raw bayer display, ideally without copying the whole msg... */
#if 0
        // May want to view raw bayer data
        if (msg->encoding.find("bayer") != std::string::npos)
        msg->encoding = "mono";
#endif

        if (img_bridge_.fromImage(*msg, "bgr8")) {
            IplImage* img = img_bridge_.toIpl();
            //cvShowImage(window_name_.c_str(), img);
            IplImage* img2 = cvCreateImage(cvGetSize(img), img->depth,
                    img->nChannels);
            cvCvtColor(img, img2, CV_BGR2YCrCb);
            ensure_encoding_context(cvGetSize(img2));

            //convert image
            th_ycbcr_buffer ycbcr_image;
            vector<unsigned char> planes[3]; //color planes
            for (int i = 0; i < 3; i++) //Size is number of pixels in padded image (chroma subsampled by factor of 2 in each dimension, which means area is 1/4)
                planes[i].resize(nearestWidth * nearestHeight / (i > 0 ? 4 : 1));

            //ROS_DEBUG("Image size: %d image width: %d image height: %d", img2->imageSize, img2->width, img2->height);
            for (int i = 0; i < img2->imageSize / 3; i++) //imageSize/3 is the number of pixels
            {
                planes[0][i] = *(unsigned char*) (img2->imageData + i * 3); //Y
                //planes[2][i] = *(unsigned char*)(img2->imageData + i*3 + 1);  //Cr
                //planes[1][i] = *(unsigned char*)(img2->imageData + i*3 + 2);  //Cb
            }

            //Note that while OpenCV uses YCbCr, theora uses YCrCb... this can make things a little confusing
            //CHROMA subsampling
            for (int planeIdx = 1; planeIdx < 3; planeIdx++) {
                int swappedIdx = planeIdx;
                if (planeIdx == 1)
                    swappedIdx = 2;
                else if (planeIdx == 2)
                    swappedIdx = 1;
                for (int i = 0; i < img2->width; i += 2)
                    for (int j = 0; j < img2->height; j += 2) {
                        int planeDataIdx = i / 2 + (j * img2->width) / 4;
                        //planes[2][planeDataIdx]
                        unsigned int total =
                                (unsigned int) ((uchar*) (img2->imageData
                                        + img2->widthStep * j))[i * 3
                                        + planeIdx];
                        unsigned int count = 1;
                        if (i < img2->width - 1) {
                            total += (unsigned int) ((uchar*) (img2->imageData
                                    + img2->widthStep * j))[(i + 1) * 3
                                    + planeIdx];
                            count++;
                        }
                        if (j < img2->height - 1) {
                            total += (unsigned int) ((uchar*) (img2->imageData
                                    + img2->widthStep * (j + 1)))[i * 3
                                    + planeIdx];
                            count++;
                        }
                        if (i < img2->width - 1 && j < img2->height - 1) {
                            total += (unsigned int) ((uchar*) (img2->imageData
                                    + img2->widthStep * (j + 1)))[(i + 1) * 3
                                    + planeIdx];
                            count++;
                        }
                        planes[swappedIdx][planeDataIdx] = (uchar) (total
                                / count);
                    }
            }

            for (int i = 0; i < 3; i++) {
                ycbcr_image[i].width = nearestWidth / (i > 0 ? 2 : 1); //
                ycbcr_image[i].height = nearestHeight / (i > 0 ? 2 : 1); //chroma is subsampled by a factor of 2
                ycbcr_image[i].stride = img2->width / (i > 0 ? 2 : 1); //
                ycbcr_image[i].data = &planes[i][0];
            }
            ROS_DEBUG("Width: %d, Height: %d, xOff: %d, yOff: %d", nearestWidth, nearestHeight, nearestXoff, nearestYoff);

            cvReleaseImage(&img2);

            int rval;
            if (encoding_context_ == null)
                ROS_DEBUG("About to encode with null encoding context.");
            rval = th_encode_ycbcr_in(encoding_context_, ycbcr_image);
            if (rval == TH_EFAULT)
                ROS_DEBUG("EFault in encoding.");
            if (rval == TH_EINVAL)
                ROS_DEBUG("EInval in encoding.");
            ROS_DEBUG("Encoding resulted in: %d", rval);

            ogg_packet oggpacket;
            compressed_video_streaming::packet output;
            ROS_DEBUG("Ready to get encoded packets.");
            while( (rval = th_encode_packetout(encoding_context_, 0, &oggpacket)) > 0) {
                oggPacketToMsg(oggpacket, output);
                ROS_DEBUG("Publishing packet!");
                pub_.publish(output);
            }
            ROS_DEBUG("Punted from while loop with rval %d", rval);
        } else
            ROS_ERROR("Unable to convert from %s to bgr", msg->encoding.c_str());

    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "stream_encoder", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    if (n.resolveName("image") == "/image") {
        ROS_DEBUG("image_compression_node: image has not been remapped! Example command-line usage:\n"
                "\t$ rosrun image_compression_node image_view image:=/forearm/image_color");
    }

    ImageCompressionNode view(n);

    ros::spin();

    return 0;
}
