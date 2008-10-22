///////////////////////////////////////////////////////////////////////////////
// The axis_cam package provides a library that talks to Axis IP-based cameras
// as well as ROS nodes which use these libraries
//
// Copyright (C) 2008, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "ros/ros_slave.h"
#include "common_flows/FlowImage.h"
#include "common_flows/ImageCodec.h"
#include "elphel_cam/elphel_cam.h"

class Elphel_Cam_Node : public ROS_Slave
{
public:
  FlowImage *image;
  ImageCodec<FlowImage> *codec;
  string elphel_host;
  Elphel_Cam *cam;
  int frame_id;
  double fps;
  int im_dec;

  Elphel_Cam_Node() : ROS_Slave(), cam(NULL), frame_id(0)
  {
    register_source(image = new FlowImage("image"));
    codec = new ImageCodec<FlowImage>(image);

    register_with_master();
    if (!get_string_param(".host", elphel_host))
      elphel_host = "192.168.0.9";
    printf("elphel_cam host set to [%s]\n", elphel_host.c_str());

    if (!get_int_param(".frame_id", &frame_id))
      frame_id = -1;
    printf("elphel_cam frame_id set to [%d]\n", frame_id);

    if (!get_double_param(".fps", &fps))
      fps = 20;
    printf("elphel_cam fps set to [%d]\n", fps);

    if (!get_int_param(".im_dec", &im_dec))
      im_dec = 4;
    printf("elphel_cam image decimation and binning set to [%d]\n", im_dec);

    cam = new Elphel_Cam(elphel_host);

    cam->init(fps, im_dec, im_dec);

    cam->start();
  }
  virtual ~Elphel_Cam_Node()
  {
    if (cam)
      delete cam;
  }
  bool take_and_send_image()
  {
    uint8_t *jpeg;
    uint32_t jpeg_size;
    if (!cam->next_jpeg(&jpeg, &jpeg_size))
    {
      ROS_ERROR("Elphel_Cam::next_jpeg returned an error");
      return false;
    }

    // Sometimes things break for no great reason.  When this happens
    // the http server returns a 1x1 GIF.
    // Skip failures and re-init cam if we get too many of them
    int failcount = 0;
    while (jpeg[0] == 0x47 && jpeg[1] == 0x49) { //THIS IS A GIF
      if (failcount++ > 10) {
	printf("Received too many failures... restarting cam\n");
	cam->init(fps, im_dec, im_dec);
	cam->start();
	failcount = 0;
      }
      if (!cam->next_jpeg(&jpeg, &jpeg_size))
      {
        ROS_ERROR("Elphel_Cam::next_jpeg returned an error");
        return false;
      }
    }

    image->set_data_size(jpeg_size);
    memcpy(image->data, jpeg, jpeg_size);

    image->compression = "jpeg";
    image->colorspace = "rgb24";

    codec->extract_header_to_flow();

    image->frame_id = frame_id;

    printf("Sending %d x %d\n", image->width, image->height);

    image->publish();
    return true;
  }
};

int main(int argc, char **argv)
{
  Elphel_Cam_Node a;
  while (a.happy())
    if (!a.take_and_send_image())
    {
      ROS_ERROR("couldn't take image.");
      break;
    }
  return 0;
}
