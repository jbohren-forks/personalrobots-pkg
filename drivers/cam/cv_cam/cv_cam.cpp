///////////////////////////////////////////////////////////////////////////////
// The cv_cam package provides a simple node which grabs images from a camera
// using OpenCV and publishes them.
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

#include "ros/node.h"
#include "std_msgs/MsgImage.h"
#include "image_utils/cv_bridge.h"
#include "opencv/highgui.h"

using namespace ros;

class CVCam : public node
{
public:
  MsgImage image_msg;
  CvBridge<MsgImage> cv_bridge;
  CvCapture *capture;
  IplImage *cam_image;
  double last_print_time;
  int show_image, fps_count, delay_us;

  CVCam() : node("cv_cam"), cv_bridge(&image_msg),
            last_print_time(0), show_image(0), fps_count(0)
  {
    advertise<MsgImage>("image");
    capture = cvCaptureFromCAM(CV_CAP_ANY);
    if (!capture)
    {
      ROS_FATAL("woah! couldn't open a camera. is one connected?");
      ROS_BREAK();
    }
    param("display", show_image, 0);
    param("delay_us", delay_us, 0);
    if (show_image)
      cvNamedWindow("cv_cam", CV_WINDOW_AUTOSIZE);
  }
  virtual ~CVCam()
  {
    if (capture)
      cvReleaseCapture(&capture);
    if (show_image)
      cvDestroyWindow("cv_cam");
  }
  bool take_and_send_picture()
  {
    if (!capture)
      return false;
    cam_image = cvQueryFrame(capture);
    if (!cam_image)
      return false;
    double t = clock.time();
    if ((last_print_time + 1 < t) && fps_count)
    {
      printf("%f fps\n", fps_count / (t - last_print_time));
      last_print_time = t;
      fps_count = 0;
    }
    fps_count++;
    cv_bridge.from_cv(cam_image);
    publish("image", image_msg);
    if (show_image)
    {
      cvShowImage("cv_cam", cam_image);
      cvWaitKey(5);
    }
    usleep(delay_us);
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  CVCam cvcam;
  while (cvcam.ok())
  {
    if (!cvcam.take_and_send_picture())
      usleep(100000);
  }
  ros::fini();
  return 0;
}

