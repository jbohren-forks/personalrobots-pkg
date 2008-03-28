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

#include "ros/ros_slave.h"
#include "common_flows/FlowImage.h"
#include "common_flows/ImageCvBridge.h"
#include "opencv/highgui.h"

class CVCam : public ROS_Slave
{
public:
  FlowImage *image_f;
  ImageCvBridge<FlowImage> *cv_bridge;
  CvCapture *capture;
  IplImage *cam_image;
  double last_image_time;
  int show_image;

  CVCam() : ROS_Slave(), last_image_time(0), show_image(0)
  {
    register_source(image_f = new FlowImage("image"));
    image_f->frame_id = 0; // frame is zero unless specified
    get_int_param(".frame_id", &image_f->frame_id);
    get_int_param(".show_image", &show_image);
    cv_bridge = new ImageCvBridge<FlowImage>(image_f);
    capture = cvCaptureFromCAM(CV_CAP_ANY);
    if (!capture)
      log(ROS::ERROR,"woah! couldn't open a camera. is one connected?\n");
  }
  virtual ~CVCam() 
  {
    if (capture)
      cvReleaseCapture(&capture);
  }
  bool take_and_send_picture()
  {
    if (!capture)
      return false;
    cam_image = cvQueryFrame(capture);
    double t = runtime();
    double dt = t - last_image_time;
    printf("dt = %f\t(%f fps)\n", dt, 1.0 / dt);
    last_image_time = t;
    if (show_image)
      cvShowImage("CV Cam", cam_image);
    image_f->compression = "raw";
    cv_bridge->from_cv(cam_image);
    cv_bridge->set_flow_data();
    image_f->publish();
    return true;
  }
};

int main(int argc, char **argv)
{
  CVCam a;
  if (a.show_image)
    cvNamedWindow("CV Cam", CV_WINDOW_AUTOSIZE);
  while (a.happy())
  {
    if (a.show_image)
      cvWaitKey(5);
    if (!a.take_and_send_picture())
      usleep(100000);
  }
  if (a.show_image)
    cvDestroyWindow("CV Cam");
  return 0;
}

