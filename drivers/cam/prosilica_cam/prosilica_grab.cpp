#include "ros/node.h"
#include "image_msgs/CvBridge.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "prosilica_cam/PolledImage.h"

static char wndname[] = "Captured image";

image_msgs::CvBridge bridge;

IplImage* callPollProsilica(int timeout)
{
  prosilica_cam::PolledImage::Request req;
  prosilica_cam::PolledImage::Response res;
  req.timeout_ms = timeout;
  if (!ros::service::call("prosilica/poll", req, res)) {
    ROS_FATAL("Service call failed\n");
    return NULL;
  }
  
  if (!bridge.fromImage(res.image, "bgr")) {
    ROS_FATAL("CvBridge::fromImage failed\n");
    return NULL;
  }
  
  return bridge.toIpl();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv);
  ros::Node n("poll_prosilica_client");

  cvNamedWindow(wndname);
  
  while (true)
  {
    int k = cvWaitKey(0);
    switch( (char) k)
    {
      case 'q':
        goto exit_main;
      case 'c':
        IplImage* display = callPollProsilica(100);
        if (display)
          cvShowImage(wndname, display);
        break;
    }
  }

exit_main:
  cvDestroyWindow(wndname);
}
