#include "ros/ros.h"
#include "prosilica_camera/PolledImage.h"

// NOTE: res must be global! CvBridge assumes it can just point
//       to the pixel data in the image message.
prosilica_cam::PolledImage::Request req;
prosilica_cam::PolledImage::Response res;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "posilica_poll");
  ros::NodeHandle n;

  while (true)
  {
    req.timeout_ms = 100;

    printf("Request\n");
    if (!ros::service::call("prosilica/poll", req, res)) 
    //if (!client.call(req))
    {
      ROS_ERROR("Service call failed");
      return NULL;
    }

    //printf("Sleeping\n");
    //usleep(100000);
  }

}
