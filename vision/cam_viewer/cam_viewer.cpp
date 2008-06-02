#include <cstdio>
#include <vector>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "ros/node.h"
#include "std_msgs/MsgImage.h"

using namespace std;
using namespace ros;

class CvView : public node
{
public:
  MsgImage image_msg;

  CvView() : node("cam_viewer")
  { 
    cvNamedWindow("cam_viewer", CV_WINDOW_AUTOSIZE);
    subscribe("image", image_msg, &CvView::image_cb);
  }
  void image_cb()
  {
    IplImage *cv_image = cvCreateImage( cvSize( image_msg.width, image_msg.height), IPL_DEPTH_8U, 3);
    memcpy(cv_image->imageData, image_msg.data, cv_image->imageSize);
    cvShowImage("cam_viewer", cv_image);
    cvWaitKey(3);
    cvReleaseImage(&cv_image);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  CvView view;
  view.spin();
  ros::fini();
  return 0;
}

