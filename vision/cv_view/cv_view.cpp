#include <cstdio>
#include <vector>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "ros/node.h"
#include "std_msgs/MsgImage.h"
#include "image_utils/cv_bridge.h"

using namespace std;
using namespace ros;

class CvView : public node
{
public:
  MsgImage image_msg;
  CvBridge<MsgImage> cv_bridge;

  CvView() : node("cv_view"), cv_bridge(&image_msg)
  { 
    cvNamedWindow("cv_view", CV_WINDOW_AUTOSIZE);
    subscribe("image", image_msg, &CvView::image_cb);
  }
  void image_cb()
  {
    IplImage *cv_image;
    if (cv_bridge.to_cv(&cv_image))
    {
      cvShowImage("cv_view", cv_image);
      cvWaitKey(3);
      cvReleaseImage(&cv_image);
    }
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

