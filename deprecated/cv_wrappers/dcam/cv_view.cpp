#include <cstdio>
#include <vector>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "ros/node.h"
#include "image_msgs/Image.h"
#include "image_msgs/CvBridge.h"

using namespace std;
using namespace ros;

class CvView : public Node
{
public:
  image_msgs::Image img;
  image_msgs::CvBridge bridge;

  CvView() : Node("cv_view")
  { 
    cvNamedWindow("cv_view", CV_WINDOW_AUTOSIZE);
    subscribe("image", img, &CvView::image_cb, 1);
  }
  void image_cb()
  {
    bridge.fromImage(img);

    cvShowImage("cv_view", bridge.toIpl());

    cvWaitKey(5);
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

