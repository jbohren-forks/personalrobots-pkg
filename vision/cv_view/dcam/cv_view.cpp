#include <cstdio>
#include <vector>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "ros/node.h"
#include "image_msgs/ImageWrapper.h"

using namespace std;
using namespace ros;

class CvView : public node
{
public:
  image_msgs::ImageWrapper img;

  CvView() : node("cv_view")
  { 
    cvNamedWindow("cv_view", CV_WINDOW_AUTOSIZE);
    subscribe("image", img, &CvView::image_cb, 1);
  }
  void image_cb()
  {
    IplImage *cv_image = img.asIplImage();

    cvShowImage("cv_view", cv_image);
    cvWaitKey(5);

    cvReleaseImageHeader(&cv_image);
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

