#include <cstdio>
#include <vector>
#include "opencv/cxcore.h"
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
    IplImage *cv_image_sc;

    if (cv_bridge.to_cv(&cv_image))
    {

    cv_image_sc = cvCreateImage(cvSize(cv_image->width, cv_image->height),
				IPL_DEPTH_8U, 1);
      //      printf("got cv image with dim: %d %d\n", cvGetSize(cv_image).width, cvGetSize(cv_image).height);
    if (cv_image->depth == IPL_DEPTH_16U)
      cvCvtScale(cv_image, cv_image_sc, 0.0625, 0);
    else
      cvCvtScale(cv_image, cv_image_sc, 1.0, 0);
      
      cvShowImage("cv_view", cv_image_sc);
      cvSaveImage("last_img.png", cv_image_sc);
      cvWaitKey(3);
      cvReleaseImage(&cv_image);
      cvReleaseImage(&cv_image_sc);
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

