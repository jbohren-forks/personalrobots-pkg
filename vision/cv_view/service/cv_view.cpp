#include <cstdio>
#include <vector>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "ros/node.h"
#include "std_msgs/Image.h"
#include "std_srvs/PolledImage.h"
#include "image_utils/cv_bridge.h"
#include "boost/thread/mutex.hpp"

#include <sys/stat.h>

using namespace std;

class CvView : public ros::node
{
public:
  CvBridge<std_msgs::Image> cv_bridge;

  std_srvs::PolledImage::request  req;
  std_srvs::PolledImage::response res;

  boost::mutex cv_mutex;

  IplImage *cv_image;

  CvView() : node("cv_view"), cv_bridge(&(res.image), CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U),
             cv_image(0)
  { 
    cvNamedWindow("cv_view", CV_WINDOW_AUTOSIZE);
  }

  ~CvView()
  {
    if (cv_image)
      cvReleaseImage(&cv_image);
  }

  void check_keys() 
  {
    cv_mutex.lock();
    if (cvWaitKey(3) == 10)
    {
      if (cv_image)
        cvReleaseImage(&cv_image);

      if (ros::service::call("polled_image", req, res))
      {
        if (cv_bridge.to_cv(&cv_image))
        {
          cvShowImage("cv_view", cv_image);
        }        
      }
      else
        printf("error calling the polled_image service\n");
    }
    cv_mutex.unlock();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  CvView view;
  while (view.ok()) 
  {
    usleep(10000);
    view.check_keys();
  }
  ros::fini();
  return 0;
}

