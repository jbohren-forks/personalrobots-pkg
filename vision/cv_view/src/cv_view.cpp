#include <cstdio>
#include <vector>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "ros/node.h"
#include "boost/thread/mutex.hpp"
#include "std_msgs/Image.h"
#include "image_utils/cv_bridge.h"

#include <sys/stat.h>

using namespace std;

class CvView : public ros::node
{
public:
  std_msgs::Image image_msg;
  CvBridge<std_msgs::Image> cv_bridge;

  boost::mutex cv_mutex;

  IplImage *cv_image;

  char dir_name[256];
  int img_cnt;
  bool made_dir;

  CvView() : node("cv_view", ros::node::ANONYMOUS_NAME), cv_bridge(&image_msg, CvBridge<std_msgs::Image>::CORRECT_BGR | CvBridge<std_msgs::Image>::MAXDEPTH_8U), 
             cv_image(0), img_cnt(0), made_dir(false)
  { 
    cvNamedWindow("cv_view", CV_WINDOW_AUTOSIZE);
    subscribe("image", image_msg, &CvView::image_cb, 1);

    time_t rawtime;
    struct tm* timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    sprintf(dir_name, "%s_images_%.2d%.2d%.2d_%.2d%.2d%.2d", 
            get_name().c_str()+1, timeinfo->tm_mon + 1, timeinfo->tm_mday,
            timeinfo->tm_year - 100,timeinfo->tm_hour, timeinfo->tm_min, 
            timeinfo->tm_sec);
  }

  ~CvView()
  {
    if (cv_image)
      cvReleaseImage(&cv_image);
  }

  void image_cb()
  {
    cv_mutex.lock();
    if (cv_image)
      cvReleaseImage(&cv_image);

    if (cv_bridge.to_cv(&cv_image))
    {
      cvShowImage("cv_view", cv_image);
    }
    cv_mutex.unlock();
  }

  void check_keys() 
  {
    cv_mutex.lock();
    if (cvWaitKey(3) == 10)
      save_image();
    cv_mutex.unlock();
  }

  void save_image() 
  {
    if (!made_dir) 
    {
      if (mkdir(dir_name, 0755)) 
      {
        printf("Failed to make directory: %s\n", dir_name);
        return;
      } 
      else 
        made_dir = true;
    }
    std::ostringstream oss;
    oss << dir_name << "/Img" << img_cnt++ << ".png";
    cvSaveImage(oss.str().c_str(), cv_image);
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

