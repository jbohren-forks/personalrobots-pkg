#include <vector>

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "ros/node.h"
#include "image_msgs/StereoInfo.h"
#include "image_msgs/Image.h"
#include "image_msgs/CvBridge.h"

#include "rosthread/condition.h"

using namespace std;

class StereoView : public ros::node
{
public:

  image_msgs::Image limage;
  image_msgs::Image rimage;
  image_msgs::Image dimage;

  image_msgs::CvBridge lbridge;
  image_msgs::CvBridge rbridge;
  image_msgs::CvBridge dbridge;

  std::string ltopic;
  std::string rtopic;
  std::string dtopic;

  ros::Time image_time;

  ros::thread::condition cond_all;

  int count;
  bool done;

  StereoView() : ros::node("cv_view"), count(0), done(false)
  { 
    ltopic = map_name("dcam") + std::string("/left/image_rect");
    rtopic = map_name("dcam") + std::string("/right/image_rect");
    dtopic = map_name("dcam") + std::string("/disparity");

    cvNamedWindow("left", CV_WINDOW_AUTOSIZE);
    subscribe(ltopic, limage, &StereoView::image_cb, &limage, 1);

    cvNamedWindow("right", CV_WINDOW_AUTOSIZE);
    subscribe(rtopic, rimage, &StereoView::image_cb, &rimage, 1);

    cvNamedWindow("disparity", CV_WINDOW_AUTOSIZE);
    subscribe(dtopic, dimage, &StereoView::image_cb, &dimage, 1);
  }

  void image_cb(void* p)
  {
    image_msgs::Image* img = (image_msgs::Image*)(p);

    cond_all.lock();

    // If first to time, wait
    if (count == 0)
    {
      wait_for_others(img);
      return;
    }

    // If behind, skip
    if (img->header.stamp < image_time)
    {
      cond_all.unlock();
      return;
    }

    // If at time, increment and signal or wait
    if (img->header.stamp == image_time)
    {
      count++;
      if (count == 3)
      {
        cond_all.broadcast();
      }
      else
      {
        while (!done && img->header.stamp == image_time)
          cond_all.wait();
      }
      
      cond_all.unlock();
      return;
    }

    // If ahead, wake up others and then wait
    if (img->header.stamp > image_time)
    {
      printf(" %s is already past with time: %d\n", img->label.c_str(), img->header.stamp.nsec);
      cond_all.broadcast();
      wait_for_others(img);
    }
  }

  void wait_for_others(image_msgs::Image* img)
  {
    count = 1;
    done = false;
    image_time = img->header.stamp;
    bool timed_out = false;

    while (count < 3 && img->header.stamp == image_time && !timed_out)
      if (!cond_all.timed_wait(1))
      {
        printf(" Timed out waiting for other images...\n");
        timed_out = true;
      }
    
    if (img->header.stamp == image_time && !timed_out)
      image_cb_all();
    else
      printf(" Got interrupted from time %d\n", img->header.stamp.nsec);

    if (img->header.stamp == image_time)
    {
      done = true;
      count = 0;
      cond_all.broadcast();
    }
    cond_all.unlock();
  }

  void image_cb_all()
  {
    lbridge.fromImage(limage);
    rbridge.fromImage(rimage);
    dbridge.fromImage(dimage);

    IplImage* disp = cvCreateImage(cvGetSize(dbridge.toIpl()), IPL_DEPTH_8U, 1);

    // Disparity has to be scaled to be be nicely displayable
    cvCvtScale(dbridge.toIpl(), disp, 1/4.0);

    cvShowImage("left", lbridge.toIpl());
    cvShowImage("right", rbridge.toIpl());
    cvShowImage("disparity", disp);

    cvWaitKey(5);

    cvReleaseImage(&disp);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  StereoView view;
  view.spin();
  ros::fini();
  return 0;
}

