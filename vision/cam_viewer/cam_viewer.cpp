#include <cstdio>
#include <vector>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "ros/node.h"
#include "std_msgs/Image.h"

using namespace std;
using namespace ros;

class CvView : public node
{
public:
  std_msgs::Image image_msg;
	char key;
	int imgnum;

  CvView() : node("cam_viewer")
  { 
    cvNamedWindow("cam_viewer", CV_WINDOW_AUTOSIZE);
    subscribe("image", image_msg, &CvView::image_cb);
		imgnum=0;
  }
  void image_cb()
  {
    IplImage *cv_image = cvCreateImage( cvSize( image_msg.width, image_msg.height), IPL_DEPTH_8U, 3);
    memcpy(cv_image->imageData, image_msg.data, cv_image->imageSize);
    cvShowImage("cam_viewer", cv_image);
    key = cvWaitKey(10);
		if (key==' ')
		{
			char imgname[30];
			sprintf(imgname, "img_%d.png", imgnum);
			cvSaveImage(imgname, cv_image);
			imgnum++;
		}

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

