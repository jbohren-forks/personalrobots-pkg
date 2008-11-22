#include <cstdio>
#include <vector>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "ros/node.h"
#include "std_msgs/Image.h"
#include "image_utils/cv_bridge.h"

using namespace std;
using namespace ros;

class CvMTurk : public node
{
public:
  std_msgs::Image image_msg;
  CvBridge<std_msgs::Image> cv_bridge;

  CvMTurk() : node("cv_mturk"), cv_bridge(&image_msg, CvBridge<std_msgs::Image>::CORRECT_BGR)
  { 
    cvNamedWindow("cv_mturk", CV_WINDOW_AUTOSIZE);
    subscribe("image", image_msg, &CvMTurk::image_cb, 1);
  }

  void image_cb()
  {
    IplImage *cv_img_to_label;

    if (cv_bridge.to_cv(&cv_img_to_label))
    {
      //VIEW THE IMAGE
      cvShowImage("cv_mturk", cv_img_to_label);

      //HANDLE KEYBOARD INPUT
      int c = cvWaitKey(3)&0xFF;
      switch(c)
      { 
      	case 27:  //ESQ -- exit
	case 'q': // or quit
        case 'Q':    
		printf("Bye bye\n");
		cvReleaseImage(&cv_img_to_label);
		self_destruct();
		break;
      }
      //MECH TURK STUFF HERE, OR ABOVE ...

     //RELEASE THIS IMAGE FOR NEXT LOOP
      cvReleaseImage(&cv_img_to_label);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  CvMTurk view;
  view.spin();  //infinite loop in node which calls back to image_cb() "image call back".  exit by calling self_destruct()
  ros::fini();
  return 0;
}

