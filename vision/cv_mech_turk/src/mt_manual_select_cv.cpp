// Mechanical Turk labeler node
// Gary Bradski, Dec 3, 2008.  Modified from Jeremey's minimal camera node and connecting to Alexander Sorokin's Mech Turk stuff
// 
// TO DO:
//Maybe make a parameter <path_to_location of submit_img.py>

#include <cstdio>
#include <vector>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "opencv_latest/CvBridge.h"
#include <stdlib.h>

using namespace std;
using namespace ros;

class MTManualSelectNode 
{
public:

  sensor_msgs::CvBridge cv_bridge;

  int image_counter_; 		
 

  ros::NodeHandle n_;
  ros::Subscriber img_to_display_sub_; //On key, we'll move from display to annotation
  ros::Publisher img_to_annotate_pub_;
  string image_topic_;


  MTManualSelectNode() : image_counter_(0)
  { 
    cvNamedWindow("mt_manual_select", CV_WINDOW_AUTOSIZE);
    image_topic_ = n_.resolveName("image");
    if (image_topic_ == "/image") {
      image_topic_ = string("/wide_stereo/left/image");  // By default
    }
    ROS_INFO_STREAM("Sending to topic " << image_topic_);
  }

  void init()
  {
    // Advertise the Images for annotation
    img_to_annotate_pub_ = n_.advertise<sensor_msgs::Image>("image_to_annotate",0);
    // Subscribe to the Images that we display
    img_to_display_sub_ = n_.subscribe(image_topic_,  (unsigned int)10, &MTManualSelectNode::onImage,this);
    // Retrieve internal message parameter, or else set default to 'pong! '
  }



  void help()
  {
	printf("\nUsage:\n  ./mt_manual_select  image:=/wide_stereo/left/image [or whatever your camera name is]\n\n"
	"   Takes keyboard input:\n"
	"\tESQ,q,Q:   Quit\n"
	"\th,H:       Print this help\n"
	"\tm,M:       Submit to Mechanical Turk (actually just send out on image_to_annotate. Run snapper.py node for the actual submission)\n\n" 
	);
  }



  void onImage(const sensor_msgs::ImageConstPtr& msg)
  {
    printf("On Image\n");

    const sensor_msgs::Image& image_msg=*msg;
    IplImage *cv_img_to_label;

    if (cv_bridge.fromImage(image_msg)) {
      cv_img_to_label = cv_bridge.toIpl();
      
      if (cv_img_to_label) {
	//VIEW THE IMAGE
	cvShowImage("mt_manual_select", cv_img_to_label);
	
	//HANDLE KEYBOARD INPUT
	int c = cvWaitKey(100)&0xFF;
	switch(c)
	  { 
	  case 27:  //ESQ -- exit
	  case 'q': // or quit
	  case 'Q':    
	    printf("Bye bye\n");
	    cvReleaseImage(&cv_img_to_label);
	    n_.shutdown();
	    break;
	  case 'h': //Help
	  case 'H':
	    help();
	    break;
	  case 'm': //Invoke Mechanical Turk submission
	  case 'M':
	    
	    image_counter_+= 1;		
	    printf("Sending\n");
	    ROS_INFO_STREAM("Sending msg " << image_counter_ );
	    img_to_annotate_pub_.publish(image_msg);
	    
	    break;
	  }
      }
    } else {
      ROS_ERROR("Unable to convert from %s to bgr", msg->encoding.c_str());
    }
  }
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "mt_manual_select");

  MTManualSelectNode view;

  view.init();

  ros::spin();  
  
  return 0;
}

