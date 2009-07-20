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

/** \brief Node to select images manually */
class MTManualSelectNode 
{
public:

  sensor_msgs::CvBridge cv_bridge;

  int image_counter_; 		
 

  ros::NodeHandle n_;
  ros::Subscriber img_to_display_sub_; //On key, we'll move from display to annotation
  ros::Publisher img_to_annotate_pub_;

  string image_topic_in_;
  string image_topic_out_;

  int key_wait_time_;

  /** \brief Clears and deallocates the entire Octree. */
  MTManualSelectNode() : image_counter_(0)
  { 
    cvNamedWindow("mt_manual_select", CV_WINDOW_AUTOSIZE);

    n_.param( std::string("~key_wait"), key_wait_time_, 300);

    n_.param( std::string("~image_in"), image_topic_in_, std::string("/wide_stereo/left/image"));
    n_.param( std::string("~image_out"), image_topic_out_, std::string("image_to_annotate"));

    ROS_INFO_STREAM("Listening topic " << image_topic_in_ << "(" << n_.resolveName(image_topic_in_) << ")");
    ROS_INFO_STREAM("Sending to topic " << image_topic_out_ << "("<< n_.resolveName(image_topic_out_) <<")");
  }

  /** \brief Create subscriptions and publishers. */
  void init()
  {
    // Advertise the Images for annotation
    img_to_annotate_pub_ = n_.advertise<sensor_msgs::Image>(image_topic_out_,0);

    // Subscribe to the Images that we display and re-publish
    img_to_display_sub_ = n_.subscribe(image_topic_in_,  (unsigned int)1, &MTManualSelectNode::onImage,this);
  }


  /** \brief Print the help/usage message. */
  void help()
  {
	printf("\nUsage:\n  ./select_image  _image_in:=/wide_stereo/left/image [or whatever your camera name is]\n\n"
	"   Takes keyboard input:\n"
	"\tESQ,q,Q:   Quit\n"
	"\th,H:       Print this help\n"
	"\tm,M,s,S:       (S)ubmit to (M)echanical Turk (actually just send out on image_to_annotate. Run snapper.py node for the actual submission)\n\n" 
	);
  }


  /** \brief Handle image. Wait for user key and republish the messages.*/
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
	int c = cvWaitKey(key_wait_time_)&0xFF;
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
	  case 's': //Invoke Mechanical Turk submission
	  case 'S':
	    
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

