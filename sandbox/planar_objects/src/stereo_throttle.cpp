/*
 * stereo_throttle.cpp
 *
 *  Created on: Jul 30, 2009
 *      Author: sturm
 */

#include "stereo_throttle.h"

#include "assert.h"
#include "box_detector.h"
#include "find_planes.h"
#include "vis_utils.h"

#include "opencv_latest/CvBridge.h"
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "vis_utils.h"

using namespace ros;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_throttle");

  planar_objects::StereoThrottle node;
  ros::spin();

  return 0;
}

namespace planar_objects
{

#define CVWINDOW(a) cvNamedWindow(a,CV_WINDOW_AUTOSIZE); cvMoveWindow(a,(cvWindows /3 )* 500,(cvWindows%3)*500); cvWindows++;
#define SQR(a) ((a)*(a))

// Constructor
StereoThrottle::StereoThrottle()
{
  nh.param("~divisor", divisor, 30);
  cout << "divisor="<<divisor<<endl;

  string stereo_ns = nh.resolveName("narrow_stereo");
  stereo_sub = nh.subscribe(stereo_ns+"/raw_stereo", 1, &StereoThrottle::stereoCallback,this);

  // advertise topics
  stereo_pub = nh.advertise<sensor_msgs::RawStereo> ("~raw_stereo", 1);

  n = 0;
}

void StereoThrottle::stereoCallback(const sensor_msgs::RawStereo::ConstPtr& stereo)
{
  n++;
  if(n % divisor != 1) {
    ROS_INFO("skipping frame");
    return;
  }
  stereo_pub.publish(stereo);
  ROS_INFO("publishing frame");
}

}
