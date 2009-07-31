/*
 * stereo_throttle.h
 *
 *  Created on: Jul 30, 2009
 *      Author: sturm
 */

#ifndef STEREO_THROTTLE_H_
#define STEREO_THROTTLE_H_

#include "ros/ros.h"
#include "topic_synchronizer2/topic_synchronizer.h"

#include "sensor_msgs/RawStereo.h"

namespace planar_objects {

class StereoThrottle
{
public:
  ros::NodeHandle nh;

  int n;

  // PARAMETERS
  int divisor;

  // MESSAGES - INCOMING
  ros::Subscriber stereo_sub;

  // MESSAGES - OUTGOING
  ros::Publisher stereo_pub;

  // Constructor
  StereoThrottle();

  // Callbacks
  void stereoCallback(const sensor_msgs::RawStereo::ConstPtr& stereo);
};

}

int main(int argc, char** argv);

#endif /* STEREO_THROTTLE_H_ */
