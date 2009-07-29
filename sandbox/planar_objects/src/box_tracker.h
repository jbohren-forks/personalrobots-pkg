/*
 * box_tracker.h
 *
 *  Created on: Jul 28, 2009
 *      Author: sturm
 */

#ifndef BOX_TRACKER_H_
#define BOX_TRACKER_H_

#include "ros/ros.h"
#include "topic_synchronizer2/topic_synchronizer.h"

#include "visualization_msgs/Marker.h"

#include "planar_objects/BoxObservations.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


namespace planar_objects
{

class BoxTracker
{
public:
  ros::NodeHandle nh;
  TopicSynchronizer sync;

  // PARAMETERS
  bool show_boxes;
  bool verbose;

  // MESSAGES - INCOMING
  ros::Subscriber observations_sub;
  BoxObservationsConstPtr observations;

  // MESSAGES - OUTGOING
  ros::Publisher filtered_observations_pub;
  ros::Publisher visualization_pub;

  // Constructor
  BoxTracker();

  // Callbacks
  void observationsCallback(const BoxObservations::ConstPtr& observations);
  void syncCallback();

  // Main loop
  bool spin();

  void visualizeObservations();
};

}

int main(int argc, char** argv);

#endif /* BOX_TRACKER_H_ */
