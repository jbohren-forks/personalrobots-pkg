/*
 * box_tracker.cpp
 *
 *  Created on: Jul 28, 2009
 *      Author: sturm
 */

#include "assert.h"
#include "box_tracker.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "box_tracker");

  planar_objects::BoxTracker node;
  node.spin();

  return 0;
}

namespace planar_objects
{

// Constructor
BoxTracker::BoxTracker() :
  sync(&BoxTracker::syncCallback, this)
{
  nh.param("~show_boxes", show_boxes, true);
  nh.param("~verbose", verbose, true);

  // subscribe to topics
  observations_sub = nh.subscribe("box_detector/observations", 1, sync.synchronize(&BoxTracker::observationsCallback,
                                                                                  this));

  // advertise topics
  filtered_observations_pub = nh.advertise<BoxObservations> ("~filtered_observations", 1);
  visualization_pub = nh.advertise<visualization_msgs::Marker> ("visualization_marker", 1);
}


bool BoxTracker::spin()
{
  while (nh.ok())
  {
    ros::spinOnce();
  }

  return true;
}

void BoxTracker::observationsCallback(const BoxObservations::ConstPtr& observations)
{
  this->observations = observations;
}

void BoxTracker::syncCallback()
{
  ROS_INFO("BoxTracker::syncCallback(), received %d observations",observations->get_obs_size());

  visualizeObservations();
}

void BoxTracker::visualizeObservations() {

}

}
