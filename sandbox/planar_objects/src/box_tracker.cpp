/*
 * box_tracker.cpp
 *
 *  Created on: Jul 28, 2009
 *      Author: sturm
 */

#include "assert.h"
#include "box_tracker.h"

#include "vis_utils.h"
#include "opencv_latest/CvBridge.h"
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"


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
  observations_msg = observations;
}

void BoxTracker::syncCallback()
{
  ROS_INFO("BoxTracker::syncCallback(), received %d observations",observations_msg->get_obs_size());

  observations.clear();
  for(size_t i=0; i<observations_msg->obs.size(); i++)
    observations.push_back(btBoxObservation(observations_msg->obs[i]));

  visualizeObservations();
}

void BoxTracker::visualizeObservations() {
  std::vector<btVector3> points;
  points.resize(5);

  std::vector<std::pair<btVector3,btVector3> > lines;
  lines.resize(5);
  for(size_t i=0; i<MIN(100,observations.size()); i++) {
    points[0] = observations[i].tf * btVector3(0,0,0);
    points[1] = observations[i].tf * btVector3(observations[i].w,0,0);
    points[2] = observations[i].tf * btVector3(observations[i].w,observations[i].h,0);
    points[3] = observations[i].tf * btVector3(0,observations[i].h,0);
    points[4] = observations[i].tf * btVector3(0,0,0.1);

    lines[0].first = points[0];  lines[0].second = points[1];
    lines[1].first = points[1];  lines[1].second = points[2];
    lines[2].first = points[2];  lines[2].second = points[3];
    lines[3].first = points[3];  lines[3].second = points[0];

    lines[4].first = points[0];  lines[4].second = points[4];

    visualizeLines(visualization_pub, observations_msg->header.frame_id, lines,
                   500+i,HSV_to_RGB(i/(double)observations_msg->obs.size(),1.0,1.0));
  }
  lines.clear();
  for(size_t i=observations.size();i<100; i++)
    visualizeLines(visualization_pub, observations_msg->header.frame_id, lines,
                   500+i,HSV_to_RGB(i/(double)observations_msg->obs.size(),1.0,1.0));
}

btBoxObservation::btBoxObservation( BoxObservation obs ) {
  btVector3 origin(obs.transform.translation.x,
                   obs.transform.translation.y,
                   obs.transform.translation.z);

  btQuaternion orientation(obs.transform.rotation.x,
                           obs.transform.rotation.y,
                           obs.transform.rotation.z,
                           obs.transform.rotation.w);

  tf = btTransform(orientation, origin);
  w = obs.w;
  h = obs.h;
  precision = obs.precision;
  recall = obs.recall;
}

}
