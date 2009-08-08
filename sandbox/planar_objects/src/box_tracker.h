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
#include "planar_objects/BoxTracks.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "vis_utils.h"
#include "track_utils.h"

namespace planar_objects
{

class BoxTracker
{
public:
  ros::NodeHandle nh;
  TopicSynchronizer sync;

  int oldLines;
  int newLines;
  int track_ids;

  // PARAMETERS
  bool show_obs;
  bool show_tracks;
  bool verbose;
  TrackParameters params;
  double timeout;

  // MESSAGES - INCOMING
  ros::Subscriber observations_sub;
  BoxObservationsConstPtr observations_msg;
  std::vector<btBoxObservation> observations;
  std::vector<btBoxTrack> tracks;

  // MESSAGES - OUTGOING
  ros::Publisher tracks_pub;
  ros::Publisher visualization_pub;

  // Constructor
  BoxTracker();

  // Callbacks
  void observationsCallback(const BoxObservations::ConstPtr& observations);
  void syncCallback();

  void visualizeObservations();
  void visualizeTracks();
  void removeOldLines();
  void removeOldTracks(ros::Duration timeout = ros::Duration(5.00) );
  void sendTracks();
};

}

int main(int argc, char** argv);

#endif /* BOX_TRACKER_H_ */
