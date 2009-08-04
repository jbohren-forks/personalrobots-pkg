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

namespace planar_objects
{

class btBoxObservation {
public:
  ros::Time stamp;
  btTransform tf;
  double w;
  double h;
  double precision;
  double recall;

  void setObservation(const BoxObservation &obs , const ros::Time stamp );
  std::vector<btBoxObservation> listAmbiguity();
  LineVector visualize();
  BoxObservation getBoxObservation();
};

class TrackParameters {
public:
  double translation_tolerance;
  double rotation_tolerance;
  double size_tolerance;
};

class btBoxTrack {
public:
  TrackParameters *param;
  int id;
  std::vector<btBoxObservation> obs_history;

  btBoxTrack(TrackParameters *param, btBoxObservation &obs, int id);
  bool withinTolerance(btBoxObservation &obs);
  void updateTrack(std::vector<btBoxObservation> &obs);
  void updateTrack(btBoxObservation &obs);
  BoxTrack getTrackMessage();
};

class BoxTracker
{
public:
  ros::NodeHandle nh;
  TopicSynchronizer sync;

  int oldLines;
  int newLines;
  int track_ids;

  // PARAMETERS
  bool show_boxes;
  bool verbose;
  TrackParameters params;

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
