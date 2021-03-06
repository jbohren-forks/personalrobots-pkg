/*
 * track_utils.h
 *
 *  Created on: Aug 3, 2009
 *      Author: sturm
 */

#ifndef TRACK_UTILS_H_
#define TRACK_UTILS_H_


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
  int plane_id;
  btBoxObservation();
  btBoxObservation(const BoxObservation &obs , const ros::Time stamp, int disambiguation=0 );
  void setObservation(const BoxObservation &obs , const ros::Time stamp );
  std::vector<btBoxObservation> listAmbiguity();
  btBoxObservation getAmbiguity(int num=0);
  LineVector visualize();
  BoxObservation getBoxObservation();
  std::string toString();
};

class TrackParameters {
public:
  double translation_tolerance;
  double rotation_tolerance;
  double size_tolerance;
  double scaling;
};

class btBoxTrack {
public:
  TrackParameters param;
  int id;
  std::vector<btBoxObservation> obs_history;
  btBoxTrack();
  btBoxTrack(TrackParameters param, btBoxObservation &obs, int id);
  btBoxTrack(const BoxTrack&  msg, const ros::Time stamp, int disambiguation=0 );
  bool withinTolerance(btBoxObservation &obs);
  void updateTrack(std::vector<btBoxObservation> &obs);
  void updateTrack(btBoxObservation &obs);
  BoxTrack getTrackMessage();
};

}

#endif /* TRACK_UTILS_H_ */
