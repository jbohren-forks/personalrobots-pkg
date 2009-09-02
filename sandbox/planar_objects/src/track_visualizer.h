/*
 * articulation_learner.h
 *
 *  Created on: Aug 3, 2009
 *      Author: sturm
 */

#ifndef TRACK_VISUALIZER_H_
#define TRACK_VISUALIZER_H_

#include "ros/ros.h"

#include "visualization_msgs/Marker.h"

#include "planar_objects/BoxObservations.h"
#include "planar_objects/BoxTracks.h"

#include "vis_utils.h"
#include "track_utils.h"

namespace planar_objects
{

class TrackVisualizer
{
public:
  ros::NodeHandle nh;

  // MESSAGES - INCOMING
  ros::Subscriber tracks_sub;
  BoxTracksConstPtr tracks_msg;
  std::vector<btBoxTrack> tracks;

  // MESSAGES - OUTGOING
  ros::Publisher visualization_pub;

  int oldLines,newLines;

  // Constructor
  TrackVisualizer();

  // Callbacks
  void tracksCallback(const BoxTracks::ConstPtr& tracks);

  void visualizeTracks();

  void removeOldLines();
};

}

int main(int argc, char** argv);


#endif /* ARTICULATED_OBJECTS_H_ */
