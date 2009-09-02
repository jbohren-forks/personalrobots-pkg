/*
 * articulation_learner.cpp
 *
 *  Created on: Aug 3, 2009
 *      Author: sturm
 */

#include "assert.h"
#include "track_visualizer.h"

#include "vis_utils.h"

using namespace std;

#define sqr(a) ((a)*(a))
#define MIN(a,b) (a<b?a:b)

int main(int argc, char **argv)
{
  ros::init(argc, argv, "track_visualizer");

  planar_objects::TrackVisualizer node;
  ros::spin();

  return 0;
}

namespace planar_objects
{

// Constructor
TrackVisualizer::TrackVisualizer()
{
  // subscribe to topics
  tracks_sub = nh.subscribe("box_tracker/tracks", 1,&TrackVisualizer::tracksCallback,this);
  oldLines = 0;
  newLines = 0;

  // advertise topics
  visualization_pub = nh.advertise<visualization_msgs::Marker> ("~tracks_marker", 0);
}

void TrackVisualizer::tracksCallback(const BoxTracks::ConstPtr& tracks_msg)
{
  ROS_INFO("received %d tracks",tracks_msg->get_tracks_size());
  this->tracks_msg = tracks_msg;
  setVisualization(&visualization_pub, NULL, tracks_msg->header);

  tracks.clear();       // forget everything
  for(size_t i =0; i<tracks_msg->tracks.size();i++) {
	  tracks.push_back( btBoxTrack(tracks_msg->tracks[i], tracks_msg->header.stamp,0) );
  }
  visualizeTracks();
  removeOldLines();
}

void TrackVisualizer::visualizeTracks() {
	  for(size_t j=0; j<tracks.size(); j++) {
	//    cout << "tracks[j].obs_history.rbegin()->stamp="<<(long)tracks[j].obs_history.rbegin()->stamp.toSec() << endl;
	//    if( observations_msg->header.stamp > tracks[j].obs_history.rbegin()->stamp + ros::Duration(1.00) ) continue;
	    for(size_t i=0; i<tracks[j].obs_history.size(); i++) {
	        visualizeLines(tracks[j].obs_history[i].visualize(),
	                       newLines++,
	                       HSV_to_RGB(
	                          j/(double)tracks.size(),
	                          1.00,
	                          1.00
	                          ));
	    }
	  }
}

void TrackVisualizer::removeOldLines() {
  LineVector lines;
  for(int l = newLines; l<oldLines; l++) {
      visualizeLines(lines,
                     l,0,0,0);
  }
  oldLines = newLines;
  newLines = 0;
}


}
