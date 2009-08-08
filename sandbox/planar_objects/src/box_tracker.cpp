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

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "box_tracker");

  planar_objects::BoxTracker node;
  ros::spin();

  return 0;
}

namespace planar_objects
{

// Constructor
BoxTracker::BoxTracker() :
  sync(&BoxTracker::syncCallback, this)
{
  nh.param("~visualize_obs", show_obs, true);
  nh.param("~visualize_tracks", show_tracks, true);
  nh.param("~verbose", verbose, true);

  nh.param("~timeout", timeout, 20.0);

  nh.param("~translation_tolerance", params.translation_tolerance, 0.20);
  nh.param("~rotation_tolerance", params.rotation_tolerance, M_PI/8);
  nh.param("~size_tolerance", params.size_tolerance, 0.03);

  newLines = 0;
  oldLines = 0;
  track_ids = 0;

  // subscribe to topics
  observations_sub = nh.subscribe("box_detector/observations", 1, sync.synchronize(&BoxTracker::observationsCallback,
                                                                                  this));

  // advertise topics
  tracks_pub = nh.advertise<BoxTracks> ("~tracks", 100);
  visualization_pub = nh.advertise<visualization_msgs::Marker> ("visualization_marker", 100);
}

void BoxTracker::observationsCallback(const BoxObservations::ConstPtr& observations)
{
  observations_msg = observations;
}

void BoxTracker::syncCallback()
{
  ROS_INFO("BoxTracker::syncCallback(), received %d observations",observations_msg->get_obs_size());

  observations.clear();

  for(size_t i=0; i<observations_msg->obs.size(); i++) {
    observations.push_back(btBoxObservation(observations_msg->obs[i], observations_msg->header.stamp));
  }

//  if(observations.size()==0) return;
  std::vector< std::vector< btBoxObservation > > data_assoc_batch;
  data_assoc_batch.resize(tracks.size());

  for(size_t i=0;i<observations.size();i++) {
    std::vector<btBoxObservation> ambiguity = observations[i].listAmbiguity();
//    std::vector<btBoxObservation> ambiguity;
//    ambiguity.push_back( observations[i] );

    // associate observation to tracks
    std::vector< std::pair<size_t,btBoxObservation> > data_assoc;
    for(size_t j=0;j<tracks.size();j++) {
      for(size_t k=0;k<ambiguity.size();k++) {
        if(tracks[j].withinTolerance(ambiguity[k])) {
          //        cout << "assigning obs "<<i<<" to track "<< j << endl;
          data_assoc.push_back( std::pair<size_t,btBoxObservation>(j,ambiguity[k]) );
        }
      }
    }

    if(data_assoc.size()==0) {
      // start new track
//      cout << "starting new track for obs "<<i << endl;
      tracks.push_back( btBoxTrack(params, observations[i], track_ids++) );
      data_assoc_batch.push_back(std::vector< btBoxObservation >());
    } else {
      // first come first serve..?
//      cout << "putting obs "<< i<<" in batch for update in track "<<data_assoc[0] << endl;
      data_assoc_batch[ data_assoc[0].first ].push_back( data_assoc[0].second );
    }
  }

  // now run batch update (this allows a tracker to incorporate several "corners" at the same time)
  for(size_t j=0;j<tracks.size();j++) {
//    cout << "batch updating track "<<j<<" with n="<<data_assoc_batch[j].size()<<" observations" << endl;
    tracks[j].updateTrack(data_assoc_batch[j]);
//    cout <<"  w="<<tracks[j].obs_history.rbegin()->w<< " h="<<tracks[j].obs_history.rbegin()->h<<endl;
  }

  ///observations = observations[0].listAmbiguity();

  removeOldTracks(ros::Duration(timeout));
  if(show_obs) {
    visualizeObservations();
  }
  if(show_tracks) {
    visualizeTracks();
  }
  removeOldLines();
  sendTracks();
}

void BoxTracker::removeOldTracks(ros::Duration timeout) {
  size_t j = 0;
  while (j<tracks.size()) {
    if( observations_msg->header.stamp > tracks[j].obs_history.rbegin()->stamp + timeout
        ||observations_msg->header.stamp < tracks[j].obs_history.rbegin()->stamp ) {
      tracks.erase(tracks.begin() + j);
    } else {
      j++;
    }
  }
}

void BoxTracker::sendTracks() {
  BoxTracks tracks_msg;
  tracks_msg.header = observations_msg->header;
  tracks_msg.set_tracks_size(tracks.size());
  for(size_t j=0; j<tracks.size(); j++) {
    tracks_msg.tracks[j] = tracks[j].getTrackMessage();
  }
  tracks_pub.publish(tracks_msg);
  ROS_INFO("Sending %d tracks",tracks.size());
}

void BoxTracker::removeOldLines() {
  LineVector lines;
  for(int l = newLines; l<oldLines; l++) {
      visualizeLines(visualization_pub, observations_msg->header.frame_id, lines,
                     l,0,0,0);
  }
  oldLines = newLines;
  newLines = 0;
}

void BoxTracker::visualizeObservations() {
  for(size_t i=0; i<observations.size(); i++) {
//    cout << "drawing line "<<newLines << endl;
    visualizeLines(visualization_pub, observations_msg->header.frame_id, observations[i].visualize(),
                   newLines++,show_tracks?HSV_to_RGB(0.0 * i,0.0,1.0):0x00ff00);
  }
}

void BoxTracker::visualizeTracks() {
//  cout << "observations_msg->header.stamp="<<(long)observations_msg->header.stamp.toSec() << endl;
  for(size_t j=0; j<tracks.size(); j++) {
//    cout << "tracks[j].obs_history.rbegin()->stamp="<<(long)tracks[j].obs_history.rbegin()->stamp.toSec() << endl;
//    if( observations_msg->header.stamp > tracks[j].obs_history.rbegin()->stamp + ros::Duration(1.00) ) continue;
    for(size_t i=0; i<tracks[j].obs_history.size(); i++) {
        visualizeLines(visualization_pub, observations_msg->header.frame_id, tracks[j].obs_history[i].visualize(),
                       newLines++,
                       HSV_to_RGB(
                          j/(double)tracks.size(),
                          MIN(1.0, 0.2*tracks[j].obs_history.size() ),
                          1.0
                          //((i+3)/((double)tracks[j].obs_history.size()+3))
//                          1.0
                          ));
    }
  }
}


}
