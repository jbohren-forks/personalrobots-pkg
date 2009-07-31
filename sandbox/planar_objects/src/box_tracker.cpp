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
  nh.param("~show_boxes", show_boxes, true);
  nh.param("~verbose", verbose, true);

  nh.param("~translation_tolerance", params.translation_tolerance, 0.2);
  nh.param("~rotation_tolerance", params.translation_tolerance, M_PI/4);
  nh.param("~size_tolerance", params.size_tolerance, 0.10);

  newLines = 0;
  oldLines = 0;

  // subscribe to topics
  observations_sub = nh.subscribe("box_detector/observations", 1, sync.synchronize(&BoxTracker::observationsCallback,
                                                                                  this));

  // advertise topics
  filtered_observations_pub = nh.advertise<BoxObservations> ("~filtered_observations", 1);
  visualization_pub = nh.advertise<visualization_msgs::Marker> ("visualization_marker", 1);
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
    btBoxObservation obs;
    obs.setObservation(observations_msg->obs[i], observations_msg->header.stamp);
    observations.push_back(obs);
  }

  if(observations.size()==0) return;
  std::vector< std::vector< btBoxObservation > > data_assoc_batch;
  data_assoc_batch.resize(tracks.size());

  for(size_t i=0;i<observations.size();i++) {
    // associate observation to tracks
    std::vector< size_t > data_assoc;
    for(size_t j=0;j<tracks.size();j++) {
      if(tracks[j].withinTolerance(observations[i])) {
//        cout << "assigning obs "<<i<<" to track "<< j << endl;
        data_assoc.push_back( j );
      }
    }

    if(data_assoc.size()==0) {
      // start new track
//      cout << "starting new track for obs "<<i << endl;
      tracks.push_back( btBoxTrack(&params, observations[i]) );
      data_assoc_batch.push_back(std::vector< btBoxObservation >());
    } else {
      // first come first serve..?
//      cout << "putting obs "<< i<<" in batch for update in track "<<data_assoc[0] << endl;
      data_assoc_batch[ data_assoc[0] ].push_back( observations[i] );
    }
  }

  // now run batch update (this allows a tracker to incorporate several "corners" at the same time)
  for(size_t j=0;j<tracks.size();j++) {
    cout << "batch updating track "<<j<<" with n="<<data_assoc_batch[j].size()<<" observations" << endl;
    tracks[j].updateTrack(data_assoc_batch[j]);
    cout <<"  w="<<tracks[j].obs_history.rbegin()->w<< " h="<<tracks[j].obs_history.rbegin()->h<<endl;
  }

  visualizeObservations();
  visualizeTracks();
  removeOldLines();
}

void BoxTracker::removeOldLines() {
  LineVector lines;
  for(int l = newLines; l<oldLines; l++) {
    for(size_t i=tracks.size();i<10; i++)
      visualizeLines(visualization_pub, observations_msg->header.frame_id, lines,
                     l,0,0,0);
  }
  oldLines = newLines;
  newLines = 0;
}

void BoxTracker::visualizeObservations() {
  for(size_t i=0; i<observations.size(); i++) {
    visualizeLines(visualization_pub, observations_msg->header.frame_id, observations[i].visualize(),
                   newLines++,HSV_to_RGB(0.0,0.0,1.0));
  }
}

void BoxTracker::visualizeTracks() {
  for(size_t j=0; j<tracks.size(); j++) {
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

// ***************************************************************************** btBoxObservation

void btBoxObservation::setObservation(const BoxObservation &obs, const ros::Time stamp ) {
  this->stamp = stamp;

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


LineVector btBoxObservation::visualize() {
  std::vector<btVector3> points;
  points.resize(5);

  std::vector<std::pair<btVector3,btVector3> > lines;
  lines.resize(5);

  points[0] = tf * btVector3(0,0,0);
  points[1] = tf * btVector3(w,0,0);
  points[2] = tf * btVector3(w,h,0);
  points[3] = tf * btVector3(0,h,0);
  points[4] = tf * btVector3(0,0,0.1);

  lines[0].first = points[0];  lines[0].second = points[1];
  lines[1].first = points[1];  lines[1].second = points[2];
  lines[2].first = points[2];  lines[2].second = points[3];
  lines[3].first = points[3];  lines[3].second = points[0];

  lines[4].first = points[0];  lines[4].second = points[4];
  return(lines);
}

// ***************************************************************************** btBoxTrack

btBoxTrack::btBoxTrack(TrackParameters *param, btBoxObservation &obs)
:param(param) {
  obs_history.push_back(obs);
}

bool btBoxTrack::withinTolerance(btBoxObservation &obs) {
  if(obs_history.size()==0) {
    ROS_ERROR("cannot compute tolerance -- no observations in history!");
    return false;
  }
  if( fabs(obs.w - obs_history.rbegin()->w ) > param->size_tolerance) return false;
  if( fabs(obs.h - obs_history.rbegin()->h ) > param->size_tolerance) return false;
  return true;
}

void btBoxTrack::updateTrack(std::vector<btBoxObservation> &obs) {
  // could merge observations first, because they are highly correlated (come from the same image)
  // not yet implemented
  for(size_t i=0;i<obs.size();i++) {
    updateTrack(obs[i]);
  }
}

void btBoxTrack::updateTrack(btBoxObservation &obs) {
  obs_history.push_back(obs);

}

}
