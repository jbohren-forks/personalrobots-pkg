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
  nh.param("~visualize", show_boxes, true);
  nh.param("~verbose", verbose, true);

  nh.param("~translation_tolerance", params.translation_tolerance, 0.1);
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
    btBoxObservation obs;
    obs.setObservation(observations_msg->obs[i], observations_msg->header.stamp);
    observations.push_back(obs);
  }

  if(observations.size()==0) return;
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
      tracks.push_back( btBoxTrack(&params, observations[i], track_ids++) );
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

  removeOldTracks(ros::Duration(1.00));
  visualizeObservations();
  visualizeTracks();
  removeOldLines();
  sendTracks();
}

void BoxTracker::removeOldTracks(ros::Duration timeout) {
  size_t j = 0;
  while (j<tracks.size()) {
    if( observations_msg->header.stamp > tracks[j].obs_history.rbegin()->stamp + timeout ) {
      tracks.erase(tracks.begin() + j);
    } else {
      j++;
    }
  }
}

void BoxTracker::sendTracks() {
  BoxTracks tracks_msg;
  tracks_msg.set_tracks_size(tracks.size());
  for(size_t j=0; j<tracks.size(); j++) {
    tracks_msg.tracks[j] = tracks[j].getTrackMessage();
  }
  tracks_pub.publish(tracks_msg);
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
                   newLines++,HSV_to_RGB(0.0 * i,0.0,1.0));
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

std::vector<btBoxObservation> btBoxObservation::listAmbiguity() {
  std::vector<btBoxObservation> allCorners;
  btBoxObservation alt = *this;

  alt.w = w; alt.h = h;
  alt.tf =  tf * btTransform(btQuaternion(btVector3(0,0,1),0 * M_PI/2),btVector3(0,0,0));
  allCorners.push_back(alt);

  alt.w = w; alt.h = h;
  alt.tf =  tf * btTransform(btQuaternion(btVector3(0,0,1),2 * M_PI/2),btVector3(w,h,0));
  allCorners.push_back(alt);

  alt.w = h; alt.h = w;
  alt.tf =  tf * btTransform(btQuaternion(btVector3(0,0,1),1 * M_PI/2),btVector3(w,0,0));
  allCorners.push_back(alt);

  alt.w = h; alt.h = w;
  alt.tf =  tf * btTransform(btQuaternion(btVector3(0,0,1),3 * M_PI/2),btVector3(0,h,0));
  allCorners.push_back(alt);

  btTransform tfinv = tf *
      btTransform(btQuaternion(btVector3(1,1,0),2 * M_PI/2),btVector3(0,0,0));

  alt.w = h; alt.h = w;
  alt.tf =  tfinv * btTransform(btQuaternion(btVector3(0,0,1),0 * M_PI/2),btVector3(0,0,0));
  allCorners.push_back(alt);

  alt.w = h; alt.h = w;
  alt.tf =  tfinv * btTransform(btQuaternion(btVector3(0,0,1),2 * M_PI/2),btVector3(h,w,0));
  allCorners.push_back(alt);

  alt.w = w; alt.h = h;
  alt.tf =  tfinv * btTransform(btQuaternion(btVector3(0,0,1),1 * M_PI/2),btVector3(h,0,0));
  allCorners.push_back(alt);

  alt.w = w; alt.h = h;
  alt.tf =  tfinv * btTransform(btQuaternion(btVector3(0,0,1),3 * M_PI/2),btVector3(0,w,0));
  allCorners.push_back(alt);

// cout << "ambiguities:"<<allCorners.size()<<endl;

  return(allCorners);
}

BoxObservation btBoxObservation::getBoxObservation() {
  BoxObservation obs;
  obs.transform.translation.x = tf.getOrigin().x();
  obs.transform.translation.y = tf.getOrigin().y();
  obs.transform.translation.z = tf.getOrigin().z();

  obs.transform.rotation.x = tf.getRotation().x();
  obs.transform.rotation.y = tf.getRotation().y();
  obs.transform.rotation.z = tf.getRotation().z();
  obs.transform.rotation.w = tf.getRotation().w();

  obs.w = w;
  obs.h = h;
  obs.precision = precision;
  obs.recall = recall;

  return(obs);
}

// ***************************************************************************** btBoxTrack

btBoxTrack::btBoxTrack(TrackParameters *param, btBoxObservation &obs, int id)
:param(param),id(id) {
  obs_history.push_back(obs);
}

bool btBoxTrack::withinTolerance(btBoxObservation &obs) {
  if(obs_history.size()==0) {
    ROS_ERROR("cannot compute tolerance -- no observations in history!");
    return false;
  }
  if( fabs(obs.w - obs_history.rbegin()->w ) > param->size_tolerance) return false;
  if( fabs(obs.h - obs_history.rbegin()->h ) > param->size_tolerance) return false;

  btTransform dist = obs_history.rbegin()->tf.inverseTimes(obs.tf);

  double phi = dist.getRotation().getAngle();
  double d = dist.getOrigin().length();


  if(phi > param->rotation_tolerance) return false;
  if(d > param->translation_tolerance) return false;

//  cout << " d= "<<d<<" phi= "<<phi/M_PI*180.0<<endl;

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

BoxTrack btBoxTrack::getTrackMessage() {
  BoxTrack result;
  result.id = id;
  result.set_obs_size( obs_history.size() );
  for(size_t i=0;i<obs_history.size();i++) {
    result.obs[i] = obs_history[i].getBoxObservation();
  }
  return(result);
}

}
