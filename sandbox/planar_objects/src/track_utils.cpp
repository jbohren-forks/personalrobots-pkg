/*
 * track_utils.cpp
 *
 *  Created on: Aug 3, 2009
 *      Author: sturm
 */

// ***************************************************************************** btBoxObservation

#include "track_utils.h"

namespace planar_objects {

btBoxObservation::btBoxObservation( ) {
  w = 0;
  h = 0;
  precision = 0;
  recall = 0;
}

btBoxObservation::btBoxObservation(const BoxObservation &obs , const ros::Time stamp ) {
  setObservation(obs,stamp);
  *this = this->getAmbiguity(1);
}

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
	for(int i=0;i<8;i++) {
		allCorners.push_back(getAmbiguity(i));
	}
	return(allCorners);
}

btBoxObservation btBoxObservation::getAmbiguity(int num) {
  btBoxObservation alt = *this;
  btTransform tfinv = tf *
      btTransform(btQuaternion(btVector3(1,1,0),2 * M_PI/2),btVector3(0,0,0));

  switch(num) {
  case 0:
		  alt.w = w; alt.h = h;
		  alt.tf =  tf * btTransform(btQuaternion(btVector3(0,0,1),0 * M_PI/2),btVector3(0,0,0));
		  break;
  case 1:
  case -1:
		  alt.w = w; alt.h = h;
		  alt.tf =  tf * btTransform(btQuaternion(btVector3(0,0,1),2 * M_PI/2),btVector3(w,h,0));
		  break;
  case 2:
  case -3:
		  alt.w = h; alt.h = w;
		  alt.tf =  tf * btTransform(btQuaternion(btVector3(0,0,1),1 * M_PI/2),btVector3(w,0,0));
		  break;
  case 3:
  case -2:
		  alt.w = h; alt.h = w;
		  alt.tf =  tf * btTransform(btQuaternion(btVector3(0,0,1),3 * M_PI/2),btVector3(0,h,0));
		  break;
  case 4:
  case -4:
		  alt.w = h; alt.h = w;
		  alt.tf =  tfinv * btTransform(btQuaternion(btVector3(0,0,1),0 * M_PI/2),btVector3(0,0,0));
		  break;
  case 5:
  case -5:
		  alt.w = h; alt.h = w;
		  alt.tf =  tfinv * btTransform(btQuaternion(btVector3(0,0,1),2 * M_PI/2),btVector3(h,w,0));
		  break;
  case 6:
  case -7:
		  alt.w = w; alt.h = h;
		  alt.tf =  tfinv * btTransform(btQuaternion(btVector3(0,0,1),1 * M_PI/2),btVector3(h,0,0));
		  break;
  case 7:
  case -6:
		  alt.w = w; alt.h = h;
		  alt.tf =  tfinv * btTransform(btQuaternion(btVector3(0,0,1),3 * M_PI/2),btVector3(0,w,0));
		  break;
  }
  return(alt);
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

btBoxTrack::btBoxTrack() {
  param.rotation_tolerance = 0.0;
  param.translation_tolerance = 0.0;
  param.size_tolerance = 0.0;
  id = -1;
}

btBoxTrack::btBoxTrack(TrackParameters param, btBoxObservation &obs, int id)
:param(param),id(id) {
  obs_history.push_back(obs);
}

btBoxTrack::btBoxTrack(const BoxTrack&  msg, const ros::Time stamp) {
  obs_history.clear();
  for(size_t i=0;i<msg.obs.size();i++) {
    obs_history.push_back( btBoxObservation( msg.obs[i],stamp ) );
  }
}

bool btBoxTrack::withinTolerance(btBoxObservation &obs) {
  if(obs_history.size()==0) {
    ROS_ERROR("cannot compute tolerance -- no observations in history!");
    return false;
  }
  if( fabs(obs.w - obs_history.rbegin()->w ) > param.size_tolerance) return false;
  if( fabs(obs.h - obs_history.rbegin()->h ) > param.size_tolerance) return false;

  btTransform dist = obs_history.rbegin()->tf.inverseTimes(obs.tf);

  double phi = dist.getRotation().getAngle();
  double d = dist.getOrigin().length();


  if(phi > param.rotation_tolerance) return false;
  if(d > param.translation_tolerance) return false;

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
