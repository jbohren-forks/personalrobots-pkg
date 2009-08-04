/*
 * articulation_learner.cpp
 *
 *  Created on: Aug 3, 2009
 *      Author: sturm
 */

#include "assert.h"
#include "articulation_learner.h"

#include "vis_utils.h"
#include "opencv_latest/CvBridge.h"
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "articulation_learner");

  planar_objects::ArticulationLearner node;
  ros::spin();

  return 0;
}

namespace planar_objects
{

// Constructor
ArticulationLearner::ArticulationLearner() :
  sync(&ArticulationLearner::syncCallback, this)
{
  nh.param("~visualize", visualize, true);
  nh.param("~verbose", verbose, true);

  nh.param("~dist_vis", dist_vis, 0.02);

  nh.param("~thres_trans", thres_trans, 0.1);
  nh.param("~thres_rot", thres_rot, 10.0 / 180.0 * M_PI);

  // subscribe to topics
  tracks_sub = nh.subscribe("box_tracker/tracks", 1, sync.synchronize(&ArticulationLearner::tracksCallback,
                                                                                  this));

  oldLines = 0;
  newLines = 0;

  // advertise topics
  visualization_pub = nh.advertise<visualization_msgs::Marker> ("visualization_marker", 100);
}

void ArticulationLearner::tracksCallback(const BoxTracks::ConstPtr& tracks_msg)
{
  this->tracks_msg = tracks_msg;
}

void ArticulationLearner::syncCallback()
{
  ROS_INFO("ArticulationLearner::syncCallback(), received %d tracks",tracks_msg->get_tracks_size());

  tracks.clear();       // forget everything
  for(size_t i =0; i<tracks_msg->tracks.size();i++) {
    tracks.push_back( btBoxTrack(tracks_msg->tracks[i], tracks_msg->header.stamp) );
  }

  cout <<"here!"<<endl;

  createModels();
  updateModels();
  filterModels();

  if(visualize) {
//    visualizeTracks();
    visualizeModels();
    removeOldLines();
  }

  releaseModels();
}

void ArticulationLearner::releaseModels() {
  for(size_t i=0;i<models.size();i++) {
    delete(models[i]);
  }
  models.clear();
}

void ArticulationLearner::createModels() {
  // create models for all tracks
  for(size_t i =0; i<tracks.size();i++) {
    models.push_back(new PrismaticModel(&tracks[i]));
  }
}

void ArticulationLearner::updateModels() {
  for(size_t i =0; i<models.size();i++) {
    models[i]->findParameters();
  }
}

void ArticulationLearner::filterModels() {
  size_t i =0;
  while(i<models.size()) {
    double err_rot=999, err_trans=999;
    models[i]->getError(err_trans,err_rot);
    cout << "model "<<i<<" error is: "<<err_rot << " " << err_trans <<" from "<<models[i]->track->obs_history.size()<<"obs."<< endl;
    if(err_trans > thres_trans || err_rot > thres_rot) {
      models.erase( models.begin() + i );
    } else {
      i++;
    }
  }
  cout << "active models: "<< models.size()<< endl;
}

void ArticulationLearner::visualizeTracks() {
//  cout << "observations_msg->header.stamp="<<(long)observations_msg->header.stamp.toSec() << endl;
  for(size_t j=0; j<tracks.size(); j++) {
//    cout << "tracks[j].obs_history.rbegin()->stamp="<<(long)tracks[j].obs_history.rbegin()->stamp.toSec() << endl;
//    if( observations_msg->header.stamp > tracks[j].obs_history.rbegin()->stamp + ros::Duration(1.00) ) continue;
    for(size_t i=0; i<tracks[j].obs_history.size(); i++) {
        visualizeLines(visualization_pub, tracks_msg->header.frame_id, tracks[j].obs_history[i].visualize(),
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

void ArticulationLearner::visualizeModels() {
  for(size_t i =0; i<models.size();i++) {
    std::vector<std::pair<double,double> > range = models[i]->getRange();
    std::vector<double> q_mean;
    q_mean.resize(models[i]->getDOFs());
    for(size_t d=0;d<models[i]->getDOFs();d++) q_mean[d] = (range[d].first + range[d].second)/2;
    for(size_t d=0;d<models[i]->getDOFs();d++) {
      std::vector<double> q = q_mean;
      int n = (int) ((range[0].second - range[0].first)/dist_vis);
      cout << "n="<<n<<endl;
      for(size_t j=0;j<(size_t)n;j++) {
//        q[d] = ((n-j-1)*range[d].first + j*range[d].second)/(n-1.0);
        q[d] = ((n-j-1)*range[d].first + j*range[d].second)/(n+1);
        btBoxObservation predObs = models[i]->getPredictedObservation(q);
//        cout <<
//          "w="<<predObs.w <<
//          " h="<<predObs.h <<
//          " x="<<predObs.tf.getOrigin().x()<<
//          " y="<<predObs.tf.getOrigin().y()<<
//          " z="<<predObs.tf.getOrigin().z()<<
//          " rx="<<predObs.tf.getRotation().x()<<
//          " ry="<<predObs.tf.getRotation().y()<<
//          " rz="<<predObs.tf.getRotation().z()<<
//          " rw="<<predObs.tf.getRotation().w()<<
//          endl;
//        cout <<
//        " x="<<models[i]->center.getOrigin().x()<<
//        " y="<<models[i]->center.getOrigin().y()<<
//        " z="<<models[i]->center.getOrigin().z()<<
//        " rx="<<models[i]->center.getRotation().x()<<
//        " ry="<<models[i]->center.getRotation().y()<<
//        " rz="<<models[i]->center.getRotation().z()<<
//        " rw="<<models[i]->center.getRotation().w()<<
//        " dx="<<models[i]->direction.x()<<
//        " dy="<<models[i]->direction.y()<<
//        " dz="<<models[i]->direction.z()<<
//          endl;
        visualizeLines(visualization_pub, tracks_msg->header.frame_id,
                       predObs.visualize(),
                       newLines++,HSV_to_RGB(j/((double)n),1.0,1.0));
      }
    }
    std::vector<double> q = models[i]->getConfiguration( models[i]->track->obs_history.rbegin()->tf );
    btBoxObservation predObs = models[i]->getPredictedObservation( q );
    visualizeLines(visualization_pub, tracks_msg->header.frame_id,
                   predObs.visualize(),
                   newLines++,HSV_to_RGB( (q[0]-range[0].first)/(range[0].second-range[0].first),1.0,1.0),0.01);
  }
}


void ArticulationLearner::removeOldLines() {
  LineVector lines;
  for(int l = newLines; l<oldLines; l++) {
      visualizeLines(visualization_pub, tracks_msg->header.frame_id, lines,
                     l,0,0,0);
  }
  oldLines = newLines;
  newLines = 0;
}


}
