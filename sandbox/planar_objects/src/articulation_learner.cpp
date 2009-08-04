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

  // subscribe to topics
  tracks_sub = nh.subscribe("box_tracker/tracks", 1, sync.synchronize(&ArticulationLearner::tracksCallback,
                                                                                  this));

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
  cout <<"here2!"<<endl;
  for(size_t i =0; i<models.size();i++) {
    cout <<"hello!"<<endl;
    models[i]->findParameters();
  }
}

void ArticulationLearner::filterModels() {
  for(size_t i =0; i<tracks.size();i++) {
//    models[i]->findParameters();
  }
}

}
