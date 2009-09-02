/*
 * box_tracker.h
 *
 *  Created on: Jul 28, 2009
 *      Author: sturm
 */

#ifndef MOCAP_EVAL_H_
#define MOCAP_EVAL_H_

#include "ros/ros.h"
#include "topic_synchronizer2/topic_synchronizer.h"
#include "tf/message_notifier.h"

#include <tf/transform_listener.h>

#include "visualization_msgs/Marker.h"
#include "mocap_msgs/MocapSnapshot.h"

#include "planar_objects/BoxObservations.h"

#include "vis_utils.h"
#include "track_utils.h"

namespace planar_objects
{

class MocapEval
{
public:
  ros::NodeHandle nh;

  int oldLines;
  int newLines;

  roslib::Header header;
  // MESSAGES - INCOMING
  ros::Subscriber observations_sub;
  BoxObservationsConstPtr observations_msg;
  std::vector<btBoxObservation> observations;

  ros::Subscriber mocap_sub;
  mocap_msgs::MocapSnapshotConstPtr mocap_msg;
  std::map<int,btVector3> mocap_markers;
  std::map<int,btTransform> mocap_bodies;
  btBoxObservation mocap_obs;

  // MESSAGES - OUTGOING
  ros::Publisher visualization_pub;
  ros::Publisher cloud_pub;

  tf::TransformListener tf;
  tf::MessageNotifier<BoxObservations>* notifier;


  // Constructor
  MocapEval();

  // Callbacks
  void observationsCallback(const BoxObservations::ConstPtr& observations);
  void mocapCallback(const mocap_msgs::MocapSnapshotConstPtr& mocap_msg);
  void callback(
    const tf::MessageNotifier<BoxObservations>::MessagePtr& msg_in);

  btTransform transformToBaseLink(std::string fromFrame, std::string toFrame, btTransform pose );
  void sendPointCloud();
  void visualizeObservations();
  void removeOldLines();
};

}

int main(int argc, char** argv);

#endif /* BOX_TRACKER_H_ */
