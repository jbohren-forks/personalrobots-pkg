/*
 * planar_node.h
 *
 *  Created on: Jul 7, 2009
 *      Author: sturm
 */

#ifndef PLANAR_NODE_H_
#define PLANAR_NODE_H_

#include "ros/ros.h"
#include "topic_synchronizer2/topic_synchronizer.h"
#include "robot_msgs/PointCloud.h"
#include "visualization_msgs/Marker.h"


class PlanarNode
{
public:
  ros::NodeHandle nh_;
  TopicSynchronizer sync_;

  // PARAMETERS
  int n_planes_max_; // number of planes to be fitted

  // MESSAGES - INCOMING
  ros::Subscriber cloud_sub_;
  robot_msgs::PointCloudConstPtr cloud_;

  // MESSAGES - OUTGOING
  ros::Publisher cloud_planes_pub_;
  ros::Publisher cloud_outliers_pub_;
  ros::Publisher visualization_pub_;

  // Constructor
  PlanarNode();

  // Callbacks
  void cloudCallback(const robot_msgs::PointCloud::ConstPtr& point_cloud);
  void syncCallback();

  // Main loop
  bool spin();
};

int main(int argc, char** argv);

#endif /* PLANAR_NODE_H_ */
