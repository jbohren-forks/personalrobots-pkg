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

#include "sensor_msgs/PointCloud.h"

#include "visualization_msgs/Marker.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/StereoInfo.h"
#include "sensor_msgs/DisparityInfo.h"
#include "sensor_msgs/CamInfo.h"

#include "opencv_latest/CvBridge.h"
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

// transform library
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class PlanarNode
{
public:
  ros::NodeHandle nh_;
  TopicSynchronizer sync_;

  // PARAMETERS
  int n_planes_max_; // number of planes to be fitted
  double point_plane_distance_; // maximally allowed point-to-plane distance

  // reprojection matrix
  double RP[16];

  // MESSAGES - INCOMING
  ros::Subscriber cloud_sub_;
  sensor_msgs::PointCloudConstPtr cloud_;

  ros::Subscriber disp_sub_;
  sensor_msgs::ImageConstPtr dimage_;
  sensor_msgs::CvBridge dbridge_;

  ros::Subscriber limage_sub_;
  sensor_msgs::ImageConstPtr limage_;
  sensor_msgs::CvBridge lbridge_;

  ros::Subscriber rimage_sub_;
  sensor_msgs::ImageConstPtr rimage_;
  sensor_msgs::CvBridge rbridge_;

  ros::Subscriber dinfo_sub_;
  sensor_msgs::DisparityInfoConstPtr dinfo_;

  ros::Subscriber linfo_sub_;
  sensor_msgs::CamInfoConstPtr linfo_;

  ros::Subscriber rinfo_sub_;
  sensor_msgs::CamInfoConstPtr rinfo_;

  tf::TransformBroadcaster broadcaster_;

  ros::Time currentTime;
  ros::Time lastTime;
  ros::Duration lastDuration;


  // MESSAGES - OUTGOING
  ros::Publisher cloud_planes_pub_;
  ros::Publisher cloud_outliers_pub_;
  ros::Publisher visualization_pub_;
//  sensor_msgs::Image pimage_;
//  sensor_msgs::CvBridge pbridge;

  // Constructor
  PlanarNode();

  // Callbacks
  void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& point_cloud);
  void dispCallback(const sensor_msgs::Image::ConstPtr& disp_img);
  void dinfoCallback(const sensor_msgs::DisparityInfo::ConstPtr& disp_img);
  void limageCallback(const sensor_msgs::Image::ConstPtr& left_img);
  void rimageCallback(const sensor_msgs::Image::ConstPtr& right_img);
  void linfoCallback(const sensor_msgs::CamInfo::ConstPtr& rinfo);
  void rinfoCallback(const sensor_msgs::CamInfo::ConstPtr& linfo);
  void syncCallback();

  void buildRP();
  btVector3 calcPt(int x, int y, std::vector<double>& coeff);

  // Main loop
  bool spin();
};

int main(int argc, char** argv);

#endif /* PLANAR_NODE_H_ */
