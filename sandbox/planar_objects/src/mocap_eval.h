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
#include "planar_objects/MocapEvalObservations.h"

#include "vis_utils.h"
#include "track_utils.h"

#include "sensor_msgs/Image.h"
#include "stereo_msgs/StereoInfo.h"
#include "stereo_msgs/DisparityInfo.h"
#include "sensor_msgs/CameraInfo.h"

#include "opencv_latest/CvBridge.h"
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"


namespace planar_objects
{

class MocapEval
{
public:
  ros::NodeHandle nh;

  int oldLines;
  int newLines;

  double min_precision;
  double min_recall;
  int plane_limit;		// 0: don't limit visible planes
						  // >0: only consider first n planes

  int calibration_type;	// 0: calibration off
						  //1 : differencing
						  //2: least-squares optimization

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
  stereo_msgs::DisparityInfoConstPtr dinfo_;
  stereo_msgs::DisparityInfo dinfo;

  ros::Subscriber linfo_sub_;
  sensor_msgs::CameraInfoConstPtr linfo_;
  sensor_msgs::CameraInfo linfo;

  ros::Subscriber rinfo_sub_;
  sensor_msgs::CameraInfoConstPtr rinfo_;
  sensor_msgs::CameraInfo rinfo;

  // MESSAGES - OUTGOING
  ros::Publisher visualization_pub;
  ros::Publisher cloud_pub;
  ros::Publisher output_pub;

  tf::TransformListener tf;
  tf::MessageNotifier<BoxObservations>* notifier;


  std::vector<btTransform> delta;
  std::vector<btTransform> deltaSteps;
  btTransform bestDelta;

  // Constructor
  MocapEval();

  // Callbacks
  void observationsCallback(const BoxObservations::ConstPtr& observations);
  void mocapCallback(const mocap_msgs::MocapSnapshotConstPtr& mocap_msg);
  void callback(
    const tf::MessageNotifier<BoxObservations>::MessagePtr& msg_in);
  void dispCallback(const sensor_msgs::Image::ConstPtr& disp_img);
  void dinfoCallback(const stereo_msgs::DisparityInfo::ConstPtr& disp_img);
  void limageCallback(const sensor_msgs::Image::ConstPtr& left_img);
  void rimageCallback(const sensor_msgs::Image::ConstPtr& right_img);
  void linfoCallback(const sensor_msgs::CameraInfo::ConstPtr& rinfo);
  void rinfoCallback(const sensor_msgs::CameraInfo::ConstPtr& linfo);

  btTransform transformToBaseLink(std::string fromFrame, std::string toFrame, btTransform pose );
  void sendPointCloud();
  void visualizeObservations();
  void removeOldLines();

  void findBestDelta();
  void visualizeObs( btBoxObservation mocap_obs);
  bool isVisible(btBoxObservation obs);

  void evaluateData( btBoxObservation mocap_obs, btBoxObservation visual_obs);
  void evaluateData( btBoxObservation mocap_obs, std::vector<btBoxObservation> visual_obs);
};

}

int main(int argc, char** argv);

#endif /* BOX_TRACKER_H_ */
