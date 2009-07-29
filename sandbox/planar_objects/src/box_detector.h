/*
 * box_detector.h
 *
 *  Created on: Jul 7, 2009
 *      Author: sturm
 */

#ifndef BOX_DETECTOR_H_
#define BOX_DETECTOR_H_

#include "ros/ros.h"
#include "topic_synchronizer2/topic_synchronizer.h"

#include "robot_msgs/PointCloud.h"

#include "visualization_msgs/Marker.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/StereoInfo.h"
#include "sensor_msgs/DisparityInfo.h"
#include "sensor_msgs/CamInfo.h"

#include "opencv_latest/CvBridge.h"
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "planar_objects/BoxObservations.h"

// transform library
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "cornercandidate.h"

namespace planar_objects {

class BoxDetector
{
public:
  ros::NodeHandle nh_;
  TopicSynchronizer sync_;

  // PARAMETERS
  int n_planes_max_; // number of planes to be fitted
  double point_plane_distance_; // maximally allowed point-to-plane distance

  bool show_colorized_planes;
  bool show_convex_hulls;
  bool show_lines;
  bool show_images;
  bool show_corners;
  bool show_rectangles;

  bool verbose;

  int select_frontplane;
  int max_lines;
  int max_corners;

  double min_precision;
  double min_recall;

  double rect_min_size;
  double rect_max_size;
  double rect_max_displace;

  // reprojection matrix
  double RP[16];
  double P[16];
  IplImage* pixDebug;

  // MESSAGES - INCOMING
  ros::Subscriber cloud_sub_;
  robot_msgs::PointCloudConstPtr cloud_;

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
  ros::Publisher visualization_pub_;
  ros::Publisher observations_pub_;
  //  sensor_msgs::Image pimage_;
  //  sensor_msgs::CvBridge pbridge;

  // Constructor
  BoxDetector();

  // Callbacks
  void cloudCallback(const robot_msgs::PointCloud::ConstPtr& point_cloud);
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

  void findFrontAndBackPlane(int& frontplane, int& backplane, std::vector<std::vector<int> >& indices, std::vector<
      std::vector<double> >& plane_coeff);
  void findCornerCandidates(IplImage* pixOccupied, IplImage *pixFree, IplImage* pixUnknown, IplImage* &pixDist,
                            std::vector<double> & plane_coeff, std::vector<CornerCandidate> &corner,int id);
  void visualizeLines(std::vector<std::pair<btVector3, btVector3> > lines, int id = 1, double r = 1.0, double b = 1.0,
                      double g = 1.0);
  std::vector<CornerCandidate> groupCorners(std::vector<CornerCandidate> &corner, double group_dist = 20);
  void visualizeCorners(std::vector<CornerCandidate> &corner, int id = 0);
  void visualizeFrontAndBackPlane(int frontplane, int backplane, const robot_msgs::PointCloud& cloud, std::vector<
      std::vector<int> >& plane_indices, std::vector<robot_msgs::PointCloud>& plane_cloud, std::vector<std::vector<
      double> >& plane_coeff, robot_msgs::PointCloud& outside, bool showConvexHull = false);
  void visualizePlanes(const robot_msgs::PointCloud& cloud, std::vector<
      std::vector<int> >& plane_indices, std::vector<robot_msgs::PointCloud>& plane_cloud, std::vector<std::vector<
      double> >& plane_coeff, robot_msgs::PointCloud& outside, bool showConvexHull = false);

  void visualizeRectangles3d(std::vector<CornerCandidate> &corner, int id = 0);
  void visualizeRectangles2d(std::vector<CornerCandidate> &corner);
  void visualizeRectangle2d(CornerCandidate &corner);
  void findRectangles(std::vector<CornerCandidate> &corner, IplImage* pixDist);
  std::vector<CornerCandidate> filterRectanglesBySupport2d(std::vector<CornerCandidate> &corner, IplImage* pixOccupied,
                                                           double min_support = 0.8);
  std::vector<CornerCandidate> filterRectanglesBySupport3d(std::vector<CornerCandidate> &corner,
                                                           const robot_msgs::PointCloud& cloud,
                                                           std::vector<int> & plane_indices, double min_support = 0.8);
  void initializeRectangle(CornerCandidate &corner, IplImage* pixDist);
};

}

int main(int argc, char** argv);

#endif /* BOX_DETECTOR_H_ */
