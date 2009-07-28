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

class CornerCandidate
{
public:
  double *P;
  double *RP;

  btTransform tf;

  double x;
  double y;

  double angle;
  double dist1;
  double dist2;

  double w;
  double h;

  btVector3 points3d[4];
  CvPoint points2d[4];

  void updatePoints3d();
  void updatePoints2d();
  double computeDistance(IplImage* distImage);
  double computeSupport(IplImage* pixOccupied,IplImage* pixDebug=NULL);
  void optimizeWidth(IplImage* distImage, double a=-0.5,double b=+0.5,int steps=20);
  void optimizeHeight(IplImage* distImage, double a=-0.5,double b=+0.5,int steps=20);
  void optimizePhi(IplImage* distImage, double a=-0.5,double b=+0.5,int steps=20);
  void optimizeX(IplImage* distImage, double a=-0.5,double b=+0.5,int steps=20);
  void optimizeY(IplImage* distImage, double a=-0.5,double b=+0.5,int steps=20);
};

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
  ros::Publisher cloud_outliers_pub_;
  ros::Publisher visualization_pub_;
  //  sensor_msgs::Image pimage_;
  //  sensor_msgs::CvBridge pbridge;

  // Constructor
  PlanarNode();

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
  void findCornerCandidates(IplImage* pixOccupied, IplImage *pixFree, IplImage* pixUnknown,IplImage* &pixDist,
      std::vector<double> & plane_coeff, std::vector<      CornerCandidate> &corner);
  void visualizeLines(std::vector<std::pair<btVector3, btVector3> > lines,int id=1,double r=1.0,double b=1.0,double g=1.0);
  std::vector<CornerCandidate> groupCorners(std::vector<CornerCandidate> &corner,double group_dist = 20);
  void visualizeCorners(std::vector<CornerCandidate> &corner,int id=0);
  void visualizeFrontAndBackPlane(int frontplane, int backplane, const robot_msgs::PointCloud& cloud, std::vector<
      std::vector<int> >& plane_indices, std::vector<robot_msgs::PointCloud>& plane_cloud, std::vector<std::vector<
      double> >& plane_coeff, robot_msgs::PointCloud& outside,bool showConvexHull=false);

  void visualizeRectangles3d(std::vector<CornerCandidate> &corner,int id=0);
  void visualizeRectangles2d(std::vector<CornerCandidate> &corner);
  void visualizeRectangle2d(CornerCandidate &corner);
  void findRectangles(std::vector<CornerCandidate> &corner, IplImage* pixDist);
  std::vector<CornerCandidate> filterRectanglesBySupport(std::vector<CornerCandidate> &corner, IplImage* pixOccupied, double min_support=0.8);
  void initializeRectangle(CornerCandidate &corner, IplImage* pixDist);
};

int main(int argc, char** argv);

#endif /* PLANAR_NODE_H_ */
